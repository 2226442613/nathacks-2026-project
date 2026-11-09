"""TCP ASCII duration client. No automatic motion retry, reconnect or resume.

Requests are serialized because the wire has no IDs. MOVE acceptance releases the
request lock; asynchronous completion uses a separate condition, leaving heartbeat
and priority STOP/ESTOP available throughout motion.
"""
from dataclasses import dataclass
from collections import deque
import socket
import threading
import time

from .protocol import (LineFramer, ProtocolError, validate_command, expected_reply,
                       parse_safety, REQUIRED_CAPABILITIES, FIXED_PWM, MAX_MOVE_MS, SIMPLE_CAPS)
from .logging import WireLogger


class CommunicationError(RuntimeError):pass


@dataclass(frozen=True)
class CommandResult:
    command:str
    transport_accepted:bool
    firmware_accepted:bool|None
    execution_completed:bool=False
    response:str|None=None
    execution_state:str='not_tracked'  # awaiting_ack/accepted/completed/aborted/unknown/rejected
    motion_id:int|None=None           # Host-only ID; never added to the wire.
    detail:str|None=None


class RobotClient:
    def __init__(self,host=None,port=3333,*,mode='simulation',timeout=.8,heartbeat_interval=.2,logger=None,firmware_profile='safe_v1_1',connect_timeout=None):
        if firmware_profile not in {'safe_v1_1','simple'}:raise ValueError('Explicit firmware profile required')
        self.firmware_profile=firmware_profile;self.simple_supported=False
        if mode not in {'simulation','physical'}:raise ValueError('Select simulation or physical mode')
        if mode=='simulation':
            if host not in {None,'localhost','127.0.0.1','::1'}:raise ValueError('Simulation only connects to loopback')
            host='127.0.0.1' if host in {None,'localhost'} else host
        elif not host:raise ValueError('Physical mode requires an explicit ESP32 host')
        if not 0<timeout<=5 or not 0<heartbeat_interval<=.3:raise ValueError('Invalid timeout/heartbeat interval')
        if connect_timeout is None:connect_timeout=timeout
        if isinstance(connect_timeout,bool) or not 0<connect_timeout<=10:raise ValueError('Connect timeout must be > 0 and <= 10 seconds')
        self.host,self.port,self.mode=host,port,mode
        self.timeout,self.heartbeat_interval=timeout,heartbeat_interval
        self.connect_timeout=connect_timeout
        self.logger=logger or WireLogger();self.sock=None;self.armed=False
        self.max_pwm=None;self.max_duration_ms=None
        self.estop_latched=False;self.safety_supported=False;self.last_error=None
        self.write_lock=threading.RLock();self.request_lock=threading.Lock()
        self.condition=threading.Condition(threading.RLock());self.pending=None
        self.telemetry=deque(maxlen=1000);self.stop_event=threading.Event()
        self.reader=None;self.heartbeat=None;self.last_rx=None
        self.needs_barrier=False;self.generation=0
        self.moves={};self.current_move=None;self.next_motion_id=1
        self.rx_sequence=0;self.events=deque(maxlen=1000);self.cleaning_after=None
        self.stop_confirmation_pending=False

    def is_connected(self):return self.sock is not None and not self.stop_event.is_set()
    def get_last_error(self):return self.last_error

    def _disarm(self):
        self.armed=False;self.max_pwm=None;self.max_duration_ms=None

    def connect(self):
        self.disconnect()
        self.stop_event=threading.Event();self.last_error=None;self.safety_supported=False
        self.simple_supported=False
        self.stop_confirmation_pending=False
        self.needs_barrier=False
        try:
            self.logger.emit('INFO',f'TCP connecting {self.host}:{self.port}; connection timeout {self.connect_timeout:g}s')
            try:
                sock=socket.create_connection((self.host,self.port),timeout=self.connect_timeout)
            except OSError as exc:
                raise CommunicationError(f'TCP connection to {self.host}:{self.port} failed: {exc}; '
                    'no commands sent. Check ESP32 current IP, shared Wi-Fi/hotspot, TCP server and other connected controllers; no automatic retry') from exc
            sock.settimeout(.05);self.sock=sock
            with self.condition:
                self.telemetry.clear();self.pending=None;self.current_move=None
                self.events.clear();self.rx_sequence=0;self.cleaning_after=None
            self.reader=threading.Thread(target=self._read,args=(sock,self.stop_event),daemon=True);self.reader.start()
            self.logger.emit('INFO',f'{self.mode.upper()} connected {self.host}:{self.port}; DISARMED; TX is not execution')
            self._request('PING')
            result=self._request('CAPS')
            if self.firmware_profile=='simple':
                if result.response!=SIMPLE_CAPS:raise CommunicationError('Exact NATHACK_V2_SIMPLE capabilities required')
                self.simple_supported=True
                # SIMPLE has no controller latch to reset. Connection never replays a task.
                self.estop_latched=False
                self.logger.emit('WARN','SIMPLE firmware: no heartbeat watchdog or latched ESTOP; bounded MOVE and STOP only')
                return self
            capabilities=set((result.response or '').split(',')[1:])
            self.safety_supported=bool(result.firmware_accepted and
                REQUIRED_CAPABILITIES<=capabilities and 'MOVE_PWM_0_255' not in capabilities)
            if self.safety_supported:
                result=self._request('SAFETY')
                if not result.firmware_accepted:raise CommunicationError('Required controller safety report rejected')
                safety=parse_safety(result.response)
                if safety['estop'] or safety['watchdog_expired']:self.estop_latched=True
            else:self.logger.emit('WARN','Missing duration/safety support or legacy PWM firmware; physical motion disabled')
            self.heartbeat=threading.Thread(target=self._heartbeat,args=(self.stop_event,),daemon=True);self.heartbeat.start()
            return self
        except (OSError,CommunicationError,ProtocolError) as exc:
            self._fail(str(exc));self.disconnect();raise CommunicationError(str(exc)) from exc

    def _check_motion_permission(self,line):
        if not (line.startswith(('MOVE,','TARGET,')) or line=='ARRIVED'):return
        if self.stop_confirmation_pending:raise CommunicationError('STOP confirmation pending; motion prohibited')
        if self.estop_latched:
            raise PermissionError('SIMPLE session fault; stop task and reconnect explicitly' if self.firmware_profile=='simple' else 'Emergency stop/fault latched; explicit reset required')
        if self.firmware_profile=='simple':
            if not self.simple_supported:raise CommunicationError('SIMPLE capabilities not confirmed')
            return
        if line.startswith('MOVE,') and not self.safety_supported:
            raise CommunicationError('Duration protocol and safety support required; MOVE meaning will not be guessed')
        if self.mode=='physical':
            if not line.startswith('MOVE,'):raise PermissionError('Physical cleaning sequence execution is outside Module 07; simulation only')
            if not self.armed or self.max_pwm is None or self.max_duration_ms is None:
                raise PermissionError('Physical mode DISARMED')
            if self.max_pwm<FIXED_PWM:raise PermissionError('Fixed PWM170 exceeds operator-validated power limit')
            if int(line.split(',')[2])>self.max_duration_ms:
                raise PermissionError('Duration exceeds operator-validated single-action limit in milliseconds')

    def _write(self,line,generation=None):
        with self.write_lock:
            if generation is not None and generation!=self.generation:
                raise CommunicationError('Queued command cancelled by priority stop')
            if not self.is_connected():raise CommunicationError('Communication unavailable; reconnect explicitly')
            self._check_motion_permission(line)
            try:self.sock.sendall((line+'\n').encode('ascii'))
            except OSError as exc:
                self._fail('Transmission failed; partial delivery possible; command will not be retried')
                raise CommunicationError(self.last_error) from exc
            self.logger.emit('TX',line)

    def _request(self,line,generation=None):
        with self.request_lock:
            if generation is not None and generation!=self.generation:
                raise CommunicationError('Queued command cancelled by priority stop')
            if self.needs_barrier:
                barrier=self._exchange('PING')
                if barrier.response!='PONG':
                    self._fail('Priority stop response barrier failed; reconnect explicitly')
                    raise CommunicationError(self.last_error)
                self.needs_barrier=False
                # No stop ACK means no confirmed abort. Never reuse the wire for
                # another motion when an untagged DONE could still be outstanding.
                with self.condition:
                    unresolved=self.current_move and self.current_move['stop_requested'] and self.current_move['state']=='unknown'
                if unresolved:
                    self._fail('Priority stop was not acknowledged; motion outcome unknown')
                    raise CommunicationError(self.last_error)
            return self._exchange(line,generation)

    def _exchange(self,line,generation=None):
        pending={'command':line,'response':None,'error':None,'move':None}
        with self.condition:
            if line.startswith('MOVE,'):
                if self.current_move and self.current_move['state'] in {'awaiting_ack','accepted','unknown'}:
                    raise CommunicationError('MOVE already in flight; wait for completion or stop explicitly')
                move={'command':line,'id':self.next_motion_id,'transport':False,'accepted':None,
                      'state':'awaiting_ack','response':None,'detail':None,'stop_requested':False,
                      'sent_at':None,
                      'deadline':time.monotonic()+int(line.split(',')[2])/1000+self.timeout}
                self.next_motion_id+=1;self.moves[move['id']]=move;self.current_move=move;pending['move']=move
            self.pending=pending
        try:
            with self.write_lock:
                with self.condition:
                    if pending['error']:raise CommunicationError(pending['error'])
                    if pending['move']:pending['move']['sent_at']=time.monotonic()
                self._write(line,generation)
                if pending['move']:pending['move']['transport']=True
            deadline=time.monotonic()+self.timeout
            with self.condition:
                while pending['response'] is None and pending['error'] is None:
                    left=deadline-time.monotonic()
                    if left<=0:break
                    self.condition.wait(left)
                if pending['error']:raise CommunicationError(pending['error'])
                response=pending['response']
            if response is None:
                self._fail(f'Command timed out: {line.split(",")[0]}; no retry')
                raise CommunicationError(self.last_error)
            if pending['move']:
                with self.condition:return self._move_snapshot(pending['move'])
            return CommandResult(line,True,not response.startswith('ERR,'),False,response)
        finally:
            with self.condition:
                move=pending['move']
                if move and not move['transport'] and move['state']=='awaiting_ack':
                    move['state']='rejected';move['accepted']=False;move['detail']='Not transmitted'
                if self.pending is pending:self.pending=None
                self.condition.notify_all()

    @staticmethod
    def _move_snapshot(move):
        return CommandResult(move['command'],move['transport'],move['accepted'],
            move['state']=='completed',move['response'],move['state'],move['id'],move['detail'])

    def get_move_result(self,result=None):
        """Return a current immutable snapshot; an OK,MOVE result is not completion."""
        with self.condition:
            move=self.moves.get(result.motion_id) if result is not None else self.current_move
            if move is None:raise ValueError('No tracked MOVE result')
            return self._move_snapshot(move)

    def wait_for_move(self,result=None,*,timeout=None,check=None):
        """Wait without holding the request lock. Heartbeats continue independently.

        A completion timeout closes/latches the connection and returns UNKNOWN;
        even a late DONE cannot make an uncertain action eligible for replay.
        """
        if timeout is not None and (isinstance(timeout,bool) or not isinstance(timeout,(int,float)) or
                                    not 0<timeout<=MAX_MOVE_MS/1000+5):
            raise ValueError('Invalid MOVE completion wait timeout')
        snapshot=self.get_move_result(result)
        move=self.moves[snapshot.motion_id]
        deadline=move['deadline'] if timeout is None else min(move['deadline'],time.monotonic()+timeout)
        while True:
            with self.condition:
                pending=move['state'] in {'awaiting_ack','accepted'} or (move['state']=='unknown' and move['stop_requested'] and self.is_connected())
                if not pending:break
                left=deadline-time.monotonic()
                if left<=0:break
                self.condition.wait(min(left,.05) if check else left)
                # DONE/fault may have woken this wait. Stationary settling and
                # reacquisition belong to the caller, not the in-motion monitor.
                monitor_pending=move['state'] in {'awaiting_ack','accepted'}
            if check and monitor_pending:check()
        if pending:self._fail('MOVE completion timed out; outcome unknown; no retry')
        return self.get_move_result(snapshot)

    def begin_cleaning(self,surface,fluid):
        if surface not in {'GROUND','WALL'} or fluid not in {'BLOOD','OTHER'}:raise ValueError('Invalid cleaning selection')
        result=self.send_command(f'TARGET,{surface},{fluid}')
        if result.response!='OK,TARGET':raise CommunicationError('TARGET rejected: '+str(result.response))
        with self.condition:self.cleaning_after=self.rx_sequence
        result=self.send_command('ARRIVED')
        if result.response!='OK,ARRIVED':raise CommunicationError('ARRIVED rejected: '+str(result.response))
        return result

    def wait_for_cleaning(self,*,timeout=30.,check=None):
        if not 0<timeout<=120 or self.cleaning_after is None:raise ValueError('Start a cleaning request before waiting')
        deadline=time.monotonic()+timeout
        invalid_done=False
        while True:
            with self.condition:
                lines=[line for sequence,line in self.events if sequence>self.cleaning_after]
                if any(line in {'EVENT,CLEAN_MOTOR_TIMEOUT','EVENT,PUMP_TIMEOUT','EVENT,LINK_LOST_ABORTED','EVENT,WATCHDOG_STOP','OK,STOP','OK,ESTOP'} or line.startswith('ERR,') for line in lines):
                    raise CommunicationError('Cleaning aborted or faulted; no automatic retry')
                if not self.is_connected():raise CommunicationError('Cleaning outcome unknown after connection loss')
                if 'DONE,CLEANING' in lines:
                    if ('EVENT,CLEANING_STARTED' not in lines or
                        lines.index('EVENT,CLEANING_STARTED')>lines.index('DONE,CLEANING')):
                        invalid_done=True;break
                    return 'DONE,CLEANING'
                left=deadline-time.monotonic()
                if left<=0:break
                self.condition.wait(min(.05,left))
            if check:check()
        self._fail('Cleaning DONE without start event; outcome uncertain; no retry' if invalid_done else 'Cleaning completion timed out; outcome unknown; no retry')
        raise CommunicationError(self.last_error)

    def _mark_move(self,state,detail):
        # Caller holds condition. Completed/rejected results are immutable history.
        move=self.current_move
        if move and move['state'] in {'awaiting_ack','accepted','unknown'}:
            move['state']=state;move['detail']=detail

    def _read(self,sock,stop):
        framing=LineFramer()
        try:
            while not stop.is_set():
                try:chunk=sock.recv(4096)
                except socket.timeout:continue
                if stop.is_set():return
                if not chunk:raise CommunicationError('ESP32 connection closed')
                framing.feed(chunk,self._received)
        except (OSError,CommunicationError,ProtocolError) as exc:
            if not stop.is_set():self._fail(str(exc))

    def _received(self,line):
        with self.write_lock:self.logger.emit('RX',line)
        with self.condition:
            self.last_rx=time.monotonic();self.telemetry.append(line)
            self.rx_sequence+=1;self.events.append((self.rx_sequence,line))
            if (self.firmware_profile=='simple' and line.startswith('ERR,') and
                self.current_move and self.current_move['state']=='accepted'):
                self._disarm();self.estop_latched=True
                self._mark_move('aborted','Unexpected controller error during MOVE: '+line)
            if line in {'EVENT,CLEAN_MOTOR_TIMEOUT','EVENT,PUMP_TIMEOUT'}:
                self._disarm();self.estop_latched=True
                self._mark_move('aborted','Controller cleaning fault: '+line)
            safety=parse_safety(line) if line.startswith('SAFETY,') else None
            if line in {'OK,ESTOP','EVENT,WATCHDOG_STOP','EVENT,LINK_LOST_ABORTED','ERR,ESTOP_LATCHED','ERR,WATCHDOG_LATCHED'} or (safety and (safety['estop'] or safety['watchdog_expired'])):
                self.estop_latched=True;self._disarm()
                self._mark_move('aborted','Controller reported a latched stop/fault')
            if line in {'OK,STOP','OK,ESTOP'}:
                self._disarm();self._mark_move('aborted','Controller acknowledged '+line[3:])
            p=self.pending
            if p and p['response'] is None and not p['error'] and expected_reply(p['command'],line):
                p['response']=line
                if p['move']:
                    move=p['move'];move['response']=line;move['accepted']=not line.startswith('ERR,')
                    if move['state']=='awaiting_ack':move['state']='accepted' if move['accepted'] else 'rejected'
            if line=='DONE,MOVE':
                move=self.current_move
                if move and move['state']=='accepted' and not move['stop_requested']:
                    now=time.monotonic()
                    if self.mode=='physical':
                        duration=int(move['command'].split(',')[2])/1000
                        # Firmware millis() can quantize the start by at most 1 ms.
                        # Never turn a premature DONE into a sleep-based success.
                        if move['sent_at'] is None or now+.001<move['sent_at']+duration:
                            raise CommunicationError('Premature DONE,MOVE before requested duration; outcome unknown; no retry')
                    if now>move['deadline']:raise CommunicationError('Late MOVE completion; outcome unknown')
                    move['state']='completed';move['detail']='Motor timer stopped; not arrival or mechanical settling'
                elif move and move['state']=='awaiting_ack':raise ProtocolError('DONE,MOVE before acceptance')
            self.condition.notify_all()

    def _heartbeat(self,stop):
        while not stop.wait(self.heartbeat_interval):
            try:
                with self.condition:
                    expired=self.current_move and self.current_move['state']=='accepted' and time.monotonic()>self.current_move['deadline']
                if expired:raise CommunicationError('MOVE completion timed out; outcome unknown; no retry')
                result=self._request('HEARTBEAT' if self.safety_supported else 'PING')
                if not result.firmware_accepted:raise CommunicationError('Heartbeat rejected')
            except CommunicationError as exc:
                if str(exc).startswith('Interrupted by priority '):continue
                if not stop.is_set():self._fail(str(exc))
                return

    def _fail(self,message):
        # TCP cannot guarantee remote actuator shutdown. Best-effort stop then
        # close; firmware must independently latch link loss and watchdog expiry.
        with self.write_lock:
            self.last_error=message;self._disarm();self.estop_latched=True;self.stop_event.set()
            sock=self.sock;self.sock=None
            if sock:
                stop='ESTOP' if self.safety_supported else 'STOP'
                try:sock.sendall((stop+'\n').encode('ascii'));self.logger.emit('TX',stop)
                except OSError:pass
                try:sock.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                sock.close()
        with self.condition:
            self._mark_move('unknown',message)
            if self.pending:self.pending['error']=message
            self.condition.notify_all()
        self.logger.emit('ERROR',message)

    def arm_physical(self,*,validated_max_pwm=None,validated_max_duration_ms=None,authorized=False):
        if self.firmware_profile=='simple':raise ValueError('SIMPLE does not use legacy PWM/ESTOP arming')
        if self.mode!='physical':raise ValueError('Arming is only meaningful in physical mode')
        with self.write_lock:self._disarm()
        if authorized is not True:raise PermissionError('Explicit operator motion authorization required')
        if isinstance(validated_max_pwm,bool) or not isinstance(validated_max_pwm,int) or not 1<=validated_max_pwm<=255:
            raise ValueError('Provide the physically validated PWM upper bound, 1..255')
        if validated_max_pwm<FIXED_PWM:raise PermissionError('Fixed PWM170 exceeds operator-validated power limit')
        if validated_max_duration_ms is None:raise PermissionError('Separate validated_max_duration_ms authorization required; PWM is not duration')
        if isinstance(validated_max_duration_ms,bool) or not isinstance(validated_max_duration_ms,int) or not 1<=validated_max_duration_ms<MAX_MOVE_MS:
            raise ValueError('Validated single-action duration must be 1..9999 milliseconds, below firmware limit')
        if not self.is_connected() or not self.safety_supported:raise CommunicationError('Duration protocol and safety support required')
        generation=self.generation
        result=self._request('SAFETY')
        safety=parse_safety(result.response) if result.firmware_accepted else None
        with self.write_lock:
            if generation!=self.generation or not self.is_connected() or not safety or safety['estop'] or safety['watchdog_expired'] or self.estop_latched:
                raise PermissionError('Emergency stop/fault remains latched; explicit reset required')
            self.max_pwm=validated_max_pwm;self.max_duration_ms=validated_max_duration_ms;self.armed=True
        self.logger.emit('WARN',f'PHYSICAL operator-authorized fixed PWM170; single MOVE <= {validated_max_duration_ms} ms; not autonomous authorization')

    def send_command(self,line):
        generation=self.generation
        validate_command(line)
        if self.firmware_profile=='simple' and line in {'HEARTBEAT','SAFETY','ESTOP','RESET_ESTOP'}:
            raise ProtocolError('Unsupported by NATHACK_V2_SIMPLE')
        if line in {'STOP','ABORT'}:return self.send_stop()
        if line=='ESTOP':return self.emergency_stop()
        if line=='RESET_ESTOP':return self.reset_estop()
        if not self.is_connected():raise CommunicationError('Communication unavailable; reconnect explicitly')
        self._check_motion_permission(line)
        return self._request(line,generation if line.startswith(('MOVE,','TARGET,')) or line=='ARRIVED' else None)

    def _priority(self,line):
        with self.write_lock:
            self.generation+=1;self._disarm()
            with self.condition:
                if self.pending:self.pending['error']='Interrupted by priority '+line
                if self.current_move and self.current_move['state'] in {'awaiting_ack','accepted','unknown'}:
                    self.current_move['stop_requested']=True
                    self._mark_move('unknown','Stop requested; awaiting controller acknowledgement')
                self.condition.notify_all()
            self._write(line);self.needs_barrier=True
        return CommandResult(line,True,None,False,None)

    def send_stop(self):return self._priority('STOP')

    def stop_and_wait(self,result=None):
        """Confirm a fresh STOP ACK and drain pre-stop replies before any new MOVE.

        An intentionally interrupted MOVE remains aborted, never completed.
        Even when DONE won the race, STOP must independently be acknowledged.
        """
        with self.write_lock:
            if self.stop_confirmation_pending:raise CommunicationError('STOP confirmation already pending')
            if self.needs_barrier:
                self._fail('Earlier priority STOP is unresolved; cannot identify a fresh acknowledgement')
                raise CommunicationError(self.last_error)
            self.stop_confirmation_pending=True
            with self.condition:after=self.rx_sequence
            self.send_stop();generation=self.generation
        try:
            self._request('PING')  # Existing priority barrier drains earlier wire replies.
            with self.write_lock,self.condition:
                lines=[line for sequence,line in self.events if sequence>after]
                if (generation!=self.generation or not self.is_connected() or 'OK,STOP' not in lines or
                    any(line.startswith('ERR,') or line.startswith('EVENT,') for line in lines)):
                    raise CommunicationError('STOP not confirmed or controller fault; motion prohibited')
                if result is not None:
                    snapshot=self.get_move_result(result)
                    if snapshot.execution_state not in {'completed','aborted'}:
                        raise CommunicationError('MOVE outcome unknown after STOP')
                else:snapshot=CommandResult('STOP',True,True,False,'OK,STOP','aborted')
                self.stop_confirmation_pending=False
                return snapshot
        except Exception as exc:
            self._fail('STOP confirmation failed: '+str(exc))
            raise CommunicationError(self.last_error) from exc

    def emergency_stop(self):
        self.estop_latched=True;self._disarm()
        if not self.safety_supported:
            self.logger.emit('WARN','Firmware has no advertised latched ESTOP; transmitting legacy STOP only')
            return self._priority('STOP')
        return self._priority('ESTOP')

    def reset_estop(self):
        if not self.safety_supported:raise CommunicationError('RESET_ESTOP unsupported by legacy firmware')
        self.estop_latched=True;self._disarm();generation=self.generation
        result=self._request('RESET_ESTOP')
        with self.write_lock:
            if result.firmware_accepted and generation==self.generation and self.is_connected():self.estop_latched=False
            self._disarm()
        return result

    def request_status(self):return self.send_command('STATUS')

    def receive_telemetry(self,timeout=0):
        if timeout<0:raise ValueError('Telemetry timeout must be nonnegative')
        deadline=time.monotonic()+timeout
        with self.condition:
            while not self.telemetry and self.is_connected():
                left=deadline-time.monotonic()
                if left<=0:return None
                self.condition.wait(left)
            return self.telemetry.popleft() if self.telemetry else None

    def disconnect(self):
        with self.write_lock:
            self._disarm();self.stop_event.set();sock=self.sock;self.sock=None
            if sock:
                self.estop_latched=True
                try:sock.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                sock.close()
        with self.condition:
            self._mark_move('unknown','Disconnected; remote stop not confirmed')
            if self.pending:self.pending['error']='Disconnected'
            self.condition.notify_all()
        for thread in (self.reader,self.heartbeat):
            if thread and thread is not threading.current_thread():thread.join(self.timeout+.1)
        self.reader=self.heartbeat=None

    def __enter__(self):return self.connect()
    def __exit__(self,*_):self.disconnect()
