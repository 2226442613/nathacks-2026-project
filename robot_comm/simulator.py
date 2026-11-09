"""Local software model of the agreed duration/safety protocol and timed cleaner.

No physical displacement or cm/s is inferred from PWM. Pump A/B select fluids;
there are no independently addressable floor/wall pumps or continuous sponge motor.
"""
import socket
import threading
import time

from .protocol import CAPS,SIMPLE_CAPS,LineFramer,ProtocolError,FIXED_PWM,PWM_FREQUENCY_HZ,MAX_MOVE_MS
from .logging import WireLogger


class FirmwareModel:
    """Model of the agreed contract, not proof that a flashed ESP32 implements it."""
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.estop=False;self.watchdog_expired=False
        self.last_seen=None;self.state='IDLE';self.target=None
        self.direction=None;self.pwm=0;self.pwm_frequency_hz=PWM_FREQUENCY_HZ
        self.move_at=None;self.move_deadline=None;self.move_owner=None
        self.pump='OFF';self.spray_deg=180;self.sponge_deg=180
        self.step_at=clock();self.cycles=0;self.sweeps=0;self.sponge_direction=-1
        self.reset_allowed=True  # Simulated hardware reset interlock; never bypassed by a query.

    def _motor_stop(self):
        self.direction=None;self.pwm=0;self.move_at=None;self.move_deadline=None;self.move_owner=None

    def _drive(self,direction,duration_ms,owner):
        self.direction=direction;self.pwm=FIXED_PWM;self.move_at=self.clock()
        self.move_deadline=self.move_at+duration_ms/1000;self.move_owner=owner

    def abort(self):
        self.state='IDLE';self.target=None;self._motor_stop()
        self.pump='OFF';self.spray_deg=180;self.sponge_deg=180

    def connected(self):
        self.abort();self.last_seen=self.clock()  # Never clears any fault latch.
        return ['CONNECTED,NATHACK_V1_1','STATUS,IDLE']

    def disconnected(self):
        self.abort();self.last_seen=None;self.watchdog_expired=True;self.estop=True

    def seen(self):
        self.last_seen=self.clock()  # Updating communication time cannot clear a fault.

    def _watchdog(self):
        if self.last_seen is not None and self.clock()-self.last_seen>=1.:
            self.abort();self.last_seen=None;self.watchdog_expired=True;self.estop=True
            return ['EVENT,WATCHDOG_STOP']
        return []

    def command(self,line):
        events=self._watchdog()
        return events+self._command(line)

    def _command(self,line):
        # Canonical host strings; parser limits remain enforced on the TCP stream.
        line=line.strip().upper()
        if line in {'STOP','ABORT'}:self.abort();self.seen();return ['OK,STOP']
        if line=='ESTOP':self.estop=True;self.abort();self.seen();return ['OK,ESTOP']
        if line=='RESET_ESTOP':
            # Requires a live/recent link, stopped outputs, and reset interlock.
            # A heartbeat may refresh the link but does not clear either latch.
            safe=(self.reset_allowed and self.last_seen is not None and
                  self.clock()-self.last_seen<1. and self.pwm==0 and self.pump=='OFF' and
                  self.state=='IDLE' and self.spray_deg==180 and self.sponge_deg==180)
            if not safe:return ['ERR,RESET_UNSAFE']
            self.abort();self.estop=False;self.watchdog_expired=False;self.seen()
            return ['OK,RESET_ESTOP']
        if line=='SAFETY':
            return [f'SAFETY,ESTOP={int(self.estop)},WATCHDOG_MS=1000,WATCHDOG={"EXPIRED" if self.watchdog_expired else "OK"}']
        if line in {'PING','STATUS','CAPS','HEARTBEAT'}:
            self.seen()
            return [{'PING':'PONG','STATUS':'STATUS,'+self.state,'CAPS':CAPS,'HEARTBEAT':'OK,HEARTBEAT'}[line]]
        if self.estop:return ['ERR,ESTOP_LATCHED']
        if self.watchdog_expired:return ['ERR,WATCHDOG_LATCHED']
        if line.startswith('MOVE,'):
            if self.state not in {'IDLE','APPROACH'} or self.move_deadline is not None:return ['ERR,BUSY']
            parts=line.split(',')
            if len(parts)!=3 or len(parts[1])!=1:return ['ERR,MOVE_FORMAT']
            if parts[1] not in 'FBLR':return ['ERR,DIRECTION']
            if not parts[2] or len(parts[2])>5 or not parts[2].isascii() or not parts[2].isdigit():return ['ERR,DURATION']
            if not 1<=int(parts[2])<=MAX_MOVE_MS:return ['ERR,DURATION_RANGE']
            self._drive(parts[1],int(parts[2]),'manual');self.seen()
            return ['OK,MOVE']
        if line.startswith('TARGET,'):
            parts=line.split(',')
            if len(parts)!=3:return ['ERR,TARGET_FORMAT']
            if parts[1] not in {'GROUND','WALL'} or parts[2] not in {'BLOOD','OTHER'}:return ['ERR,UNKNOWN_TARGET']
            self.seen()
            if self.state!='IDLE' or self.move_deadline is not None:return ['ERR,BUSY']
            self.target=tuple(parts[1:]);self.state='APPROACH';return ['OK,TARGET']
        if line=='ARRIVED':
            self.seen()
            if self.state!='APPROACH':return ['ERR,NO_TARGET']
            if self.move_deadline is not None:return ['ERR,BUSY']
            self._motor_stop();self.cycles=0
            if self.target[0]=='WALL':self.spray_deg=90;self.state='SPRAY_LIFT'
            else:self.pump='B' if self.target[1]=='BLOOD' else 'O';self.state='SPRAYING'
            self.step_at=self.clock()
            return ['EVENT,CLEANING_STARTED','OK,ARRIVED']
        return ['ERR,UNKNOWN_COMMAND']

    def tick(self):
        events=self._watchdog()
        if self.estop or self.watchdog_expired:return events
        now=self.clock()
        # Independent motor timer runs even if the cleaning state machine stalls.
        if self.move_deadline is not None and now>=self.move_deadline:
            owner=self.move_owner;self._motor_stop()
            if owner=='manual':events.append('DONE,MOVE')
        elapsed=now-self.step_at
        if self.state=='SPRAY_LIFT' and elapsed>=.7:
            self.pump='B' if self.target[1]=='BLOOD' else 'O';self.state='SPRAYING';self.step_at=now
        elif self.state=='SPRAYING' and elapsed>=1.2:
            self.pump='OFF';self.step_at=now
            if self.target[0]=='WALL':self.spray_deg=180;self.state='SPRAY_LOWER'
            else:self._drive('F',350,'cleaning');self.state='GROUND_FORWARD'
        elif self.state=='SPRAY_LOWER' and elapsed>=.7:
            self.state='WALL_WIPE';self.sponge_deg=180;self.sponge_direction=-1;self.sweeps=0;self.step_at=now
        elif self.state in {'GROUND_FORWARD','GROUND_BACKWARD'} and elapsed>=.35:
            if self.state=='GROUND_FORWARD':self._drive('B',350,'cleaning');self.state='GROUND_BACKWARD'
            else:
                self.cycles+=1
                if self.cycles>=3:self.abort();return events+['DONE,CLEANING']
                self._drive('F',350,'cleaning');self.state='GROUND_FORWARD'
            self.step_at=now
        elif self.state=='WALL_WIPE' and elapsed>=.015:
            self.step_at=now;self.sponge_deg+=self.sponge_direction*2
            if self.sponge_deg<=0:self.sponge_deg=0;self.sponge_direction=1
            elif self.sponge_deg>=180:
                self.sponge_deg=180;self.sponge_direction=-1;self.sweeps+=1
            if self.sweeps>=3:self.abort();return events+['DONE,CLEANING']
        return events


class SimpleFirmwareModel(FirmwareModel):
    """Explicit NATHACK_V2_SIMPLE profile, following the supplied Robot 3 source."""
    def __init__(self,clock=time.monotonic):
        super().__init__(clock);self.pump_at=None

    def _watchdog(self):return []

    def connected(self):
        self.abort();self.last_seen=self.clock()
        return ['CONNECTED,NATHACK_V2','STATUS,IDLE']

    def _drive(self,direction,duration_ms,owner):
        super()._drive(direction,duration_ms,owner)
        if owner=='cleaning':self.move_deadline=self.move_at+.6

    def _command(self,line):
        if line=='CAPS':return [SIMPLE_CAPS]
        if line in {'HEARTBEAT','SAFETY','ESTOP','RESET_ESTOP'}:return ['ERR,UNKNOWN_COMMAND']
        previous=self.pump;result=super()._command(line)
        if self.pump!='OFF' and previous=='OFF':self.pump_at=self.clock()
        return result

    def tick(self):
        if self.move_owner=='cleaning' and self.clock()-self.move_at>=.6:
            self.abort();return ['EVENT,CLEAN_MOTOR_TIMEOUT']
        if self.pump!='OFF' and self.pump_at is not None and self.clock()-self.pump_at>=1.7:
            self.abort();return ['EVENT,PUMP_TIMEOUT']
        previous=self.pump;result=super().tick()
        if self.pump!='OFF' and previous=='OFF':self.pump_at=self.clock()
        return result

    def disconnected(self):
        self.abort();self.last_seen=None  # SIMPLE has no watchdog/ESTOP latch.


class RobotSimulator:
    def __init__(self,host='127.0.0.1',port=3333,logger=None,*,firmware_profile='safe_v1_1'):
        if firmware_profile not in {'safe_v1_1','simple'}:raise ValueError('Explicit firmware profile required')
        if host not in {'localhost','127.0.0.1','::1'}:raise ValueError('Simulator binds loopback only')
        self.host='127.0.0.1' if host=='localhost' else host
        self.port=port;self.logger=logger or WireLogger()
        self.model=SimpleFirmwareModel() if firmware_profile=='simple' else FirmwareModel()
        self.stop_event=threading.Event();self.ready=threading.Event();self.server=None;self.client=None;self.thread=None
        self.start_error=None
        self.last_logged_state=None

    def start(self):
        self.thread=threading.Thread(target=self.serve_forever,daemon=True);self.thread.start()
        if not self.ready.wait(2):raise RuntimeError('Simulator failed to start')
        if self.start_error:raise self.start_error
        return self

    def serve_forever(self):
        try:
            self.server=socket.socket(socket.AF_INET6 if ':' in self.host else socket.AF_INET,socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            self.server.bind((self.host,self.port));self.port=self.server.getsockname()[1]
            self.server.listen(1);self.server.settimeout(.05)
        except OSError as exc:
            if self.server:self.server.close()
            self.server=None;self.start_error=exc;self.ready.set()
            if self.thread is threading.current_thread():return
            raise
        self.logger.emit('INFO',f'SIMULATION ONLY listening {self.host}:{self.port}; fixed PWM170 / 20kHz; MOVE is milliseconds')
        self.ready.set()
        try:
            while not self.stop_event.is_set():
                try:conn,_=self.server.accept()
                except socket.timeout:continue
                except OSError:break
                self.client=conn;conn.settimeout(.03)
                framing=LineFramer()
                try:
                    self.reply(self.model.connected())
                    while not self.stop_event.is_set():
                        self.reply(self.model.tick())
                        self.log_state()
                        try:chunk=conn.recv(4096)
                        except socket.timeout:continue
                        if not chunk:break
                        framing.feed(chunk,self.received)
                except ProtocolError as exc:
                    self.model.disconnected();self.logger.emit('ERROR',str(exc))
                    try:self.reply(['ERR,LINE_TOO_LONG' if '120' in str(exc) else 'ERR,ENCODING'])
                    except OSError:pass
                except OSError:pass
                finally:
                    self.model.disconnected();conn.close();self.client=None
                    self.logger.emit('INFO','Client disconnected; simulated motors/pumps stopped, servos home')
        finally:
            self.server.close();self.server=None

    def reply(self,lines):
        for line in lines:
            self.client.sendall((line+'\n').encode('ascii'));self.logger.emit('TX',line)

    def received(self,line):
        self.logger.emit('RX',line)
        self.reply(self.model.command(line))
        self.log_state()

    def log_state(self):
        m=self.model
        state=(m.state,m.direction,m.pwm,m.pump,m.spray_deg,m.sponge_deg)
        if state!=self.last_logged_state:
            self.last_logged_state=state
            self.logger.emit('INFO',f'SIMULATED commanded state={m.state} direction={m.direction} PWM={m.pwm} pump={m.pump} spray_deg={m.spray_deg} sponge_deg={m.sponge_deg}; not physical feedback')

    def close(self):
        self.stop_event.set()
        if self.client:
            try:self.client.shutdown(socket.SHUT_RDWR)
            except OSError:pass
        if self.thread:self.thread.join(2)

    def __enter__(self):return self.start()
    def __exit__(self,*_):self.close()
