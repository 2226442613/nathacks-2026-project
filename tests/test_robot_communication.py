"""Local-only protocol integration. Never connect to a physical ESP32."""
import io
import json
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
import shutil

import pytest

from robot_comm import (RobotClient,RobotSimulator,FirmwareModel,WireLogger,LineFramer,
                        ProtocolError,CommunicationError,validate_command)
from robot_comm.cli import main as comm_main
from robot_comm.protocol import CAPS

LEGACY_CAPS='CAPS,NATHACK_V1_1,MOVE_PWM_0_255,STOP,ESTOP,RESET_ESTOP,HEARTBEAT,SAFETY,TARGET,ARRIVED'


def quiet():return WireLogger(stream=io.StringIO())


@pytest.mark.parametrize('connect_timeout,expected',[(None,.8),(5.,5.)])
def test_tcp_connection_timeout_is_specific_and_does_not_retry(monkeypatch,connect_timeout,expected):
    attempts=[]
    def unavailable(address,timeout):
        attempts.append((address,timeout))
        raise socket.timeout('timed out')
    monkeypatch.setattr(socket,'create_connection',unavailable)
    client=RobotClient('127.0.0.1',3333,mode='physical',logger=quiet(),connect_timeout=connect_timeout)
    with pytest.raises(CommunicationError,match=r'TCP connection to 127\.0\.0\.1:3333 failed: timed out; no commands sent'):
        client.connect()
    assert attempts==[(('127.0.0.1',3333),expected)]
    assert client.timeout==.8 and not client.armed and not client.is_connected()
    assert client.estop_latched and client.get_last_error()
    assert '[TX]' not in client.logger.stream.getvalue()


@pytest.mark.parametrize('value',[0,-1,11,True,float('nan'),float('inf')])
def test_invalid_tcp_connect_timeout_rejected(value):
    with pytest.raises(ValueError,match='Connect timeout'):
        RobotClient(connect_timeout=value)


@pytest.mark.parametrize('outcome',['completed','aborted','unknown','rejected'])
def test_done_or_fault_during_wait_does_not_run_in_motion_camera_check(monkeypatch,outcome):
    client=RobotClient(logger=quiet())
    move=dict(command='MOVE,L,500',id=1,transport=True,accepted=True,state='accepted',
              response='OK,MOVE',detail=None,deadline=time.monotonic()+1,stop_requested=False)
    client.moves[1]=client.current_move=move
    monkeypatch.setattr(client.condition,'wait',lambda timeout:move.update(state=outcome))
    def in_motion():raise AssertionError('Completed/faulted MOVE must not run in-motion camera monitor')
    result=client.wait_for_move(check=in_motion)
    assert result.execution_state==outcome and result.execution_completed==(outcome=='completed')


def test_pending_move_still_runs_camera_monitor(monkeypatch):
    client=RobotClient(logger=quiet())
    move=dict(command='MOVE,L,500',id=1,transport=True,accepted=True,state='accepted',
              response='OK,MOVE',detail=None,deadline=time.monotonic()+1,stop_requested=False)
    client.moves[1]=client.current_move=move
    monkeypatch.setattr(client.condition,'wait',lambda timeout:None)
    checks=[]
    def check():checks.append(True);move['state']='completed'
    assert client.wait_for_move(check=check).execution_completed and checks==[True]


def test_separate_tcp_connect_timeout_keeps_command_deadline(sim,monkeypatch):
    create=socket.create_connection;attempts=[]
    def connect(address,timeout):
        attempts.append(timeout)
        return create(address,timeout=timeout)
    monkeypatch.setattr(socket,'create_connection',connect)
    with RobotClient(port=sim.port,timeout=.3,connect_timeout=5.,logger=quiet()) as client:
        assert attempts==[5.] and client.timeout==.3
        assert client.request_status().firmware_accepted and not client.armed


@pytest.fixture
def sim():
    with RobotSimulator(port=0,logger=quiet()) as server:yield server


@pytest.mark.parametrize('line',['PING','STATUS','MOVE,F,1','MOVE,R,10000','TARGET,WALL,OTHER','ARRIVED','ESTOP','RESET_ESTOP','HEARTBEAT'])
def test_plain_text_serialization(line):assert validate_command(line)==line


@pytest.mark.parametrize('line',['','PING\nSTOP','PING\r','☃','MOVE,F,0','MOVE,F,10001','MOVE,F,-1','MOVE,F,1.2','DRIVE 10 0',
                                'MOVE,CM_S,10','FLOOR_SPRAY ON','WALL_NOZZLE_HEIGHT 12','{"cmd":"STOP"}','A'*121])
def test_invalid_commands_units_unsupported_rejected(line):
    with pytest.raises(ProtocolError):validate_command(line)


def test_partial_multiple_crlf_and_invalid_framing():
    f=LineFramer();assert f.feed(b'PO')==[]
    assert f.feed(b'NG\r\nSTATUS,IDLE\nOK,STOP\n')==['PONG','STATUS,IDLE','OK,STOP']
    for data in [b'\xff\n',b'A'*121,b'PING\x00\n',b'PI\rNG\n']:
        with pytest.raises(ProtocolError):LineFramer().feed(data)


def test_exact_tx_rx_logging_and_nonoverwriting_file(sim,tmp_path,capsys):
    path=tmp_path/'wire.log';logger=WireLogger(path)
    with RobotClient(port=sim.port,logger=logger) as client:
        result=client.send_command('MOVE,F,80')
        assert result.transport_accepted and result.firmware_accepted and not result.execution_completed
        assert result.response=='OK,MOVE'
    logger.close()
    text=capsys.readouterr().out
    assert '[TX] MOVE,F,80\n' in text and '[RX] OK,MOVE\n' in text
    assert text.count('[TX] MOVE,F,80\n')==1
    assert path.read_text()==text
    with pytest.raises(FileExistsError):WireLogger(path)


def test_no_tx_log_when_transport_fails(monkeypatch):
    log=quiet();client=RobotClient(logger=log)
    sock=type('Broken',(),{'sendall':lambda self,data:(_ for _ in ()).throw(OSError('broken')),
       'shutdown':lambda *a:None,'close':lambda *a:None})()
    client.sock=sock
    with pytest.raises(CommunicationError):client.send_command('PING')
    assert '[TX] PING' not in log.stream.getvalue()


def test_connected_lifecycle_status_heartbeat_and_explicit_reconnect(sim):
    client=RobotClient(port=sim.port,logger=quiet()).connect()
    assert client.is_connected() and not client.armed and client.safety_supported
    assert client.request_status().response=='STATUS,IDLE'
    time.sleep(.25)
    assert '[TX] HEARTBEAT' in client.logger.stream.getvalue()
    client.disconnect();assert not client.is_connected()
    with pytest.raises(CommunicationError):client.send_command('MOVE,F,20')
    client.connect();assert not client.armed and client.request_status().firmware_accepted
    client.disconnect()


def test_estop_priority_latch_reset_and_reconnect(sim):
    client=RobotClient(port=sim.port,logger=quiet()).connect()
    client.send_command('MOVE,F,50')
    result=client.emergency_stop()
    assert result.transport_accepted and result.firmware_accepted is None
    assert not result.execution_completed
    with pytest.raises(PermissionError):client.send_command('MOVE,F,50')
    assert 'ESTOP=1' in client.send_command('SAFETY').response
    client.disconnect();client.connect()
    assert client.estop_latched
    with pytest.raises(PermissionError):client.send_command('MOVE,F,50')
    client.reset_estop();assert not client.estop_latched and not client.armed
    assert client.send_command('MOVE,F,50').firmware_accepted
    client.send_stop();client.disconnect()


def test_physical_default_disarmed_limits_and_explicit_authorization(sim):
    # Explicit physical mode is directed ONLY at our loopback simulator in this test.
    with RobotClient('127.0.0.1',sim.port,mode='physical',logger=quiet()) as client:
        with pytest.raises(PermissionError,match='DISARMED'):client.send_command('MOVE,F,20')
        with pytest.raises(PermissionError):client.arm_physical(validated_max_pwm=30)
        with pytest.raises(ValueError):client.arm_physical(validated_max_pwm=True,authorized=True)
        client.arm_physical(validated_max_pwm=170,validated_max_duration_ms=30,authorized=True)
        assert client.send_command('MOVE,F,20').firmware_accepted
        with pytest.raises(PermissionError,match='limit'):client.send_command('MOVE,F,31')
        with pytest.raises(PermissionError,match='outside Module 07'):client.send_command('TARGET,GROUND,BLOOD')
        client.emergency_stop()
        with pytest.raises(PermissionError):client.arm_physical(validated_max_pwm=170,validated_max_duration_ms=30,authorized=True)
        client.reset_estop();assert not client.armed


def test_simulation_never_resolves_remote_endpoint_and_physical_requires_host():
    with pytest.raises(ValueError):RobotClient('192.0.2.1')
    with pytest.raises(ValueError):RobotClient(mode='physical')
    with pytest.raises(ValueError):RobotSimulator('0.0.0.0')


class Clock:
    def __init__(self):self.t=0
    def __call__(self):return self.t
    def advance(self,n):self.t+=n


def test_model_watchdogs_stop_cleaning_and_manual_drive():
    clock=Clock();m=FirmwareModel(clock);m.connected();m.command('MOVE,F,100')
    clock.advance(.451);m.tick();assert m.pwm==0
    m.command('TARGET,GROUND,BLOOD');m.command('ARRIVED');assert m.pump=='B'
    clock.advance(1.001)
    assert m.tick()==['EVENT,WATCHDOG_STOP']
    assert m.pwm==0 and m.pump=='OFF' and m.sponge_deg==180 and m.spray_deg==180
    assert m.watchdog_expired


def test_simulated_floor_cleaning_ack_is_not_done_and_duplicate_arrived_rejected():
    clock=Clock();m=FirmwareModel(clock);m.connected()
    assert m.command('TARGET,GROUND,BLOOD')==['OK,TARGET']
    assert m.command('ARRIVED')==['EVENT,CLEANING_STARTED','OK,ARRIVED']
    assert m.command('ARRIVED')==['ERR,NO_TARGET']
    assert m.pump=='B'
    events=[];directions=set()
    for _ in range(100):
        clock.advance(.05);m.command('HEARTBEAT');events+=m.tick()
        if m.direction:directions.add(m.direction)
    assert events.count('DONE,CLEANING')==1 and m.state=='IDLE' and m.pump=='OFF'
    assert directions=={'F','B'}


def test_simulated_wall_pump_lift_sponge_and_no_independent_actuator_invention():
    clock=Clock();m=FirmwareModel(clock);m.connected()
    m.command('TARGET,WALL,OTHER');m.command('ARRIVED')
    assert m.spray_deg==90 and m.pump=='OFF'
    seen_pump=False;seen_sponge=False;events=[]
    for _ in range(1500):
        clock.advance(.02);m.command('HEARTBEAT');events+=m.tick()
        seen_pump|=m.pump=='O';seen_sponge|=m.sponge_deg>0
    assert seen_pump and seen_sponge and 'DONE,CLEANING' in events
    assert m.pump=='OFF' and m.spray_deg==180 and m.sponge_deg==180
    assert m.command('WALL_SPONGE ON')==['ERR,UNKNOWN_COMMAND']


@pytest.mark.parametrize('line,reply',[
 ('MOVE,Z,10','ERR,DIRECTION'),('MOVE,F,no','ERR,DURATION'),('MOVE,F,10001','ERR,DURATION_RANGE'),
 ('MOVE,F,100000000000000000000000','ERR,DURATION'),('MOVE,F,0','ERR,DURATION_RANGE'),
 ('MOVE,FF,2','ERR,MOVE_FORMAT'),('TARGET,WALL,OTHER,extra','ERR,TARGET_FORMAT'),
 ('TARGET,NOPE,OTHER','ERR,UNKNOWN_TARGET'),('NOPE','ERR,UNKNOWN_COMMAND')])
def test_simulator_duration_parameter_errors(line,reply):assert FirmwareModel().command(line)==[reply]


def test_client_no_opencv_dependency():
    result=subprocess.run([sys.executable,'-c','import robot_comm; import sys; assert "cv2" not in sys.modules'],capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def test_cli_preserves_main_commands_and_safe_defaults(sim,capsys):
    import main
    assert main.main(['robot-client','--port',str(sim.port),'--command','PING','--command','STATUS','--listen','0'])==0
    assert '[RX] PONG' in capsys.readouterr().out
    assert comm_main(['robot-client','--port',str(sim.port),'--demo','--listen','0'])==1  # Disconnect latched.
    with RobotSimulator(port=0,logger=quiet()) as fresh:
        assert comm_main(['robot-client','--port',str(fresh.port),'--demo','--listen','0'])==0
    assert comm_main(['robot-client','--mode','physical','--demo'])==1
    for command in ['track-robot','calibrate-robot-plane','calibrate-robot-interactive','detect-spills','plan-cleaning','calibrate']:
        assert command in main.build_parser()._subparsers._group_actions[0].choices


@pytest.fixture(scope='module')
def firmware_binary(tmp_path_factory):
    compiler=shutil.which('clang++') or shutil.which('g++')
    if not compiler:pytest.skip('No native C++ compiler for stub-based parser tests')
    output=tmp_path_factory.mktemp('firmware')/'parser'
    result=subprocess.run([compiler,'-std=c++17','-Itests/firmware_stubs','-Ifirmware/module07',
       'firmware/module07/Protocol.cpp','firmware/module07/CommunicationSafety.cpp','tests/firmware_stubs/harness.cpp','-o',str(output)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    result=subprocess.run([compiler,'-std=c++17','-fsyntax-only','-x','c++','-Itests/firmware_stubs','-Ifirmware/module07',
                           'firmware/module07/NatHack_Robot.ino'],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    return output


def firmware_run(binary,commands):
    result=subprocess.run([str(binary)],input='\n'.join(commands)+'\n',capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    return [line for line in result.stdout.splitlines() if not line.startswith('@STATE')],result.stdout


def test_real_patched_parser_capabilities_legacy_and_watchdog(firmware_binary):
    replies,state=firmware_run(firmware_binary,['PING','CAPS','MOVE,F,100','TARGET,WALL,OTHER','ARRIVED','@TIME,1001','SAFETY'])
    assert replies[:3]==['PONG',LEGACY_CAPS,'OK,MOVE']
    assert 'EVENT,CLEANING_STARTED' in replies and 'EVENT,WATCHDOG_STOP' in replies
    assert '@STATE,IDLE,F,0,0' in state


@pytest.mark.parametrize('line,reply',[
 ('MOVE,Z,10','ERR,DIRECTION'),('MOVE,F,no','ERR,SPEED'),('MOVE,F,256','ERR,SPEED_RANGE'),
 ('MOVE,F,99999999999999999999999999999999999999','ERR,SPEED_RANGE'),
 ('MOVE,FF,10','ERR,MOVE_FORMAT'),('TARGET,GROUND,WATER','ERR,UNKNOWN_TARGET')])
def test_archived_pwm_firmware_retains_its_original_errors(firmware_binary,line,reply):
    assert firmware_run(firmware_binary,[line])[0]==[reply]


def test_firmware_estop_survives_reconnect_stop_and_no_auto_resume(firmware_binary):
    replies,state=firmware_run(firmware_binary,['MOVE,F,100','ESTOP','@DISCONNECT','@CONNECT','MOVE,F,100','STOP','MOVE,F,100',
                                             'RESET_ESTOP','STATUS'])
    assert replies.count('ERR,ESTOP_LATCHED')==2 and replies[-2:]==['OK,RESET_ESTOP','STATUS,IDLE']
    assert '@STATE,IDLE,F,0,0' in state


def test_firmware_oversized_line_never_executes_tail(firmware_binary):
    replies,_=firmware_run(firmware_binary,['@RAW,'+'X'*121+'MOVE,F,255','PING'])
    assert replies==['ERR,LINE_TOO_LONG','PONG']


class Endpoint:
    """Fault-injection endpoint, bound strictly to loopback."""
    def __init__(self,behavior):
        self.behavior=behavior;self.lines=[];self.stop=threading.Event();self.conn=None
        self.listener=socket.socket();self.listener.bind(('127.0.0.1',0));self.listener.listen(1)
        self.listener.settimeout(.05);self.port=self.listener.getsockname()[1]
        self.thread=threading.Thread(target=self.serve,daemon=True);self.thread.start()

    def serve(self):
        try:
            while not self.stop.is_set():
                try:self.conn,_=self.listener.accept();break
                except socket.timeout:continue
            if self.conn is None:return
            self.conn.settimeout(.03);framing=LineFramer()
            while not self.stop.is_set():
                try:chunk=self.conn.recv(4096)
                except socket.timeout:continue
                if not chunk:break
                for line in framing.feed(chunk):
                    self.lines.append(line)
                    reply=self.behavior(line)
                    if reply is False:self.conn.close();return
                    if reply is not None:self.conn.sendall(reply)
        except OSError:pass

    def close(self):
        self.stop.set();self.listener.close()
        if self.conn:
            try:self.conn.shutdown(socket.SHUT_RDWR)
            except OSError:pass
            self.conn.close()
        self.thread.join(1)

    def __enter__(self):return self
    def __exit__(self,*_):self.close()


def standard_reply(line):
    return {'PING':b'PONG\n','CAPS':(CAPS+'\n').encode(),'SAFETY':b'SAFETY,ESTOP=0,WATCHDOG_MS=1000,WATCHDOG=OK\n',
            'HEARTBEAT':b'OK,HEARTBEAT\n','STATUS':b'STATUS,IDLE\n','ESTOP':b'OK,ESTOP\n',
            'STOP':b'OK,STOP\n'}.get(line,b'OK,MOVE\n')


def wait_until(predicate,seconds=1):
    deadline=time.monotonic()+seconds
    while not predicate() and time.monotonic()<deadline:time.sleep(.005)
    assert predicate()


def test_command_timeout_disconnects_and_never_replays_motion():
    with Endpoint(lambda line:None if line.startswith('MOVE,') else standard_reply(line)) as endpoint:
        client=RobotClient(port=endpoint.port,timeout=.12,logger=quiet()).connect()
        with pytest.raises(CommunicationError,match='timed out'):client.send_command('MOVE,F,50')
        assert not client.is_connected() and 'timed out' in client.get_last_error()
        with pytest.raises(CommunicationError):client.send_command('MOVE,F,50')
        assert endpoint.lines.count('MOVE,F,50')==1
        client.disconnect()


def test_missing_heartbeat_response_closes_client_and_disarms():
    with Endpoint(lambda line:None if line=='HEARTBEAT' else standard_reply(line)) as endpoint:
        client=RobotClient(port=endpoint.port,timeout=.1,heartbeat_interval=.03,logger=quiet()).connect()
        wait_until(lambda:not client.is_connected())
        assert 'timed out' in client.get_last_error() and not client.armed
        client.disconnect()


@pytest.mark.parametrize('bad',[b'\xff\n',b'X'*121+b'\n'])
def test_bad_response_framing_terminates_client(bad):
    with Endpoint(lambda line:bad if line=='STATUS' else standard_reply(line)) as endpoint:
        client=RobotClient(port=endpoint.port,timeout=.2,logger=quiet()).connect()
        with pytest.raises(CommunicationError):client.request_status()
        assert not client.is_connected()
        client.disconnect()


def test_unexpected_disconnect_pending_request_fails_without_reconnect():
    with Endpoint(lambda line:False if line=='STATUS' else standard_reply(line)) as endpoint:
        client=RobotClient(port=endpoint.port,timeout=.2,logger=quiet()).connect()
        with pytest.raises(CommunicationError,match='closed'):client.request_status()
        assert not client.is_connected()
        client.disconnect()


def test_priority_estop_does_not_wait_for_inflight_command_timeout():
    with Endpoint(lambda line:None if line.startswith('MOVE,') else standard_reply(line)) as endpoint:
        client=RobotClient(port=endpoint.port,timeout=.8,logger=quiet()).connect();errors=[]
        def move():
            try:client.send_command('MOVE,F,50')
            except CommunicationError as exc:errors.append(str(exc))
        worker=threading.Thread(target=move);worker.start()
        wait_until(lambda:'MOVE,F,50' in endpoint.lines)
        before=time.monotonic();client.emergency_stop()
        wait_until(lambda:'ESTOP' in endpoint.lines)
        assert time.monotonic()-before<.3
        worker.join(1)
        assert errors and 'Interrupted by priority ESTOP' in errors[0]
        # PING barrier drains prior replies before subsequent ordinary requests.
        assert client.request_status().response=='STATUS,IDLE' and client.is_connected()
        assert endpoint.lines.count('MOVE,F,50')==1
        client.disconnect()


def test_estop_can_interrupt_a_heartbeat_without_failing_connection():
    with Endpoint(lambda line:None if line=='HEARTBEAT' else standard_reply(line)) as endpoint:
        client=RobotClient(port=endpoint.port,timeout=.8,heartbeat_interval=.2,logger=quiet()).connect()
        wait_until(lambda:'HEARTBEAT' in endpoint.lines)
        client.emergency_stop()
        assert client.request_status().firmware_accepted and client.is_connected()
        client.disconnect()


def test_legacy_firmware_queries_work_but_arming_and_latched_estop_are_not_claimed():
    with Endpoint(lambda line:b'ERR,UNKNOWN_COMMAND\n' if line=='CAPS' else standard_reply(line)) as endpoint:
        client=RobotClient('127.0.0.1',endpoint.port,mode='physical',logger=quiet()).connect()
        assert not client.safety_supported and client.request_status().firmware_accepted
        with pytest.raises(CommunicationError,match='safety support'):client.arm_physical(validated_max_pwm=170,validated_max_duration_ms=20,authorized=True)
        result=client.emergency_stop()
        assert result.command=='STOP' and result.firmware_accepted is None
        with pytest.raises(CommunicationError):client.reset_estop()
        assert 'legacy STOP only' in client.logger.stream.getvalue()
        client.disconnect()


def test_real_socket_fragmentation_multiple_commands_and_protocol_fault_stop(sim):
    with socket.create_connection(('127.0.0.1',sim.port),timeout=1) as sock:
        sock.settimeout(1)
        stream=sock.makefile('rb')
        assert stream.readline().startswith(b'CONNECTED,')
        assert stream.readline()==b'STATUS,IDLE\n'
        sock.sendall(b'PI');sock.sendall(b'NG\nSTATUS\nMOVE,F,50\n')
        assert [stream.readline() for _ in range(3)]==[b'PONG\n',b'STATUS,IDLE\n',b'OK,MOVE\n']
        sock.sendall(b'X'*121+b'MOVE,F,255\n')
        assert stream.readline()==b'ERR,LINE_TOO_LONG\n'
        wait_until(lambda:sim.model.pwm==0)
        stream.close()


def test_telemetry_done_does_not_get_mistaken_for_ack():
    with Endpoint(lambda line:b'OK,ARRIVED\nDONE,CLEANING\n' if line=='ARRIVED' else standard_reply(line)) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet()) as client:
            result=client.send_command('ARRIVED')
            assert result.firmware_accepted and not result.execution_completed
            wait_until(lambda:'DONE,CLEANING' in client.telemetry)
            assert any(client.receive_telemetry()== 'DONE,CLEANING' for _ in range(len(client.telemetry)))


def test_firmware_invalid_ascii_discard_rest_of_line(firmware_binary):
    replies,_=firmware_run(firmware_binary,['@RAW,\tMOVE,F,255','PING'])
    assert replies==['ERR,ENCODING','PONG']


def test_complete_prefix_lines_are_logged_even_if_same_read_ends_with_bad_encoding():
    lines=[]
    with pytest.raises(ProtocolError):LineFramer().feed(b'PONG\nOK,MOVE\n\xff',lines.append)
    assert lines==['PONG','OK,MOVE']


def test_incomplete_or_spoofed_safety_capabilities_do_not_arm_physical():
    def reply(line):
        if line=='CAPS':return b'CAPS,FAKE_NATHACK_V1_1,ESTOP\n'
        return standard_reply(line)
    with Endpoint(reply) as endpoint:
        with RobotClient('127.0.0.1',endpoint.port,mode='physical',logger=quiet()) as client:
            assert not client.safety_supported
            with pytest.raises(CommunicationError):client.arm_physical(validated_max_pwm=170,validated_max_duration_ms=10,authorized=True)


def test_malformed_safety_state_fails_connection():
    with Endpoint(lambda line:b'SAFETY,WATCHDOG_MS=1000,ESTOP=MAYBE\n' if line=='SAFETY' else standard_reply(line)) as endpoint:
        client=RobotClient(port=endpoint.port,logger=quiet())
        with pytest.raises(CommunicationError,match='safety report'):client.connect()
        assert not client.is_connected()


def test_priority_stop_cancels_queued_motion_instead_of_sending_it_later(sim):
    with RobotClient(port=sim.port,logger=quiet()) as client:
        errors=[];started=threading.Event()
        client.request_lock.acquire()
        def queued():
            started.set()
            try:client.send_command('MOVE,F,90')
            except CommunicationError as exc:errors.append(str(exc))
        worker=threading.Thread(target=queued);worker.start();started.wait(1)
        # Ensure the request actually entered the locked wait before changing generation.
        time.sleep(.02)
        client.send_stop();client.request_lock.release();worker.join(1)
        assert errors and 'cancelled' in errors[0]
        assert '[TX] MOVE,F,90' not in client.logger.stream.getvalue()
        assert client.request_status().firmware_accepted


@pytest.mark.parametrize('capabilities',[
    LEGACY_CAPS, CAPS.replace(',MOVE_DURATION_MS',''), CAPS.replace(',FIXED_PWM_170',''),
    CAPS.replace(',STOP',''), CAPS.replace(',TARGET',''), CAPS+',MOVE_PWM_0_255'])
def test_duration_caps_require_complete_unambiguous_contract(capabilities):
    with Endpoint(lambda line:(capabilities+'\n').encode() if line=='CAPS' else standard_reply(line)) as endpoint:
        with RobotClient('127.0.0.1',endpoint.port,mode='physical',logger=quiet()) as client:
            assert not client.safety_supported and not client.armed
            with pytest.raises(CommunicationError):
                client.arm_physical(validated_max_pwm=170,validated_max_duration_ms=50,authorized=True)
            with pytest.raises((CommunicationError,PermissionError)):client.send_command('MOVE,R,1500')
            assert not any(line.startswith('MOVE,') for line in endpoint.lines)


def test_fixed_power_and_duration_are_independent_authorizations(sim):
    with RobotClient('127.0.0.1',sim.port,mode='physical',logger=quiet()) as client:
        for power in [1,169]:
            with pytest.raises(PermissionError,match='PWM170'):
                client.arm_physical(validated_max_pwm=power,validated_max_duration_ms=100,authorized=True)
        with pytest.raises(PermissionError,match='duration'):
            client.arm_physical(validated_max_pwm=170,authorized=True)  # Old API cannot silently authorize time.
        for duration in [0,-1,True,1.5,10000]:
            with pytest.raises(ValueError):
                client.arm_physical(validated_max_pwm=170,validated_max_duration_ms=duration,authorized=True)
        client.arm_physical(validated_max_pwm=170,validated_max_duration_ms=300,authorized=True)
        assert client.max_pwm==170 and client.max_duration_ms==300
        result=client.send_command('MOVE,F,256')  # Valid duration, not bounded by PWM170/255.
        assert client.wait_for_move(result).execution_completed
        with pytest.raises(PermissionError,match='Duration'):client.send_command('MOVE,F,301')
        assert 'MOVE,F,301' not in client.logger.stream.getvalue()
        with pytest.raises(PermissionError):
            client.arm_physical(validated_max_pwm=169,validated_max_duration_ms=100,authorized=True)
        assert not client.armed and client.max_duration_ms is None


def test_ok_move_then_async_done_with_heartbeat_and_status_during_wait(sim):
    with RobotClient(port=sim.port,timeout=.2,heartbeat_interval=.04,logger=quiet()) as client:
        result=client.send_command('MOVE,R,1200')  # Longer than the 1000 ms watchdog.
        assert result.firmware_accepted and not result.execution_completed and result.execution_state=='accepted'
        completed=[]
        worker=threading.Thread(target=lambda:completed.append(client.wait_for_move(result)))
        worker.start()
        assert client.request_status().response=='STATUS,IDLE'
        worker.join(2)
        assert len(completed)==1 and completed[0].execution_completed
        assert completed[0].execution_state=='completed' and completed[0].response=='OK,MOVE'
        assert client.get_move_result(result)==completed[0]
        log=client.logger.stream.getvalue()
        assert log.count('[TX] MOVE,R,1200')==1 and log.count('[TX] HEARTBEAT')>=4
        assert not sim.model.watchdog_expired and sim.model.pwm==0


@pytest.mark.parametrize('stop_command',['STOP','ABORT','ESTOP'])
def test_acknowledged_stop_aborts_move_and_never_emits_done(sim,stop_command):
    with RobotClient(port=sim.port,logger=quiet()) as client:
        result=client.send_command('MOVE,F,600')
        client.send_command(stop_command)
        stopped=client.wait_for_move(result)
        assert stopped.execution_state=='aborted' and not stopped.execution_completed
        time.sleep(.65)
        assert 'DONE,MOVE' not in client.telemetry and sim.model.pwm==0
        assert not client.armed


def test_missing_stop_ack_leaves_unknown_and_closes_connection():
    with Endpoint(lambda line:None if line=='STOP' else standard_reply(line)) as endpoint:
        with RobotClient(port=endpoint.port,timeout=.12,logger=quiet()) as client:
            result=client.send_command('MOVE,F,200')
            client.send_stop()
            with pytest.raises(CommunicationError,match='not acknowledged'):client.request_status()
            assert client.get_move_result(result).execution_state=='unknown'
            assert not client.is_connected() and client.estop_latched


def test_same_packet_done_is_not_confused_with_acceptance_or_status():
    with Endpoint(lambda line:b'OK,MOVE\nDONE,MOVE\n' if line.startswith('MOVE,') else standard_reply(line)) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet()) as client:
            result=client.send_command('MOVE,F,20')
            assert client.wait_for_move(result).execution_state=='completed'
            assert client.request_status().response=='STATUS,IDLE'


def test_rejected_move_is_not_accepted_or_completed():
    with Endpoint(lambda line:b'ERR,BUSY\n' if line.startswith('MOVE,') else standard_reply(line)) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet()) as client:
            result=client.send_command('MOVE,F,20')
            assert not result.firmware_accepted and result.execution_state=='rejected'
            assert client.wait_for_move(result).execution_state=='rejected'


def test_missing_done_times_out_unknown_no_retry_and_fault_latches():
    with Endpoint(standard_reply) as endpoint:
        with RobotClient(port=endpoint.port,timeout=.12,heartbeat_interval=.03,logger=quiet()) as client:
            result=client.send_command('MOVE,F,20')
            outcome=client.wait_for_move(result)
            assert outcome.execution_state=='unknown' and outcome.firmware_accepted
            assert not outcome.execution_completed and not client.is_connected() and client.estop_latched
            assert endpoint.lines.count('MOVE,F,20')==1
            with pytest.raises(CommunicationError):client.send_command('MOVE,F,20')


def test_disconnect_after_acceptance_never_becomes_completion_or_auto_resume():
    with Endpoint(lambda line:False if line=='STATUS' else standard_reply(line)) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet()) as client:
            result=client.send_command('MOVE,F,500')
            with pytest.raises(CommunicationError):client.request_status()
            assert client.wait_for_move(result).execution_state=='unknown'
            assert client.estop_latched and not client.armed
            assert endpoint.lines.count('MOVE,F,500')==1


def test_reset_failure_preserves_latch_success_still_requires_reauthorization(sim):
    with RobotClient('127.0.0.1',sim.port,mode='physical',logger=quiet()) as client:
        client.emergency_stop();client.emergency_stop()
        assert 'ESTOP=1' in client.send_command('SAFETY').response
        sim.model.reset_allowed=False
        assert client.reset_estop().response=='ERR,RESET_UNSAFE'
        assert client.estop_latched and not client.armed
        sim.model.reset_allowed=True
        assert client.reset_estop().response=='OK,RESET_ESTOP'
        assert not client.estop_latched and not client.armed and client.max_duration_ms is None
        with pytest.raises(PermissionError,match='DISARMED'):client.send_command('MOVE,F,20')
        client.arm_physical(validated_max_pwm=170,validated_max_duration_ms=50,authorized=True)
        client.disconnect();client.connect()
        assert client.estop_latched and not client.armed
        with pytest.raises(PermissionError):client.send_command('MOVE,F,20')
        assert client.reset_estop().firmware_accepted
        assert not client.armed


def test_model_duration_boundaries_busy_and_cancelled_timer():
    clock=Clock();m=FirmwareModel(clock);m.connected()
    assert m.command('MOVE,R,1500')==['OK,MOVE']
    assert m.pwm==170 and m.pwm_frequency_hz==20000
    assert m.command('MOVE,F,20')==['ERR,BUSY']
    clock.advance(.8);m.command('HEARTBEAT');assert m.tick()==[]
    clock.advance(.7);m.command('HEARTBEAT');assert m.tick()==['DONE,MOVE']
    assert m.pwm==0 and m.tick()==[]
    assert m.command('MOVE,L,10000')==['OK,MOVE']
    m.command('STOP');clock.advance(.1);assert m.tick()==[]
    assert m.command('MOVE,B,1')==['OK,MOVE']
    clock.advance(.0011);assert m.tick()==['DONE,MOVE']


def test_model_fault_queries_heartbeats_stop_reconnect_do_not_clear_latches():
    clock=Clock();m=FirmwareModel(clock);m.connected();m.command('MOVE,F,2000')
    clock.advance(1.001);assert m.tick()==['EVENT,WATCHDOG_STOP']
    assert m.estop and m.watchdog_expired and m.pwm==0
    for line in ['SAFETY','PING','CAPS','HEARTBEAT','STOP','ESTOP','ESTOP']:
        m.command(line)
        assert m.estop and m.watchdog_expired
    assert 'EXPIRED' in m.command('SAFETY')[0]
    m.disconnected();m.connected();assert m.estop and m.watchdog_expired
    assert m.command('MOVE,F,1')==['ERR,ESTOP_LATCHED']
    assert m.command('RESET_ESTOP')==['OK,RESET_ESTOP']
    assert not m.estop and not m.watchdog_expired and m.state=='IDLE'


def test_cleaning_motor_has_timer_independent_of_cleaning_state_progress():
    clock=Clock();m=FirmwareModel(clock);m.connected()
    m.command('TARGET,GROUND,OTHER');m.command('ARRIVED')
    clock.advance(.7);m.command('HEARTBEAT');m.tick()
    clock.advance(.5);m.command('HEARTBEAT');m.tick()
    assert m.direction=='F' and m.pwm==170
    assert m.move_deadline==pytest.approx(clock()+.350)
    m.state='STALLED_TEST_STATE'  # Simulated cleaner logic failure; motor timer still runs.
    clock.advance(.351);m.command('HEARTBEAT');assert m.tick()==[]
    assert m.pwm==0 and m.direction is None
    m.command('ESTOP');assert m.pump=='OFF' and m.spray_deg==m.sponge_deg==180


def test_cleaning_exact_three_cycles_power_home_and_wall_sweep():
    for surface in ['GROUND','WALL']:
        clock=Clock();m=FirmwareModel(clock);m.connected()
        m.command('TARGET,'+surface+',OTHER');m.command('ARRIVED')
        events=[];directions=[];angles=[];last_direction=None
        for _ in range(1600):
            clock.advance(.02);m.command('HEARTBEAT');events+=m.tick()
            angles.append(m.sponge_deg)
            if m.direction:
                assert m.pwm==170
                if m.direction!=last_direction:directions.append(m.direction)
            last_direction=m.direction
        assert events.count('DONE,CLEANING')==1 and 'DONE,MOVE' not in events
        assert m.pwm==0 and m.pump=='OFF' and m.spray_deg==m.sponge_deg==180
        if surface=='GROUND':assert directions==['F','B']*3 and m.cycles==3
        else:assert min(angles)==0 and max(angles)==180 and m.sweeps==3


def test_late_done_after_stop_cannot_complete_the_next_move():
    def reply(line):
        if line=='STOP':return b'OK,STOP\nDONE,MOVE\n'  # Fault injection: firmware must not do this.
        return standard_reply(line)
    with Endpoint(reply) as endpoint:
        with RobotClient(port=endpoint.port,timeout=.15,logger=quiet()) as client:
            first=client.send_command('MOVE,F,100')
            client.send_stop()
            assert client.request_status().firmware_accepted  # Explicit response barrier drains old lines.
            assert client.get_move_result(first).execution_state=='aborted'
            second=client.send_command('MOVE,F,20')
            assert second.execution_state=='accepted' and not second.execution_completed
            assert client.wait_for_move(second).execution_state=='unknown'
            assert endpoint.lines.count('MOVE,F,20')==1


def test_missing_heartbeat_while_move_waits_returns_unknown_and_latches():
    with Endpoint(lambda line:None if line=='HEARTBEAT' else standard_reply(line)) as endpoint:
        with RobotClient(port=endpoint.port,timeout=.12,heartbeat_interval=.03,logger=quiet()) as client:
            result=client.send_command('MOVE,F,2000')
            outcome=client.wait_for_move(result)
            assert outcome.execution_state=='unknown' and not outcome.execution_completed
            assert client.estop_latched and not client.is_connected()
            assert endpoint.lines.count('MOVE,F,2000')==1


class ReportedSafetyPeer:
    """Test double for the user's reported heartbeat-only/startup-lock behavior.

    Not the unchanged FirmwareModel and not a hardware validation.
    """
    def __init__(self):
        self.started=time.monotonic();self.last_heartbeat=None
        self.estop=True;self.expired=False

    def __call__(self,line):
        now=time.monotonic()
        if now-(self.last_heartbeat if self.last_heartbeat is not None else self.started)>=1:
            self.expired=True;self.estop=True
        if line=='HEARTBEAT':self.last_heartbeat=now;return b'OK,HEARTBEAT\n'
        if line=='PING':return b'PONG\n'
        if line=='CAPS':return (CAPS+'\n').encode()
        if line=='STATUS':return b'STATUS,IDLE\n'
        if line=='SAFETY':return f'SAFETY,ESTOP={int(self.estop)},WATCHDOG_MS=1000,WATCHDOG={"EXPIRED" if self.expired else "OK"}\n'.encode()
        if line=='ESTOP':self.estop=True;return b'OK,ESTOP\n'
        if line in {'STOP','ABORT'}:return b'OK,STOP\n'
        if line=='RESET_ESTOP':
            if self.last_heartbeat is None or now-self.last_heartbeat>=1:return b'ERR,RESET_UNSAFE\n'
            self.estop=False;self.expired=False;return b'OK,RESET_ESTOP\n'
        if line.startswith('MOVE,'):return b'ERR,ESTOP_LATCHED\n' if self.estop else b'OK,MOVE\n'
        return b'ERR,UNKNOWN_COMMAND\n'


def test_bench_pause_resume_keeps_tcp_and_queries_cannot_refresh_reported_watchdog():
    from tools.module07_bench import BenchSession
    peer=ReportedSafetyPeer()
    with Endpoint(peer) as endpoint, socket.create_connection(('127.0.0.1',endpoint.port)) as sock:
        bench=BenchSession(sock,logger=quiet())
        bench.tick();bench.pause()
        descriptor=sock.fileno();heartbeats=endpoint.lines.count('HEARTBEAT')
        deadline=time.monotonic()+1.06
        while time.monotonic()<deadline:
            assert bench.query('PING')=='PONG'
            bench.tick()  # Paused tick must not transmit HEARTBEAT.
            time.sleep(.04)
        assert endpoint.lines.count('HEARTBEAT')==heartbeats
        assert bench.query('SAFETY')=='SAFETY,ESTOP=1,WATCHDOG_MS=1000,WATCHDOG=EXPIRED'
        bench.resume()
        assert endpoint.lines.count('HEARTBEAT')==heartbeats+1
        assert bench.query('SAFETY')=='SAFETY,ESTOP=1,WATCHDOG_MS=1000,WATCHDOG=EXPIRED'
        assert sock.fileno()==descriptor
        assert not any(x in {'RESET_ESTOP'} or x.startswith('MOVE,') for x in endpoint.lines)
        assert bench.query('RESET_ESTOP')=='OK,RESET_ESTOP'  # Separate explicit action.
        assert bench.query('SAFETY')=='SAFETY,ESTOP=0,WATCHDOG_MS=1000,WATCHDOG=OK'


def test_bench_probe_confirms_actual_firmware_reply_not_client_interception():
    from tools.module07_bench import BenchSession
    with Endpoint(ReportedSafetyPeer()) as endpoint, socket.create_connection(('127.0.0.1',endpoint.port)) as sock:
        bench=BenchSession(sock,isolated=True,logger=quiet())
        assert bench.probe_estop()=='ERR,ESTOP_LATCHED'
        assert endpoint.lines.count('MOVE,F,1')==1
        assert '[RX] ERR,ESTOP_LATCHED' in bench.logger.stream.getvalue()
        assert 'RESET_ESTOP' not in endpoint.lines


def test_bench_probe_without_isolation_and_normal_move_send_nothing():
    from tools.module07_bench import BenchSession,BenchError
    with Endpoint(ReportedSafetyPeer()) as endpoint, socket.create_connection(('127.0.0.1',endpoint.port)) as sock:
        bench=BenchSession(sock,logger=quiet())
        with pytest.raises(BenchError,match='isolated'):bench.probe_estop()
        with pytest.raises(BenchError,match='No normal MOVE'):bench.query('MOVE,F,1')
        assert endpoint.lines==[]


def test_bench_probe_unexpected_acceptance_fails_and_never_retries_move():
    from tools.module07_bench import BenchSession,BenchError
    peer=ReportedSafetyPeer()
    def broken(line):return b'OK,MOVE\n' if line.startswith('MOVE,') else peer(line)
    with Endpoint(broken) as endpoint, socket.create_connection(('127.0.0.1',endpoint.port)) as sock:
        bench=BenchSession(sock,isolated=True,logger=quiet())
        with pytest.raises(BenchError,match='FAIL'):bench.probe_estop()
        wait_until(lambda:'STOP' in endpoint.lines)
        assert endpoint.lines.count('MOVE,F,1')==1 and endpoint.lines.count('ESTOP')==2


def test_bench_physical_requires_isolation_before_connect(monkeypatch):
    from tools.module07_bench import main as bench_main
    def forbidden(*args,**kwargs):raise AssertionError('Must not connect')
    monkeypatch.setattr(socket,'create_connection',forbidden)
    with pytest.raises(SystemExit):bench_main(['--mode','physical','--host','192.0.2.1'])
    with pytest.raises(SystemExit):bench_main(['--host','192.0.2.1'])


def test_reported_startup_lock_reset_requires_first_heartbeat_and_never_arms():
    with Endpoint(ReportedSafetyPeer()) as endpoint:
        with RobotClient('127.0.0.1',endpoint.port,mode='physical',heartbeat_interval=.3,logger=quiet()) as client:
            assert client.safety_supported and client.estop_latched and not client.armed
            assert client.reset_estop().response=='ERR,RESET_UNSAFE'
            assert client.estop_latched and not client.armed
            wait_until(lambda:'HEARTBEAT' in endpoint.lines)
            assert client.reset_estop().response=='OK,RESET_ESTOP'
            assert not client.estop_latched and not client.armed
            with pytest.raises(PermissionError):client.send_command('MOVE,F,1')
            assert not any(line.startswith('MOVE,') for line in endpoint.lines)


@pytest.mark.parametrize('fault',['EVENT,WATCHDOG_STOP','EVENT,LINK_LOST_ABORTED',
    'SAFETY,ESTOP=1,WATCHDOG_MS=1000,WATCHDOG=EXPIRED'])
def test_async_known_fault_then_invalid_done_never_completes_or_replays(fault):
    def reply(line):
        if line.startswith('MOVE,'):return ('OK,MOVE\n'+fault+'\nDONE,MOVE\n').encode()
        return standard_reply(line)
    with Endpoint(reply) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet()) as client:
            result=client.send_command('MOVE,F,500')
            assert client.wait_for_move(result).execution_state=='aborted'
            assert not client.get_move_result(result).execution_completed
            assert client.estop_latched and not client.armed
            assert endpoint.lines.count('MOVE,F,500')==1
