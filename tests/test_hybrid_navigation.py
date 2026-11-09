"""Hybrid control and STOP response races, entirely synthetic or loopback."""
import threading
import time
from types import SimpleNamespace

import pytest

import main
from navigation.adapters import CameraPoses
from navigation.controller import Navigator,NavigationConfig,NavigationFailure,Pose
from navigation.geometry import Arena
from navigation.timing import MovementTiming,load_timing
from robot_comm import RobotClient,CommunicationError
from robot_comm.protocol import SIMPLE_CAPS
from test_navigation import rig,quiet
from test_robot_communication import Endpoint,standard_reply


@pytest.mark.parametrize('surface',['FLOOR','WALL'])
def test_hybrid_cycle_reduces_move_count_and_keeps_complete_lifecycle(surface):
    nav,plant,transport,plan=rig(surface)
    hybrid=nav.run(plan,'NON_RED_SPILL')
    assert hybrid['state']=='READY' and nav.cleaning_done and nav.ready
    assert any(e.get('movement_mode')=='LONG_MONITORED' for e in nav.events)
    assert any(e.get('movement_mode')=='SHORT_CORRECTION' for e in nav.events)
    assert all(int(line.split(',')[2])<=1000 for line in transport.lines if line.startswith('MOVE,'))
    assert not any(line in {'HEARTBEAT','SAFETY','ESTOP','RESET_ESTOP'} for line in transport.lines)
    old,_,_,oldplan=rig(surface);old.timing.data['stopping_samples']=[]
    fallback=old.run(oldplan,'NON_RED_SPILL')
    assert fallback['state']=='READY' and hybrid['movements']<fallback['movements']
    if surface=='FLOOR':
        early=[e for e in nav.events if e.get('early_stop')]
        assert early and all(e['stop_confirmed'] for e in early)
        assert any(e.get('segment_outcome')=='early_stopped' and not e['movement_completed'] for e in nav.events)


def test_measured_stopping_margin_uses_independent_upper_bounds_without_physical_defaults():
    timing=MovementTiming({'schema_version':1,'mode':'PHYSICAL','samples':[]})
    timing.add(direction='F',duration_ms=400,observed_change=10,verified=True,source='test measurement')
    with pytest.raises(ValueError,match='stopping'):timing.stopping('F',.15)
    timing.add_stopping(direction='F',pose_latency_ms=80,stop_latency_ms=60,maximum_speed_per_second=30,
                        coast_units=.4,verified=False,source='unverified')
    with pytest.raises(ValueError):timing.stopping('F',.15)
    timing.data['stopping_samples'][0]['verified']=True
    assert timing.stopping('F',.15)['margin']==pytest.approx(30*(.15+.08+.06)+.4)
    with pytest.raises(ValueError):timing.stopping('B',.15)
    physical=load_timing('config/movement_timing.json')
    assert physical.data.get('stopping_samples',[])==[] and physical.data['samples']==[]


@pytest.mark.parametrize('field,value',[('pose_latency_ms',-1),('stop_latency_ms',float('nan')),
                                     ('maximum_speed_per_second',0),('coast_units',-1)])
def test_invalid_stopping_measurements_rejected(field,value):
    timing=MovementTiming({'schema_version':1,'mode':'PHYSICAL','samples':[]})
    values=dict(direction='R',pose_latency_ms=80,stop_latency_ms=60,maximum_speed_per_second=30,
                coast_units=.4,verified=True,source='measurement')
    values[field]=value
    with pytest.raises(ValueError):timing.add_stopping(**values)


def test_uncalibrated_stopping_data_uses_only_short_monitored_pulses():
    nav,plant,transport,plan=rig()
    nav.timing.data['stopping_samples']=[]
    nav.axis('X',70)
    assert all(int(line.split(',')[2])<=500 for line in transport.lines if line.startswith('MOVE,'))
    assert all(e.get('stopping_margin') is None for e in nav.events if 'command' in e)
    assert not any(e.get('movement_mode')=='LONG_MONITORED' for e in nav.events)


def test_uncalibrated_physical_segment_cannot_be_raised_above_1000ms():
    nav,plant,transport,plan=rig(config=NavigationConfig(maximum_segment_ms=1500))
    nav.timing.data.update(mode='PHYSICAL',stopping_samples=[])
    with pytest.raises(NavigationFailure,match='1000'):nav.axis('X',70)
    assert not transport.lines
    with pytest.raises(ValueError):NavigationConfig(maximum_segment_ms=10000)


@pytest.mark.parametrize('condition',['stale','lost','unsafe'])
def test_active_motion_failure_requests_confirmed_stop_without_next_move(monkeypatch,condition):
    nav,plant,transport,plan=rig()
    original=transport.wait_for_move
    def fail(*args,**kwargs):
        if condition=='stale':plant.stale=True
        elif condition=='lost':plant.lost=True
        else:plant.y=1
        return original(*args,**kwargs)
    monkeypatch.setattr(transport,'wait_for_move',fail)
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state'] in {'PAUSED','BLOCKED'} and not nav.ready
    assert nav.stop_confirmed and transport.lines[-2:]==['STOP','PING']
    assert sum(line.startswith('MOVE,') for line in transport.lines)==1


def test_missing_stop_ack_faults_even_when_camera_loss_was_the_original_cause(monkeypatch):
    nav,plant,transport,plan=rig();transport.missing_stop_ack=True
    original=transport.wait_for_move
    def fail(*args,**kwargs):plant.lost=True;return original(*args,**kwargs)
    monkeypatch.setattr(transport,'wait_for_move',fail)
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']=='ERROR' and nav.faulted and not nav.ready
    before=list(transport.lines)
    assert nav.run(plan,'NON_RED_SPILL')['state']=='ERROR'
    assert transport.lines==before  # A latched task never automatically resumes.


@pytest.mark.parametrize('move_reply,stop_reply,expected',[
    (b'OK,MOVE\n',b'OK,STOP\n','aborted'),
    (b'OK,MOVE\n',b'DONE,MOVE\nOK,STOP\n','aborted'),
    (b'OK,MOVE\n',b'OK,STOP\nDONE,MOVE\n','aborted'),
    (b'OK,MOVE\nDONE,MOVE\n',b'OK,STOP\n','completed'),
])
def test_client_stop_done_races_drain_old_replies_before_next_move(move_reply,stop_reply,expected):
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        if line.startswith('MOVE,'):return move_reply
        if line=='STOP':return stop_reply
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet(),firmware_profile='simple') as client:
            accepted=client.send_command('MOVE,F,1000')
            stopped=client.stop_and_wait(accepted)
            assert stopped.execution_state==expected
            # STOP's old DONE has been drained; it cannot complete this new request.
            next_result=client.send_command('MOVE,B,1000')
            if b'DONE,MOVE' not in move_reply:
                assert client.get_move_result(next_result).execution_state=='accepted'
            client.stop_and_wait(next_result)
            assert endpoint.lines.count('MOVE,F,1000')==1 and endpoint.lines.count('MOVE,B,1000')==1


@pytest.mark.parametrize('already_completed',[False,True])
def test_missing_stop_ack_cannot_be_hidden_by_done_or_pong(already_completed):
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        if line.startswith('MOVE,'):return b'OK,MOVE\n'+(b'DONE,MOVE\n' if already_completed else b'')
        if line=='STOP':return b''
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet(),firmware_profile='simple',timeout=.1) as client:
            result=client.send_command('MOVE,F,1000')
            with pytest.raises(CommunicationError):client.stop_and_wait(result)
            assert client.estop_latched and not client.is_connected()
            with pytest.raises(CommunicationError):client.send_command('MOVE,B,100')
            assert 'MOVE,B,100' not in endpoint.lines


def test_stop_interrupts_move_even_before_its_acceptance_arrives():
    from test_robot_communication import wait_until
    started=threading.Event();errors=[]
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        if line.startswith('MOVE,'):
            started.set();time.sleep(.04);return b'OK,MOVE\n'
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet(),firmware_profile='simple') as client:
            def move():
                try:client.send_command('MOVE,F,1000')
                except CommunicationError as exc:errors.append(str(exc))
            sender=threading.Thread(target=move);sender.start()
            try:
                assert started.wait(.5)
                assert client.stop_and_wait().response=='OK,STOP'
            finally:sender.join(1)
            assert errors and 'priority STOP' in errors[0]
            assert client.get_move_result().execution_state=='aborted'
            assert endpoint.lines.count('MOVE,F,1000')==1


def test_unresolved_earlier_stop_cannot_supply_ack_for_new_stop():
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet(),firmware_profile='simple') as client:
            client.send_stop()
            with pytest.raises(CommunicationError,match='Earlier'):client.stop_and_wait()
            assert not client.is_connected() and client.estop_latched


def test_post_stop_overshoot_is_corrected_from_new_pose_not_assumed_stopped_position(monkeypatch):
    nav,plant,transport,plan=rig();original=nav._halt;shifted=False
    def coast(result=None):
        nonlocal shifted
        value=original(result)
        if not shifted:plant.x+=8;shifted=True  # Deliberate unmodeled post-STOP slip, still inside arena.
        return value
    monkeypatch.setattr(nav,'_halt',coast)
    nav.axis('X',70)
    assert shifted and abs(plant.x-70)<=2.5
    assert any(line.startswith('MOVE,B,') for line in transport.lines)
    assert any(e.get('segment_outcome')=='early_stopped' for e in nav.events)


def test_long_segment_full_sweep_is_checked_before_transmission():
    nav,plant,transport,plan=rig()
    nav.arena=Arena(obstacles=[[50,20,51,40]],margin=.1)
    with pytest.raises(ValueError,match='swept area'):nav.axis('X',70)
    assert not transport.lines


def test_correction_attempt_budget_still_applies_to_hybrid():
    nav,plant,transport,plan=rig(config=NavigationConfig(maximum_attempts=1))
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']=='BLOCKED' and not nav.ready
    assert 'Maximum heading correction' in result['reason']
    assert transport.lines[-2:]==['STOP','PING']


def test_camera_worker_timeout_does_not_block_motion_monitor(monkeypatch):
    import navigation.adapters as module
    tracker=SimpleNamespace(error=None,marker_plane=object(),config=SimpleNamespace(smoothing_alpha=1))
    monkeypatch.setattr(module,'build_robot_tracker',lambda *a,**k:tracker)
    release=threading.Event()
    source=CameraPoses(None,'cal','robot','map')
    monkeypatch.setattr(source,'_read_pose',lambda:(release.wait(),Pose(32,29,0,time.monotonic(),1))[1])
    source.start(.03)
    try:
        start=time.monotonic()
        with pytest.raises(ValueError,match='timed out'):source.observe()
        assert time.monotonic()-start<.25
    finally:release.set();source.close()


def test_post_settling_observation_waits_for_new_capture_instead_of_using_cached_latest():
    nav,plant,transport,plan=rig()
    clock=nav.clock;now=clock()
    rows=iter([Pose(32,29,0,now-.05,1),Pose(32,29,0,now+.002,2)])
    nav.poses=SimpleNamespace(observe=lambda:next(rows))
    clock.sleep(.003)
    assert nav.observe(after=now).frame_id==2


def test_stopping_calibration_cli_keeps_source_immutable(tmp_path):
    source=tmp_path/'timing.json'
    source.write_text('{"schema_version":1,"mode":"PHYSICAL","samples":[]}')
    before=source.read_bytes()
    assert main.main(['movement-calibration','add-stop','--file',str(source),'--direction','F',
        '--pose-latency-ms','80','--stop-latency-ms','60','--maximum-speed','30','--coast-units','0.4','--confirm-measured'])==0
    assert source.read_bytes()==before
    assert load_timing(tmp_path/'timing_001.json').data['stopping_samples'][0]['verified'] is True
