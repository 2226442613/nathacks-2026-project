"""Module06 deterministic feedback and SIMPLE compatibility; no hardware endpoints."""
from copy import deepcopy
from dataclasses import replace
from datetime import datetime,timezone,timedelta
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import main
from navigation.cli import make_demo,measured_sample
from navigation.controller import Navigator,NavigationConfig,NavigationFailure,Pose
from navigation.geometry import Arena
from navigation.simulation import SimClock,SimPlant,SimTransport,demo_timing
from navigation.timing import MovementTiming,angle_error,save_timing,load_timing
from robot_comm import RobotClient,RobotSimulator,CommunicationError,WireLogger
from robot_comm.simulator import SimpleFirmwareModel
from robot_comm.protocol import SIMPLE_CAPS
from test_robot_communication import Endpoint,standard_reply,wait_until


def quiet():return WireLogger(stream=io.StringIO())


def rig(surface='FLOOR',config=None,**plant_options):
    c=config or NavigationConfig()
    clock,plant,transport,timing,arena,adapter,plan=make_demo(surface,c,**plant_options)
    nav=Navigator(plant,transport,timing,arena,c,clock=clock,sleep=clock.sleep,check_approach=adapter.check)
    return nav,plant,transport,plan


@pytest.mark.parametrize('surface',['FLOOR','WALL'])
def test_complete_closed_loop_task_goes_home_only_after_cleaning_done(surface):
    nav,plant,transport,plan=rig(surface)
    result=nav.run(plan,'RED_SPILL')
    assert result['state']=='READY' and nav.ready and nav.cleaning_done and transport.cleaned
    states=[e['state'] for e in nav.events]
    assert states.index('WAITING_FOR_CLEANING_DONE')<states.index('RETURNING_X')<states.index('RETURNING_Y')<states.index('RESTORING_HOME_HEADING')<states.index('VERIFYING_HOME')<states.index('READY')
    assert math.hypot(plant.x-nav.home['x_cm'],plant.y-nav.home['y_cm'])<=3
    assert abs(angle_error(nav.home['heading_deg'],plant.heading))<=10
    assert transport.lines.count('ARRIVED')==1
    assert f'TARGET,{"GROUND" if surface=="FLOOR" else "WALL"},BLOOD' in transport.lines
    assert all(e['home_verified'] is False for e in nav.events if e['state'] not in {'COMPLETE','READY'})
    if surface=='WALL':
        assert all(abs(e['pose']['y_cm']-13)<1e-8 for e in nav.events if 'pose' in e)
        assert not any(line.startswith(('MOVE,L','MOVE,R')) for line in transport.lines)


def test_actual_tool_approach_not_stain_centroid_and_module05_original_retained():
    nav,plant,transport,plan=rig()
    assert plan['approach_pose']['x_cm']==pytest.approx(74)
    assert plan['approach_pose']['x_cm']!=90
    assert 'module05_approach_pose' in plan
    assert plan['execution_scope'].startswith('single fixed SIMPLE')


@pytest.mark.parametrize('gain',[.65,1.6])
def test_axis_under_and_overshoot_corrected_from_new_pose(gain):
    nav,plant,transport,plan=rig(move_gain=gain,turn_gain=1.)
    nav.axis('Y',38)
    assert abs(plant.y-38)<=2.5
    corrected=[e for e in nav.events if e['state']=='CORRECTING_Y' and 'command' in e]
    assert corrected
    if gain>1:assert any(e['command'].startswith('MOVE,B,') for e in corrected)
    completed=[e for e in nav.events if e.get('movement_completed')]
    assert all(e['fresh_pose']!=e['previous_pose'] for e in completed)


@pytest.mark.parametrize('gain',[.65,1.35])
def test_heading_under_overshoot_and_wraparound(gain):
    nav,plant,transport,plan=rig(config=NavigationConfig(maximum_pulse_ms=4000,maximum_segment_ms=4000,correction_gain=1.),turn_gain=gain)
    nav.timing.data['stopping_samples']=[]  # Explicit fallback: verify pulse overshoot correction without early braking.
    nav.align(90,'ALIGNING_Y_HEADING')
    assert abs(angle_error(90,plant.heading))<=7
    commands=[line for line in transport.lines if line.startswith('MOVE,')]
    assert len(commands)>1
    if gain>1:assert any(line.startswith('MOVE,L,') for line in commands)
    assert angle_error(2,358)==4 and angle_error(358,2)==-4
    count=len(commands);plant.heading=358
    nav.align(2,'ALIGNING_X_HEADING')
    assert len(transport.lines)==count  # Four-degree wraparound is inside deadband.


def test_x_axis_and_deadband_no_unnecessary_move():
    nav,plant,transport,plan=rig(turn_gain=1.)
    nav.axis('X',70)
    assert abs(plant.x-70)<=2.5
    count=len(transport.lines)
    nav.axis('X',plant.x+2.49);nav.align(plant.heading+6.99,'ALIGNING_X_HEADING')
    assert len(transport.lines)==count


def test_y_before_x_and_return_x_before_y_log_invariants():
    nav,plant,transport,plan=rig()
    assert nav.run(plan,'NON_RED_SPILL')['state']=='READY'
    outbound=[e for e in nav.events if e['state'] in {'MOVING_Y','CORRECTING_Y','MOVING_X','CORRECTING_X'} and 'command' in e]
    axes=[e['state'][-1] for e in outbound]
    assert 'Y' in axes and 'X' in axes and max(i for i,a in enumerate(axes) if a=='Y')<min(i for i,a in enumerate(axes) if a=='X')
    assert any('cross_axis_drift_cm' in e for e in nav.events)


def test_no_progress_and_maximum_attempts_stop_instead_of_infinite_corrections():
    nav,plant,transport,plan=rig(move_gain=0.,turn_gain=1.)
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']=='BLOCKED' and 'no measurable progress' in result['reason']
    assert transport.stopped and not transport.cleaned and not nav.ready
    nav,plant,transport,plan=rig(config=NavigationConfig(maximum_attempts=1),turn_gain=.5)
    assert nav.run(plan,'NON_RED_SPILL')['state']=='BLOCKED'
    assert len(transport.lines)<10


def test_minimum_effective_pulse_blocks_without_sending_ineffective_move():
    nav,plant,transport,plan=rig(config=NavigationConfig(minimum_move_ms=400))
    with pytest.raises(NavigationFailure,match='minimum'):nav.axis('X',35)
    assert not transport.lines


@pytest.mark.parametrize('failure',['stale','lost','invalid','reused'])
def test_invalid_or_stale_localization_stops_navigation(failure):
    nav,plant,transport,plan=rig()
    if failure=='stale':plant.stale=True
    elif failure=='lost':plant.lost=True
    elif failure=='invalid':plant.valid=False
    else:
        original=plant.observe;first=original();plant.observe=lambda:first
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']=='PAUSED' and transport.stopped and not nav.ready
    assert not transport.cleaned


def test_pre_movement_cached_frame_cannot_complete_feedback(monkeypatch):
    nav,plant,transport,plan=rig()
    original=transport.wait_for_move
    def finish(result,**kwargs):
        value=original(result,**kwargs)
        frozen=plant.observe();plant.observe=lambda:frozen
        return value
    monkeypatch.setattr(transport,'wait_for_move',finish)
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']=='PAUSED'
    assert 'Stale or pre-movement' in result['reason'] and transport.stopped


@pytest.mark.parametrize('fault',['busy','no_done','disconnected','clean_fault'])
def test_transport_failure_never_replays_or_marks_ready(fault):
    nav,plant,transport,plan=rig()
    setattr(transport,fault,True)
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']=='ERROR' and not result['home_verified'] and transport.stopped
    if fault in {'busy','no_done','disconnected'}:
        assert sum(line.startswith('MOVE,') for line in transport.lines)<=1
    assert transport.lines.count('ARRIVED')<=1


def test_large_cross_drift_replans_in_order_then_stops_at_budget():
    nav,plant,transport,plan=rig(cross_drift=4,turn_gain=1.)
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state'] in {'BLOCKED','PAUSED','ERROR'} and not nav.ready
    if result['state']=='ERROR':assert 'swept area' in result['reason'] and transport.stopped
    assert any(e.get('reason')=='Cross-axis drift requires new ordered route' for e in nav.events)
    assert max(e.get('route_revision',0) for e in nav.events)<=nav.config.maximum_replans


def test_full_footprint_rotation_sweep_and_wall_reference_take_priority():
    arena=Arena(margin=.1)
    start={'x_cm':40,'y_cm':13,'heading_deg':0}
    assert arena.safe(start)
    end={**start,'heading_deg':90}
    with pytest.raises(ValueError,match='swept area'):arena.check_segment(start,end,turn_change=90)
    nav,plant,transport,plan=rig()
    plant.y=13
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']=='ERROR' and transport.stopped
    assert not any(line.startswith('MOVE,') for line in transport.lines)
    nav,plant,transport,plan=rig('WALL')
    plan['approach_pose']['y_cm']=20
    assert nav.run(plan,'NON_RED_SPILL')['state']=='BLOCKED'


def test_travel_swept_area_detects_obstacle_even_when_endpoints_are_clear():
    arena=Arena(obstacles=[[50,20,51,40]],margin=.1)
    a={'x_cm':30,'y_cm':30,'heading_deg':0};b={**a,'x_cm':80}
    assert arena.safe(a) and arena.safe(b)
    with pytest.raises(ValueError,match='swept area'):arena.check_segment(a,b)


def test_physical_estimates_preserve_history_and_use_observed_turn_heading_signs():
    timing=load_timing('config/movement_timing.json')
    assert timing.data['mode']=='PHYSICAL'
    assert [(s['direction'],s['duration_ms']) for s in timing.data['references']]==[('R',1500),('R',3000)]
    assert timing.rate('F')==pytest.approx((62.5,1))
    assert timing.rate('B')==pytest.approx((1000/18.5,-1))
    assert timing.rate('L')==pytest.approx((4000/90,-1))
    assert timing.rate('R')==pytest.approx((3000/90,1))
    assert timing.turn_direction(45)=='R' and timing.turn_direction(-45)=='L'


def test_timing_multiple_samples_signed_turns_immutable_revisions(tmp_path):
    timing=MovementTiming({'schema_version':1,'mode':'PHYSICAL','samples':[]})
    timing.add(direction='R',duration_ms=3000,observed_change=-90,verified=False,source='approximate report')
    with pytest.raises(ValueError):timing.rate('R')
    timing.add(direction='R',duration_ms=3000,observed_change=-90,verified=True,source='measured')
    timing.add(direction='R',duration_ms=3300,observed_change=-90,verified=True,source='measured')
    assert timing.rate('R')==pytest.approx((35,-1))
    with pytest.raises(ValueError):timing.rate('L')
    path=tmp_path/'timing.json';one=save_timing(path,timing);before=one.read_bytes()
    two=save_timing(path,timing)
    assert two!=one and one.read_bytes()==before
    assert load_timing(two).data==timing.data


@pytest.mark.parametrize('duration',[0,10001,True,1.5])
def test_timing_duration_boundaries(duration):
    timing=demo_timing()
    with pytest.raises(ValueError):timing.add(direction='F',duration_ms=duration,observed_change=10,verified=True,source='bad')


def test_module04_pose_adapter_uses_ground_center_not_marker_pixels(monkeypatch):
    import navigation.adapters as module
    pose=SimpleNamespace(tracking_status='VISIBLE',localization_status='VERIFIED',suitable_for_navigation=True,position_cm={'x':40.,'y':30.},heading_deg=90,
                         pose_quality={'observation_current':True,'mapping_method':'EMPIRICAL_FIXED_HEIGHT_PLANE'},marker_center_pixels=(900,500))
    tracker=SimpleNamespace(error=None,marker_plane=object(),config=SimpleNamespace(smoothing_alpha=1),update=lambda f:pose)
    monkeypatch.setattr(module,'build_robot_tracker',lambda *a,**k:tracker)
    source=module.CameraPoses(SimpleNamespace(read=lambda:object()),'cal','robot','mapping')
    observed=source.observe()
    assert (observed.x_cm,observed.y_cm,observed.heading_deg)==(40,30,90)
    pose.tracking_status='LOST'
    with pytest.raises(ValueError,match='ground-center'):source.observe()


def pose_record(x,y,heading,seconds=0):
    return {'message_type':'ROBOT_POSE','tracking_status':'VISIBLE','localization_status':'UNVERIFIED',
            'position_cm':{'x':x,'y':y},'heading_deg':heading,'calibration_id':'cal','marker_configuration':{'height':16.6},
            'timestamp':(datetime(2026,1,1,tzinfo=timezone.utc)+timedelta(seconds=seconds)).isoformat(),
            'pose_quality':{'mapping_method':'EMPIRICAL_FIXED_HEIGHT_PLANE'}}


def test_calibration_from_actual_ground_center_before_after_and_no_fabricated_left_rate():
    before=pose_record(40,30,90)
    assert measured_sample('F',before,pose_record(40,40,90,1))==pytest.approx(10)
    assert measured_sample('B',before,pose_record(40,20,90,1))==pytest.approx(-10)
    assert measured_sample('R',before,pose_record(40,30,0,3))==pytest.approx(-90)
    with pytest.raises(ValueError):measured_sample('F',before,pose_record(45,40,90,1))


def test_cli_simulation_and_timing_commands_preserve_files(tmp_path):
    output=tmp_path/'run'
    assert main.main(['simulate-navigation','--output-dir',str(output)])==0
    result=json.loads((output/'summary.json').read_text())
    assert result['state']=='READY' and result['mode']=='SIMULATION' and not result['physical_accuracy_claim']
    events=[json.loads(line) for line in (output/'progress.jsonl').read_text().splitlines()]
    assert events[-1]['phase']=='Ready' and events[-1]['home_verified'] and events[-1]['cleaning_done_received']
    assert not any('HEARTBEAT' in str(e) or 'HOME,' in str(e) for e in events)
    assert main.main(['simulate-navigation','--output-dir',str(output)])==1
    source=tmp_path/'timing.json';source.write_text(json.dumps({'schema_version':1,'mode':'PHYSICAL','samples':[]}))
    before=source.read_bytes()
    assert main.main(['movement-calibration','add','--file',str(source),'--direction','R','--duration-ms','3000','--angle-deg','-90','--confirm-measured'])==0
    assert source.read_bytes()==before
    assert load_timing(tmp_path/'timing_001.json').rate('R')==pytest.approx((3000/90,-1))


def test_simple_client_never_transmits_legacy_safety_and_distinguishes_done():
    with RobotSimulator(port=0,logger=quiet(),firmware_profile='simple') as server:
        with RobotClient(port=server.port,logger=quiet(),firmware_profile='simple') as client:
            assert client.simple_supported and not client.safety_supported and client.heartbeat is None
            accepted=client.send_command('MOVE,R,30')
            assert accepted.firmware_accepted and not accepted.execution_completed
            assert client.wait_for_move(accepted).execution_completed
            client.begin_cleaning('WALL','OTHER')
            client.send_stop()
            with pytest.raises(CommunicationError):client.wait_for_cleaning(timeout=.2)
            log=client.logger.stream.getvalue()
            for command in ('HEARTBEAT','SAFETY','ESTOP','RESET_ESTOP'):
                assert '[TX] '+command+'\n' not in log
            assert client.send_command('STATUS').response=='STATUS,IDLE'


@pytest.mark.parametrize('reply',[b'OK,ARRIVED\nEVENT,CLEANING_STARTED\nDONE,CLEANING\n',
                                   b'EVENT,CLEANING_STARTED\nDONE,CLEANING\nOK,ARRIVED\n'])
def test_simple_cleaning_event_ack_order(reply):
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        if line.startswith('TARGET,'):return b'OK,TARGET\n'
        if line=='ARRIVED':return reply
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet(),firmware_profile='simple') as client:
            client.begin_cleaning('GROUND','BLOOD')
            assert client.wait_for_cleaning(timeout=.2)=='DONE,CLEANING'


@pytest.mark.parametrize('fault',['EVENT,PUMP_TIMEOUT','EVENT,CLEAN_MOTOR_TIMEOUT'])
def test_simple_cleaning_fault_cannot_be_mistaken_for_done(fault):
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        if line.startswith('TARGET,'):return b'OK,TARGET\n'
        if line=='ARRIVED':return ('OK,ARRIVED\n'+fault+'\nDONE,CLEANING\n').encode()
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet(),firmware_profile='simple') as client:
            client.begin_cleaning('GROUND','OTHER')
            with pytest.raises(CommunicationError,match='aborted'):client.wait_for_cleaning(timeout=.2)
            assert endpoint.lines.count('ARRIVED')==1


def test_simple_explicit_profile_refuses_wrong_capabilities():
    with Endpoint(standard_reply) as endpoint:
        client=RobotClient(port=endpoint.port,logger=quiet(),firmware_profile='simple')
        with pytest.raises(CommunicationError,match='SIMPLE'):client.connect()
        assert 'SAFETY' not in endpoint.lines and 'HEARTBEAT' not in endpoint.lines


@pytest.mark.parametrize('reply',[b'DONE,MOVE\nOK,MOVE\n',b'OK,MOVE\nERR,CONTROLLER_FAULT\nDONE,MOVE\n'])
def test_simple_move_fault_or_done_before_ack_is_never_completed(reply):
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        if line.startswith('MOVE,'):return reply
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet(),firmware_profile='simple') as client:
            try:
                result=client.send_command('MOVE,R,100')
                result=client.wait_for_move(result)
                assert not result.execution_completed
            except CommunicationError:pass
            assert endpoint.lines.count('MOVE,R,100')==1


@pytest.mark.parametrize('failure',['timeout','disconnect'])
def test_simple_cleaning_unknown_never_replays_or_recovers(failure):
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        if line.startswith('TARGET,'):return b'OK,TARGET\n'
        if line=='ARRIVED':return b'OK,ARRIVED\n'
        if line=='STATUS' and failure=='disconnect':return False
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,logger=quiet(),firmware_profile='simple') as client:
            client.begin_cleaning('GROUND','OTHER')
            if failure=='disconnect':
                with pytest.raises(CommunicationError):client.request_status()
            with pytest.raises(CommunicationError,match='unknown'):client.wait_for_cleaning(timeout=.05)
            assert not client.is_connected() and client.estop_latched
            with pytest.raises(CommunicationError):client.send_command('MOVE,F,10')
            assert endpoint.lines.count('ARRIVED')==1


@pytest.mark.parametrize('stage',['move','cleaning'])
def test_localization_loss_during_active_wait_stops_without_next_action(monkeypatch,stage):
    nav,plant,transport,plan=rig()
    method='wait_for_move' if stage=='move' else 'wait_for_cleaning'
    original=getattr(transport,method)
    def lose(*args,**kwargs):
        plant.lost=True
        return original(*args,**kwargs)
    monkeypatch.setattr(transport,method,lose)
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']=='PAUSED' and transport.stopped and not nav.ready
    assert transport.lines[-2:]==['STOP','PING']  # Fresh STOP confirmation and old-reply barrier.
    if stage=='move':assert not transport.cleaned and transport.lines.count('ARRIVED')==0


@pytest.mark.parametrize('surface',['FLOOR','WALL'])
def test_simple_cleaning_turns_pump_off_homes_servos_and_sends_done_once(surface):
    clock=SimClock();model=SimpleFirmwareModel(clock)
    assert model.connected()[0]=='CONNECTED,NATHACK_V2'
    model.command(f'TARGET,{"GROUND" if surface=="FLOOR" else "WALL"},OTHER');model.command('ARRIVED')
    events=[];directions=[]
    for _ in range(800):
        clock.sleep(.02);events+=model.tick()
        if model.direction:directions.append(model.direction);assert model.pwm==170
    assert events.count('DONE,CLEANING')==1
    assert model.pwm==0 and model.pump=='OFF' and model.spray_deg==180 and model.sponge_deg==180
    assert set(directions)==({'F','B'} if surface=='FLOOR' else set())


def test_final_home_verification_rejects_outside_radial_tolerance(monkeypatch):
    nav,plant,transport,plan=rig()
    original=nav.route
    def route(goal,**kwargs):
        original(goal,**kwargs)
        if kwargs.get('returning'):plant.x=nav.home['x_cm']+3.1
    monkeypatch.setattr(nav,'route',route)
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']=='BLOCKED' and 'HOME' in result['reason'] and not nav.ready
    assert nav.cleaning_done and transport.stopped


def test_simple_model_has_independent_600_1700_bounds_and_no_watchdog():
    clock=SimClock();model=SimpleFirmwareModel(clock);model.connected()
    assert model.command('HEARTBEAT')==['ERR,UNKNOWN_COMMAND']
    assert model.command('TARGET,GROUND,OTHER')==['OK,TARGET']
    model.command('ARRIVED');model.state='STALLED_TEST'
    clock.sleep(1.701)
    assert model.tick()==['EVENT,PUMP_TIMEOUT'] and model.pump=='OFF'
    model.command('TARGET,GROUND,OTHER');model.command('ARRIVED')
    clock.sleep(1.201);model.tick()
    assert model.pwm==170 and model.pump=='OFF'
    model.state='STALLED_TEST';clock.sleep(.601)
    assert model.tick()==['EVENT,CLEAN_MOTOR_TIMEOUT'] and model.pwm==0
    assert model.command('MOVE,F,2000')==['OK,MOVE']
    clock.sleep(2.001);assert model.tick()==['DONE,MOVE']
    assert not model.watchdog_expired


def test_dashboard_progress_does_not_complete_at_cleaning_done(tmp_path):
    from dashboard.service import DashboardService
    from dashboard.demo import demo_event
    path=tmp_path/'progress.jsonl'
    service=DashboardService(calibration='data/module03_demo/calibration.json',navigation_events=path)
    event=demo_event(1,'NON_RED_SPILL','FLOOR',service.arena,service.calibration_id,1)
    service.ingest(event,'DEMO');key='demo:0:1'
    rows=[{'event_type':'NAVIGATION_PROGRESS','mode':'SIMULATION','task_key':key,'phase':'Returning',
           'home_verified':False,'cleaning_done_received':True}]
    path.write_text(json.dumps(rows[0])+'\n')
    task=service.snapshot()['tasks'][0]
    assert task['task_status']=='IN_PROGRESS' and task['navigation_progress']['label']=='Returning'
    with pytest.raises(ValueError,match='home'):service.transition(key,'COMPLETED')
    with path.open('a') as stream:stream.write(json.dumps({**rows[0],'phase':'Ready','home_verified':True})+'\n')
    task=service.snapshot()['tasks'][0]
    assert task['task_status']=='COMPLETED' and task['navigation_progress']['label']=='Ready'
    assert 'pose' not in task['navigation_progress']
