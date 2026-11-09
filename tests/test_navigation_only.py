"""Supervised navigation/trust regression: synthetic poses and loopback TCP only."""
from copy import deepcopy
from dataclasses import asdict,replace
import io
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import main
from navigation.timing import load_timing,angle_error
from robot_comm import RobotClient,CommunicationError,WireLogger
from robot_comm.protocol import SIMPLE_CAPS
from vision import RobotConfig,RobotTracker,CameraRegistration
from vision.robot_cli import build_robot_tracker
from test_navigation import rig
from test_robot_tracking import floor,mapping,training_data,observed,marker_scene
from test_robot_communication import Endpoint,standard_reply


GOAL={'x_cm':64.,'y_cm':36.,'heading_deg':0.}


def verification_evidence(config):
    training,validation=training_data()
    return dict(training=training,validation=validation,metadata={
        'attested_measured':True,'measurement_reference':'MARKER_GROUND_PROJECTION',
        'calibration_id':'cal','frame_size':[800,600],'mounting_height_cm':10,
        'robot_configuration':asdict(config),'heading_alignment_verified':False,
        'chassis_reference_conversion_supported':False})


def verified_tracker(floor):
    config=RobotConfig(mounting_height_cm=10)
    return RobotTracker(floor,config,calibration_id='cal',marker_plane=mapping()),verification_evidence(config)


def test_plane_pass_is_separate_from_current_context_verification(floor,monkeypatch):
    tracker,evidence=verified_tracker(floor)
    monkeypatch.setattr(tracker.detector,'detect',lambda f:observed(center=(420,250)))
    original=deepcopy(evidence)
    pose=tracker.update(marker_scene(),timestamp=10)
    assert pose.pose_quality['mapping_validation_status']=='PASS'
    assert pose.localization_status=='UNVERIFIED' and not pose.suitable_for_navigation
    with pytest.raises(ValueError,match='camera'):tracker.verify_context(evidence)
    with pytest.raises(ValueError,match='chassis'):tracker.verify_context(evidence,fixed_camera=True)
    tracker.verify_context(evidence,fixed_camera=True,ground_reference_confirmed=True)
    pose=tracker.update(marker_scene(),timestamp=11)
    assert pose.localization_status=='VERIFIED' and pose.suitable_for_navigation
    assert pose.position_cm==pytest.approx({'x':40,'y':30})
    assert not pose.motion_authorized and not pose.actionable and evidence==original
    message=json.loads(json.dumps(pose.to_dict(),allow_nan=False))
    schema=json.loads(Path('config/robot_pose_v1.schema.json').read_text())
    assert message['localization_status'] in schema['properties']['localization_status']['enum']
    assert all(k in message for k in schema['required'])
    monkeypatch.setattr(tracker.detector,'detect',lambda f:None)
    lost=tracker.update(marker_scene(),timestamp=12)
    assert not lost.suitable_for_navigation and lost.position_cm is None


@pytest.mark.parametrize('damage',['calibration','frame','height','marker_size','offset','heading','points','measurement'])
def test_context_mismatch_cannot_unlock_localization(floor,damage):
    tracker,evidence=verified_tracker(floor)
    tracker.verify_context(evidence,fixed_camera=True,ground_reference_confirmed=True)
    m=evidence['metadata']
    if damage=='calibration':m['calibration_id']='other'
    elif damage=='frame':m['frame_size']=[1280,720]
    elif damage=='height':m['mounting_height_cm']=11
    elif damage=='marker_size':m['robot_configuration']['marker_size_cm']=8
    elif damage=='offset':m['robot_configuration']['marker_offset_forward_cm']=2
    elif damage=='heading':m['robot_configuration']['marker_to_robot_heading_deg']=90
    elif damage=='points':evidence['validation'][0]['pixel'][0]+=1
    else:m['attested_measured']=False
    with pytest.raises(ValueError):tracker.verify_context(evidence,fixed_camera=True,ground_reference_confirmed=True)
    assert tracker.verified_context is None


@pytest.mark.parametrize('registration',[CameraRegistration(status='MOVED'),
    CameraRegistration(status='MATCHED',current_to_calibration=np.eye(3),compensated=True),
    CameraRegistration(status='MATCHED',current_to_calibration=np.eye(3),override_active=True)])
def test_moved_or_overridden_camera_never_produces_verified_pose(floor,monkeypatch,registration):
    tracker,evidence=verified_tracker(floor)
    tracker.verify_context(evidence,fixed_camera=True,ground_reference_confirmed=True)
    monkeypatch.setattr(tracker.detector,'detect',lambda f:observed(center=(420,250)))
    pose=tracker.update(marker_scene(),registration=registration,timestamp=10)
    assert pose.localization_status=='UNVERIFIED' and not pose.suitable_for_navigation


def test_wrong_live_marker_scale_remains_unverified(floor,monkeypatch):
    tracker,evidence=verified_tracker(floor)
    tracker.verify_context(evidence,fixed_camera=True,ground_reference_confirmed=True)
    obs=observed(center=(420,250))
    corners=np.asarray([420,250])+(np.asarray(obs.corners_pixels)-[420,250])*2
    # Only the size matters here; marker center and heading remain unchanged.
    monkeypatch.setattr(tracker.detector,'detect',lambda f:replace(obs,corners_pixels=tuple(map(tuple,corners))))
    assert not tracker.update(marker_scene(),timestamp=10).suitable_for_navigation


def test_saved_physical_mapping_and_settings_match_without_rewriting_them():
    paths=['config/camera_config.json','config/robot_config.json','config/robot_marker_plane.json',
           'data/robot_plane_points.json','data/robot_plane_points_003.json','data/robot_plane_points_marker9.json']
    if not Path(paths[0]).is_file():pytest.skip('Local physical camera calibration is backed up separately, not distributed in Git')
    before={p:Path(p).read_bytes() for p in paths}
    tracker=build_robot_tracker(paths[1],paths[0],marker_plane_path=paths[2],verification_points_path=paths[5],
        fixed_camera=True,ground_reference_confirmed=True)
    assert tracker.error is None and tracker.verified_context
    assert tracker.marker_plane.report['rms_error_cm']==pytest.approx(1.04459768)
    assert tracker.marker_plane.report['maximum_error_cm']==pytest.approx(1.25286084)
    assert (tracker.config.marker_size_cm,tracker.config.mounting_height_cm)==(9,16.6)
    # Historical nominal size remains 6; no mismatch exception is bypassed.
    with pytest.raises(ValueError,match='settings mismatch'):
        build_robot_tracker(paths[1],paths[0],marker_plane_path=paths[2],verification_points_path=paths[4],
            fixed_camera=True,ground_reference_confirmed=True)
    # Original 4+1 interrupted snapshot cannot verify the finished 4+3 map.
    with pytest.raises(ValueError,match='points'):
        build_robot_tracker(paths[1],paths[0],marker_plane_path=paths[2],verification_points_path=paths[3],
            fixed_camera=True,ground_reference_confirmed=True)
    assert all(Path(p).read_bytes()==before[p] for p in paths)


def test_marker9_revision_changes_only_nominal_size_and_keeps_original_measurements():
    import hashlib
    original_path=Path('data/robot_plane_points_003.json')
    original=json.loads(original_path.read_text())
    corrected=json.loads(Path('data/robot_plane_points_marker9.json').read_text())
    correction=corrected['metadata'].pop('configuration_corrections')[0]
    assert correction['previous_value']==6 and correction['corrected_value']==9
    assert correction['original_evidence_sha256']==hashlib.sha256(original_path.read_bytes()).hexdigest()
    assert corrected['metadata']['robot_configuration']['marker_size_cm']==9
    corrected['metadata']['robot_configuration']['marker_size_cm']=6
    assert corrected==original  # Includes every original pixel observation and capture record.


def test_nine_cm_marker_fixes_scale_gate_without_changing_empirical_pose(floor,monkeypatch):
    obs=observed(center=(420,250))
    corners=np.array([420,250])+(np.asarray(obs.corners_pixels)-[420,250])*1.5
    obs=replace(obs,corners_pixels=tuple(map(tuple,corners)))
    results=[]
    for size in (6.,9.):
        config=RobotConfig(mounting_height_cm=10,marker_size_cm=size)
        tracker=RobotTracker(floor,config,calibration_id='cal',marker_plane=mapping())
        tracker.verify_context(verification_evidence(config),fixed_camera=True,ground_reference_confirmed=True)
        monkeypatch.setattr(tracker.detector,'detect',lambda frame:obs)
        results.append(tracker.update(marker_scene(),timestamp=10))
    old,new=results
    assert old.localization_status=='UNVERIFIED' and old.pose_quality['marker_scale_discrepancy_fraction']==pytest.approx(.5)
    assert new.localization_status=='VERIFIED' and new.suitable_for_navigation
    assert old.position_cm==new.position_cm and old.heading_deg==new.heading_deg
    assert new.pose_quality['marker_scale_discrepancy_fraction']==pytest.approx(0,abs=1e-6)


def test_auto_navigation_loads_marker9_and_accepts_verified_pose_without_hardware(monkeypatch):
    from navigation.adapters import CameraPoses
    from vision.robot_tracking import MarkerObservation,project_points
    args=main.build_parser().parse_args(['auto-clean','--navigation-only','--mode','physical',
        '--host','never-connect.invalid','--confirm-navigation-test'])
    if not Path(args.calibration).exists():pytest.skip('Private camera calibration not distributed')
    frame=np.zeros((args.height,args.width,3),np.uint8)
    source=CameraPoses(SimpleNamespace(read=lambda:frame),args.calibration,args.robot_settings,args.marker_plane,
        verification_points_path=args.verification_points,fixed_camera=True,ground_reference_confirmed=True)
    assert source.tracker.config.marker_size_cm==9
    inverse=np.linalg.inv(source.tracker.marker_plane.matrix)
    pixels=project_points([[60,35],[57.75,35],[62.25,35],
        [55.5,30.5],[64.5,30.5],[64.5,39.5],[55.5,39.5]],inverse)
    obs=MarkerObservation(0,tuple(map(tuple,pixels[3:])),tuple(pixels[0]),tuple(map(tuple,pixels[1:3])),{})
    monkeypatch.setattr(source.tracker.detector,'detect',lambda f:obs)
    pose=source.observe()
    assert pose.verified and (pose.x_cm,pose.y_cm,pose.heading_deg)==pytest.approx((58,34,0),abs=1e-5)


def test_unverified_adapter_and_physical_controller_reject_before_move(monkeypatch):
    import navigation.adapters as module
    pose=SimpleNamespace(tracking_status='VISIBLE',localization_status='UNVERIFIED',suitable_for_navigation=False,
        position_cm={'x':40,'y':30},heading_deg=0,
        pose_quality={'observation_current':True,'mapping_method':'EMPIRICAL_FIXED_HEIGHT_PLANE'})
    tracker=SimpleNamespace(error=None,marker_plane=object(),config=SimpleNamespace(smoothing_alpha=1),update=lambda f:pose)
    monkeypatch.setattr(module,'build_robot_tracker',lambda *a,**k:tracker)
    source=module.CameraPoses(SimpleNamespace(read=lambda:object()),'cal','settings','map')
    with pytest.raises(ValueError,match='ground-center'):source.observe()
    nav,plant,transport,_=rig()
    nav.timing=load_timing('config/movement_timing.json');original=plant.observe
    monkeypatch.setattr(plant,'observe',lambda:replace(original(),verified=False))
    result=nav.run_navigation_only(GOAL)
    assert result['state']=='PAUSED' and 'VERIFIED' in result['reason']
    assert not any(line.startswith('MOVE,') for line in transport.lines)


def test_camera_startup_retries_missing_frames_with_existing_tracker(floor,monkeypatch):
    import time
    import navigation.adapters as module
    tracker,evidence=verified_tracker(floor)
    tracker.verify_context(evidence,fixed_camera=True,ground_reference_confirmed=True)
    detections=iter([None,None,observed(center=(420,250))])
    monkeypatch.setattr(tracker.detector,'detect',lambda frame:next(detections,observed(center=(420,250))))
    monkeypatch.setattr(module,'build_robot_tracker',lambda *a,**k:tracker)
    def read():
        time.sleep(.005)
        return marker_scene()
    logs=[]
    source=module.CameraPoses(SimpleNamespace(read=read),'cal','robot','mapping',log=logs.append)
    try:
        source.start(.1,startup_wait_seconds=1)
        pose=source.observe()
        assert pose.verified and (pose.x_cm,pose.y_cm)==pytest.approx((40,30))
        assert source.frame>=3
        assert any('MARKER_NOT_VISIBLE' in line for line in logs)
        assert any('POSE_VERIFIED' in line for line in logs)
        # Runtime observation still fails immediately on loss; capture remains
        # available for an explicitly stationary reacquisition window.
        monkeypatch.setattr(tracker.detector,'detect',lambda frame:None)
        deadline=time.monotonic()+1
        while source.capture_error is None and time.monotonic()<deadline:time.sleep(.005)
        with pytest.raises(ValueError,match='STALE_POSE'):source.observe()
        assert source.worker.is_alive() and source.latest is None
    finally:source.close()


def test_camera_startup_without_marker_is_bounded(floor,monkeypatch):
    import time
    import navigation.adapters as module
    tracker,evidence=verified_tracker(floor)
    tracker.verify_context(evidence,fixed_camera=True,ground_reference_confirmed=True)
    monkeypatch.setattr(tracker.detector,'detect',lambda frame:None)
    monkeypatch.setattr(module,'build_robot_tracker',lambda *a,**k:tracker)
    logs=[]
    source=module.CameraPoses(SimpleNamespace(read=marker_scene),'cal','robot','mapping',log=logs.append)
    started=time.monotonic()
    try:
        source.start(.1,startup_wait_seconds=.05)
        with pytest.raises(ValueError,match='STARTUP_TIMEOUT|MARKER_NOT_VISIBLE'):source.observe()
        assert source.latest is None and time.monotonic()-started<1
        assert any('MARKER_NOT_VISIBLE' in line for line in logs)
    finally:source.close()


@pytest.mark.parametrize('damage,code',[
    ('unverified','LOCALIZATION_UNVERIFIED'),('raw_floor','MAPPING_ERROR'),
    ('stale','STALE_POSE'),('invalid','LOCALIZATION_INVALID'),('camera','CAMERA_ERROR')])
def test_camera_pose_failure_diagnostics(monkeypatch,damage,code):
    import navigation.adapters as module
    pose=SimpleNamespace(tracking_status='VISIBLE',localization_status='VERIFIED',suitable_for_navigation=True,
        position_cm={'x':40,'y':30},heading_deg=0,
        pose_quality={'observation_current':True,'mapping_method':'EMPIRICAL_FIXED_HEIGHT_PLANE','reason':'tracker detail'})
    if damage=='unverified':pose.localization_status='UNVERIFIED';pose.suitable_for_navigation=False
    if damage=='invalid':pose.localization_status='INVALID';pose.suitable_for_navigation=False
    if damage=='raw_floor':pose.pose_quality['mapping_method']='APPROXIMATE_FLOOR_RAY_INTERSECTION'
    if damage=='stale':pose.pose_quality['observation_current']=False
    tracker=SimpleNamespace(error=None,marker_plane=object(),config=SimpleNamespace(smoothing_alpha=1),update=lambda f:pose)
    monkeypatch.setattr(module,'build_robot_tracker',lambda *a,**k:tracker)
    logs=[]
    source=module.CameraPoses(SimpleNamespace(read=lambda:None if damage=='camera' else object()),'cal','robot','mapping',log=logs.append)
    try:
        source.start(.1)
        with pytest.raises(ValueError,match=code):source.observe()
        assert source.latest is None
        assert any(code in line for line in logs)
    finally:source.close()


def test_camera_mapping_context_mismatch_is_logged(monkeypatch):
    import navigation.adapters as module
    def mismatch(*a,**k):raise ValueError('Verification points do not match the saved validated mapping')
    monkeypatch.setattr(module,'build_robot_tracker',mismatch)
    logs=[]
    with pytest.raises(ValueError,match='Verification points'):
        module.CameraPoses(None,'cal','robot','mapping',log=logs.append)
    assert 'MAPPING_CONTEXT_ERROR' in logs[0]


def test_navigation_only_round_trip_never_calls_cleaning(monkeypatch):
    nav,plant,transport,_=rig()
    def forbidden(*a,**k):raise AssertionError('Navigation-only invoked cleaning')
    monkeypatch.setattr(transport,'begin_cleaning',forbidden)
    monkeypatch.setattr(transport,'wait_for_cleaning',forbidden)
    log=WireLogger(stream=io.StringIO());transport.logger=log
    result=nav.run_navigation_only(GOAL)
    assert result['state']=='READY' and result['arrival_verified'] and result['home_verified']
    assert not nav.cleaning_done and not transport.cleaned
    assert all(not line.startswith(('TARGET','ARRIVED')) for line in transport.lines)
    assert all(int(line.split(',')[2])<=1000 for line in transport.lines if line.startswith('MOVE,'))
    states=[e['state'] for e in nav.events]
    assert states.index('MOVING_Y')<states.index('MOVING_X')<states.index('VERIFYING_ARRIVAL')<states.index('RETURNING_X')<states.index('RETURNING_Y')<states.index('RESTORING_HOME_HEADING')<states.index('VERIFYING_HOME')<states.index('READY')
    assert any(e.get('early_stop') for e in nav.events)
    assert abs(angle_error(nav.home['heading_deg'],plant.heading))<=nav.config.home_heading_tolerance_deg
    text=log.stream.getvalue()
    for token in ('[TX] MOVE,','[RX] OK,MOVE','[RX] DONE,MOVE','[TX] STOP','[RX] OK,STOP'):
        assert token in text
    assert all(e['navigation_only'] and not e['cleaning_done_received'] for e in nav.events)


@pytest.mark.parametrize('fault',['lost','stale','no_done','disconnected','missing_stop_ack'])
def test_navigation_only_fault_stops_and_locks_without_return_or_retry(monkeypatch,fault):
    nav,plant,transport,_=rig()
    if fault in {'lost','stale'}:setattr(plant,fault,True)
    else:setattr(transport,fault,True)
    result=nav.run_navigation_only(GOAL)
    assert result['state']!='READY' and nav.faulted and transport.stopped
    assert not any(e['state'].startswith('RETURNING') for e in nav.events)
    previous=list(transport.lines)
    assert nav.run_navigation_only(GOAL)['state']=='ERROR'
    assert transport.lines==previous


def test_navigation_only_checks_complete_route_clearance_before_first_move():
    nav,plant,transport,_=rig()
    plant.y=13  # Chassis fits stationary but there is no safe in-place 90-degree turn.
    result=nav.run_navigation_only(GOAL)
    assert result['state']!='READY' and 'swept area' in result['reason']
    assert not any(line.startswith('MOVE,') for line in transport.lines)


def test_observed_physical_signs_and_rates_round_trip_with_synthetic_feedback():
    nav,plant,transport,_=rig()
    nav.timing=load_timing('config/movement_timing.json')
    plant.actual_rates.update(F=62.5,B=1000/18.5,L=4000/90,R=3000/90)
    assert nav.run_navigation_only(GOAL)['state']=='READY'
    commands=[line for line in transport.lines if line.startswith('MOVE,')]
    assert any(line.startswith('MOVE,L,') for line in commands)
    assert any(line.startswith('MOVE,R,') for line in commands)
    assert all(int(line.split(',')[2])<=500 for line in commands)


def test_navigation_only_cli_is_simulation_by_default_and_preserves_evidence(tmp_path):
    output=tmp_path/'round-trip';args=['navigate-test','--x-cm','64','--y-cm','36','--output-dir',str(output)]
    assert main.main(args)==0
    result=json.loads((output/'summary.json').read_text())
    assert result['mode']=='SIMULATION' and result['state']=='READY' and result['navigation_only']
    log=(output/'wire.log').read_bytes()
    assert b'[TX] MOVE,' in log and b'[RX] DONE,MOVE' in log
    assert b'TARGET,' not in log and b'ARRIVED' not in log
    assert main.main(args)==1 and (output/'wire.log').read_bytes()==log


def test_physical_cli_requires_explicit_checks_before_camera_or_network(tmp_path,monkeypatch):
    import navigation.cli as module
    def forbidden(*a,**k):raise AssertionError('Hardware accessed before operator checks')
    monkeypatch.setattr(module,'CameraManager',forbidden);monkeypatch.setattr(module,'RobotClient',forbidden)
    assert main.main(['navigate-test','--mode','physical','--host','never-connect.invalid',
        '--x-cm','64','--y-cm','36','--output-dir',str(tmp_path/'physical')])==1


@pytest.mark.parametrize('reply',[b'OK,ARRIVED\nDONE,CLEANING\n',
    b'OK,ARRIVED\nDONE,CLEANING\nEVENT,CLEANING_STARTED\n'])
def test_cleaning_done_without_prior_start_cannot_start_return(reply):
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        if line.startswith('TARGET,'):return b'OK,TARGET\n'
        if line=='ARRIVED':return reply
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,firmware_profile='simple',logger=WireLogger(stream=io.StringIO())) as client:
            client.begin_cleaning('GROUND','OTHER')
            with pytest.raises(CommunicationError,match='without start'):client.wait_for_cleaning(timeout=.1)
            assert not client.is_connected() and endpoint.lines.count('ARRIVED')==1


def test_cleaning_done_on_failed_connection_cannot_start_return():
    def response(line):
        if line=='CAPS':return (SIMPLE_CAPS+'\n').encode()
        if line.startswith('TARGET,'):return b'OK,TARGET\n'
        if line=='ARRIVED':return b'EVENT,CLEANING_STARTED\nDONE,CLEANING\nOK,ARRIVED\n'
        return standard_reply(line)
    with Endpoint(response) as endpoint:
        with RobotClient(port=endpoint.port,firmware_profile='simple',logger=WireLogger(stream=io.StringIO())) as client:
            client.begin_cleaning('GROUND','OTHER');client._fail('Synthetic link loss')
            with pytest.raises(CommunicationError,match='connection loss'):client.wait_for_cleaning(timeout=.1)


@pytest.mark.parametrize('fault',['clean_fault','lost_after_clean','wrong_done'])
def test_cleaning_fault_or_unknown_pose_never_automatically_returns(monkeypatch,fault):
    nav,plant,transport,plan=rig()
    original=transport.wait_for_cleaning
    def fail(*a,**k):
        if fault=='clean_fault':raise RuntimeError('EVENT,PUMP_TIMEOUT')
        if fault=='wrong_done':return 'OK,ARRIVED'
        value=original(*a,**k);plant.lost=True;return value
    monkeypatch.setattr(transport,'wait_for_cleaning',fail)
    result=nav.run(plan,'NON_RED_SPILL')
    assert result['state']!='READY' and nav.faulted and transport.stopped
    assert not any(e['state'].startswith('RETURNING') for e in nav.events)
    after_arrived=transport.lines[transport.lines.index('ARRIVED')+1:]
    assert not any(line.startswith('MOVE,') for line in after_arrived)
