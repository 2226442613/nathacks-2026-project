"""Automatic MVP integration: actual image detection/localization, synthetic robot."""
import json
from pathlib import Path
from dataclasses import replace
import io

import main
import cv2
import numpy as np
from navigation.simulation import SimTransport
import pytest


def test_auto_connect_timeout_defaults_and_invalid_values_do_not_open_hardware(tmp_path,monkeypatch):
    import navigation.automatic as module
    parser=main.build_parser()
    assert parser.parse_args(['auto-clean']).connect_timeout==5.
    assert parser.parse_args(['auto-clean','--connect-timeout','3']).connect_timeout==3.
    def forbidden(*a,**k):raise AssertionError('Hardware must not open')
    monkeypatch.setattr(module,'RobotClient',forbidden)
    monkeypatch.setattr(module,'CameraManager',forbidden)
    for value in ['0','-1','11','nan','inf']:
        out=tmp_path/value
        assert main.main(['auto-clean','--connect-timeout',value,'--output-dir',str(out)])==1
        assert not out.exists()


def test_automatic_detect_prioritize_clean_and_return_twice(tmp_path):
    out=tmp_path/'automatic'
    assert main.main(['auto-clean','--return-home','--output-dir',str(out)])==0
    result=json.loads((out/'summary.json').read_text())
    assert result['state']=='READY' and not result['skipped']
    assert [t['target']['category'] for t in result['completed']]==['RED_SPILL','NON_RED_SPILL']
    assert all(t['result']['home_verified'] for t in result['completed'])
    assert result['completed'][0]['result']['home']==result['completed'][1]['result']['home']
    log=(out/'wire.log').read_text()
    assert log.count('[TX] ARRIVED')==2 and log.count('[RX] DONE,CLEANING')==2
    assert log.index('[TX] TARGET,GROUND,BLOOD')<log.index('[TX] TARGET,GROUND,OTHER')
    events=[json.loads(line) for line in (out/'progress.jsonl').read_text().splitlines()]
    assert [e['priority'] for e in events if e['event_type']=='AUTO_TASK_SELECTED']==['HIGH','LOW']
    assert len([e for e in events if e.get('state')=='READY' and e['home_verified']])==2
    verified=[i for i,e in enumerate(events) if e['event_type']=='BUMPER_CLEARANCE_VERIFIED']
    cleaning=[i for i,e in enumerate(events) if e.get('state')=='CLEANING']
    assert len(verified)==2 and all(v<c for v,c in zip(verified,cleaning))
    assert all(abs(events[i]['clearance_cm']-5)<=2.5 and events[i]['facing_error_deg']<=7 for i in verified)
    before=(out/'summary.json').read_bytes()
    assert main.main(['auto-clean','--output-dir',str(out)])==1
    assert (out/'summary.json').read_bytes()==before


def test_automatic_cleaning_fault_does_not_return_or_take_next_task(tmp_path,monkeypatch):
    def fail(*a,**k):raise RuntimeError('Synthetic cleaning outcome unknown')
    monkeypatch.setattr(SimTransport,'wait_for_cleaning',fail)
    out=tmp_path/'fault'
    assert main.main(['auto-clean','--output-dir',str(out)])==1
    log=(out/'wire.log').read_text()
    assert log.count('[TX] ARRIVED')==1 and '[TX] STOP' in log
    assert '[TX] MOVE,' not in log.split('[TX] ARRIVED',1)[1]
    assert json.loads((out/'summary.json').read_text())['state']=='ERROR'


def test_shutdown_stop_failure_preserves_original_fault_and_saves_summary(tmp_path,monkeypatch,caplog):
    import navigation.automatic as module
    def fail(*a,**k):raise RuntimeError('STOP confirmation failed: PING timed out')
    def unavailable(*a,**k):raise RuntimeError('Communication unavailable; reconnect explicitly')
    monkeypatch.setattr(module,'run_queue',fail)
    monkeypatch.setattr(SimTransport,'send_stop',unavailable)
    out=tmp_path/'fault'
    assert main.main(['auto-clean','--output-dir',str(out)])==1
    summary=json.loads((out/'summary.json').read_text())
    assert summary['reason']=='STOP confirmation failed: PING timed out'
    assert summary['stop_error']=='Communication unavailable; reconnect explicitly'
    assert 'STOP confirmation failed: PING timed out' in caplog.text
    log=(out/'wire.log').read_text()
    assert '[TX] MOVE,' not in log and '[TX] ARRIVED' not in log


def test_automatic_physical_requires_authorization_before_hardware(tmp_path,monkeypatch):
    import navigation.automatic as module
    def fail(*a,**k):raise AssertionError('Physical hardware accessed')
    monkeypatch.setattr(module,'CameraManager',fail);monkeypatch.setattr(module,'RobotClient',fail)
    assert main.main(['auto-clean','--mode','physical','--output-dir',str(tmp_path/'physical')])==1
    assert not (tmp_path/'physical').exists()


def test_auto_navigation_detects_high_first_and_returns_without_cleaning(tmp_path,monkeypatch):
    def forbidden(*a,**k):raise AssertionError('Navigation-only invoked cleaning')
    monkeypatch.setattr(SimTransport,'begin_cleaning',forbidden)
    monkeypatch.setattr(SimTransport,'wait_for_cleaning',forbidden)
    out=tmp_path/'navigation'
    assert main.main(['auto-clean','--navigation-only','--output-dir',str(out)])==0
    result=json.loads((out/'summary.json').read_text())
    assert result['navigation_only'] and result['state']=='READY'
    assert len(result['completed'])==1  # Supervised test defaults to one target.
    task=result['completed'][0]
    assert task['target']['category']=='RED_SPILL' and task['result']['arrival_verified'] and task['result']['home_verified']
    log=(out/'wire.log').read_text()
    assert '[TX] MOVE,' in log and '[RX] DONE,MOVE' in log
    assert 'TARGET,' not in log and 'ARRIVED' not in log and 'CLEANING' not in log
    events=[json.loads(s) for s in (out/'progress.jsonl').read_text().splitlines()]
    assert len([e for e in events if e['event_type']=='BUMPER_CLEARANCE_VERIFIED'])==1
    states=[e.get('state') for e in events]
    assert states.index('MOVING_Y')<states.index('MOVING_X')<states.index('VERIFYING_ARRIVAL')<states.index('RETURNING_X')<states.index('RETURNING_Y')<states.index('RESTORING_HOME_HEADING')<states.index('VERIFYING_HOME')


def test_auto_navigation_does_not_need_cleaning_contact_geometry():
    from navigation.automatic import navigation_destination
    from navigation.geometry import Arena
    from navigation.cli import synthetic_calibration
    from vision.cleaning_config import CleaningHardware
    hardware=CleaningHardware(json.loads(Path('config/cleaning_hardware.json').read_text()))
    assert hardware.missing('FLOOR') and hardware.missing('WALL')
    arena=Arena(side_tool=hardware.side_collision_envelope())
    home={'x_cm':32.,'y_cm':29.,'heading_deg':0.}
    target={'surface':'FLOOR','position_cm':{'x':80.,'y':35.},
            'geometry_cm':{'bounds':dict(min_x=79.,min_y=34.,max_x=81.,max_y=36.)}}
    goal=navigation_destination(target,home,arena,synthetic_calibration(),1.5)
    assert arena.safe(goal) and goal==dict(x_cm=61.,y_cm=35.,heading_deg=0.)
    wall=navigation_destination({'surface':'WALL','position_cm':{'x':60.,'h':15.}},{**home,'heading_deg':90.},arena,synthetic_calibration(),1.5)
    assert arena.safe(wall) and wall['y_cm']>15 and wall['heading_deg'] in (0.,180.)
    with pytest.raises(RuntimeError,match='clearance'):
        navigation_destination(target,home,Arena(width=85,depth=40),synthetic_calibration(),1.5)


def test_auto_navigation_fault_stops_without_cleaning_or_home(tmp_path,monkeypatch):
    def lost(*a,**k):raise RuntimeError('Synthetic camera/link loss')
    monkeypatch.setattr(SimTransport,'wait_for_move',lost)
    out=tmp_path/'fault_navigation'
    assert main.main(['auto-clean','--navigation-only','--output-dir',str(out)])==1
    log=(out/'wire.log').read_text()
    assert '[TX] STOP' in log and 'TARGET,' not in log and 'ARRIVED' not in log
    events=[json.loads(s) for s in (out/'progress.jsonl').read_text().splitlines()]
    assert not any(e.get('state','').startswith('RETURNING') for e in events)


def test_navigation_confirmation_cannot_authorize_cleaning(tmp_path):
    assert main.main(['auto-clean','--confirm-navigation-test','--output-dir',str(tmp_path/'reject')])==1
    assert not (tmp_path/'reject').exists()


def geometry_scanner(*,shift=0.,textured=True,navigation_only=True,matrix=None):
    from navigation.automatic import SceneScanner
    from vision import BackgroundReference,GroundCalibration
    from robot_comm import WireLogger
    gray=np.random.default_rng(42).integers(60,180,(360,480),dtype=np.uint8) if textured else np.full((360,480),150,np.uint8)
    clean=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR);scene=clean.copy()
    cv2.circle(scene,(240,230),15,(20,20,220),-1)
    transform=np.float32([[1,0,shift],[0,1,0]]) if matrix is None else matrix
    frame=cv2.warpAffine(scene,transform,(480,360))
    calibration=GroundCalibration([[0,0],[479,0],[479,359],[0,359]],120,90,480,360)
    log=WireLogger(stream=io.StringIO())
    scanner=SceneScanner(BackgroundReference(clean,confirmed_clean=True),calibration,'synthetic-calibration',
        'config/spill_config.json',lambda:(frame,None),physical=True,navigation_only=navigation_only,logger=log)
    return scanner,frame,log


@pytest.mark.parametrize('shift,textured',[(0.,False),(.5,True),(4.,True)])
def test_navigation_detection_warnings_do_not_block_confirmed_targets(shift,textured):
    scanner,frame,log=geometry_scanner(shift=shift,textured=textured)
    records=scanner.scan()
    assert any(r['category']=='RED_SPILL' for r in records)
    assert all(r['localization_status']=='UNVERIFIED' and not r['motion_authorized'] for r in records)
    text=log.stream.getvalue()
    assert '[GEOMETRY]' in text and '[WARN]' in text
    report=json.loads(next(s.split('[GEOMETRY] ',1)[1] for s in text.splitlines() if '[GEOMETRY]' in s))
    assert report['hazard'] is None and report['frame_size']==[480,360]
    assert report['small_alignment_limit_pixels']==2 and report['limits']['center_shift_pixels']==8
    if shift==4:
        assert report['detection']['status']=='MOVED'
        assert 'small-shift' in report['detection']['reason']
        assert report['detection']['center_shift_pixels']==pytest.approx(4,abs=.3)
        assert report['detection']['corner_displacement_pixels']==pytest.approx(4,abs=.3)


@pytest.mark.parametrize('matrix',[
    np.float32([[1,0,18],[0,1,12]]),
    cv2.getRotationMatrix2D((240,180),5,1),
    cv2.getRotationMatrix2D((240,180),0,1.08)])
def test_background_framing_changes_warn_without_redefining_calibration(matrix):
    scanner,_,log=geometry_scanner(matrix=matrix)
    records=scanner.scan()
    assert records and all(r['calibration_id']=='synthetic-calibration' for r in records)
    assert all(r['camera_registration']['reference_id']=='synthetic-calibration' for r in records)
    text=log.stream.getvalue()
    assert 'Image framing exceeds configured motion limits' in text and 'inlier_count' in text
    assert 'center_shift_pixels' in text and 'rotation_degrees' in text and 'scale_change' in text
    assert '[WARN]' in text


def test_navigation_rejects_invalid_transform_and_wrong_resolution():
    scanner,frame,_=geometry_scanner()
    result=scanner.detector.detect(frame)
    from vision.localization import CameraRegistration
    with pytest.raises(RuntimeError,match='Invalid image registration transform'):
        scanner._check_geometry(replace(result,background_alignment_transform=np.zeros((3,3))),frame,CameraRegistration())
    scanner.frame_source=lambda:(frame[:100],None)
    with pytest.raises(ValueError,match='resolution.*differs from background'):scanner.scan()


def test_cleaning_uses_current_frame_estimates_despite_detection_quality_warning():
    scanner,_,_=geometry_scanner(textured=False,navigation_only=False)
    records=scanner.scan()
    assert records and all(r['localization_status']=='UNVERIFIED' and not r['motion_authorized'] for r in records)
