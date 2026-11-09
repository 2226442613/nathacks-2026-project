"""Operator wizard regressions; synthetic observations are not physical acceptance."""
import json
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision.calibration import GroundCalibration
from vision.robot_cli import load_robot_config
from vision.robot_tracking import MarkerObservation, RobotMarkerDetector, RobotTracker
from vision.robot_plane import MarkerPlaneMapping, MarkerPlaneValidationError
from vision.robot_calibration_wizard import (CaptureWindow, CalibrationSession, WizardUI,
                                           run_wizard, digest, write_new_json)
from test_robot_tracking import training_data, marker_scene

pytestmark = pytest.mark.filterwarnings('ignore:Legacy floor-only calibration:UserWarning')


@pytest.fixture
def inputs(tmp_path):
    cal = tmp_path/'arena.json'
    GroundCalibration([[0,0],[799,0],[799,599],[0,599]],79.9,59.9,800,600).save(cal)
    config=json.loads(Path('config/robot_config.json').read_text())
    config.update(mounting_height_cm=10,marker_offset_forward_cm=2,marker_offset_left_cm=1,
                  body_length_cm=26,body_width_cm=15.2)
    settings=tmp_path/'robot.json'; settings.write_text(json.dumps(config))
    return cal,settings


@pytest.fixture
def session(inputs):
    return CalibrationSession(*inputs)


def observation(pixel):
    x,y=pixel
    return MarkerObservation(0,((x-3,y-3),(x+3,y-3),(x+3,y+3),(x-3,y+3)),
                             (x,y),((x-1,y),(x+1,y)),{})


def window_at(pixel=(320,250),serial=0,jitter=None):
    window=CaptureWindow()
    for i in range(10):
        frame=np.full((20,20,3),0,np.uint8)
        frame[0,0]=[(serial+i)//256,(serial+i)%256,123]
        offset=np.zeros(2) if jitter is None else jitter[i]
        window.add(frame,observation(np.array(pixel)+offset),serial+i*.06,f'UTC-{serial}-{i}')
    return window,serial+.55


def fill_session(session,validation_error=0):
    session.attested=True
    training,validation=training_data()
    for serial,(mode,points) in enumerate([('training',training),('validation',validation)]):
        for i,p in enumerate(points):
            w,now=window_at(p['pixel'],serial*100+i*10)
            ground=list(p['ground_cm'])
            if mode=='validation': ground[0]+=validation_error
            session.capture(mode,ground,w,now)
    return session


def test_raw_detector_center_capture_and_schema(session):
    detector=RobotMarkerDetector(session.config)
    obs=detector.detect(marker_scene())
    assert obs is not None
    w,now=window_at(obs.center_pixels)
    session.attested=True
    session.capture('training',[40,30],w,now)
    points=session.measurements()
    assert points['training'][0]=={'pixel':list(obs.center_pixels),'ground_cm':[40.,30.]}
    assert points['validation']==[]
    meta=points['metadata']
    assert meta['measurement_reference']=='MARKER_GROUND_PROJECTION'
    assert meta['frame_size']==[800,600] and meta['mounting_height_cm']==10
    assert meta['robot_configuration']['marker_offset_forward_cm']==2
    assert meta['capture_quality'][0]['quality']['distinct_frames']==10
    assert not meta['heading_alignment_verified']
    assert not meta['chassis_reference_conversion_supported']


def test_requires_reference_attestation_and_finite_in_bounds_ground(session):
    w,now=window_at()
    with pytest.raises(ValueError,match='Press A'): session.capture('training',[30,20],w,now)
    session.attested=True
    for coords in ([float('nan'),1],[-1,1],[80,30],[1,60]):
        with pytest.raises(ValueError,match='inside the floor'): session.capture('training',coords,w,now)


@pytest.mark.parametrize('missing',[None,'held_pose'])
def test_missing_or_held_last_never_captured(session,missing):
    w,now=window_at()
    obs=RobotTracker().update(marker_scene()) if missing else None
    w.add(np.ones((20,20,3),np.uint8),obs,now+.1,'now')
    session.attested=True
    with pytest.raises(ValueError,match='fresh genuine'): session.capture('training',[40,30],w,now+.1)


def test_duplicate_frames_and_timestamp_replays_rejected():
    w,now=window_at()
    repeated=np.zeros((20,20,3),np.uint8);repeated[0,0]=[0,9,123]
    w.add(repeated,observation((320,250)),now+.1,'now')
    with pytest.raises(ValueError,match='fresh genuine'): w.representative(now+.1)
    with pytest.raises(ValueError,match='strictly increase'): w.add(repeated,observation((320,250)),now+.1,'now')


def test_timestamp_expiry_and_minimum_span():
    w,now=window_at()
    with pytest.raises(ValueError,match='fresh genuine'):w.representative(now+.3)
    short=CaptureWindow(minimum_span=1)
    short.samples=w.samples;short.current=True
    with pytest.raises(ValueError,match='too short'): short.representative(now)
    small=CaptureWindow();small.current=True;small.samples.extend(list(w.samples)[:2])
    with pytest.raises(ValueError,match='Need 8'):small.representative(.07)


def test_median_rejects_single_outlier_and_reports_actual_variation():
    offsets=np.zeros((10,2));offsets[4]=[25,25];offsets[2]=[.1,-.2]
    w,now=window_at(jitter=offsets)
    pixel,quality=w.representative(now)
    assert pixel==pytest.approx([320,250])
    assert quality['outlier_frames']==1 and quality['inlier_frames']==9
    assert quality['maximum_inlier_radius_px']>0
    assert len(quality['frames'])==10
    assert 'not sensor exposure' in quality['timestamp_basis']


@pytest.mark.parametrize('offsets',[np.column_stack((np.arange(10)*2,np.zeros(10))),
                                    np.array([[i*20,0] if i<4 else [0,0] for i in range(10)])])
def test_unstable_capture_rejected(offsets):
    w,now=window_at(jitter=offsets)
    with pytest.raises(ValueError,match='Unstable'): w.representative(now)


def test_capture_clears_window_duplicate_location_and_observation_rejected(session):
    session.attested=True
    w,now=window_at()
    reused=w.samples.copy()
    session.capture('training',[40,30],w,now)
    assert not w.samples
    w.samples=reused;w.current=True
    with pytest.raises(ValueError,match='Observation already used'):session.capture('validation',[45,30],w,now)
    fresh,now=window_at(serial=100)
    with pytest.raises(ValueError,match='Duplicate'):session.capture('validation',[40,30],fresh,now)
    other,now=window_at((400,300),serial=200)
    with pytest.raises(ValueError,match='Duplicate'):session.capture('validation',[40,30],other,now)


def test_undo_across_modes_and_invalidates_fitted_state(session):
    fill_session(session);session.fit()
    session.undo()
    assert len(session.points['training'])==4 and len(session.points['validation'])==2
    assert session.mapping is None and session.report is None
    with pytest.raises(ValueError,match='>=4 training and >=3'): session.fit()
    session.undo();session.undo();session.undo()
    assert len(session.points['training'])==3
    with pytest.raises(ValueError,match='>=4 training'):session.fit()


def test_empty_counts_and_undo(session):
    with pytest.raises(ValueError,match='No captured'):session.undo()
    session.attested=True
    with pytest.raises(ValueError,match='>=4 training'):session.fit()


def test_fit_uses_existing_calibrator_and_validation_errors(session,monkeypatch):
    fill_session(session,validation_error=.6)
    original=MarkerPlaneMapping.fit
    fit=Mock(wraps=original);monkeypatch.setattr(MarkerPlaneMapping,'fit',fit)
    report=session.fit()
    assert fit.call_count==1
    assert report['training_count']==4 and report['validation_count']==3
    assert report['maximum_error_cm']==pytest.approx(.6,abs=1e-5)
    assert report['rms_error_cm']==pytest.approx(.6,abs=1e-5)
    assert report['points'][0]['error_x_cm']==pytest.approx(-.6,abs=1e-5)
    assert report['points'][0]['error_y_cm']==pytest.approx(0,abs=1e-5)
    assert report['tolerance_satisfied']


def test_failed_validation_diagnostics_and_preserved_points(session,tmp_path):
    fill_session(session,validation_error=3)
    before=json.dumps(session.points)
    with pytest.raises(MarkerPlaneValidationError):session.fit()
    assert session.report['maximum_error_cm']==pytest.approx(3,abs=1e-5)
    assert not session.report['tolerance_satisfied'] and session.mapping is None
    assert json.dumps(session.points)==before
    points=session.save_points(tmp_path/'points.json')
    assert json.loads(points.read_text())['metadata']['independent_validation']['points']
    with pytest.raises(ValueError,match='Fit must pass'):session.save_mapping(tmp_path/'map.json')
    assert not (tmp_path/'map.json').exists()


def test_mapping_compatible_and_no_overwrite_and_offsets_after_map(session,tmp_path,monkeypatch):
    fill_session(session);session.fit()
    output=tmp_path/'map.json';session.save_mapping(output)
    before=output.read_bytes()
    with pytest.raises(FileExistsError):session.save_mapping(output)
    assert output.read_bytes()==before
    mapping=MarkerPlaneMapping(json.loads(output.read_text()))
    mapping.validate_context(session.calibration_id,session.config,(800,600))
    tracker=session.preview_tracker
    # Plane mapping gives marker (40,30), heading 0; offsets yield chassis (38,29).
    monkeypatch.setattr(tracker.detector,'detect',lambda f:observation((420,250)))
    pose=tracker.update(marker_scene())
    assert pose.position_cm==pytest.approx({'x':38,'y':29},abs=1e-5)
    assert pose.localization_status=='UNVERIFIED' and not pose.motion_authorized
    with pytest.raises(ValueError,match='height mismatch'):
        mapping.validate_context(session.calibration_id,replace(session.config,mounting_height_cm=11),(800,600))
    with pytest.raises(ValueError,match='identity mismatch'):
        mapping.validate_context('other',session.config,(800,600))


def test_point_snapshots_exclusive_and_original_cli_compatibility(session,tmp_path):
    fill_session(session)
    base=tmp_path/'points.json';first=session.save_points(base);before=first.read_bytes()
    session.fit()
    second=session.save_points(base)
    assert second!=first and first.read_bytes()==before
    assert session.save_points(base)==second
    output=tmp_path/'map.json'
    assert main.main(['calibrate-robot-plane','--calibration',str(session.calibration_path),
                      '--points',str(second),'--mounting-height-cm','10','--confirm-measured',
                      '--output',str(output)])==0
    assert MarkerPlaneMapping(json.loads(output.read_text())).report['maximum_error_cm']<1e-4
    with pytest.raises(FileExistsError):write_new_json(first,{})
    assert first.read_bytes()==before


def test_context_change_and_height_mismatch(inputs,session,tmp_path):
    with pytest.raises(ValueError,match='height differs'):CalibrationSession(*inputs,height_cm=16.6)
    fill_session(session)
    inputs[0].write_text(inputs[0].read_text()+'\n')
    for call in [session.fit,lambda:session.save_points(tmp_path/'points.json')]:
        with pytest.raises(ValueError,match='changed during wizard'):call()


@pytest.mark.parametrize('kwargs',[{'height_cm':None},{'height_cm':0},{'tolerance_cm':0},{'camera':1}])
def test_missing_height_invalid_tolerance_or_source(inputs,kwargs):
    if kwargs=={'height_cm':None}:
        config=json.loads(inputs[1].read_text());config['mounting_height_cm']=None
        inputs[1].write_text(json.dumps(config))
    with pytest.raises(ValueError):CalibrationSession(*inputs,**kwargs)


def test_keyboard_edit_capture_modes_fit_failure_preserve_and_quit(session,tmp_path):
    ui=WizardUI(session,tmp_path/'points.json',tmp_path/'map.json')
    w,now=window_at()
    for key in 'a40':assert ui.key(ord(key),w,now)
    ui.key(9,w,now)
    for key in '30':ui.key(ord(key),w,now)
    ui.key(ord(' '),w,now)
    assert len(session.points['training'])==1
    ui.key(ord('f'),w,now)
    assert '>=4 training' in ui.message and (tmp_path/'points.json').exists()
    ui.key(ord('v'),w,now);assert ui.mode=='validation'
    ui.key(ord('u'),w,now);assert not session.captures
    ui.key(ord('c'),w,now);assert ui.fields==['','']
    ui.key(ord('p'),w,now);assert not ui.preview
    assert 'Save a passing mapping' in ui.message
    assert not ui.key(ord('q'),w,now)


def test_ui_failure_report_paginated_and_preview_no_schema_changes(session,tmp_path,monkeypatch):
    fill_session(session);session.fit();session.save_mapping(tmp_path/'map.json')
    ui=WizardUI(session,tmp_path/'points.json',tmp_path/'map.json');w,now=window_at()
    ui.key(ord('p'),w,now);assert ui.preview
    obs=RobotMarkerDetector(session.config).detect(marker_scene())
    monkeypatch.setattr(session.preview_tracker.detector,'detect',lambda f:observation((420,250)))
    view=ui.render(marker_scene(),obs,w,now)
    assert view.dtype==np.uint8 and view.shape[1]>800 and view.shape[0]>=760
    ui.key(ord(' '),w,now);assert 'read-only preview' in ui.message
    ui.key(ord(']'),w,now);assert ui.report_page==0


def wizard_args(inputs,tmp_path,*extra):
    return main.build_parser().parse_args(['calibrate-robot-interactive','--calibration',str(inputs[0]),
       '--robot-settings',str(inputs[1]),'--width','800','--height','600',
       '--points-output',str(tmp_path/'points.json'),'--output',str(tmp_path/'map.json'),*extra])


def test_cli_dispatch_live_detection_camera_release_and_input_preservation(inputs,tmp_path,monkeypatch):
    import vision.robot_calibration_wizard as module
    before=[p.read_bytes() for p in inputs]
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.return_value=marker_scene()
    monkeypatch.setattr(module,'CameraManager',Mock(return_value=camera))
    for method in ('namedWindow','resizeWindow','imshow','destroyWindow','destroyAllWindows'):
        monkeypatch.setattr(cv2,method,Mock())
    monkeypatch.setattr(cv2,'waitKey',Mock(return_value=ord('q')))
    monkeypatch.setattr(cv2,'getWindowProperty',Mock(return_value=1))
    args=wizard_args(inputs,tmp_path)
    assert main.main(['calibrate-robot-interactive','--calibration',str(inputs[0]),'--robot-settings',str(inputs[1]),
      '--width','800','--height','600','--points-output',str(args.points_output),'--output',str(args.output)])==0
    camera.__exit__.assert_called_once()
    cv2.imshow.assert_called_once()
    assert [p.read_bytes() for p in inputs]==before
    assert not Path(args.output).exists() and not Path(args.points_output).exists()


def test_cli_refuses_existing_or_conflicting_outputs_and_wrong_frame_before_camera(inputs,tmp_path,monkeypatch):
    import vision.robot_calibration_wizard as module
    camera=Mock();monkeypatch.setattr(module,'CameraManager',camera)
    (tmp_path/'points.json').write_text('{}')
    with pytest.raises(ValueError,match='Output exists'):run_wizard(wizard_args(inputs,tmp_path))
    other=tmp_path/'other';other.mkdir()
    with pytest.raises(ValueError,match='distinct'):
        run_wizard(wizard_args(inputs,other,'--output',str(inputs[0])))
    with pytest.raises(ValueError,match='Requested frame dimensions'):
        run_wizard(wizard_args(inputs,other,'--width','1280'))
    camera.assert_not_called()


def test_real_frame_mismatch_and_disconnect_release_camera(inputs,tmp_path,monkeypatch):
    import vision.robot_calibration_wizard as module
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.return_value=np.zeros((720,1280,3),np.uint8)
    monkeypatch.setattr(module,'CameraManager',Mock(return_value=camera))
    for method in ('namedWindow','resizeWindow','destroyWindow'):monkeypatch.setattr(cv2,method,Mock())
    with pytest.raises(ValueError,match='Actual camera frame'):run_wizard(wizard_args(inputs,tmp_path))
    camera.__exit__.assert_called_once()


def test_suggested_locations_require_full_body_clearance_for_any_heading(session):
    assert not session.suggestion_clearance((10,10))
    assert session.suggestion_clearance((25,18))
    assert not session.suggestion_clearance((95,48))  # Fixture arena is only 79.9 cm wide.
    session.config=replace(session.config,body_length_cm=None,body_width_cm=None)
    assert not session.suggestion_clearance((25,18))


def test_complete_keyboard_camera_workflow_fit_save_preview(inputs,tmp_path,monkeypatch):
    """Drive the real controller and existing fitter through a simulated live stream."""
    import vision.robot_calibration_wizard as module
    from itertools import count
    training,validation=training_data()
    script=[]
    for mode,points in [('training',training),('validation',validation)]:
        script.append((points[0]['pixel'],ord('t' if mode=='training' else 'v')))
        for p in points:
            keys=[ord('a'),ord('c')]+[ord(k) for k in str(p['ground_cm'][0])]+[9]+[ord(k) for k in str(p['ground_cm'][1])]
            keys += [-1]*10+[ord(' ')]
            script.extend((p['pixel'],k) for k in keys)
    script.extend((validation[-1]['pixel'],ord(k)) for k in 'fmpq')
    frames=iter(enumerate(script));active={}
    def read():
        index,(pixel,key)=next(frames)
        active.update(pixel=pixel,key=key)
        frame=marker_scene();frame[0,0]=[index//256,index%256,99]
        return frame
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=read
    monkeypatch.setattr(module,'CameraManager',Mock(return_value=camera))
    monkeypatch.setattr(module.RobotMarkerDetector,'detect',lambda self,f:observation(active['pixel']))
    ticks=count(100,.06);monkeypatch.setattr(module.time,'monotonic',lambda:next(ticks))
    for method in ('namedWindow','resizeWindow','imshow','destroyWindow','destroyAllWindows'):
        monkeypatch.setattr(cv2,method,Mock())
    monkeypatch.setattr(cv2,'getWindowProperty',Mock(return_value=1))
    monkeypatch.setattr(cv2,'waitKey',lambda _:active['key'])
    run_wizard(wizard_args(inputs,tmp_path))
    output=json.loads((tmp_path/'map.json').read_text())
    assert len(output['training'])==4 and len(output['validation'])==3
    assert output['mapping_type']=='FIXED_HEIGHT_MARKER_PLANE'
    assert output['validation_report']['maximum_error_cm']<1e-4
    snapshots=list(tmp_path.glob('points*.json'))
    assert snapshots
    assert any(json.loads(p.read_text())['metadata']['independent_validation'] for p in snapshots)
    camera.__exit__.assert_called_once()


def test_failed_ui_fit_preserves_report_without_mapping(session,tmp_path):
    fill_session(session,validation_error=3)
    ui=WizardUI(session,tmp_path/'points.json',tmp_path/'map.json')
    w,now=window_at(serial=500)
    ui.key(ord('f'),w,now)
    assert 'inspect validation rows' in ui.message
    assert session.mapping is None and session.report['points'][0]['euclidean_error_cm']>2
    ui.key(ord('m'),w,now)
    assert not (tmp_path/'map.json').exists()
    snapshots=list(tmp_path.glob('points*.json'))
    assert any(json.loads(p.read_text())['metadata']['independent_validation'] for p in snapshots)


def test_failure_during_camera_read_preserves_captured_points(inputs,tmp_path,monkeypatch):
    import vision.robot_calibration_wizard as module
    session=fill_session(CalibrationSession(*inputs))
    monkeypatch.setattr(module,'CalibrationSession',lambda *a,**k:session)
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=RuntimeError('camera disconnected')
    monkeypatch.setattr(module,'CameraManager',Mock(return_value=camera))
    for method in ('namedWindow','resizeWindow','destroyWindow'):monkeypatch.setattr(cv2,method,Mock())
    with pytest.raises(RuntimeError,match='disconnected'):run_wizard(wizard_args(inputs,tmp_path))
    assert len(json.loads((tmp_path/'points.json').read_text())['training'])==4
    camera.__exit__.assert_called_once()


def test_suggested_holdouts_cover_current_arena_without_relaxing_existing_rule():
    from vision.robot_calibration_wizard import SUGGESTIONS
    validation=np.array(SUGGESTIONS['validation'],np.float32)
    assert cv2.contourArea(cv2.convexHull(validation[:3]))<.01*121.6*68.4
    assert cv2.contourArea(cv2.convexHull(validation))>=.01*121.6*68.4


@pytest.fixture
def interrupted_points(session,tmp_path):
    fill_session(session)
    session.undo();session.undo()  # The real interrupted shape: 4 training, 1 validation.
    path=session.save_points(tmp_path/'points.json')
    data=json.loads(path.read_text())
    data['metadata']['operator_note']='Preserve this original measurement record'
    path.write_text(json.dumps(data))
    return path


def test_resume_restores_four_plus_one_and_next_validation_two(inputs,interrupted_points,tmp_path):
    before=interrupted_points.read_bytes()
    saved=json.loads(before)
    resumed=CalibrationSession(*inputs)
    resumed.resume_points(interrupted_points)
    ui=WizardUI(resumed,interrupted_points,tmp_path/'map.json')
    assert resumed.points=={k:saved[k] for k in ('training','validation')}
    assert resumed.captures==saved['metadata']['capture_quality']
    assert resumed.measurements()==saved
    assert ui.mode=='validation' and 'next validation #2' in ui.message
    assert resumed.attested and resumed.mapping is None and resumed.report is None
    assert resumed.save_points(interrupted_points)==interrupted_points  # No write just for opening.
    assert resumed.resume_backup is None and interrupted_points.read_bytes()==before


def test_resume_capture_two_remaining_holdouts_keeps_original_and_fitter(inputs,interrupted_points,tmp_path):
    original=interrupted_points.read_bytes();saved=json.loads(original)
    resumed=CalibrationSession(*inputs);resumed.resume_points(interrupted_points)
    ui=WizardUI(resumed,interrupted_points,tmp_path/'map.json')
    _,validation=training_data()
    for i,point in enumerate(validation[1:]):
        w,now=window_at(point['pixel'],serial=500+i*100)
        ui.fields=[str(v) for v in point['ground_cm']]
        assert ui.key(ord(' '),w,now)
        assert len(resumed.points['validation'])==2+i
        assert 'Saved:' in ui.message
        assert resumed.last_points_path!=interrupted_points
        assert resumed.resume_backup.read_bytes()==original
        assert interrupted_points.read_bytes()==original
        snapshot=json.loads(resumed.last_points_path.read_text())
        assert snapshot['training']==saved['training']
        assert snapshot['validation'][0]==saved['validation'][0]
        assert snapshot['metadata']['capture_quality'][:5]==saved['metadata']['capture_quality']
        assert snapshot['metadata']['created_at_utc']==saved['metadata']['created_at_utc']
        assert snapshot['metadata']['operator_note']==saved['metadata']['operator_note']
        if i==0:
            with pytest.raises(ValueError,match='>=4 training and >=3'):resumed.fit()
    report=resumed.fit()
    assert report['maximum_error_cm']<1e-4 and report['tolerance_cm']==2
    assert report['training_count']==4 and report['validation_count']==3
    resumed.save_mapping(tmp_path/'map.json')
    assert resumed.preview_tracker is not None  # Original tracker/trust behavior stays intact.
    assert interrupted_points.read_bytes()==original


@pytest.mark.parametrize('field,value',[
    ('frame_size',[1280,720]),('frame_size',[True,600]),('calibration_id','sha256:other'),
    ('mounting_height_cm',11),('mounting_height_cm',True),('measurement_reference','CHASSIS_CENTER'),
    ('attested_measured',False),('heading_alignment_verified',True),
    ('chassis_reference_conversion_supported',True),('capture_quality',[]),('created_at_utc',None)])
def test_resume_metadata_mismatch_rejected_before_camera(inputs,interrupted_points,tmp_path,monkeypatch,field,value):
    import vision.robot_calibration_wizard as module
    data=json.loads(interrupted_points.read_text());data['metadata'][field]=value
    interrupted_points.write_text(json.dumps(data));before=interrupted_points.read_bytes()
    camera=Mock();monkeypatch.setattr(module,'CameraManager',camera)
    with pytest.raises(ValueError,match='Cannot resume'):
        run_wizard(wizard_args(inputs,tmp_path,'--resume'))
    camera.assert_not_called()
    assert interrupted_points.read_bytes()==before
    assert not list(tmp_path.glob('*resume_backup*'))


@pytest.mark.parametrize('damage',[
    'bad_json','missing_metadata','wrong_schema','bad_coordinate','robot_mismatch',
    'invalid_observation','reused_observation','bad_capture_index'])
def test_resume_invalid_schema_or_observations_preserves_file(inputs,interrupted_points,damage):
    data=json.loads(interrupted_points.read_text())
    if damage=='missing_metadata':del data['metadata']
    elif damage=='wrong_schema':data['schema_version']=2
    elif damage=='bad_coordinate':data['training'][0]['pixel'][0]=float('nan')
    elif damage=='robot_mismatch':data['metadata']['robot_configuration']['marker_size_cm']=7
    elif damage=='invalid_observation':data['metadata']['capture_quality'][0]['quality']['frames'][1]['received_at']=-1
    elif damage=='reused_observation':
        first=data['metadata']['capture_quality'][0]
        data['metadata']['capture_quality'][1]['frame_ids']=first['frame_ids']
        data['metadata']['capture_quality'][1]['quality']=first['quality']
    elif damage=='bad_capture_index':data['metadata']['capture_quality'][0]['index']=True
    interrupted_points.write_text('{' if damage=='bad_json' else json.dumps(data))
    before=interrupted_points.read_bytes();resumed=CalibrationSession(*inputs)
    with pytest.raises(ValueError,match='Cannot resume'):resumed.resume_points(interrupted_points)
    assert resumed.points=={'training':[],'validation':[]} and not resumed.captures
    assert interrupted_points.read_bytes()==before


def test_resume_reused_old_observations_and_duplicate_locations_still_rejected(inputs,interrupted_points):
    resumed=CalibrationSession(*inputs);resumed.resume_points(interrupted_points)
    w,now=window_at(training_data()[0][0]['pixel'],serial=0)
    with pytest.raises(ValueError,match='Observation already used'):
        resumed.capture('validation',[45,30],w,now)
    w,now=window_at(training_data()[0][0]['pixel'],serial=1000)
    with pytest.raises(ValueError,match='Duplicate'):
        resumed.capture('validation',[45,30],w,now)
    assert len(resumed.points['training'])==4 and len(resumed.points['validation'])==1


def test_resume_backup_and_snapshot_are_atomic_on_publish_failure(inputs,interrupted_points,tmp_path,monkeypatch):
    import vision.robot_calibration_wizard as module
    resumed=CalibrationSession(*inputs);resumed.resume_points(interrupted_points)
    original=interrupted_points.read_bytes()
    point=training_data()[1][1];w,now=window_at(point['pixel'],serial=500)
    resumed.capture('validation',point['ground_cm'],w,now)
    original_link=module.os.link
    def fail_publish(source,destination):
        if Path(destination).name=='points_001.json':
            assert resumed.resume_backup.read_bytes()==original  # Backup precedes publication.
            raise OSError('simulated interrupted publish')
        return original_link(source,destination)
    monkeypatch.setattr(module.os,'link',fail_publish)
    with pytest.raises(OSError,match='interrupted publish'):resumed.save_points(interrupted_points)
    assert interrupted_points.read_bytes()==original
    assert not (tmp_path/'points_001.json').exists() and not list(tmp_path.glob('.*.tmp'))
    monkeypatch.setattr(module.os,'link',original_link)
    saved=resumed.save_points(interrupted_points)
    assert len(json.loads(saved.read_text())['validation'])==2
    assert resumed.resume_backup.read_bytes()==original


def test_resume_never_overwrites_existing_backup_or_external_edits(inputs,interrupted_points,tmp_path):
    prior=tmp_path/'points_resume_backup.json';prior.write_bytes(b'older backup')
    resumed=CalibrationSession(*inputs);resumed.resume_points(interrupted_points)
    original=interrupted_points.read_bytes()
    point=training_data()[1][1];w,now=window_at(point['pixel'],serial=500)
    resumed.capture('validation',point['ground_cm'],w,now)
    resumed.save_points(interrupted_points)
    assert prior.read_bytes()==b'older backup'
    assert resumed.resume_backup.name=='points_resume_backup_001.json'
    assert resumed.resume_backup.read_bytes()==original
    interrupted_points.write_bytes(original+b'\n')
    with pytest.raises(ValueError,match='changed during wizard'):resumed.save_points(interrupted_points)
    assert interrupted_points.read_bytes()==original+b'\n'


def test_resume_camera_interruption_saves_new_capture_and_releases(inputs,interrupted_points,tmp_path,monkeypatch):
    import vision.robot_calibration_wizard as module
    resumed=CalibrationSession(*inputs)
    original_resume=resumed.resume_points
    def load_and_capture(path):
        original_resume(path)
        point=training_data()[1][1];w,now=window_at(point['pixel'],serial=500)
        resumed.capture('validation',point['ground_cm'],w,now)
    monkeypatch.setattr(resumed,'resume_points',load_and_capture)
    monkeypatch.setattr(module,'CalibrationSession',lambda *a,**k:resumed)
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=RuntimeError('camera disconnected')
    monkeypatch.setattr(module,'CameraManager',Mock(return_value=camera))
    for method in ('namedWindow','resizeWindow','destroyWindow'):monkeypatch.setattr(cv2,method,Mock())
    before=interrupted_points.read_bytes()
    with pytest.raises(RuntimeError,match='disconnected'):
        run_wizard(wizard_args(inputs,tmp_path,'--resume'))
    camera.__exit__.assert_called_once()
    saved=json.loads(resumed.last_points_path.read_text())
    assert len(saved['training'])==4 and len(saved['validation'])==2
    assert interrupted_points.read_bytes()==before and resumed.resume_backup.read_bytes()==before


def test_resume_wrong_camera_resolution_missing_source_and_existing_mapping_refused(inputs,interrupted_points,tmp_path,monkeypatch):
    import vision.robot_calibration_wizard as module
    camera=Mock();monkeypatch.setattr(module,'CameraManager',camera)
    with pytest.raises(ValueError,match='Requested frame dimensions'):
        run_wizard(wizard_args(inputs,tmp_path,'--resume','--width','1280'))
    with pytest.raises(FileNotFoundError):
        run_wizard(wizard_args(inputs,tmp_path,'--resume','--points-output',str(tmp_path/'missing.json')))
    (tmp_path/'map.json').write_text('existing map')
    with pytest.raises(ValueError,match='Output exists'):
        run_wizard(wizard_args(inputs,tmp_path,'--resume'))
    assert (tmp_path/'map.json').read_text()=='existing map'
    camera.assert_not_called()


def test_optional_real_four_plus_one_snapshot_is_loadable_with_historical_settings_without_writing(tmp_path):
    path=Path('data/robot_plane_points.json')
    if not path.exists() or not Path('config/camera_config.json').exists():
        pytest.skip('Private operator calibration not distributed')
    before=path.read_bytes();data=json.loads(before)
    # Current marker9 configuration must not silently resume historical marker6 metadata.
    with pytest.raises(ValueError,match='configuration'):
        CalibrationSession('config/camera_config.json','config/robot_config.json').resume_points(path)
    settings=tmp_path/'historical_robot.json'
    settings.write_text(json.dumps(data['metadata']['robot_configuration']))
    resumed=CalibrationSession('config/camera_config.json',settings)
    resumed.resume_points(path)
    assert resumed.points=={k:data[k] for k in ('training','validation')}
    assert resumed.captures==data['metadata']['capture_quality']
    assert resumed.mapping is None and resumed.preview_tracker is None
    assert path.read_bytes()==before
