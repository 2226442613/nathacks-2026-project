"""Module 04 synthetic detections and geometry; not physical camera validation."""
from dataclasses import replace
import json
import math

import cv2
import numpy as np
import pytest

from vision import (RobotConfig,RobotTracker,RobotMarkerDetector,RobotPose,MarkerPlaneMapping,
                    GroundCalibration,CameraRegistration,BackgroundReference,SpillDetector)
from vision.robot_tracking import generate_marker,MarkerObservation,project_points


def marker_scene(config=None,angle=0):
    marker=generate_marker(config or RobotConfig(),240,40)
    if angle: marker=cv2.rotate(marker,{90:cv2.ROTATE_90_CLOCKWISE,180:cv2.ROTATE_180,270:cv2.ROTATE_90_COUNTERCLOCKWISE}[angle])
    frame=np.full((600,800,3),255,np.uint8)
    frame[140:460,240:560]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
    return frame


@pytest.fixture
def floor(): return GroundCalibration([[0,0],[799,0],[799,599],[0,599]],79.9,59.9,800,600)


def observed(center=(400,300),angle=0):
    theta=math.radians(angle);u=np.array([math.cos(theta),math.sin(theta)]);v=np.array([-u[1],u[0]])
    center=np.asarray(center,float)
    corners=np.asarray([center+a*30*u+b*30*v for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]])
    return MarkerObservation(0,tuple(map(tuple,corners)),tuple(center),tuple(map(tuple,[center-15*u,center+15*u])),{'synthetic':True})


def test_marker_generation_dictionary_id_and_border():
    image=generate_marker(RobotConfig(),1200,200)
    assert image.shape==(1600,1600) and image[:200].min()==255
    assert image[200,200]==0 and image[199,199]==255
    with pytest.raises(ValueError): generate_marker(RobotConfig(),240,1)
    with pytest.raises(ValueError): RobotConfig(marker_id=50)


@pytest.mark.parametrize('angle',[0,90,180,270])
def test_real_aruco_detection_rotations_and_corner_order(angle):
    detector=RobotMarkerDetector();o=detector.detect(marker_scene(angle=angle))
    assert o.marker_id==0 and o.center_pixels==pytest.approx((399.5,299.5),abs=.2)
    p=np.asarray(o.corners_pixels)
    expected=np.array([[279.5,179.5],[519.5,179.5],[519.5,419.5],[279.5,419.5]])
    expected=np.roll(expected,-angle//90,axis=0)
    # Rotation of the canonical first corner moves with the decoded bits.
    assert p==pytest.approx(expected,abs=.6)
    axis=np.asarray(o.axis_pixels)[1]-np.asarray(o.axis_pixels)[0]
    assert math.degrees(math.atan2(axis[1],axis[0]))%360==pytest.approx(angle,abs=.5)


def test_marker_perspective_center_is_not_mean_corners():
    source=marker_scene()
    h=cv2.getPerspectiveTransform(np.float32([[0,0],[799,0],[799,599],[0,599]]),np.float32([[25,50],[750,0],[680,580],[70,540]]))
    frame=cv2.warpPerspective(source,h,(800,600),borderValue=(255,255,255))
    o=RobotMarkerDetector().detect(frame)
    assert o is not None
    expected=project_points([[399.5,299.5]],h)[0]
    assert o.center_pixels==pytest.approx(expected,abs=1.)
    assert np.linalg.norm(np.asarray(o.center_pixels)-np.mean(o.corners_pixels,axis=0))>1


@pytest.mark.parametrize('mode',['missing','unrelated','occluded','low_contrast','duplicate'])
def test_no_false_pose_for_missing_or_ambiguous_marker(mode):
    frame=marker_scene(RobotConfig(marker_id=1)) if mode=='unrelated' else marker_scene()
    if mode=='missing': frame[:]=255
    elif mode=='occluded': frame[175:315,275:410]=255
    elif mode=='low_contrast': frame[:]=220
    elif mode=='duplicate':
        frame=np.full((600,800,3),255,np.uint8)
        m=cv2.cvtColor(generate_marker(RobotConfig(),120,20),cv2.COLOR_GRAY2BGR)
        frame[100:260,100:260]=m;frame[300:460,500:660]=m
    assert RobotMarkerDetector().detect(frame) is None


def test_multiple_ids_select_only_configured_robot():
    frame=np.full((600,800,3),255,np.uint8)
    for i,x in [(0,50),(1,400)]:
        m=cv2.cvtColor(generate_marker(RobotConfig(marker_id=i),180,30),cv2.COLOR_GRAY2BGR)
        frame[150:390,x:x+240]=m
    assert RobotMarkerDetector().detect(frame).marker_id==0


@pytest.mark.parametrize('angle',[0,90,180,270])
def test_ground_heading_axes_not_raw_image_angle(floor,monkeypatch,angle):
    tracker=RobotTracker(floor)
    monkeypatch.setattr(tracker.detector,'detect',lambda frame:observed(angle=angle))
    pose=tracker.update(marker_scene(),timestamp=1000)
    assert pose.position_cm==pytest.approx({'x':40,'y':30})
    assert pose.heading_deg==pytest.approx(angle,abs=1e-6)
    assert pose.localization_status=='UNVERIFIED'
    assert 'parallax' in ' '.join(pose.pose_quality['warnings'])
    assert not pose.suitable_for_navigation and not pose.motion_authorized
    json.dumps(pose.to_dict(),allow_nan=False)


def test_heading_perspective_correction_and_mounting_offsets(monkeypatch):
    floor=GroundCalibration([[100,120],[700,90],[650,550],[180,500]],120,80,800,600)
    center=np.array([50.,30.]);theta=math.radians(90)
    world=np.array([[47,27],[53,27],[53,33],[47,33]])
    pix=np.array([floor.world_to_pixel(*p) for p in world])
    c=floor.world_to_pixel(*center)
    axis=[floor.world_to_pixel(50,28),floor.world_to_pixel(50,32)]
    o=MarkerObservation(0,tuple(map(tuple,pix)),c,tuple(axis),{})
    config=RobotConfig(marker_offset_forward_cm=5,marker_offset_left_cm=2,marker_to_robot_heading_deg=0)
    tracker=RobotTracker(floor,config)
    monkeypatch.setattr(tracker.detector,'detect',lambda f:o)
    pose=tracker.update(marker_scene(),timestamp=10)
    assert pose.heading_deg==pytest.approx(90,abs=1e-5)
    assert pose.position_cm==pytest.approx({'x':52,'y':25},abs=1e-5)
    rotated=RobotTracker(floor,replace(config,marker_to_robot_heading_deg=90))
    monkeypatch.setattr(rotated.detector,'detect',lambda f:o)
    assert rotated.update(marker_scene(),timestamp=10).heading_deg==pytest.approx(180,abs=1e-5)


def test_wraparound_smoothing_and_reacquisition(floor,monkeypatch):
    tracker=RobotTracker(floor,RobotConfig(smoothing_alpha=.5,missed_frame_expiration=1))
    detections=iter([observed(angle=359),observed(angle=1),None,None,observed(angle=90)])
    monkeypatch.setattr(tracker.detector,'detect',lambda f:next(detections))
    poses=[tracker.update(marker_scene(),timestamp=1000+i*.1) for i in range(5)]
    assert poses[1].heading_deg==pytest.approx(0,abs=1e-6)
    assert poses[2].tracking_status=='LOST' and poses[2].position_cm is None and poses[2].heading_deg is None
    assert poses[2].last_known_pose['position_cm'] is not None
    assert poses[3].tracking_status=='EXPIRED'
    assert poses[4].tracking_event=='TRACKING_GAINED' and poses[4].heading_deg==pytest.approx(90)
    assert all(p.robot_id==0 for p in poses)


def test_missing_calibration_preserves_pixel_pose():
    pose=RobotTracker().update(marker_scene(),timestamp=10)
    assert pose.tracking_status=='VISIBLE' and pose.position_cm is None
    assert pose.marker_corners_pixels and pose.localization_status=='INVALID'


def training_data(height=10):
    # Elevated plane: image u=10*x+20, v=10*y-50. Applying floor map
    # directly introduces +2cm X and -5cm Y parallax in this model.
    training=[{'pixel':[10*x+20,10*y-50],'ground_cm':[x,y]} for x,y in [(10,10),(60,10),(60,50),(10,50)]]
    validation=[{'pixel':[10*x+20,10*y-50],'ground_cm':[x,y]} for x,y in [(20,20),(50,20),(30,40)]]
    return training,validation


def mapping(cal_id='cal'):
    training,validation=training_data()
    return MarkerPlaneMapping.fit(training,validation,calibration_id=cal_id,frame_size=(800,600),floor_size_cm=(79.9,59.9),height_cm=10,attested=True)


def test_empirical_elevated_plane_removes_known_parallax(floor,monkeypatch):
    config=RobotConfig(mounting_height_cm=10)
    empirical=RobotTracker(floor,config,calibration_id='cal',marker_plane=mapping())
    fallback=RobotTracker(floor,config)
    for tracker in (empirical,fallback): monkeypatch.setattr(tracker.detector,'detect',lambda f:observed(center=(420,250)))
    good=empirical.update(marker_scene(),timestamp=10)
    approximate=fallback.update(marker_scene(),timestamp=10)
    assert good.position_cm==pytest.approx({'x':40,'y':30})
    assert approximate.position_cm==pytest.approx({'x':42,'y':25})
    assert good.pose_quality['mapping_method']=='EMPIRICAL_FIXED_HEIGHT_PLANE'
    assert good.localization_status=='UNVERIFIED' and not good.actionable
    reloaded=MarkerPlaneMapping(mapping().to_dict())
    assert reloaded.map([[420,250]])[0]==pytest.approx([40,30])


@pytest.mark.parametrize('damage',['height','identity','nonfinite','collinear','validation','attestation'])
def test_empirical_map_rejects_invalid_context_or_measurements(floor,damage):
    plane=mapping()
    if damage in ('height','identity'):
        tracker=RobotTracker(floor,RobotConfig(mounting_height_cm=11 if damage=='height' else 10),calibration_id='other' if damage=='identity' else 'cal',marker_plane=plane)
        assert tracker.update(marker_scene(),timestamp=10).position_cm is None
    else:
        training,validation=training_data()
        if damage=='nonfinite': training[0]['pixel'][0]=float('nan')
        if damage=='collinear': training=[{'pixel':[100+i*10,100],'ground_cm':[10+i,10]} for i in range(4)]
        if damage=='validation': validation[0]['ground_cm'][0]+=10
        with pytest.raises(ValueError):
            MarkerPlaneMapping.fit(training,validation,calibration_id='cal',frame_size=(800,600),floor_size_cm=(79.9,59.9),height_cm=10,attested=damage!='attestation')


def test_camera_alignment_once_and_large_shift_fallback(floor,monkeypatch):
    tracker=RobotTracker(floor)
    monkeypatch.setattr(tracker.detector,'detect',lambda f:observed(center=(401,300)))
    small=CameraRegistration.from_reference_to_current([[1,0,1],[0,1,0]],(800,600))
    pose=tracker.update(marker_scene(),timestamp=10,registration=small)
    assert pose.position_cm==pytest.approx({'x':40,'y':30})
    large=CameraRegistration.from_reference_to_current([[1,0,18],[0,1,0]],(800,600))
    pose=tracker.update(marker_scene(),timestamp=11,registration=large)
    assert pose.position_cm==pytest.approx({'x':40.1,'y':30})
    assert pose.camera_registration['fallback_active'] and pose.localization_status=='UNVERIFIED'


def test_dynamic_body_exclusion_clears_immediately_and_expires(floor):
    tracker=RobotTracker(floor,RobotConfig(exclusion_padding_pixels=25,body_length_cm=30,body_width_cm=20))
    scene=marker_scene();pose=tracker.update(scene,timestamp=10)
    mask=tracker.exclusion_mask(pose,(800,600),timestamp=10)
    assert mask[300,270] and mask[300,400] and not mask[20,20]
    assert not tracker.exclusion_mask(pose,(800,600),timestamp=11).any()
    missing=tracker.update(np.full_like(scene,255),timestamp=10.1)
    assert not tracker.exclusion_mask(missing,(800,600)).any()
    detector=SpillDetector(BackgroundReference(np.full_like(scene,255),confirmed_clean=True))
    assert detector.detect(scene).detections
    assert not detector.detect(scene,exclusion_mask=mask).detections


@pytest.mark.parametrize('kwargs',[{'marker_size_cm':0},{'mounting_height_cm':-1},{'smoothing_alpha':0},
    {'body_length_cm':10},{'marker_offset_forward_cm':float('nan')},{'exclusion_padding_pixels':0}])
def test_robot_config_validation(kwargs):
    with pytest.raises(ValueError): RobotConfig(**kwargs)


def test_serialized_observation_freshness_is_not_assumed(floor):
    from vision import robot_pose_is_current
    pose=RobotTracker(floor).update(marker_scene(),timestamp=100)
    message=json.loads(json.dumps(pose.to_dict()))
    assert robot_pose_is_current(message,timestamp=100.2)
    assert not robot_pose_is_current(message,timestamp=100.6)
    assert not robot_pose_is_current(message,timestamp=99.9)
    assert not robot_pose_is_current(dict(message,tracking_status='LOST'),timestamp=100.1)


def test_marker_mapping_uses_same_module03_floor_axes(monkeypatch):
    from test_localization import dual, observation, identity
    # Use the fixture factory without pytest wrapping to keep one calibration model.
    cal=dual.__wrapped__()
    from vision import SurfaceLocalizer
    pixel=cal.floor_to_pixel(50,30)
    target,mask=observation(pixel,radius=1)
    localized=SurfaceLocalizer(cal).localize(target,mask,frame_size=(800,600),registration=identity())
    tracker=RobotTracker(cal)
    o=observed(center=localized.pixel_anchor)
    monkeypatch.setattr(tracker.detector,'detect',lambda frame:o)
    robot=tracker.update(marker_scene(),timestamp=10,registration=identity())
    assert robot.position_cm==pytest.approx(localized.world_coordinates_cm)
    assert robot.coordinate_frame['shared_floor_boundary']=='top'


def test_alternate_dictionary_and_id():
    config=RobotConfig(dictionary='DICT_5X5_100',marker_id=17)
    scene=marker_scene(config)
    assert RobotMarkerDetector(config).detect(scene).marker_id==17
    assert RobotMarkerDetector().detect(scene) is None


def test_low_contrast_marker_is_not_a_fabricated_pose():
    scene=marker_scene()
    scene=np.where(scene>127,222,220).astype(np.uint8)
    observation=RobotMarkerDetector().detect(scene)
    if observation is not None:
        # If OpenCV can decode it, retain the decoded ID, never a different robot.
        assert observation.marker_id==0
    else:
        assert RobotTracker().update(scene,timestamp=10).position_cm is None


def test_default_exclusion_clock_does_not_reuse_old_last_update(floor):
    pose=RobotTracker(floor).update(marker_scene(),timestamp=10)
    assert not RobotTracker(floor).exclusion_mask(pose,(800,600)).any()


def test_pose_schema_required_fields_and_loss_invariants(floor):
    from pathlib import Path
    tracker=RobotTracker(floor)
    schema=json.loads(Path('config/robot_pose_v1.schema.json').read_text())
    for frame,stamp in ((marker_scene(),10),(np.full((600,800,3),255,np.uint8),10.1)):
        pose=tracker.update(frame,timestamp=stamp)
        message=pose.to_dict()
        assert all(k in message for k in schema['required'])
        if message['tracking_status']!='VISIBLE':
            assert message['position_cm'] is None and message['heading_deg'] is None
            assert message['marker_corners_pixels'] is None
        assert message['localization_status'] in schema['properties']['localization_status']['enum']


@pytest.mark.parametrize('timestamp',[float('nan'),-1,True])
def test_invalid_robot_timestamps_rejected(floor,timestamp):
    with pytest.raises(ValueError): RobotTracker(floor).update(marker_scene(),timestamp=timestamp)


def test_robot_time_cannot_run_backward(floor):
    tracker=RobotTracker(floor);tracker.update(marker_scene(),timestamp=10)
    with pytest.raises(ValueError): tracker.update(marker_scene(),timestamp=9)


def test_reusable_marker_observation_has_utc_timestamp():
    from datetime import datetime
    observation=RobotMarkerDetector().detect(marker_scene())
    data=observation.to_dict()
    assert datetime.fromisoformat(data['timestamp'].replace('Z','+00:00')).tzinfo is not None
    assert len(data['corners_pixels'])==4
    json.dumps(data,allow_nan=False)
