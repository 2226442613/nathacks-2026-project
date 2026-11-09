"""Synthetic Module 03 mathematics and durable event contracts, no physical accuracy claims."""
from dataclasses import replace
import json

import cv2
import numpy as np
import pytest

from vision import (GroundCalibration,WallCalibration,DualPlaneCalibration,SpillDetection,SpillCategory,
                    SurfaceLocalizer,LocalizationConfig,CameraRegistration,TargetEventStore)
from vision.localization_runtime import LocalizationRegistration, image_identity
from vision.localization_validation import evaluate_measurements
from vision import PhysicalValidationPoint,BackgroundReference,SpillDetector,SpillTracker,DetectorConfig


def project(x,y,z):
    return ((2*x+.3*y+.1*z+100)/(1+.002*y+.001*z),(.1*x+2*y-2*z+300)/(1+.002*y+.001*z))


@pytest.fixture
def dual():
    floor = GroundCalibration([project(0,0,0),project(120,0,0),project(120,80,0),project(0,80,0)],120,80,800,600)
    wall = WallCalibration([project(0,0,60),project(120,0,60),project(120,0,0),project(0,0,0)],120,60,800,600)
    return DualPlaneCalibration(floor,wall,'top')


def observation(center=(200,350),radius=3,mask=None,target_id=1):
    if mask is None:
        mask = np.zeros((600,800),np.uint8)
        cv2.circle(mask,tuple(round(v) for v in center),radius,255,-1)
    ys,xs = np.nonzero(mask)
    x,y,w,h = int(xs.min()),int(ys.min()),int(np.ptp(xs)+1),int(np.ptp(ys)+1)
    contour = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0][0][:,0]
    target = SpillDetection(SpillCategory.RED_SPILL,(0.,0.),(x,y,w,h),tuple(tuple(int(v) for v in p) for p in contour),
        int(len(xs)),.9,'synthetic exact mask',target_id=target_id,confirmed=True,first_seen=1000.,last_seen=1002.)
    return target,mask


def identity(reference_id=None):
    return CameraRegistration.from_reference_to_current(np.eye(3),(800,600),reference_id=reference_id)


@pytest.mark.parametrize('surface,xy', [('FLOOR',(3,5)),('FLOOR',(115,75)),('WALL',(30,3)),('WALL',(115,55)),
                                       ('FLOOR',(47,33)),('WALL',(47,33))])
def test_floor_wall_perspective_points_and_axes(dual,surface,xy):
    pixel = dual.floor_to_pixel(*xy) if surface=='FLOOR' else dual.wall_to_pixel(*xy)
    target,mask = observation(pixel,radius=1)
    record = SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=identity())
    assert record.surface == surface and record.localization_status=='UNVERIFIED'
    assert record.world_coordinates_cm['x'] == pytest.approx(xy[0],abs=.6)
    assert record.world_coordinates_cm['y' if surface=='FLOOR' else 'h'] == pytest.approx(xy[1],abs=.6)
    assert record.component_pixel_count == target.area_pixels
    assert record.surface_confidence > .85 and not record.actionable


def test_boundary_overlap_and_shared_coordinates_are_not_forced(dual):
    pixel = dual.wall_to_pixel(50,0)
    assert dual.pixel_to_floor(*pixel) == pytest.approx((50,0),abs=1e-4)
    assert dual.pixel_to_wall(*pixel) == pytest.approx((50,0),abs=1e-4)
    target,mask = observation(pixel,radius=7)
    record = SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=identity())
    assert record.surface=='BOUNDARY' and record.world_coordinates_cm is None
    assert record.floor_pixel_count>0 and record.wall_pixel_count>0
    assert record.target_message()['detection_state']=='DETECTED'


def test_outside_polygons_unknown_without_coordinates(dual):
    target,mask = observation((700,500))
    record = SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=identity())
    assert record.surface=='UNKNOWN' and record.world_coordinates_cm is None
    assert record.outside_overlap_ratio==1


def test_irregular_mask_anchor_is_actual_member_not_box_or_hole_center(dual):
    mask = np.zeros((600,800),np.uint8)
    center = tuple(round(v) for v in dual.floor_to_pixel(50,40))
    cv2.circle(mask,center,15,255,-1)
    cv2.circle(mask,center,8,0,-1)
    target,mask = observation(mask=mask)
    record = SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=identity())
    x,y = (round(v) for v in record.pixel_anchor)
    assert mask[y,x] and (x,y)!=center
    assert record.world_coordinates_cm == dict(zip(('x','y'),dual.pixel_to_floor(*record.calibration_pixel_anchor)))


def test_wall_reversed_x_and_non_top_native_floor_convention():
    floor = GroundCalibration([[200,200],[400,200],[400,400],[200,400]],120,80,600,600)
    wall = WallCalibration([[400,500],[200,500],[200,400],[400,400]],120,60,600,600)
    cal = DualPlaneCalibration(floor,wall,'bottom',True)
    pixel = cal.wall_to_pixel(30,20)
    target,mask = observation(pixel)
    mask = mask[:600,:600]
    record = SurfaceLocalizer(cal).localize(target,mask,frame_size=(600,600),
        registration=CameraRegistration.from_reference_to_current(np.eye(3),(600,600)))
    assert record.surface=='WALL'
    assert record.world_coordinates_cm == pytest.approx({'x':30,'h':20},abs=.6)
    assert record.coordinate_frame['wall_x_reversed'] is True
    assert record.coordinate_frame['shared_floor_boundary']=='bottom'


@pytest.mark.parametrize('damage',['missing','singular','nonfinite','dimensions','empty','origin','bad_registration'])
def test_invalid_inputs_yield_diagnostics_not_crashes(dual,damage):
    target,mask = observation(dual.floor_to_pixel(50,40))
    size,origin,registration = (800,600),(0,0),identity()
    cal = dual
    if damage=='missing': cal=None
    if damage=='singular': dual.floor.homography[:]=0
    if damage=='nonfinite': dual.wall.homography[0,0]=np.nan
    if damage=='dimensions': size=(640,480)
    if damage=='empty': mask[:]=0
    if damage=='origin': origin=(-1,0)
    if damage=='bad_registration': registration.current_to_calibration=np.zeros((3,3))
    record = SurfaceLocalizer(cal).localize(target,mask,frame_size=size,origin=origin,registration=registration)
    assert record.localization_status=='INVALID' and record.world_coordinates_cm is None
    assert record.reason
    json.dumps(record.to_dict(),allow_nan=False)


def test_accepted_pixel_warning_retained_not_cm_verification():
    floor = GroundCalibration([[200,200],[400,200],[400,400],[200,400]],108.5,43,600,600)
    cal = DualPlaneCalibration.from_floor_and_wall_top(floor,[[196,96],[400,104]],108.5,29,'top')
    cal.accept_quality_warnings()
    target,mask = observation(cal.floor_to_pixel(50,20))
    mask=mask[:600,:600]
    with pytest.warns(UserWarning,match='Calibration quality warning'):
        model = SurfaceLocalizer(cal)
    record = model.localize(target,mask,frame_size=(600,600),
        registration=CameraRegistration.from_reference_to_current(np.eye(3),(600,600)))
    assert record.world_coordinates_cm is not None and record.localization_status=='UNVERIFIED'
    assert record.warnings and record.calibration_quality['accepted_warnings']


@pytest.mark.parametrize('surface', ['FLOOR','WALL'])
def test_small_shift_inverse_direction_no_double_application_and_stable_cm(dual,surface):
    pixel = dual.floor_to_pixel(50,30) if surface=='FLOOR' else dual.wall_to_pixel(50,30)
    target,mask = observation(pixel)
    model = SurfaceLocalizer(dual)
    original = model.localize(target,mask,frame_size=(800,600),registration=identity())
    forward = np.float64([[1,0,1],[0,1,1],[0,0,1]])
    shifted_mask = cv2.warpPerspective(mask,forward,(800,600),flags=cv2.INTER_NEAREST)
    shifted_target,shifted_mask = observation(mask=shifted_mask)
    reg = CameraRegistration.from_reference_to_current(forward,(800,600))
    record = model.localize(shifted_target,shifted_mask,frame_size=(800,600),registration=reg)
    assert reg.compensated and reg.status=='MATCHED'
    assert record.pixel_anchor == pytest.approx(np.asarray(original.pixel_anchor)+1)
    assert record.calibration_pixel_anchor == pytest.approx(original.pixel_anchor)
    assert record.world_coordinates_cm == pytest.approx(original.world_coordinates_cm,abs=1e-9)
    assert record.localization_status=='UNVERIFIED'


@pytest.mark.parametrize('matrix', [np.float64([[1,0,.2],[0,1,.3],[1e-7,0,1]]),np.eye(3)*2])
def test_bounded_projective_registration(matrix):
    reg = CameraRegistration.from_reference_to_current(matrix,(800,600))
    assert reg.status=='MATCHED' and np.isfinite(reg.current_to_calibration).all()


@pytest.mark.parametrize('matrix',[np.zeros((3,3)),np.full((3,3),np.nan),np.eye(2),np.float64([[1,0,0],[0,1,0],[.01,0,-1]])])
def test_bad_registration_is_not_a_valid_transform(matrix):
    assert CameraRegistration.from_reference_to_current(matrix,(800,600)).current_to_calibration is None


def test_large_motion_and_override_publish_only_unverified_estimates(dual):
    target,mask = observation(dual.floor_to_pixel(50,30))
    reg = CameraRegistration.from_reference_to_current([[1,0,20],[0,1,0]],(800,600),override_active=True)
    record = SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=reg)
    assert reg.status=='MOVED' and record.world_coordinates_cm is not None
    assert record.surface=='FLOOR' and record.geometry_status=='UNVERIFIED'
    assert record.camera_registration['fallback_active']
    assert record.localization_status!='VERIFIED'
    assert not record.target_message()['actionable']


def test_detection_registration_exposed_and_reused_without_extra_feature_fit(dual,monkeypatch):
    reference = np.random.default_rng(7).integers(40,180,(600,800,3),dtype=np.uint8)
    forward = np.float32([[1,0,1],[0,1,0]])
    frame = cv2.warpAffine(reference,forward,(800,600))
    center = tuple(round(v) for v in np.asarray(dual.floor_to_pixel(50,30))+[1,0])
    cv2.circle(frame,center,8,(10,10,220),-1)
    detection = SpillDetector(BackgroundReference(reference,confirmed_clean=True),calibration=dual).detect(frame)
    assert detection.reference_to_current.shape==(3,3)
    bridge = LocalizationRegistration(reference,reference,declared=True,config=DetectorConfig())
    monkeypatch.setattr(bridge.guard,'check',lambda *args:pytest.fail('Should reuse detector registration'))
    reg = bridge.for_frame(frame,detection)
    targets = SpillTracker().update(detection.detections)
    targets = [replace(t,confirmed=True) for t in targets]
    records = SurfaceLocalizer(dual).localize_detections(detection,targets,frame_size=(800,600),registration=reg)
    assert records and records[0].surface=='FLOOR'
    assert records[0].world_coordinates_cm == pytest.approx({'x':50,'y':30},abs=1.)


def test_baseline_re_registration_does_not_rebind_calibration_view(dual):
    image = np.random.default_rng(8).integers(40,180,(600,800,3),dtype=np.uint8)
    shifted = cv2.warpAffine(image,np.float32([[1,0,20],[0,1,0]]),(800,600))
    result = SpillDetector(BackgroundReference(shifted,confirmed_clean=True)).detect(shifted)
    bridge = LocalizationRegistration(image,image,declared=True,config=DetectorConfig())
    reg = bridge.for_frame(shifted,result,baseline_changed=True)
    assert reg.status=='MOVED' and reg.current_to_calibration is None
    unknown = LocalizationRegistration(shifted,shifted,baseline_unverified=True,config=DetectorConfig())
    assert unknown.for_frame(shifted,result).current_to_calibration is None


@pytest.mark.parametrize('surface,keys',[('FLOOR',{'x','y'}),('WALL',{'x','h'})])
def test_versioned_robot_message_units_schema_and_safety(dual,surface,keys):
    point=dual.floor_to_pixel(50,30) if surface=='FLOOR' else dual.wall_to_pixel(50,30)
    target,mask = observation(point)
    record = SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=identity())
    message = record.target_message()
    assert message['schema_version']==1 and message['message_type']=='TARGET_DETECTION'
    assert set(message['position_cm'])==keys and message['units']=='cm'
    assert message['detection_state']=='LOCALIZED_ESTIMATE' and not message['actionable']
    assert not message['motion_authorized'] and 'mask' not in message
    assert message['calibration_version']==2 and message['calibration_id'].startswith('sha256:')
    assert message['timestamp']==record.timestamp
    assert json.loads(json.dumps(message,allow_nan=False))==message


def measured_report(dual):
    model = SurfaceLocalizer(dual)
    points=[]
    for surface,xy in [('FLOOR',(10,5)),('FLOOR',(110,5)),('FLOOR',(60,70)),
                       ('WALL',(10,5)),('WALL',(110,5)),('WALL',(60,55))]:
        pixel = dual.floor_to_pixel(*xy) if surface=='FLOOR' else dual.wall_to_pixel(*xy)
        points.append(PhysicalValidationPoint(surface,pixel,xy))
    return evaluate_measurements(dual,points,calibration_id=model.calibration_id,reference_id='sha256:measured-view',attested_measured=True)


def test_verification_needs_measured_evidence_same_reference_no_override_or_motion(dual):
    target,mask = observation(dual.floor_to_pixel(60,30))
    report = measured_report(dual)
    assert report['physical_validation_passed']
    model = SurfaceLocalizer(dual,physical_evidence=report)
    verified = model.localize(target,mask,frame_size=(800,600),registration=identity('sha256:measured-view'))
    assert verified.localization_status=='VERIFIED'
    assert verified.target_message()['detection_state']=='LOCALIZED_VERIFIED'
    assert not verified.target_message()['actionable']
    for reg in [identity('sha256:other'),replace(identity('sha256:measured-view'),compensated=True),
                replace(identity('sha256:measured-view'),override_active=True)]:
        assert model.localize(target,mask,frame_size=(800,600),registration=reg).localization_status=='UNVERIFIED'
    bad = SurfaceLocalizer(dual,physical_evidence={'physical_validation_passed':True})
    assert bad.localize(target,mask,frame_size=(800,600),registration=identity()).localization_status=='UNVERIFIED'
    edge_target,edge_mask = observation(dual.floor_to_pixel(115,70),radius=1)
    assert model.localize(edge_target,edge_mask,frame_size=(800,600),registration=identity('sha256:measured-view')).localization_status=='UNVERIFIED'


def test_measurement_errors_attestation_and_non_collinear_coverage(dual):
    points = [PhysicalValidationPoint('FLOOR',dual.floor_to_pixel(x,y),(x+3,y)) for x,y in [(10,5),(50,20),(90,35)]]
    report = evaluate_measurements(dual,points,calibration_id='cal',reference_id='ref',attested_measured=True)
    assert not report['physical_validation_passed'] and report['max_error_cm']==pytest.approx(3)
    assert report['points'][0]['absolute_axis_error_cm'][0]==pytest.approx(3)
    assert report['coverage']['FLOOR']['physical_hull_area_fraction']<.05


def test_persistent_multiple_ids_updates_inactive_and_timestamp_preservation(dual,tmp_path):
    records=[]
    for index,xy in enumerate([(40,30),(70,35)],1):
        target,mask=observation(dual.floor_to_pixel(*xy),target_id=index)
        records.append(SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=identity(),timestamp='2026-11-07T23:00:00Z'))
    path=tmp_path/'events.jsonl'
    store=TargetEventStore(path,session_id='run')
    store.update(records,timestamp='2026-11-07T23:00:01Z')
    updated=replace(records[0],world_coordinates_cm={'x':42.,'y':30.},timestamp='2026-11-07T23:00:02Z')
    events=store.update([updated],timestamp='2026-11-07T23:00:03Z')
    states=TargetEventStore.read(path)
    assert len(states)==2 and states['run:0:1']['active'] and not states['run:0:2']['active']
    assert states['run:0:1']['localization']['world_coordinates_cm']['x']==42
    assert states['run:0:2']['target_message']['position_cm'] is None
    assert states['run:0:2']['target_message']['last_known_position_cm']==records[1].world_coordinates_cm
    assert states['run:0:2']['last_seen']=='2026-11-07T23:00:00Z'
    assert states['run:0:1']['first_seen']==records[0].first_observed_timestamp
    store.update([updated],epoch=1)
    assert 'run:1:1' in store.states and not store.states['run:0:1']['active']
    later=TargetEventStore(path,session_id='other')
    assert not any(item['active'] for item in later.states.values())
    later.update([updated])
    assert later.states['other:0:1']['active'] and 'run:1:1' in later.states


@pytest.mark.parametrize('settings',[{'ownership_fraction':0},{'boundary_margin_pixels':-1},
                                    {'world_boundary_tolerance_cm':float('nan')},{'max_outside_fraction':2}])
def test_localization_settings_validation(settings):
    with pytest.raises(ValueError): LocalizationConfig(**settings)


@pytest.mark.parametrize('damage',['nan_transform','nan_origin','nan_time'])
def test_invalid_anchor_context_remains_finite_json(dual,damage):
    target,mask=observation(dual.floor_to_pixel(50,30))
    reg=identity();origin=(0,0)
    if damage=='nan_transform': reg.current_to_calibration=np.full((3,3),np.nan)
    elif damage=='nan_origin': origin=(np.nan,0)
    else: target=replace(target,last_seen=np.nan)
    record=SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=reg,origin=origin)
    assert record.localization_status=='INVALID' and record.world_coordinates_cm is None
    json.dumps(record.to_dict(),allow_nan=False)


def test_numpy_serialization_and_message_coordinate_invariants(dual):
    target,mask=observation(dual.floor_to_pixel(50,30))
    record=SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=identity())
    native=replace(record,target_id=np.int64(9),world_coordinates_cm={'x':np.float32(50),'y':np.float64(30)})
    assert json.loads(json.dumps(native.target_message(),allow_nan=False))['target_id']==9
    with pytest.raises(ValueError,match='position keys'):
        replace(record,world_coordinates_cm={'x':50.,'h':30.}).target_message()
    assert replace(record,surface='BOUNDARY').target_message()['position_cm'] is None
    assert replace(record,surface='UNKNOWN').target_message()['position_cm'] is None
    with pytest.raises(ValueError,match='Non-finite'):
        replace(record,world_coordinates_cm={'x':np.nan,'y':30.}).target_message()


def test_current_messages_expire_after_process_stops_observing(dual,tmp_path):
    target,mask=observation(dual.floor_to_pixel(50,30))
    record=SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=identity(),timestamp='2026-11-07T23:00:00Z')
    store=TargetEventStore(tmp_path/'fresh.jsonl',session_id='fresh')
    store.update([record])
    assert len(store.current_messages(now='2026-11-07T23:00:01Z'))==1
    before=(tmp_path/'fresh.jsonl').read_bytes()
    assert len(TargetEventStore.read_current_messages(tmp_path/'fresh.jsonl',now='2026-11-07T23:00:01Z'))==1
    assert (tmp_path/'fresh.jsonl').read_bytes()==before
    assert store.current_messages(now='2026-11-07T23:00:03Z')==[]
    assert store.current_messages(now='2026-11-07T22:59:59Z')==[]
    assert store.states['fresh:0:1']['localization']['world_coordinates_cm'] is not None
