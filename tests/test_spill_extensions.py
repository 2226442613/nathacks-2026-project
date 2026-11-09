"""Incremental Module 02 coverage: reference aggregation, framing, lighting and ownership-free outputs."""
from dataclasses import replace
import json
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import (BackgroundReference, DetectionError, DetectorConfig, DualPlaneCalibration,
    GroundCalibration, WallCalibration, SpillDetector, SpillCategory, SpillTracker, TrackerConfig,
    exclusion_mask_from_polygons, load_exclusion_polygons)


@pytest.fixture
def clean():
    return np.full((320,400,3),200,np.uint8)


def add_blob(frame,center,color=(20,20,220),radius=18):
    image = frame.copy()
    cv2.circle(image,center,radius,color,-1)
    return image


def calibration():
    floor = GroundCalibration([[20,140],[300,140],[300,300],[20,300]],108.5,43,400,320)
    wall = WallCalibration([[20,20],[300,20],[300,140],[20,140]],108.5,29,400,320)
    return DualPlaneCalibration(floor,wall,'top')


@pytest.mark.parametrize('surface,center', [('FLOOR',(100,220)),('WALL',(100,80))])
@pytest.mark.parametrize('color,category', [((20,20,220),SpillCategory.RED_SPILL),((20,180,180),SpillCategory.NON_RED_SPILL)])
def test_red_and_nonred_on_each_plane(clean,surface,center,color,category):
    detector = SpillDetector(BackgroundReference(clean,confirmed_clean=True),calibration=calibration())
    result = detector.detect(add_blob(clean,center,color))
    assert len(result.detections) == 1
    target = result.detections[0]
    assert target.category == category
    assert target.pixel_center == pytest.approx(center,abs=1)
    assert all(0 <= x < 400 and 0 <= y < 320 for x,y in target.contour)
    assert np.count_nonzero(result.mask_for(target)) == target.area_pixels
    assert np.isfinite(target.to_dict()['pixel_center']).all()
    assert 'surface' not in target.to_dict()  # Ownership and physical localization await Module 03.


def test_three_simultaneous_floor_wall_targets_and_clean_confirmation(clean):
    detector = SpillDetector(BackgroundReference(clean,confirmed_clean=True),calibration=calibration())
    tracker = SpillTracker()
    for _ in range(5):
        assert tracker.update(detector.detect(clean).detections) == []
    frame = add_blob(clean,(100,220))
    frame = add_blob(frame,(90,80),(20,180,180))
    frame = add_blob(frame,(230,80))
    for _ in range(3):
        observed = tracker.update(detector.detect(frame).detections)
    assert len(observed) == 3 and len({item.target_id for item in observed}) == 3
    assert all(item.confirmed for item in observed)
    assert [item.category for item in observed].count(SpillCategory.RED_SPILL) == 2
    assert tracker.update(detector.detect(clean).detections) == []


def test_unchanged_red_reference_object_is_ignored(clean):
    reference = add_blob(clean,(100,200))
    detector = SpillDetector(BackgroundReference(reference,confirmed_clean=True))
    assert detector.detect(reference).detections == []
    result = detector.detect(add_blob(reference,(260,200),(20,180,180)))
    assert len(result.detections) == 1
    assert result.detections[0].category == SpillCategory.NON_RED_SPILL


@pytest.mark.parametrize('brightness',[-20,15,25])
def test_selected_global_lighting_changes_are_suppressed(clean,brightness):
    detector = SpillDetector(BackgroundReference(clean,confirmed_clean=True))
    changed = np.clip(clean.astype(np.int16)+brightness,0,255).astype(np.uint8)
    result = detector.detect(changed)
    assert result.detections == []
    assert result.illumination_offset_bgr == (brightness,)*3
    assert result.foreground_mask.max() == 0
    target = detector.detect(add_blob(changed,(100,200)))
    assert target.detections[0].category == SpillCategory.RED_SPILL


def test_selected_minor_shadow_and_compression_noise_do_not_confirm(clean):
    rng = np.random.default_rng(4)
    frame = clean.astype(np.int16)
    frame[160:240,70:140] -= 12
    frame += rng.integers(-4,5,frame.shape,dtype=np.int16)
    frame = np.clip(frame,0,255).astype(np.uint8)
    detector = SpillDetector(BackgroundReference(clean,confirmed_clean=True))
    tracker = SpillTracker()
    for _ in range(4):
        assert tracker.update(detector.detect(frame).detections) == []


@pytest.mark.parametrize('method', ['mean','median'])
def test_aggregated_reference_noise_resolution_and_metadata(clean,tmp_path,method):
    rng = np.random.default_rng(9)
    frames = [np.clip(clean.astype(np.int16)+rng.integers(-12,13,clean.shape),0,255).astype(np.uint8) for _ in range(7)]
    reference = BackgroundReference.from_frames(frames,method=method,confirmed_clean=True)
    assert np.std(reference.image.astype(float)-clean) < np.std(frames[0].astype(float)-clean)
    assert reference.resolution == (400,320)
    assert reference.metadata['frame_count'] == 7
    path = tmp_path/'reference.png'
    reference.save(path)
    assert Path_metadata(path)['frame_width'] == 400
    loaded = BackgroundReference.load(path,confirmed_clean=True)
    assert np.array_equal(loaded.image,reference.image)
    assert loaded.metadata == reference.metadata


def Path_metadata(path):
    return json.loads(path.with_name(path.name+'.json').read_text())


def test_reference_median_suppresses_single_frame_outlier(clean):
    outlier = np.full_like(clean,255)
    reference = BackgroundReference.from_frames([clean,outlier,clean],confirmed_clean=True)
    assert np.array_equal(reference.image,clean)


def test_invalid_aggregation_and_metadata_are_rejected(clean,tmp_path):
    for frames,kwargs in [([],{}),([clean,clean[:200]],{}),([clean]*32,{}),([clean],{'method':'bad'})]:
        with pytest.raises(DetectionError):
            BackgroundReference.from_frames(frames,confirmed_clean=True,**kwargs)
    with pytest.raises(DetectionError,match='Confirm'):
        BackgroundReference.from_frames([clean])
    path = tmp_path/'reference.png'
    BackgroundReference(clean,confirmed_clean=True).save(path)
    metadata_path = path.with_name(path.name+'.json')
    data = Path_metadata(path)
    data['frame_width'] = 800
    metadata_path.write_text(json.dumps(data))
    with pytest.raises(DetectionError,match='resolution'):
        BackgroundReference.load(path,confirmed_clean=True)


@pytest.fixture
def textured():
    # Broad distributed immutable background texture supports a camera-movement diagnostic.
    return np.random.default_rng(13).integers(40,180,(360,480,3),dtype=np.uint8)


def test_unchanged_textured_reference_and_local_target_keep_geometry_matched(textured):
    detector = SpillDetector(BackgroundReference(textured,confirmed_clean=True))
    assert detector.detect(textured).geometry.status == 'MATCHED'
    changed = add_blob(textured,(220,180))
    result = detector.detect(changed)
    assert result.geometry.status == 'MATCHED'
    assert len(result.detections) == 1


@pytest.mark.parametrize('kind',['translation','rotation','zoom'])
def test_major_framing_change_nonblocking_when_background_matches_exist(textured,kind):
    if kind == 'translation':
        matrix = np.float32([[1,0,18],[0,1,12]])
    else:
        matrix = cv2.getRotationMatrix2D((240,180),5 if kind == 'rotation' else 0,1.08 if kind == 'zoom' else 1)
    changed = cv2.warpAffine(textured,matrix,(480,360))
    detector = SpillDetector(BackgroundReference(textured,confirmed_clean=True))
    result=detector.detect(changed)
    assert result.geometry.status=='MOVED' and not result.detection_paused


def test_insufficient_texture_is_explicitly_unverified(clean):
    reference = BackgroundReference(clean,confirmed_clean=True)
    result = SpillDetector(reference).detect(clean)
    assert result.geometry.status == 'UNVERIFIED'
    assert result.geometry.center_shift_pixels is None
    result = SpillDetector(reference,DetectorConfig(geometry_check_enabled=False)).detect(clean)
    assert result.geometry.status == 'DISABLED'


def test_manual_polygon_static_and_dynamic_exclusions(clean,tmp_path):
    polygons = [[[60,160],[140,160],[140,240],[60,240]]]
    mask = exclusion_mask_from_polygons(clean.shape[:2],polygons)
    path = tmp_path/'excluded.json'
    path.write_text(json.dumps({'frame_width':400,'frame_height':320,'polygons':polygons}))
    assert np.array_equal(load_exclusion_polygons(path,clean.shape[:2]),mask)
    frame = add_blob(clean,(100,200))
    reference = BackgroundReference(clean,confirmed_clean=True)
    assert SpillDetector(reference,exclusion_mask=mask).detect(frame).detections == []
    assert SpillDetector(reference).detect(frame,exclusion_mask=mask).detections == []
    for polygons in [[[[1,1],[1,1],[1,1]]],[[[-1,1],[4,1],[4,4]]],[[[1,1],[4,float('inf')],[4,4]]]]:
        with pytest.raises(DetectionError):
            exclusion_mask_from_polygons(clean.shape[:2],polygons)
    with pytest.raises(DetectionError,match='resolution'):
        load_exclusion_polygons(path,(640,400))


def test_roi_opt_out_and_absolute_area_limit(clean):
    reference = BackgroundReference(clean,confirmed_clean=True)
    outside = add_blob(clean,(350,200))
    assert SpillDetector(reference,calibration=calibration()).detect(outside).detections == []
    detector = SpillDetector(reference,DetectorConfig(roi_enabled=False),calibration=calibration())
    assert len(detector.detect(outside).detections) == 1
    detector = SpillDetector(reference,DetectorConfig(max_area_pixels=500))
    assert detector.detect(add_blob(clean,(100,200))).detections == []


def test_component_masks_preserve_internal_holes(clean):
    frame = add_blob(clean,(100,200),radius=27)
    cv2.circle(frame,(100,200),11,(200,200,200),-1)
    detector = SpillDetector(BackgroundReference(clean,confirmed_clean=True))
    result = detector.detect(frame)
    target = result.detections[0]
    exact = result.mask_for(target)
    assert exact[200,100] == 0
    assert np.count_nonzero(exact) == target.area_pixels
    assert result.mask_for(replace(target,target_id=9)).shape == clean.shape[:2]
    with pytest.raises(DetectionError):
        result.mask_for(replace(target,component_id=0))


def test_first_last_seen_are_observation_times_across_short_gaps(clean):
    detector = SpillDetector(BackgroundReference(clean,confirmed_clean=True))
    candidates = detector.detect(add_blob(clean,(100,200))).detections
    tracker = SpillTracker(TrackerConfig(max_missed_frames=1))
    first = tracker.update(candidates,timestamp=1000)[0]
    second = tracker.update(candidates,timestamp=1001)[0]
    assert first.first_seen == second.first_seen == 1000
    assert second.last_seen == 1001
    tracker.update([],timestamp=1002)
    after_gap = tracker.update(candidates,timestamp=1003)[0]
    assert after_gap.target_id == first.target_id and after_gap.first_seen == 1000
    assert after_gap.last_seen == 1003 and not after_gap.confirmed
    assert after_gap.to_dict()['first_seen'] == 1000
    for timestamp in (999,-1,float('nan'),True):
        with pytest.raises(DetectionError):
            tracker.update(candidates,timestamp=timestamp)


def test_background_aggregation_live_manager_and_polygon_cli(clean,tmp_path,monkeypatch):
    import vision.spill_cli as cli
    for name in ('imshow','waitKey','destroyAllWindows'):
        monkeypatch.setattr(cv2,name,Mock(side_effect=AssertionError('Headless')))
    camera = Mock()
    camera.__enter__ = Mock(return_value=camera)
    camera.__exit__ = Mock(return_value=False)
    camera.read.side_effect = [np.full_like(clean,value) for value in (190,200,250)]
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    background = tmp_path/'reference.png'
    assert main.main(['background-reference','--camera','2','--reference-frames','3',
        '--aggregation','median','--output',str(background),'--confirm-clean']) == 0
    camera.__exit__.assert_called_once()
    reference = BackgroundReference.load(background,confirmed_clean=True)
    assert np.array_equal(reference.image,clean)
    assert reference.metadata['frame_count'] == 3
    image = tmp_path/'spill.png'
    cv2.imwrite(str(image),add_blob(clean,(100,200)))
    polygons = tmp_path/'polygons.json'
    polygons.write_text(json.dumps({'frame_width':400,'frame_height':320,
        'polygons':[[[50,150],[150,150],[150,250],[50,250]]]}))
    output = tmp_path/'detections.jsonl'
    assert main.main(['detect-spills','--image',str(image),'--background',str(background),
        '--confirm-clean','--headless','--exclusion-polygons',str(polygons),'--output',str(output)]) == 0
    record = json.loads(output.read_text())
    assert record['detections'] == []
    assert record['geometry']['status'] == 'UNVERIFIED'


def test_camera_resources_release_after_nonblocking_mismatch_and_end_of_frames(textured,tmp_path,monkeypatch):
    import vision.spill_cli as cli
    background = tmp_path/'reference.png'
    BackgroundReference(textured,confirmed_clean=True).save(background)
    moved = cv2.warpAffine(textured,np.float32([[1,0,18],[0,1,12]]),(480,360))
    camera = Mock()
    camera.__enter__ = Mock(return_value=camera)
    camera.__exit__ = Mock(return_value=False)
    camera.read.side_effect = [moved,None]
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    monkeypatch.setattr(cv2,'destroyAllWindows',Mock(side_effect=AssertionError('Headless')))
    assert main.main(['detect-spills','--camera','2','--background',str(background),
        '--confirm-clean','--headless','--output',str(tmp_path/'out.jsonl')]) == 0
    record=json.loads((tmp_path/'out.jsonl').read_text())
    assert not record['diagnostics']['detection_paused']
    camera.__exit__.assert_called_once()


@pytest.mark.parametrize('values',[{'roi_enabled':1},{'geometry_min_matches':2},
    {'geometry_max_shift_pixels':0},{'geometry_min_inlier_ratio':2},{'illumination_max_shift':-1},
    {'max_area_pixels':1}])
def test_extension_settings_validation(values):
    with pytest.raises(DetectionError):
        DetectorConfig(**values)
