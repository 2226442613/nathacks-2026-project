"""Same original color/mask convention on floor/wall; synthetic, not field accuracy."""
import json

import cv2
import numpy as np
import pytest

import main
from vision import BackgroundReference, DualPlaneCalibration, GroundCalibration
from vision.spill_detection import DetectorConfig, SpillDetector, SpillCategory
from vision.spill_tracking import SpillTracker


@pytest.fixture
def scene():
    image = np.full((360,480,3),(95,145,185),np.uint8)
    cv2.rectangle(image,(350,35),(405,70),(10,10,220),-1)  # Legal printed logo.
    return image


def planes():
    floor = GroundCalibration([[20,180],[460,180],[460,340],[20,340]],108.5,43,480,360)
    return DualPlaneCalibration.from_floor_and_wall_top(floor,[[20,20],[460,20]],108.5,29,'top')


def detector(scene, **settings):
    # Color/ROI tests isolate geometry registration, which has its own regressions.
    return SpillDetector(BackgroundReference(scene,confirmed_clean=True),
                         DetectorConfig(geometry_check_enabled=False,**settings),planes())


def target_frame(scene, center, color, half_width=12, half_height=16):
    frame = scene.copy()
    x,y = center
    frame[y-half_height:y+half_height,x-half_width:x+half_width] = color
    return frame


@pytest.mark.parametrize('surface,center', [('FLOOR',(120,260)),('WALL',(120,90))])
@pytest.mark.parametrize('color', [(10,10,220),(5,5,45),(115,115,180),(90,70,180)])
def test_same_red_on_both_planes_under_shadow_and_white_balance(scene,surface,center,color):
    model = detector(scene)
    calibration_before = model.polygons[surface].copy()
    frame = target_frame(scene,center,color)
    before = frame.copy()
    result = model.detect(frame,color_diagnostics=True)
    assert len(result.detections) == 1
    target = result.detections[0]
    stats = result.color_diagnostics[target.component_id]
    assert target.category == SpillCategory.RED_SPILL
    assert stats['surface_roi'] == surface
    assert stats['total_component_pixel_count'] == target.area_pixels
    mask = result.mask_for(target) != 0
    assert mask.shape == scene.shape[:2] and mask[center[1],center[0]]
    assert np.count_nonzero(mask) == target.area_pixels
    assert stats['red_pixel_count'] == np.count_nonzero(mask & (result.red_mask != 0))
    hsv = cv2.cvtColor(frame[mask].reshape(-1,1,3),cv2.COLOR_BGR2HSV)[:,0]
    assert stats['mean_hsv'] == pytest.approx(hsv.mean(axis=0))
    assert stats['median_hsv'] == pytest.approx(np.median(hsv,axis=0))
    assert np.array_equal(frame,before) and np.array_equal(model.polygons[surface],calibration_before)


def test_original_strict_sv_failure_reproduced_and_repaired(scene):
    frame = target_frame(scene,(120,90),(5,5,45))
    strict = detector(scene,red_shadow_enabled=False,red_interior_erosion_pixels=0).detect(frame,color_diagnostics=True)
    assert strict.detections[0].category == SpillCategory.NON_RED_SPILL
    repaired = detector(scene).detect(frame,color_diagnostics=True)
    assert repaired.detections[0].category == SpillCategory.RED_SPILL
    stats = repaired.color_diagnostics[repaired.detections[0].component_id]
    assert stats['strict_red_pixel_count'] == 0
    assert stats['supplementary_red_pixel_count'] > 0
    assert stats['median_hsv'][2] == 45


def test_perspective_thin_object_brown_blur_halo_uses_guarded_interior(scene):
    frame = target_frame(scene,(120,90),(10,10,220),half_width=2,half_height=15)
    old = detector(scene,red_interior_erosion_pixels=0).detect(frame,color_diagnostics=True)
    assert old.detections[0].category == SpillCategory.NON_RED_SPILL
    model = detector(scene)
    result = model.detect(frame,color_diagnostics=True)
    target = result.detections[0]
    stats = result.color_diagnostics[target.component_id]
    assert target.category == SpillCategory.RED_SPILL
    assert .5 <= stats['red_pixel_ratio'] < .7
    assert stats['interior_red_pixel_ratio'] == 1
    assert stats['classification_basis'] == 'supported_component_interior'
    # Interior is a color sample only: geometry/exact component stay identical.
    assert target.bounding_box == old.detections[0].bounding_box
    assert np.array_equal(result.mask_for(target),old.mask_for(old.detections[0]))


def test_noisy_nonred_boundary_still_accepts_red_majority(scene):
    frame = scene.copy()
    frame[70:101,100:131] = (160,210,230)
    frame[73:98,103:128] = (10,10,220)
    model = detector(scene,blur_kernel=1,opening_kernel=1,closing_kernel=1)
    result = model.detect(frame,color_diagnostics=True)
    target = result.detections[0]
    info = result.color_diagnostics[target.component_id]
    assert target.category == SpillCategory.RED_SPILL
    assert .5 < info['red_pixel_ratio'] < .7
    assert info['interior_red_pixel_ratio'] > .7


def test_tiny_valid_component_uses_full_mask_without_eroding_geometry(scene):
    frame = target_frame(scene,(120,90),(10,10,220),half_width=2,half_height=10)
    model = detector(scene,blur_kernel=1,opening_kernel=1,closing_kernel=1)
    result = model.detect(frame,color_diagnostics=True)
    target = result.detections[0]
    info = result.color_diagnostics[target.component_id]
    assert target.category == SpillCategory.RED_SPILL and target.area_pixels == 80
    assert info['interior_red_pixel_ratio'] is None
    assert info['red_pixel_ratio'] == 1 and np.count_nonzero(result.mask_for(target)) == 80


@pytest.mark.parametrize('hue', [0,5,175,179])
def test_both_red_hue_ranges_in_dim_and_low_saturation_light(scene,hue):
    color = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[hue,90,45]]]),cv2.COLOR_HSV2BGR)[0,0])
    result = detector(scene).detect(target_frame(scene,(120,90),color))
    assert result.detections[0].category == SpillCategory.RED_SPILL


@pytest.mark.parametrize('hue', [14,18,22,30,35,100])
def test_orange_yellow_and_blue_outside_red_range_remain_nonred(scene,hue):
    color = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[hue,220,200]]]),cv2.COLOR_HSV2BGR)[0,0])
    result = detector(scene).detect(target_frame(scene,(120,90),color),color_diagnostics=True)
    assert result.detections[0].category == SpillCategory.NON_RED_SPILL
    info = result.color_diagnostics[result.detections[0].component_id]
    assert info['red_pixel_count'] == 0


def test_small_red_core_in_nonred_majority_cannot_be_rescued(scene):
    frame = scene.copy()
    frame[65:115,95:145] = (20,220,220)
    frame[78:102,108:132] = (10,10,220)
    result = detector(scene).detect(frame,color_diagnostics=True)
    target = result.detections[0]
    assert target.category == SpillCategory.NON_RED_SPILL
    assert result.color_diagnostics[target.component_id]['red_pixel_ratio'] < .5


def test_legal_cardboard_and_existing_logo_never_become_color_targets(scene):
    model = detector(scene)
    for _ in range(4):
        result = model.detect(scene,color_diagnostics=True)
        assert result.detections == [] and result.color_diagnostics == {}
    changed = target_frame(scene,(120,90),(5,5,45))
    result = model.detect(changed,color_diagnostics=True)
    assert len(result.detections) == 1
    assert not result.foreground_mask[50,375]


@pytest.mark.parametrize('surface,center', [('FLOOR',(230,260)),('WALL',(230,90))])
def test_reliable_alignment_uses_original_current_colors_and_coordinates(surface,center):
    reference = np.random.default_rng(27).integers(40,180,(360,480,3),dtype=np.uint8)
    shifted = cv2.warpAffine(reference,np.float32([[1,0,1],[0,1,0]]),(480,360))
    color = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[179,160,45]]]),cv2.COLOR_HSV2BGR)[0,0])
    frame = target_frame(shifted,center,color)
    result = SpillDetector(BackgroundReference(reference,confirmed_clean=True),calibration=planes()).detect(frame,color_diagnostics=True)
    assert result.geometry.status == 'MATCHED' and result.alignment_applied
    target = result.detections[0]
    info = result.color_diagnostics[target.component_id]
    assert target.category == SpillCategory.RED_SPILL and info['surface_roi'] == surface
    assert target.pixel_center == pytest.approx((center[0]-.5,center[1]-.5),abs=1)
    assert info['median_hsv'] == pytest.approx(cv2.cvtColor(np.uint8([[color]]),cv2.COLOR_BGR2HSV)[0,0])
    assert result.mask_for(target)[center[1],center[0]] == 255


def test_tracking_and_immutable_legal_baseline_remain_stable(scene):
    model = detector(scene)
    tracker = SpillTracker()
    expected = None
    for index in range(8):
        frame = target_frame(scene,(120,260),(10,10,75))
        frame = target_frame(frame,(120,90),(5,5,45+index%2*5))
        targets = tracker.update(model.detect(frame,color_diagnostics=True).detections)
        ids = {tuple(round(v) for v in t.pixel_center):t.target_id for t in targets}
        assert expected is None or ids == expected
        expected = ids
        assert len(targets) == 2 and all(t.category == SpillCategory.RED_SPILL for t in targets)
        assert all(t.confirmed == (index >= 2) for t in targets)
    assert np.array_equal(model._reference,scene)
    assert tracker.update(model.detect(scene).detections) == []


def test_debug_records_synchronize_original_frame_exact_mask_and_target_stats(scene,tmp_path):
    reference,image,calibration,output,debug = (tmp_path/name for name in ('ref.png','wall.png','cal.json','out.jsonl','debug'))
    BackgroundReference(scene,confirmed_clean=True).save(reference)
    planes().save(calibration)
    before = (reference.read_bytes(),reference.with_suffix('.png.json').read_bytes(),calibration.read_bytes())
    frame = target_frame(scene,(120,90),(5,5,45))
    cv2.imwrite(str(image),frame)
    args = ['detect-spills','--image',str(image),'--background',str(reference),'--confirm-clean',
            '--calibration',str(calibration),'--headless','--confirmation-frames','1','--output',str(output)]
    assert main.main(args+['--debug-dir',str(debug)]) == 0
    record = json.loads(output.read_text())
    assert record == json.loads((debug/'frame_000000_diagnostics.json').read_text())
    target,info = record['detections'][0],record['color_diagnostics'][0]
    assert info['target_id'] == target['target_id'] and info['category'] == 'RED_SPILL'
    assert info['surface_roi'] == 'WALL' and info['color_source'] == 'original_bgr_frame'
    assert info['coordinate_system'] == 'original_image_pixels'
    assert info['strict_red_pixel_count'] == 0 and info['supplementary_red_pixel_count'] > 0
    labels = np.load(debug/'frame_000000_components.npz',allow_pickle=False)['component_labels']
    mask = labels == target['component_id']
    red = cv2.imread(str(debug/'frame_000000_red.png'),cv2.IMREAD_GRAYSCALE)
    assert int(mask.sum()) == info['total_component_pixel_count'] == target['area_pixels']
    assert int(np.count_nonzero(mask & (red!=0))) == info['red_pixel_count']
    assert info['red_pixel_ratio'] == pytest.approx(info['red_pixel_count']/info['total_component_pixel_count'])
    assert np.array_equal(cv2.imread(str(debug/'frame_000000_input.png')),frame)
    assert before == (reference.read_bytes(),reference.with_suffix('.png.json').read_bytes(),calibration.read_bytes())
    assert main.main(args) == 0
    assert 'color_diagnostics' not in json.loads(output.read_text())


@pytest.mark.parametrize('settings', [dict(red_shadow_enabled=1),dict(red_shadow_min_saturation=-1),
    dict(red_shadow_min_value=256),dict(red_shadow_min_normalized_excess=float('nan')),
    dict(red_interior_erosion_pixels=4),dict(red_interior_min_pixels=0),
    dict(red_interior_min_fraction=2),dict(red_interior_min_full_ratio=0)])
def test_new_red_settings_are_validated(settings):
    with pytest.raises(ValueError):
        DetectorConfig(**settings)
