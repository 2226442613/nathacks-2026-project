from dataclasses import replace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from vision import (BackgroundReference, CalibrationError, DetectorConfig, DetectionError,
                    DualPlaneCalibration, GroundCalibration, WallCalibration, SpillCategory,
                    SpillDetector, SpillTracker, TrackerConfig, render_detections)
from vision.spill_tracking import load_spill_settings, save_spill_settings


@pytest.fixture
def clean():
    return np.full((320, 400, 3), 200, np.uint8)


@pytest.fixture
def detector(clean):
    return SpillDetector(BackgroundReference(clean, confirmed_clean=True))


def blob(clean, color=(0,0,220), center=(100,160), radius=22):
    frame = clean.copy()
    cv2.circle(frame, center, radius, color, -1)
    return frame


def test_red_foreground_and_structured_output(clean, detector):
    result = detector.detect(blob(clean))
    assert len(result.detections) == 1
    target = result.detections[0]
    assert target.category == SpillCategory.RED_SPILL
    assert target.pixel_center == pytest.approx((100,160), abs=1)
    assert target.area_pixels > 1400
    assert len(target.contour) >= 4
    assert 0 < target.detection_score <= 1
    assert not target.confirmed
    record = target.to_dict()
    assert record['category'] == 'RED_SPILL'
    assert isinstance(record['contour'][0], list)
    assert len(record['bounding_box']) == 4
    assert result.red_mask[160,100] == 255


@pytest.mark.parametrize('hue', [0, 5, 175, 179])
def test_red_hue_wraparound(clean, detector, hue):
    color = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[hue,230,230]]]), cv2.COLOR_HSV2BGR)[0,0])
    result = detector.detect(blob(clean, color))
    assert result.detections[0].category == SpillCategory.RED_SPILL


def test_non_red_foreground_not_every_non_red_pixel(clean, detector):
    assert detector.detect(clean).detections == []
    frame = blob(clean, (15,180,180))
    result = detector.detect(frame)
    assert len(result.detections) == 1
    assert result.detections[0].category == SpillCategory.NON_RED_SPILL
    assert result.red_mask.max() == 0
    assert result.foreground_mask[0,0] == 0


def test_clean_and_small_noise(clean, detector):
    assert detector.detect(clean).detections == []
    frame = clean.copy()
    for x, y in [(10,10),(70,100),(210,140),(310,250)]:
        cv2.circle(frame, (x,y), 1, (0,0,220), -1)
    assert detector.detect(frame).detections == []


def test_multiple_regions_deduplicated_by_components(clean, detector):
    frame = blob(clean)
    cv2.circle(frame, (280,180), 25, (20,180,180), -1)
    result = detector.detect(frame)
    assert len(result.detections) == 2
    assert {target.category for target in result.detections} == {SpillCategory.RED_SPILL, SpillCategory.NON_RED_SPILL}


def test_background_confirmation_and_immutable_storage(clean, detector, tmp_path):
    with pytest.raises(DetectionError, match='Confirm'):
        BackgroundReference(clean)
    reference = BackgroundReference(clean, confirmed_clean=True)
    clean[0,0] = 0
    assert np.all(reference.image[0,0] == 200)
    external = reference.image
    external[:] = 0
    assert reference.image.mean() == 200
    path = tmp_path / 'reference.png'
    reference.save(path)
    with pytest.raises(DetectionError, match='Confirm'):
        BackgroundReference.load(path)
    loaded = BackgroundReference.load(path, confirmed_clean=True)
    assert np.array_equal(loaded.image, reference.image)
    with pytest.raises(DetectionError, match='lossless'):
        reference.save(tmp_path / 'lossy.jpg')
    original = detector._reference.copy()
    for _ in range(5):
        detector.detect(blob(loaded.image))
    assert np.array_equal(detector._reference, original)


def test_mismatched_reference_and_invalid_input(clean, detector):
    with pytest.raises(DetectionError, match='resolution'):
        detector.detect(clean[:200])
    for invalid in (clean[:,:,0], clean.astype(float), None, np.zeros((0,0,3), np.uint8)):
        with pytest.raises(DetectionError):
            detector.detect(invalid)


def calibrated_planes():
    floor = GroundCalibration([[20,140],[300,140],[300,300],[20,300]], 120,80,400,320)
    wall = WallCalibration([[20,20],[300,20],[300,140],[20,140]], 120,60,400,320)
    return DualPlaneCalibration(floor, wall, 'top')


def test_module_one_polygons_only_and_outside_roi(clean, monkeypatch):
    calibration = calibrated_planes()
    for plane in (calibration.floor, calibration.wall):
        monkeypatch.setattr(plane, 'pixel_to_world', Mock(side_effect=AssertionError('No physical transforms in Module 02')))
    detector = SpillDetector(BackgroundReference(clean, confirmed_clean=True), calibration=calibration)
    assert detector.detect(blob(clean, center=(355,160))).detections == []
    result = detector.detect(blob(clean, center=(100,200)))
    assert result.detections[0].category == SpillCategory.RED_SPILL
    wall_result = detector.detect(blob(clean, center=(100,80)))
    assert wall_result.detections[0].category == SpillCategory.RED_SPILL
    assert set(detector.polygons) == {'FLOOR','WALL'}


def test_floor_only_roi_and_resolution_mismatch(clean):
    cal = calibrated_planes().floor
    detector = SpillDetector(BackgroundReference(clean, confirmed_clean=True), calibration=cal)
    assert detector.detect(blob(clean, center=(100,80))).detections == []
    with pytest.warns(UserWarning, match='recalibration'), pytest.raises(CalibrationError):
        SpillDetector(BackgroundReference(clean[:200], confirmed_clean=True), calibration=cal)


def test_shared_boundary_candidate_is_not_split_and_retains_boundary_flag(clean):
    detector = SpillDetector(BackgroundReference(clean, confirmed_clean=True), calibration=calibrated_planes())
    result = detector.detect(blob(clean, center=(150,140)))
    assert len(result.detections) == 1
    candidate = result.detections[0]
    assert candidate.category == SpillCategory.RED_SPILL
    assert candidate.boundary_overlap
    assert 'floor-wall boundary' in candidate.classification_reason


def test_static_and_per_frame_exclusions(clean):
    frame = blob(clean)
    exclusion = np.zeros(clean.shape[:2], np.uint8)
    cv2.circle(exclusion, (100,160), 28, 255, -1)
    reference = BackgroundReference(clean, confirmed_clean=True)
    detector = SpillDetector(reference, exclusion_mask=exclusion)
    assert detector.detect(frame).detections == []
    detector = SpillDetector(reference)
    assert detector.detect(frame, exclusion_mask=exclusion).detections == []
    assert len(detector.detect(frame).detections) == 1  # Dynamic exclusion is not retained.
    for x in (60,90,120,150):
        moving = blob(clean, (220,40,20), (x,100), 15)
        mask = np.zeros(clean.shape[:2], np.uint8)
        cv2.circle(mask, (x,100), 22, 255, -1)
        assert detector.detect(moving, exclusion_mask=mask).detections == []
    for mask in (np.zeros((10,10),np.uint8), clean, np.zeros(clean.shape[:2],float)):
        with pytest.raises(DetectionError, match='mask'):
            detector.detect(frame, exclusion_mask=mask)


@pytest.mark.parametrize('color', [(40,40,40),(210,210,210)])
def test_unsaturated_new_object_is_nonred_unchanged_color_ignored(clean, detector, color):
    result = detector.detect(blob(clean, color))
    if color == (210,210,210):
        assert result.detections == []
    else:
        assert result.detections[0].category == SpillCategory.NON_RED_SPILL
        assert 0 < result.detections[0].detection_score <= 1


def test_mixed_and_elongated_objects_use_foreground_red_ratio(clean, detector):
    frame = clean.copy()
    cv2.rectangle(frame, (70,110), (100,150), (0,0,220), -1)
    cv2.rectangle(frame, (101,110), (130,150), (10,180,180), -1)
    result = detector.detect(frame)
    assert result.detections[0].category == SpillCategory.NON_RED_SPILL
    frame = clean.copy()
    cv2.rectangle(frame, (60,80), (220,90), (0,0,220), -1)
    assert detector.detect(frame).detections[0].category == SpillCategory.RED_SPILL


def test_partial_roi_region_keeps_truncation_diagnostic(clean):
    detector = SpillDetector(BackgroundReference(clean, confirmed_clean=True), calibration=calibrated_planes())
    target = detector.detect(blob(clean, center=(25,220))).detections[0]
    assert target.category == SpillCategory.RED_SPILL
    assert 'truncated' in target.classification_reason


def test_temporal_id_stability_confirmation_and_disappearance(clean, detector):
    tracker = SpillTracker(TrackerConfig(confirmation_frames=3, max_missed_frames=1))
    ids = []
    for index in range(4):
        detected = detector.detect(blob(clean, center=(100+index%2,160))).detections
        tracked = tracker.update(detected)
        ids.append(tracked[0].target_id)
        assert tracked[0].confirmed == (index >= 2)
    assert len(set(ids)) == 1
    assert tracker.update([]) == []
    assert tracker.update([]) == []
    new = tracker.update(detector.detect(blob(clean)).detections)[0]
    assert new.target_id != ids[0]
    assert not new.confirmed


def test_temporary_artifact_and_movement_do_not_confirm(clean, detector):
    tracker = SpillTracker()
    candidates = detector.detect(blob(clean)).detections
    assert not tracker.update(candidates)[0].confirmed
    assert not tracker.update(candidates)[0].confirmed
    assert tracker.update([]) == []
    observed = tracker.update(candidates)[0]
    assert not observed.confirmed and observed.consecutive_frames == 1
    tracker.reset()
    ids = []
    for x in (60,72,84,96,108,120):
        tracked = tracker.update(detector.detect(blob(clean, (220,20,20), (x,160))).detections)
        assert not tracked[0].confirmed
        ids.append(tracked[0].target_id)
    assert len(set(ids)) == 1  # Associated, but not stationary enough for confirmation.


def test_multiple_ids_and_category_change_reset(clean, detector):
    tracker = SpillTracker()
    frame = blob(clean)
    cv2.circle(frame, (280,180), 25, (20,180,180), -1)
    first = tracker.update(detector.detect(frame).detections)
    second = tracker.update(list(reversed(detector.detect(frame).detections)))
    assert {d.category:d.target_id for d in first} == {d.category:d.target_id for d in second}
    changed = tracker.update(detector.detect(blob(clean, (20,180,180))).detections)[0]
    assert changed.target_id == first[0].target_id
    assert changed.consecutive_frames == 1 and not changed.confirmed


@pytest.mark.parametrize('values', [{'blur_kernel':2}, {'difference_threshold':0}, {'min_area_pixels':-1},
    {'red_hue_high_min':180}, {'min_color_fraction':float('nan')}, {'typo':4},
    {'red_fraction_threshold':.1}, {'exclusion_padding_pixels':-1}, {'max_area_fraction':2}])
def test_invalid_detector_settings(values):
    with pytest.raises(DetectionError):
        DetectorConfig.from_dict(values)


@pytest.mark.parametrize('values', [{'confirmation_frames':0}, {'max_missed_frames':-1},
    {'max_match_distance_pixels':float('inf')}, {'stationary_distance_pixels':100}, {'typo':4}])
def test_invalid_tracker_settings(values):
    with pytest.raises(DetectionError):
        TrackerConfig.from_dict(values)


def test_settings_persistence_and_debug_render(clean, detector, tmp_path):
    path = tmp_path / 'settings.json'
    save_spill_settings(path, DetectorConfig(), TrackerConfig())
    assert load_spill_settings(path) == (DetectorConfig(), TrackerConfig())
    path.write_text('{"settings_version":99}')
    with pytest.raises(DetectionError):
        load_spill_settings(path)
    frame = blob(clean)
    detections = SpillTracker().update(detector.detect(frame).detections)
    before = frame.copy()
    rendered = render_detections(frame, detections, debug=True)
    assert np.array_equal(frame, before)
    assert not np.array_equal(frame, rendered)
