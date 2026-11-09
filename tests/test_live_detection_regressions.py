"""Real scalar failure and conservative photometric/temporal regressions."""
from dataclasses import replace
import json
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import BackgroundReference, DualPlaneCalibration, GroundCalibration
from vision.serialization import json_line, json_native
from vision.spill_detection import DetectorConfig, SpillCategory, SpillDetection, SpillDetector
from vision.spill_tracking import SpillTracker, TrackerConfig


@pytest.mark.parametrize('value,expected', [
    (np.bool_(True), True), (np.bool_(False), False), (np.int64(17), 17),
    (np.uint64(2**63), 2**63), (np.float32(1.25), 1.25), (np.float64(2.5), 2.5),
    (np.array([[1, 2], [3, 4]]), [[1, 2], [3, 4]]),
    ({'nested': [np.bool_(True), {'x': np.array([np.float32(1.25)])}]},
     {'nested': [True, {'x': [1.25]}]}),
])
def test_numpy_values_preserve_data_and_native_types(value, expected):
    actual = json_native(value)
    assert actual == expected
    assert json.loads(json_line(value)) == expected
    if not isinstance(expected, (list, dict)):
        assert type(actual) is type(expected)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf'),
    np.float32('nan'), np.float64('inf'), np.array([1, np.nan])])
def test_nonfinite_nested_data_rejected_with_path(value):
    with pytest.raises(ValueError, match=r'Non-finite JSON number at \$\.nested'):
        json_line({'nested': value})


def test_unsupported_data_is_not_silently_dropped():
    with pytest.raises(TypeError, match='Unsupported JSON value'):
        json_line({'x': object()})


def candidate(**changes):
    target = SpillDetection(SpillCategory.UNKNOWN, (60., 60.), (50, 50, 20, 20),
                           ((50, 50), (70, 50), (70, 70)), 300, .4, 'ambiguous color')
    return replace(target, **changes)


def test_original_numpy_boundary_boolean_serialization_and_schema():
    target = candidate(boundary_overlap=np.bool_(True), area_pixels=np.int64(300),
                       pixel_center=(np.float32(60), np.float64(60)))
    result = target.to_dict()
    assert result['boundary_overlap'] is True
    assert result['bounding_box'] == [50, 50, 20, 20]
    assert result['contour'] == [[50, 50], [70, 50], [70, 70]]
    assert result['category'] == 'UNKNOWN'
    assert json.loads(json_line({'detections': [result]}))['detections'][0] == result


def test_real_dual_plane_boundary_detection_jsonl(tmp_path):
    clean = np.full((200, 240, 3), 180, np.uint8)
    floor = GroundCalibration([[20, 100], [220, 100], [220, 190], [20, 190]], 108.5, 43, 240, 200)
    dual = DualPlaneCalibration.from_floor_and_wall_top(floor, [[20, 10], [220, 10]], 108.5, 29, 'top')
    calibration, reference, image, output = (tmp_path/name for name in ('cal.json', 'ref.png', 'frame.png', 'live.jsonl'))
    calibration.write_text(json.dumps(dual.to_dict()))
    BackgroundReference(clean, confirmed_clean=True).save(reference)
    frame = clean.copy()
    cv2.circle(frame, (120, 100), 20, (10, 10, 230), -1)
    cv2.imwrite(str(image), frame)
    assert main.main(['detect-spills', '--image', str(image), '--background', str(reference),
        '--calibration', str(calibration), '--confirm-clean', '--headless', '--output', str(output)]) == 0
    record = json.loads(output.read_text())
    assert record['detections'][0]['boundary_overlap'] is True
    assert record['detections'][0]['category'] == 'RED_SPILL'
    assert not record['detections'][0]['confirmed']


def test_bounded_exposure_gain_on_printed_background_preserves_targets():
    clean = np.full((240, 320, 3), 170, np.uint8)
    for x in range(10, 310, 30):
        cv2.rectangle(clean, (x, 20), (x+12, 100), (45, 45, 45), -1)
    exposed = np.clip(clean.astype(float)*1.12+3, 0, 255).astype(np.uint8)
    # Repetitive bars intentionally cannot establish global camera registration;
    # isolate the photometric correction under test from that separate safeguard.
    detector = SpillDetector(BackgroundReference(clean, confirmed_clean=True), DetectorConfig(geometry_check_enabled=False))
    result = detector.detect(exposed)
    assert result.detections == []
    assert all(1.08 < gain < 1.15 for gain in result.illumination_gain_bgr)
    cv2.circle(exposed, (85, 175), 20, (15, 15, 230), -1)
    cv2.circle(exposed, (230, 175), 20, (230, 30, 30), -1)
    result = detector.detect(exposed)
    assert {d.category for d in result.detections} == {SpillCategory.RED_SPILL, SpillCategory.NON_RED_SPILL}
    assert np.count_nonzero(result.raw_foreground_mask) >= np.count_nonzero(result.foreground_mask)


def test_identical_reference_minor_noise_and_brightness():
    clean = np.full((150, 200, 3), 180, np.uint8)
    detector = SpillDetector(BackgroundReference(clean, confirmed_clean=True))
    assert detector.detect(clean).detections == []
    noise = np.random.default_rng(9).integers(-4, 5, clean.shape)
    changed = np.clip(clean.astype(int)+noise+9, 0, 255).astype(np.uint8)
    assert detector.detect(changed).detections == []
    assert np.array_equal(detector._reference, clean)


@pytest.mark.parametrize('changes', [dict(bounding_box=(55, 40, 10, 40)), dict(area_pixels=150)])
def test_same_centroid_fluctuating_footprint_never_confirms(changes):
    tracker = SpillTracker()
    for index in range(8):
        target = candidate(**(changes if index % 2 else {}))
        result = tracker.update([target])[0]
        assert not result.confirmed
        assert result.consecutive_frames == 1


def test_stable_unknown_is_retained_and_confirmable():
    tracker = SpillTracker()
    observed = [tracker.update([candidate()])[0] for _ in range(3)]
    assert [d.confirmed for d in observed] == [False, False, True]
    assert observed[-1].category == SpillCategory.UNKNOWN


@pytest.mark.parametrize('config', [dict(illumination_max_gain_change=1), dict(illumination_gain_correction=1)])
def test_invalid_photometric_settings(config):
    with pytest.raises(ValueError):
        DetectorConfig(**config)


@pytest.mark.parametrize('config', [dict(confirmation_min_iou=-.1), dict(confirmation_max_area_ratio=float('nan'))])
def test_invalid_persistence_settings(config):
    with pytest.raises(ValueError):
        TrackerConfig(**config)


def test_serialization_failure_reports_frame_and_releases_camera_and_gui(tmp_path, monkeypatch, caplog):
    import vision.spill_cli as cli
    clean = np.full((120, 160, 3), 180, np.uint8)
    reference = tmp_path/'ref.png'
    BackgroundReference(clean, confirmed_clean=True).save(reference)
    camera = Mock()
    camera.__enter__ = Mock(return_value=camera)
    camera.__exit__ = Mock(return_value=False)
    camera.read.return_value = clean
    monkeypatch.setattr(cli, 'CameraManager', Mock(return_value=camera))
    monkeypatch.setattr(cli.SpillTracker, 'update', lambda *args: [candidate(detection_score=np.float32('nan'))])
    close = Mock()
    monkeypatch.setattr(cv2, 'destroyAllWindows', close)
    monkeypatch.setattr(cv2, 'imshow', Mock(side_effect=AssertionError('Serialization must fail before GUI')))
    output = tmp_path/'output.jsonl'
    assert main.main(['detect-spills', '--camera', '0', '--background', str(reference),
        '--confirm-clean', '--output', str(output), '--max-frames', '1']) == 1
    camera.__exit__.assert_called_once()
    close.assert_called_once()
    assert 'frame 0' in caplog.text and 'source 0' in caplog.text
    assert 'Non-finite JSON number' in caplog.text
    assert any(record.exc_info for record in caplog.records)
    assert output.read_text() == ''


def test_live_numpy_detections_write_multiple_valid_jsonl_records(tmp_path, monkeypatch):
    import vision.spill_cli as cli
    clean = np.full((120, 160, 3), 180, np.uint8)
    reference, output = tmp_path/'ref.png', tmp_path/'live.jsonl'
    BackgroundReference(clean, confirmed_clean=True).save(reference)
    camera = Mock()
    camera.__enter__ = Mock(return_value=camera)
    camera.__exit__ = Mock(return_value=False)
    camera.read.return_value = clean
    monkeypatch.setattr(cli, 'CameraManager', Mock(return_value=camera))
    target = candidate(boundary_overlap=np.bool_(True), confirmed=np.bool_(False),
                       area_pixels=np.int32(300), detection_score=np.float32(.4))
    monkeypatch.setattr(cli.SpillTracker, 'update', lambda *args: [target])
    assert main.main(['detect-spills', '--camera', '0', '--background', str(reference),
        '--confirm-clean', '--headless', '--output', str(output), '--max-frames', '3']) == 0
    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert [r['frame_index'] for r in records] == [0, 1, 2]
    assert all(r['detections'][0]['boundary_overlap'] is True for r in records)
    camera.__exit__.assert_called_once()


def test_unexpected_programming_error_logs_traceback_and_closes(tmp_path, monkeypatch, caplog):
    import vision.spill_cli as cli
    clean = np.full((120, 160, 3), 180, np.uint8)
    reference = tmp_path/'ref.png'
    BackgroundReference(clean, confirmed_clean=True).save(reference)
    camera = Mock()
    camera.__enter__ = Mock(return_value=camera)
    camera.__exit__ = Mock(return_value=False)
    camera.read.return_value = clean
    monkeypatch.setattr(cli, 'CameraManager', Mock(return_value=camera))
    monkeypatch.setattr(cli.SpillDetector, 'detect', Mock(side_effect=TypeError('test programming failure')))
    close = Mock()
    monkeypatch.setattr(cv2, 'destroyAllWindows', close)
    assert main.main(['detect-spills', '--camera', '0', '--background', str(reference),
        '--confirm-clean', '--max-frames', '1']) == 1
    camera.__exit__.assert_called_once()
    close.assert_called_once()
    assert 'test programming failure' in caplog.text
    assert any(record.exc_info for record in caplog.records)
