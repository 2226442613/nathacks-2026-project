import json

import numpy as np
import pytest

from vision import CalibrationError, GroundCalibration


@pytest.fixture
def calibration():
    return GroundCalibration([[100, 100], [700, 100], [700, 500], [100, 500]], 120, 80, 800, 600)


def test_known_rectangle(calibration):
    assert calibration.pixel_to_world(400, 300) == pytest.approx((60, 40))
    assert calibration.pixel_to_world(700, 500) == pytest.approx((120, 80))


def test_inverse(calibration):
    assert calibration.world_to_pixel(60, 40) == pytest.approx((400, 300))


def test_perspective_round_trip():
    cal = GroundCalibration([[90, 70], [720, 130], [650, 530], [140, 490]], 120, 80, 800, 600)
    for point in [(300, 200), (100, 100), (540, 420)]:
        assert cal.world_to_pixel(*cal.pixel_to_world(*point)) == pytest.approx(point, abs=1e-7)
    for pixel, world in zip(cal.corners_pixels, [(0, 0), (120, 0), (120, 80), (0, 80)]):
        assert cal.pixel_to_world(*pixel) == pytest.approx(world, abs=1e-5)


@pytest.mark.parametrize('width,height', [(0, 80), (-1, 80), (120, 0), (120, -3), (float('nan'), 80), (120, float('inf'))])
def test_invalid_dimensions(width, height):
    with pytest.raises(CalibrationError):
        GroundCalibration([[1, 1], [9, 1], [9, 9], [1, 9]], width, height, 10, 10)


@pytest.mark.parametrize('points', [
    [[1, 1]] * 4,
    [[1, 1], [2, 2], [3, 3], [4, 4]],
    [[1, 1], [9, 9], [9, 1], [1, 9]],
    [[1, 1], [9, 1], [3, 3], [1, 9]],
    [[1, 1], [9, 1], [9, 9], [float('nan'), 9]],
    [[-1, 1], [9, 1], [9, 9], [1, 9]],
    [[1, 1], [10, 1], [9, 9], [1, 9]],
    [[1, 1], [9, 1], [9, 9]],
])
def test_invalid_points(points):
    with pytest.raises(CalibrationError):
        GroundCalibration(points, 120, 80, 10, 10)


def test_persistence(calibration, tmp_path):
    path = tmp_path / 'config' / 'camera.json'
    calibration.save(path)
    loaded = GroundCalibration.load(path)
    assert loaded.to_dict() == calibration.to_dict()
    assert loaded.pixel_to_world(400, 300) == pytest.approx((60, 40))


@pytest.mark.parametrize('damage', ['missing', 'singular', 'nan', 'inconsistent', 'version', 'frame'])
def test_invalid_config(calibration, tmp_path, damage):
    data = calibration.to_dict()
    if damage == 'missing':
        del data['arena']
    elif damage == 'singular':
        data['homography'] = np.zeros((3, 3)).tolist()
    elif damage == 'nan':
        data['homography'][0][0] = float('nan')
    elif damage == 'inconsistent':
        data['homography'][0][2] += 5
    elif damage == 'version':
        data['calibration_version'] = 42
    else:
        data['camera']['frame_width'] = 0
    path = tmp_path / 'bad.json'
    path.write_text(json.dumps(data))
    with pytest.raises(CalibrationError):
        GroundCalibration.load(path)


def test_broken_and_absent_json(tmp_path):
    path = tmp_path / 'missing.json'
    with pytest.raises(CalibrationError):
        GroundCalibration.load(path)
    path.write_text('{broken')
    with pytest.raises(CalibrationError):
        GroundCalibration.load(path)


def test_resolution_mismatch(calibration):
    calibration.validate_resolution(800, 600)
    with pytest.warns(UserWarning, match='recalibration required'), pytest.raises(CalibrationError):
        calibration.validate_resolution(1280, 720)


@pytest.mark.parametrize('point', [(float('nan'), 1), (1, float('inf'))])
def test_nonfinite_coordinates(calibration, point):
    for transform in (calibration.pixel_to_world, calibration.world_to_pixel):
        with pytest.raises(CalibrationError):
            transform(*point)


def test_birdseye(calibration):
    frame = np.zeros((600, 800, 3), dtype=np.uint8)
    frame[295:306, 395:406] = (255, 0, 0)
    result = calibration.birdseye(frame, 5)
    assert result.shape == (401, 601, 3)
    assert result[200, 300, 0] == 255
    bounded = calibration.birdseye(frame, 1e100)
    assert max(bounded.shape[:2]) <= 2048
    assert (bounded.shape[1] - 1) / (bounded.shape[0] - 1) == pytest.approx(1.5, rel=.002)
    with pytest.raises(CalibrationError):
        calibration.birdseye(frame, 0)
