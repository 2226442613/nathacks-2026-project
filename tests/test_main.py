from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import GroundCalibration


@pytest.fixture
def gui(monkeypatch):
    for name in ('namedWindow', 'imshow', 'destroyWindow', 'destroyAllWindows'):
        monkeypatch.setattr(cv2, name, Mock())
    monkeypatch.setattr(cv2, 'getWindowProperty', lambda *_: 1)
    monkeypatch.setattr(cv2, 'waitKey', lambda *_: ord('q'))


def test_screenshot_unique_and_readable(tmp_path):
    frame = np.full((48, 64, 3), 90, np.uint8)
    first = main.save_screenshot(frame, tmp_path / 'shots')
    second = main.save_screenshot(frame, tmp_path / 'shots')
    assert first != second
    assert np.array_equal(cv2.imread(str(first)), frame)


def test_screenshot_write_error(monkeypatch, tmp_path):
    monkeypatch.setattr(cv2, 'imwrite', lambda *_: False)
    with pytest.raises(OSError, match='Could not save'):
        main.save_screenshot(np.zeros((10, 10, 3), np.uint8), tmp_path)


def mock_camera(monkeypatch, frames):
    camera = Mock()
    camera.__enter__ = Mock(return_value=camera)
    camera.__exit__ = Mock(return_value=False)
    camera.read.side_effect = frames
    monkeypatch.setattr(main, 'CameraManager', Mock(return_value=camera))
    return camera


def test_preview_screenshot_exit(monkeypatch, tmp_path, gui):
    camera = mock_camera(monkeypatch, [np.zeros((48, 64, 3), np.uint8)] * 2)
    keys = iter([ord('s'), ord('q')])
    monkeypatch.setattr(cv2, 'waitKey', lambda *_: next(keys))
    assert main.main(['preview', '--camera', '2', '--screenshots', str(tmp_path)]) == 0
    assert len(list(tmp_path.glob('*.png'))) == 1
    camera.__exit__.assert_called_once()
    assert cv2.imshow.call_count == 2
    cv2.destroyAllWindows.assert_called_once()


def test_preview_eof(monkeypatch, gui):
    camera = mock_camera(monkeypatch, [None])
    assert main.main(['preview', '--video', 'example.avi']) == 0
    camera.__exit__.assert_called_once()


def test_calibration_click_reset_save(monkeypatch, tmp_path, gui):
    frame = np.zeros((100, 140, 3), np.uint8)
    monkeypatch.setattr(cv2, 'imread', lambda *_: frame)
    callback = None

    def install(_window, fn):
        nonlocal callback
        callback = fn

    monkeypatch.setattr(cv2, 'setMouseCallback', install)
    calls = 0

    def key(_delay):
        nonlocal calls
        calls += 1
        if calls == 1:
            callback(cv2.EVENT_LBUTTONDOWN, 3, 3, 0, None)
            return ord('r')
        if calls == 2:
            for x, y in [(10, 10), (130, 10), (130, 90), (10, 90)]:
                callback(cv2.EVENT_LBUTTONDOWN, x, y, 0, None)
            return 13
        return ord('q')

    monkeypatch.setattr(cv2, 'waitKey', key)
    path = tmp_path / 'calibration.json'
    assert main.main(['calibrate', '--image', 'frame.png', '--calibration-source', '2',
                      '--arena-width-cm', '120', '--arena-height-cm', '80', '--config', str(path)]) == 0
    cal = GroundCalibration.load(path)
    assert cal.source == 2
    assert cal.pixel_to_world(70, 50) == pytest.approx((60, 40))
    assert cv2.imshow.call_count == 3


def test_verification_geometry_mismatch(monkeypatch, tmp_path, gui):
    cal = GroundCalibration([[1, 1], [90, 1], [90, 90], [1, 90]], 120, 80, 100, 100)
    path = tmp_path / 'cal.json'
    cal.save(path)
    camera = mock_camera(monkeypatch, [np.zeros((50, 50, 3), np.uint8)])
    with pytest.warns(UserWarning, match='recalibration'):
        assert main.main(['verify-calibration', '--config', str(path)]) == 1
    camera.__exit__.assert_called_once()


def test_invalid_camera_options_clean_error(gui):
    assert main.main(['preview', '--width', '0']) == 1


def test_interrupted_preview_releases(monkeypatch, gui):
    camera = mock_camera(monkeypatch, [])
    camera.read.side_effect = KeyboardInterrupt
    assert main.main(['preview']) == 0
    camera.__exit__.assert_called_once()


def dual_cli_arguments(tmp_path):
    return ['calibrate', '--image', 'frame.png', '--calibration-source', '2', '--dual-plane',
            '--arena-width-cm', '120', '--arena-depth-cm', '80',
            '--wall-width-cm', '120', '--wall-height-cm', '60', '--floor-boundary', 'top',
            '--grid', '--config', str(tmp_path / 'dual.json')]


def test_dual_calibration_same_frame_save_and_verify(monkeypatch, tmp_path, gui):
    from vision import DualPlaneCalibration
    frame = np.zeros((600, 800, 3), np.uint8)
    monkeypatch.setattr(cv2, 'imread', lambda *_: frame)
    callback = None
    selection = 0

    def install(_window, fn):
        nonlocal callback
        callback = fn

    def key(_delay):
        nonlocal selection
        selections = [((200,200),(400,200),(400,400),(200,400)),
                      ((200,100),(400,100))]
        if selection < 2:
            for x, y in selections[selection]:
                callback(cv2.EVENT_LBUTTONDOWN, x, y, 0, None)
            selection += 1
            return 13
        if selection == 2:
            assert not (tmp_path / 'dual.json').exists()  # Review must precede saving.
            selection += 1
            return 13
        return ord('q')

    monkeypatch.setattr(cv2, 'setMouseCallback', install)
    monkeypatch.setattr(cv2, 'waitKey', key)
    assert main.main(dual_cli_arguments(tmp_path)) == 0
    calibration = DualPlaneCalibration.load(tmp_path / 'dual.json')
    assert calibration.source == 2
    assert calibration.pixel_to_wall(300,150) == pytest.approx((60,30))
    titles = [call.args[0] for call in cv2.imshow.call_args_list]
    assert any(title.startswith('Review six points') for title in titles)
    assert np.array_equal(calibration.wall.corners_pixels[3], calibration.floor.corners_pixels[0])
    assert np.array_equal(calibration.wall.corners_pixels[2], calibration.floor.corners_pixels[1])
    assert 'Floor rectified (cm) - Q exit' in titles
    assert 'Wall rectified (cm) - Q exit' in titles
    cv2.imshow.reset_mock()
    camera = mock_camera(monkeypatch, [frame])
    assert main.main(['verify-calibration', '--camera', '2', '--config', str(tmp_path / 'dual.json'), '--grid']) == 0
    assert cv2.imshow.call_count == 3  # Original frame plus both rectified planes.
    camera.__exit__.assert_called_once()


def test_cancel_wall_keeps_existing_configuration(monkeypatch, tmp_path, gui):
    path = tmp_path / 'dual.json'
    path.write_text('previous calibration')
    monkeypatch.setattr(cv2, 'imread', lambda *_: np.zeros((600,800,3), np.uint8))
    monkeypatch.setattr(main, 'select_corners', Mock(side_effect=[
        [(200,200),(400,200),(400,400),(200,400)], None]))
    assert main.main(dual_cli_arguments(tmp_path)) == 0
    assert path.read_text() == 'previous calibration'


def test_invalid_wall_does_not_replace_configuration(monkeypatch, tmp_path, gui):
    path = tmp_path / 'dual.json'
    path.write_text('previous calibration')
    monkeypatch.setattr(cv2, 'imread', lambda *_: np.zeros((600,800,3), np.uint8))
    monkeypatch.setattr(main, 'select_corners', Mock(side_effect=[
        [(200,200),(400,200),(400,400),(200,400)], [(10,10)] * 2, None]))
    assert main.main(dual_cli_arguments(tmp_path)) == 0  # Invalid wall can be corrected or cancelled in-session.
    assert path.read_text() == 'previous calibration'


def test_incomplete_dual_configuration_is_not_used_as_floor(monkeypatch, tmp_path, gui):
    path = tmp_path / 'bad.json'
    path.write_text('{"calibration_version": 2, "camera": {}, "floor": {}}')
    camera_class = Mock()
    monkeypatch.setattr(main, 'CameraManager', camera_class)
    assert main.main(['verify-calibration', '--config', str(path)]) == 1
    camera_class.assert_not_called()
