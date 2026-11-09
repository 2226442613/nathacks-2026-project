from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from vision import CameraError, CameraManager


def fake_capture(opened=True):
    capture = Mock()
    capture.isOpened.return_value = opened
    capture.get.return_value = 30
    capture.read.return_value = (True, np.zeros((48, 64, 3), dtype=np.uint8))
    return capture


def test_camera_read_and_release(monkeypatch):
    capture = fake_capture()
    monkeypatch.setattr(cv2, 'VideoCapture', Mock(return_value=capture))
    with CameraManager(2, 640, 480, 25) as camera:
        assert camera.read().shape == (48, 64, 3)
        assert camera.settings['fps'] == 30
        capture.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set.assert_any_call(cv2.CAP_PROP_BUFFERSIZE, 1)
    capture.release.assert_called_once()
    with pytest.raises(CameraError, match='not open'):
        camera.read()


def test_mac_backend_fallback(monkeypatch):
    import vision.camera as module
    monkeypatch.setattr(module.platform, 'system', lambda: 'Darwin')
    first, second = fake_capture(False), fake_capture()
    factory = Mock(side_effect=[first, second])
    monkeypatch.setattr(cv2, 'VideoCapture', factory)
    with CameraManager(3):
        pass
    assert factory.call_args_list[0].args == (3, cv2.CAP_AVFOUNDATION)
    assert factory.call_args_list[1].args == (3, cv2.CAP_ANY)
    first.release.assert_called_once()
    second.release.assert_called_once()


def test_unavailable_camera(monkeypatch):
    capture = fake_capture(False)
    monkeypatch.setattr(cv2, 'VideoCapture', Mock(return_value=capture))
    with pytest.raises(CameraError, match='disconnected'):
        CameraManager().open()
    assert capture.release.called


def test_read_failure_releases(monkeypatch):
    capture = fake_capture()
    capture.read.return_value = (False, None)
    monkeypatch.setattr(cv2, 'VideoCapture', Mock(return_value=capture))
    with pytest.raises(CameraError, match='frame read failed'):
        with CameraManager() as camera:
            camera.read()
    capture.release.assert_called_once()


def test_missing_video(tmp_path):
    with pytest.raises(CameraError, match='does not exist'):
        CameraManager(tmp_path / 'missing.avi').open()


def test_real_video_file(tmp_path):
    path = tmp_path / 'input.avi'
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 10, (64, 48))
    assert writer.isOpened(), 'Test environment must support MJPG AVI encoding'
    for value in (30, 90, 150):
        writer.write(np.full((48, 64, 3), value, dtype=np.uint8))
    writer.release()
    with CameraManager(path) as camera:
        assert camera.settings['width'] == 64
        frames = [camera.read() for _ in range(3)]
        assert all(frame.shape == (48, 64, 3) for frame in frames)
        assert frames[0].mean() == pytest.approx(30, abs=3)
        assert camera.read() is None


@pytest.mark.parametrize('kwargs', [{'source': -1}, {'width': 0}, {'height': -1}, {'fps': float('nan')}])
def test_invalid_requests(kwargs):
    with pytest.raises(ValueError):
        CameraManager(**kwargs)
