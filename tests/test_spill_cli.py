import json
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from tools.generate_spill_demo import generate_demo
from vision import BackgroundReference
from vision.spill_cli import capture_background


@pytest.fixture
def demo(tmp_path):
    return generate_demo(tmp_path / 'demo')


@pytest.fixture
def no_gui(monkeypatch):
    for name in ('namedWindow','imshow','waitKey','destroyWindow','destroyAllWindows'):
        monkeypatch.setattr(cv2, name, Mock(side_effect=AssertionError('No GUI calls allowed in headless mode')))


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_offline_image_cli_json_and_debug_masks(demo, no_gui):
    output = demo / 'image.jsonl'
    debug = demo / 'debug'
    assert main.main(['detect-spills','--image',str(demo/'spills.png'),
        '--background',str(demo/'clean.png'),'--confirm-clean','--headless',
        '--confirmation-frames','1','--output',str(output),'--debug-dir',str(debug)]) == 0
    result = records(output)
    assert len(result) == 1
    assert {d['category'] for d in result[0]['detections']} == {'RED_SPILL','NON_RED_SPILL'}
    assert all(d['confirmed'] for d in result[0]['detections'])
    assert len(list(debug.glob('*.png'))) == 8
    assert cv2.imread(str(debug/'frame_000000_annotated.png')).shape == (320,400,3)


def test_real_video_tracking_and_temporary_artifacts(demo, no_gui):
    output = demo / 'video.jsonl'
    assert main.main(['detect-spills','--video',str(demo/'spills.avi'),
        '--background',str(demo/'clean.png'),'--exclusion-mask',str(demo/'exclude.png'),
        '--confirm-clean','--headless','--output',str(output)]) == 0
    frames = records(output)
    assert len(frames) == 18
    assert all(frame['detections'] == [] for frame in frames[:3])
    for category in ('RED_SPILL','NON_RED_SPILL'):
        observed = [d for frame in frames[3:] for d in frame['detections']
                    if d['category'] == category and d['pixel_center'][0] > 90]
        assert len({d['target_id'] for d in observed}) == (2 if category == 'NON_RED_SPILL' else 1)
        assert any(d['confirmed'] for d in observed)
    artifacts = [d for frame in frames for d in frame['detections'] if d['pixel_center'][0] < 90 and d['pixel_center'][1] > 220]
    assert len(artifacts) == 2
    assert not any(d['confirmed'] for d in artifacts)
    assert not any(d['pixel_center'][1] < 65 for frame in frames for d in frame['detections'])


def test_single_image_never_fakes_multiple_observations(demo, no_gui):
    output = demo/'pending.jsonl'
    assert main.main(['detect-spills','--image',str(demo/'spills.png'),
        '--background',str(demo/'clean.png'),'--confirm-clean','--headless','--output',str(output)]) == 0
    assert all(not d['confirmed'] and d['consecutive_frames'] == 1 for d in records(output)[0]['detections'])


def test_reference_capture_from_image_and_video(demo, no_gui):
    output = demo/'reference.png'
    assert main.main(['background-reference','--image',str(demo/'clean.png'),
        '--output',str(output),'--confirm-clean']) == 0
    assert np.array_equal(BackgroundReference.load(output, confirmed_clean=True).image,
                          cv2.imread(str(demo/'clean.png')))
    assert main.main(['background-reference','--video',str(demo/'spills.avi'),
        '--frame-index','1','--output',str(demo/'video_reference.png'),'--confirm-clean']) == 0
    # The video demo has a moving-object lane, so compare a pixel outside that lane.
    assert cv2.imread(str(demo/'video_reference.png'))[200,200].mean() == pytest.approx(200,abs=3)
    assert main.main(['background-reference','--video',str(demo/'spills.avi'),
        '--frame-index','99','--output',str(demo/'missing.png'),'--confirm-clean']) == 1


def test_missing_clean_confirmation_is_rejected(demo, no_gui):
    with pytest.raises(SystemExit):
        main.main(['detect-spills','--image',str(demo/'spills.png'),
                   '--background',str(demo/'clean.png'),'--headless'])
    with pytest.raises(SystemExit):
        main.main(['background-reference','--image',str(demo/'clean.png'),'--output',str(demo/'reference.png')])


def test_unreadable_files_invalid_thresholds_and_input_overwrite(demo, no_gui):
    common = ['detect-spills','--background',str(demo/'clean.png'),'--confirm-clean','--headless']
    assert main.main(common + ['--image',str(demo/'missing.png')]) == 1
    assert main.main(common + ['--image',str(demo/'spills.png'),'--confirmation-frames','0']) == 1
    assert main.main(common + ['--image',str(demo/'spills.png'),'--max-frames','0']) == 1
    assert main.main(common + ['--image',str(demo/'spills.png'),'--exclusion-mask',str(demo/'missing.png')]) == 1
    before = (demo/'spills.png').read_bytes()
    assert main.main(common + ['--image',str(demo/'spills.png'),'--output',str(demo/'spills.png')]) == 1
    assert (demo/'spills.png').read_bytes() == before


def test_detect_camera_uses_existing_manager_and_releases(demo, monkeypatch, no_gui):
    import vision.spill_cli as cli
    camera = Mock()
    camera.__enter__ = Mock(return_value=camera)
    camera.__exit__ = Mock(return_value=False)
    camera.read.side_effect = [cv2.imread(str(demo/'spills.png')), KeyboardInterrupt]
    factory = Mock(return_value=camera)
    monkeypatch.setattr(cli,'CameraManager',factory)
    assert main.main(['detect-spills','--camera','2','--background',str(demo/'clean.png'),
        '--confirm-clean','--headless','--output',str(demo/'camera.jsonl')]) == 0
    factory.assert_called_once_with(2,1280,720,30)
    camera.__exit__.assert_called_once()


def test_native_debug_preview_control_flow(demo, monkeypatch):
    for name in ('imshow','destroyAllWindows'):
        monkeypatch.setattr(cv2,name,Mock())
    monkeypatch.setattr(cv2,'waitKey',lambda *_: ord('q'))
    assert main.main(['detect-spills','--image',str(demo/'spills.png'),
        '--background',str(demo/'clean.png'),'--confirm-clean','--show-masks',
        '--output',str(demo/'preview.jsonl')]) == 0
    assert cv2.imshow.call_count == 6  # Annotated frame and five diagnostic masks.
    cv2.destroyAllWindows.assert_called_once()
