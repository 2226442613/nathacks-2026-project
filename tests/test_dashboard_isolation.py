"""Replay the same recorded pixels through test-vision and Dashboard LIVE."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

import main
import dashboard.live as live
from vision.calibration import load_calibration
from vision.localization import SurfaceLocalizer


@pytest.mark.parametrize('variant', ['normal', 'calibration_source', 'baseline_source',
                                     'unverified_baseline', 'legacy_baseline', 'recorded_camera'])
def test_recorded_cli_and_live_outputs_match_before_presentation(tmp_path, monkeypatch, variant):
    source = Path('data/module03_demo')
    baseline = tmp_path/'baseline.png'
    calibration = tmp_path/'calibration.json'
    real_camera = variant == 'recorded_camera'
    original_baseline = Path('data/iphone_reference.png') if real_camera else source/'baseline.png'
    original_calibration = Path('config/camera_config.json') if real_camera else source/'calibration.json'
    shutil.copyfile(original_baseline, baseline)
    shutil.copyfile(original_calibration, calibration)
    metadata = json.loads(Path(str(original_baseline)+'.json').read_text())
    if variant == 'baseline_source':
        metadata['source'] = 7
    if variant == 'unverified_baseline':
        metadata['calibration_may_be_invalid'] = True
    if variant != 'legacy_baseline':
        Path(str(baseline)+'.json').write_text(json.dumps(metadata))
    camera_index = 7 if variant == 'calibration_source' else 0

    capture = cv2.VideoCapture(str(source/'targets.avi'))
    frames = []
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frames.append(frame)
    finally:
        capture.release()
    assert len(frames) == 16
    if real_camera:
        paths = sorted(Path('data/iphone_debug').glob('*_input.png'))[:8]
        if not paths:
            pytest.skip('Optional saved physical-camera frames are not available')
        frames = [cv2.imread(str(path)) for path in paths]
        assert all(frame is not None for frame in frames)

    class ReplayCamera:
        def __init__(self, *args):
            self.frames = iter(frames)
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def read(self):
            frame = next(self.frames, None)
            return frame.copy() if frame is not None else None

    # Wall-clock time and producer-session IDs are not detection differences.
    import vision.live_test as cli
    import vision.localization as localization
    import vision.localization_runtime as runtime
    fixed = 1794189600.
    utc = localization.utc_timestamp
    monkeypatch.setattr('vision.spill_tracking.time.time', lambda: fixed)
    monkeypatch.setattr(localization, 'utc_timestamp', lambda value=None: utc(fixed if value is None else value))
    monkeypatch.setattr(runtime, 'utc_timestamp', lambda: utc(fixed))
    monkeypatch.setattr(cli.time, 'monotonic', lambda: fixed)
    monkeypatch.setattr(cli, 'CameraManager', ReplayCamera)
    monkeypatch.setattr(live, 'CameraManager', ReplayCamera)
    protected = [baseline, calibration, Path('config/spill_config.json')]
    before = [p.read_bytes() for p in protected]
    records, images = [], []
    process = cli.LiveVision.process
    def observed(self, frame, **kwargs):
        image, summary = process(self, frame, **kwargs)
        records.append([r.to_dict() for r in self.records])
        return image, summary
    monkeypatch.setattr(cli.LiveVision, 'process', observed)
    monkeypatch.setattr(cv2, 'namedWindow', lambda *args: None)
    monkeypatch.setattr(cv2, 'imshow', lambda name, image: images.append(image.copy()))
    keys = iter([-1]*(len(frames)-1)+[ord('q')])
    monkeypatch.setattr(cv2, 'waitKey', lambda *args: next(keys))
    monkeypatch.setattr(cv2, 'getWindowProperty', lambda *args: 1)
    monkeypatch.setattr(cv2, 'destroyAllWindows', lambda: None)
    assert main.main(['test-vision', '--camera', str(camera_index), '--background', str(baseline),
                      '--calibration', str(calibration)]) == 0
    standalone = deepcopy(records);records.clear()
    received, live_images, errors = [], [], []
    def receive(image, events, **kwargs):
        live_images.append(image.copy());received.append(deepcopy(events))
    service = SimpleNamespace(calibration=load_calibration(calibration),calibration_path=str(calibration),
        calibration_id=SurfaceLocalizer(load_calibration(calibration)).calibration_id,
        live_frame=receive,
        live_unavailable=errors.append)
    height, width = frames[0].shape[:2]
    monitor = live.LiveMonitor(service, camera=camera_index, width=width, height=height, background=str(baseline))
    def wait(timeout):
        if len(received)==len(frames):monitor.stop_event.set()
        return monitor.stop_event.is_set()
    monkeypatch.setattr(monitor.stop_event, 'wait', wait)
    monitor.run()
    assert errors==['Live monitoring stopped']
    assert len(received) == len(standalone) == len(records) == len(images) == len(live_images)
    assert records==standalone
    assert all(np.array_equal(a,b) for a,b in zip(images,live_images))
    for actual, expected in zip(received, standalone):
        assert [event['localization'] for event in actual if event['active']] == expected
    observations = [record for frame in standalone for record in frame]
    if not real_camera:
        assert {record['category'] for record in observations} == {'RED_SPILL', 'NON_RED_SPILL'}
        assert {record['surface'] for record in observations} == {'FLOOR', 'WALL', 'BOUNDARY'}
        for surface in ('FLOOR', 'WALL', 'BOUNDARY'):
            assert len({r['target_id'] for r in observations if r['surface'] == surface}) == 1
    assert [p.read_bytes() for p in protected] == before


def test_original_cli_does_not_require_web_dependencies():
    script = '''
import importlib.abc
import sys
class NoWeb(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'flask', 'werkzeug'} or fullname in {'dashboard.app', 'dashboard.service', 'dashboard.live'}:
            raise ImportError('Standalone CLI must not load web runtime: '+fullname)
sys.meta_path.insert(0, NoWeb())
import main
parser = main.build_parser()
expected = {'preview', 'calibrate', 'verify-calibration', 'background-reference', 'detect-spills',
            'validate-localization', 'generate-aruco', 'track-robot', 'calibrate-robot-plane',
            'calibrate-robot-interactive', 'plan-cleaning', 'robot-simulator', 'robot-client', 'dashboard'}
assert expected <= set(parser._subparsers._group_actions[0].choices)
args = parser.parse_args(['detect-spills', '--camera', '0', '--background', 'baseline.png',
                          '--confirm-clean', '--localize', '--localization-boundaries'])
assert args.localize and args.localization_boundaries
'''
    result = subprocess.run([sys.executable, '-c', script], text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
