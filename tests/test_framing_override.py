"""Operator-managed framing policy: warnings never pause; R/Y is still explicit."""
import json
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import BackgroundReference, GroundCalibration, DualPlaneCalibration
from vision.framing_session import FramingSession
from vision.spill_detection import SpillDetector
from vision.spill_tracking import SpillTracker


@pytest.fixture
def live(tmp_path,monkeypatch):
    import vision.spill_cli as cli
    clean = np.random.default_rng(35).integers(40,185,(360,480,3),dtype=np.uint8)
    moved = cv2.warpAffine(clean,np.float32([[1,0,18],[0,1,12]]),(480,360))
    baseline, calibration = tmp_path/'baseline.png',tmp_path/'calibration.json'
    BackgroundReference(clean,confirmed_clean=True).save(baseline)
    floor = GroundCalibration([[10,180],[470,180],[470,350],[10,350]],108.5,43,480,360)
    DualPlaneCalibration.from_floor_and_wall_top(floor,[[10,10],[470,10]],108.5,29,'top').save(calibration)
    camera = Mock()
    camera.__enter__ = Mock(return_value=camera)
    camera.__exit__ = Mock(return_value=False)
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    show, close, visible = Mock(),Mock(),Mock(return_value=1)
    monkeypatch.setattr(cv2,'imshow',show)
    monkeypatch.setattr(cv2,'destroyAllWindows',close)
    monkeypatch.setattr(cv2,'getWindowProperty',visible)
    texts = []
    original = cv2.putText
    def capture_text(image,text,*args,**kwargs):
        texts.append(text)
        return original(image,text,*args,**kwargs)
    monkeypatch.setattr(cv2,'putText',capture_text)
    def run(keys,frames=None,extra=()):
        frames = frames if frames is not None else [moved]*len(keys)
        camera.read.side_effect = frames
        wait = Mock(side_effect=keys)
        monkeypatch.setattr(cv2,'waitKey',wait)
        output = tmp_path/f'results_{camera.__enter__.call_count}.jsonl'
        code = main.main(['detect-spills','--camera','0','--background',str(baseline),
            '--calibration',str(calibration),'--confirm-clean','--reregister-frames','3',
            '--output',str(output),*extra])
        records = [json.loads(line) for line in output.read_text().splitlines()] if output.exists() else []
        return code,records,wait
    return dict(clean=clean,moved=moved,baseline=baseline,calibration=calibration,
                camera=camera,show=show,close=close,texts=texts,run=run)


def pair_bytes(path):
    return path.read_bytes(),Path(str(path)+'.json').read_bytes()


def test_mismatch_keeps_live_window_responsive_and_quit_preserves_files(live):
    baseline,cal = pair_bytes(live['baseline']),live['calibration'].read_bytes()
    code,records,wait = live['run']([-1,-1,ord('q')])
    assert code == 0
    assert len(records) == wait.call_count == live['show'].call_count == 3
    assert all(not r['detections'] for r in records)  # Aligned empty texture is not a target.
    assert all(r['diagnostics']['alignment_applied'] for r in records)
    assert all(r['diagnostics']['framing']['state'] == 'NORMAL' for r in records)
    assert all(not r['diagnostics']['detection_paused'] for r in records)
    assert pair_bytes(live['baseline']) == baseline and live['calibration'].read_bytes() == cal
    assert any('UNVERIFIED' in s for s in live['texts'])
    assert not any('WAITING_FOR_OPERATOR' in s or 'OVERRIDE_ACTIVE' in s or 'CALIBRATION MAY BE INVALID' in s for s in live['texts'])
    live['camera'].__exit__.assert_called_once()
    live['close'].assert_called_once()


def test_c_is_optional_noop_and_warning_does_not_reset_target_ids(live):
    baseline,cal = pair_bytes(live['baseline']),live['calibration'].read_bytes()
    scene = live['moved'].copy()
    cv2.circle(scene,(250,250),18,(20,20,230),-1)
    code,records,_ = live['run']([-1,ord('c'),ord('c'),-1,ord('q')],
        frames=[scene]*5,extra=('--confirmation-frames','1'))
    assert code == 0
    assert all(r['diagnostics']['framing']['state']=='NORMAL' for r in records)
    assert all(not r['diagnostics']['detection_paused'] for r in records)
    assert all(r['geometry']['status']=='MOVED' for r in records)
    assert all(len(r['detections'])==1 and r['detections'][0]['category']=='RED_SPILL' for r in records)
    assert records[0]['detections'][0]['target_id']==records[-1]['detections'][0]['target_id']
    assert all(not r['diagnostics']['calibration']['navigation_allowed'] for r in records)
    assert pair_bytes(live['baseline'])==baseline and live['calibration'].read_bytes()==cal
    events=[e for r in records for e in r['diagnostics']['framing']['events']]
    assert events.count('FRAMING_WARNING_NONBLOCKING')==1
    assert 'FRAMING_OVERRIDE_ACCEPTED' not in events


def test_r_requires_different_confirmation_key_and_cancel_never_writes(live):
    before = pair_bytes(live['baseline'])
    code,records,_ = live['run']([ord('r'),ord('r'),ord('n'),ord('q')])
    assert code == 0 and pair_bytes(live['baseline']) == before
    assert all(not r['diagnostics']['detection_paused'] for r in records)
    assert records[1]['diagnostics']['framing']['confirming_baseline_registration']
    assert any('ALL currently visible objects' in s for s in live['texts'])
    assert any('Y: confirm capture' in s for s in live['texts'])


def test_r_y_captures_median_persists_resumes_and_clears_old_ids(live,monkeypatch):
    import vision.spill_cli as cli
    before = live['calibration'].read_bytes()
    original = live['clean'].copy()
    cv2.circle(original,(140,250),18,(10,10,230),-1)
    moved = cv2.warpAffine(original,np.float32([[1,0,18],[0,1,12]]),(480,360))
    new = moved.copy()
    cv2.circle(new,(350,250),18,(230,30,20),-1)
    tracker = SpillTracker()
    reset = Mock(wraps=tracker.reset)
    tracker.reset = reset
    monkeypatch.setattr(cli,'SpillTracker',lambda config: tracker)
    frames = [original]*3+[moved]*5+[new]
    keys = [-1,-1,-1,ord('r'),ord('y'),-1,-1,-1,ord('q')]
    code,records,wait = live['run'](keys,frames,extra=('--confirmation-frames','1'))
    assert code == 0 and len(records) == wait.call_count == 9
    assert records[2]['detections'][0]['confirmed']
    assert all(not r['detections'] for r in records[5:8])  # Only explicit capture pauses.
    assert records[-1]['diagnostics']['framing']['state'] == 'NORMAL'
    assert records[-1]['geometry']['status'] == 'MATCHED'
    assert len(records[-1]['detections']) == 1
    assert records[-1]['detections'][0]['target_id'] == 1
    assert records[-1]['detections'][0]['category'] == 'NON_RED_SPILL'
    reset.assert_called()
    saved = BackgroundReference.load(live['baseline'],confirmed_clean=True)
    assert np.array_equal(saved.image,moved)
    assert saved.metadata['aggregation_method'] == 'median' and saved.metadata['frame_count'] == 3
    assert saved.metadata['registration_trigger'] == 'interactive_R_then_Y'
    assert saved.metadata['calibration_may_be_invalid'] is True
    assert live['calibration'].read_bytes() == before
    assert records[-1]['diagnostics']['calibration']['physical_coordinate_status'] == 'UNVERIFIED_AFTER_FRAMING_CHANGE'
    # Matching the new baseline on another startup must not validate old calibration.
    code,restarted,_ = live['run']([ord('q')],frames=[moved])
    assert code == 0 and restarted[0]['diagnostics']['framing']['calibration_may_be_invalid']


def test_quit_during_capture_does_not_write_partial_reference(live):
    before = pair_bytes(live['baseline'])
    code,records,wait = live['run']([ord('r'),ord('y'),ord('q')])
    assert code == 0 and wait.call_count == 3
    assert records[-1]['diagnostics']['framing']['state'] == 'BASELINE_REREGISTERING'
    assert pair_bytes(live['baseline']) == before
    live['camera'].__exit__.assert_called_once()


def test_optional_c_does_not_enable_override_on_next_session(live):
    assert live['run']([ord('c'),ord('q')])[0] == 0
    code,records,_ = live['run']([ord('q')])
    assert code == 0
    assert records[0]['diagnostics']['framing']['state'] == 'NORMAL'
    assert not records[0]['diagnostics']['framing']['override_active']


def test_cancel_registration_resumes_nonblocking_operation():
    session = FramingSession()
    session.mismatch('Moved')
    assert session.key(ord('c')) is None
    assert session.key(ord('r')) == 'register'
    assert session.key(ord('n')) == 'cancel'
    assert not session.override_active and session.state == 'NORMAL'


def test_repeated_keys_cannot_confirm_or_restart_capture():
    session = FramingSession(capture_count=3)
    session.mismatch('Moved')
    session.key(ord('r'))
    for _ in range(5):
        session.key(ord('r'))
    assert session.confirming_registration and session.frames == []
    assert session.key(ord('y')) == 'confirm_registration'
    for key in (ord('y'),ord('r'),ord('c'),-1,ord('y')):
        assert session.key(key) is None
    assert session.state == 'BASELINE_REREGISTERING'


def test_detector_mismatch_is_nonblocking_but_resolution_is_invalid(live):
    detector = SpillDetector(BackgroundReference.load(live['baseline'],confirmed_clean=True))
    normal = detector.detect(live['moved'])
    assert normal.geometry.status=='MOVED' and not normal.detection_paused
    result = detector.detect(live['moved'],framing_override=True)
    assert result.geometry.status == 'MOVED' and not result.detection_paused
    with pytest.raises(ValueError,match='resolution'):
        detector.detect(live['moved'][:200],framing_override=True)


def test_exceeded_small_shift_limit_keeps_warning_without_prompt(live):
    moved = cv2.warpAffine(live['clean'],np.float32([[1,0,4],[0,1,0]]),(480,360))
    code,records,_ = live['run']([ord('c'),ord('q')],frames=[moved,moved])
    assert code == 0 and not records[0]['diagnostics']['detection_paused']
    assert records[0]['diagnostics']['framing']['state'] == 'NORMAL'
    assert 'safe small-shift alignment' in records[0]['diagnostics']['framing']['reason']
    assert records[-1]['diagnostics']['framing']['state'] == 'NORMAL'
    assert all(record['geometry']['status'] == 'MOVED' for record in records)


@pytest.mark.parametrize('stage', ['encode','stage','metadata_replace','png_replace'])
def test_atomic_save_failure_preserves_reference_pair(live,monkeypatch,stage):
    import vision.reference_store as store
    before = pair_bytes(live['baseline'])
    fresh = BackgroundReference(live['moved'],confirmed_clean=True)
    if stage == 'encode':
        monkeypatch.setattr(store.cv2,'imencode',Mock(return_value=(False,None)))
    elif stage == 'stage':
        original = store._stage
        calls = 0
        def failing_stage(path,data):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise OSError('injected staging failure')
            return original(path,data)
        monkeypatch.setattr(store,'_stage',failing_stage)
    else:
        original = store.os.replace
        destination = live['baseline'] if stage == 'png_replace' else Path(str(live['baseline'])+'.json')
        failed = False
        def failing_replace(source,target):
            nonlocal failed
            if Path(target) == destination and not failed:
                failed = True
                raise OSError('injected commit failure')
            return original(source,target)
        monkeypatch.setattr(store.os,'replace',failing_replace)
    with pytest.raises(OSError):
        fresh.save(live['baseline'],overwrite=True)
    assert pair_bytes(live['baseline']) == before
    assert np.array_equal(BackgroundReference.load(live['baseline'],confirmed_clean=True).image,live['clean'])
    assert list(live['baseline'].parent.glob('.*')) == []


def test_reregistration_io_failure_resumes_without_losing_reference(live,monkeypatch,caplog):
    import vision.reference_store as store
    before = pair_bytes(live['baseline'])
    monkeypatch.setattr(store.cv2,'imencode',Mock(return_value=(False,None)))
    code,records,_ = live['run']([ord('r'),ord('y'),-1,-1,-1,ord('q')])
    assert code == 0
    assert records[-1]['diagnostics']['framing']['state'] == 'NORMAL'
    assert pair_bytes(live['baseline']) == before
    assert 'Interactive baseline replacement failed' in caplog.text
    assert any(record.exc_info for record in caplog.records)


def test_interrupted_pair_checksum_is_rejected(live):
    cv2.imwrite(str(live['baseline']),live['moved'])
    with pytest.raises(ValueError,match='checksum mismatch'):
        BackgroundReference.load(live['baseline'],confirmed_clean=True)


@pytest.mark.parametrize('value', ['1','32'])
def test_invalid_capture_length_is_rejected_before_camera_open(live,value):
    code,records,_ = live['run']([ord('q')],extra=('--reregister-frames',value))
    assert code == 1
    live['camera'].__enter__.assert_not_called()
