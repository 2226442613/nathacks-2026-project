"""iPhone-ready CLI integration is simulated through the existing CameraManager, never real hardware."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import BackgroundReference, GroundCalibration, DualPlaneCalibration
from vision.spill_cli import calibration_diagnostics


@pytest.fixture
def headless(monkeypatch):
    for name in ('imshow','waitKey','namedWindow','destroyAllWindows'):
        monkeypatch.setattr(cv2,name,Mock(side_effect=AssertionError('Headless must not call HighGUI')))


def test_json_and_overlay_report_actual_fps_resolution_counts_and_weak_evidence(tmp_path,monkeypatch,headless):
    import vision.spill_cli as cli
    clean = np.full((320,400,3),200,np.uint8)
    frame = clean.copy()
    for center,color in [((80,150),(20,20,220)),((250,150),(20,180,180)),((250,260),(40,40,40))]:
        cv2.circle(frame,center,20,color,-1)
    background, image, output = (tmp_path/name for name in ('ref.png','image.png','results.jsonl'))
    BackgroundReference(clean,confirmed_clean=True).save(background)
    cv2.imwrite(str(image),frame)
    ticks = iter([10.0,10.25])
    monkeypatch.setattr(cli,'time',SimpleNamespace(perf_counter=lambda:next(ticks)))
    texts = []
    original = cv2.putText
    def capture_text(image,text,*args,**kwargs):
        texts.append(text)
        return original(image,text,*args,**kwargs)
    monkeypatch.setattr(cv2,'putText',capture_text)
    assert main.main(['detect-spills','--image',str(image),'--background',str(background),
        '--confirm-clean','--headless','--debug-dir',str(tmp_path/'debug'),'--output',str(output)]) == 0
    record = json.loads(output.read_text())
    diag = record['diagnostics']
    assert diag['frame_resolution'] == {'width':400,'height':320}
    assert diag['processing_fps'] == pytest.approx(4.0)  # Requested capture FPS is 30.
    assert diag['background']['status'] == 'READY'
    assert diag['background']['width'] == 400
    assert diag['calibration']['status'] == 'NOT_LOADED'
    assert diag['target_count'] == 3 and diag['confirmed_count'] == 0 and diag['pending_count'] == 3
    assert diag['category_counts'] == {'RED_SPILL':1,'NON_RED_SPILL':2,'UNKNOWN':0}
    assert diag['weak_evidence_count'] == 0
    assert diag['input_mode'] == 'IMAGE'
    assert record['geometry']['status'] == 'UNVERIFIED'
    assert any('400x320' in text and '4.0' in text for text in texts)
    assert any('BG READY' in text and 'CAL NONE' in text for text in texts)
    assert any('confirmed 0' in text and 'pending 3' in text for text in texts)
    assert any('RED 1' in text and 'UNKNOWN 0' in text for text in texts)


def test_calibration_status_exposes_accepted_warnings_without_changing_geometry():
    floor = GroundCalibration([[200,200],[400,200],[400,400],[200,400]],108.5,43,600,600)
    assert calibration_diagnostics(floor,roi_enabled=True)['status'] == 'FLOOR_ONLY'
    dual = DualPlaneCalibration.from_floor_and_wall_top(floor,[[196,96],[400,104]],108.5,29,'top')
    dual.accept_quality_warnings()
    before = dual.to_dict()
    diag = calibration_diagnostics(dual,roi_enabled=True)
    assert diag['status'] == 'DUAL_PLANE_WITH_WARNINGS'
    assert diag['quality_warnings'] and diag['warnings_accepted']
    assert diag['roi_mode'] == 'CALIBRATED'
    assert calibration_diagnostics(dual,roi_enabled=False)['roi_mode'] == 'FULL_IMAGE'
    assert dual.to_dict() == before


def fake_capture(monkeypatch,reads,opened=True):
    import vision.camera as camera_module
    capture = Mock()
    capture.isOpened.return_value = opened
    capture.set.return_value = True
    capture.get.side_effect = lambda prop: {cv2.CAP_PROP_FRAME_WIDTH:1280,
        cv2.CAP_PROP_FRAME_HEIGHT:720,cv2.CAP_PROP_FPS:30}.get(prop,0)
    capture.read.side_effect = reads
    factory = Mock(return_value=capture)
    monkeypatch.setattr(cv2,'VideoCapture',factory)
    monkeypatch.setattr(camera_module.platform,'system',lambda:'Darwin')
    return capture,factory


def test_live_pipeline_uses_existing_avfoundation_manager_tracks_and_reports_disconnection(tmp_path,monkeypatch,headless,caplog):
    clean = np.full((720,1280,3),200,np.uint8)
    spill = clean.copy()
    cv2.circle(spill,(200,450),30,(20,20,220),-1)
    background, config, output = (tmp_path/name for name in ('ref.png','camera.json','live.jsonl'))
    BackgroundReference(clean,confirmed_clean=True).save(background)
    floor = GroundCalibration([[50,300],[1200,300],[1200,700],[50,700]],108.5,43,1280,720,source=3)
    DualPlaneCalibration.from_floor_and_wall_top(floor,[[50,50],[1200,50]],108.5,29,'top').save(config)
    before = config.read_bytes()
    capture,factory = fake_capture(monkeypatch,[(True,clean),(True,spill),(True,spill),(True,spill),(True,clean),(False,None)])
    assert main.main(['detect-spills','--camera','3','--background',str(background),
        '--confirm-clean','--calibration',str(config),'--headless','--output',str(output)]) == 1
    factory.assert_called_once_with(3,cv2.CAP_AVFOUNDATION)
    capture.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH,1280)
    capture.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT,720)
    capture.release.assert_called_once()
    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert len(records) == 5
    assert records[0]['diagnostics']['target_count'] == 0
    assert records[3]['diagnostics']['confirmed_count'] == 1
    assert records[-1]['diagnostics']['target_count'] == 0  # Removal is not a camera error or UNKNOWN.
    assert records[3]['diagnostics']['calibration']['status'] == 'DUAL_PLANE'
    assert records[3]['diagnostics']['source'] == 3
    assert {record['detections'][0]['target_id'] for record in records[1:4]} == {1}
    assert records[3]['diagnostics']['category_counts']['RED_SPILL'] == 1
    assert 'frame read failed' in caplog.text and 'Continuity Camera' in caplog.text
    assert config.read_bytes() == before


def test_live_reference_resolution_mismatch_is_error_not_unknown(tmp_path,monkeypatch,headless,caplog):
    reference = tmp_path/'small.png'
    BackgroundReference(np.full((320,400,3),200,np.uint8),confirmed_clean=True).save(reference)
    capture,_factory = fake_capture(monkeypatch,[(True,np.full((720,1280,3),200,np.uint8))])
    output = tmp_path/'live.jsonl'
    assert main.main(['detect-spills','--camera','3','--background',str(reference),
        '--confirm-clean','--headless','--output',str(output)]) == 1
    assert 'Frame resolution 1280x720 differs from background 400x320' in caplog.text
    assert output.read_text() == ''
    capture.release.assert_called_once()


def test_live_initialization_failure_is_clear_and_releases_backends(tmp_path,monkeypatch,headless,caplog):
    reference = tmp_path/'reference.png'
    BackgroundReference(np.full((720,1280,3),200,np.uint8),confirmed_clean=True).save(reference)
    capture,factory = fake_capture(monkeypatch,[],opened=False)
    assert main.main(['detect-spills','--camera','3','--background',str(reference),
        '--confirm-clean','--headless','--output',str(tmp_path/'empty.jsonl')]) == 1
    assert [call.args for call in factory.call_args_list] == [(3,cv2.CAP_AVFOUNDATION),(3,cv2.CAP_ANY)]
    assert capture.release.call_count == 2
    assert 'Cannot open 3' in caplog.text and 'unavailable' in caplog.text
