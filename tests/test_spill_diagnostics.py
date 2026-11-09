"""Production-path diagnostics and image-only alignment; no physical accuracy claims."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import BackgroundReference
from vision.frame_geometry import FrameGeometryGuard
from vision.spill_detection import DetectorConfig, SpillDetector
from vision.spill_tracking import SpillTracker
from tools.analyze_spill_diagnostics import summarize


def test_diagnostic_collection_preserves_outputs_sources_and_lossless_pixels(tmp_path,monkeypatch):
    monkeypatch.setattr('vision.spill_tracking.time.time',lambda:1794189600.)
    source = Path('data/module03_demo')
    paths = [source/'baseline.png',source/'baseline.png.json',source/'calibration.json',Path('config/spill_config.json')]
    before = [p.read_bytes() for p in paths]
    args = ['detect-spills','--video',str(source/'targets.avi'),'--background',str(paths[0]),
            '--calibration',str(paths[2]),'--confirm-clean','--headless']
    ordinary,observed = tmp_path/'ordinary.jsonl',tmp_path/'observed.jsonl'
    directory = tmp_path/'run'
    assert main.main(args+['--output',str(ordinary)]) == 0
    assert main.main(args+['--output',str(observed),'--diagnostic-dir',str(directory),'--diagnostic-panels']) == 0
    load = lambda p:[json.loads(line) for line in p.read_text().splitlines()]
    assert [r['detections'] for r in load(ordinary)] == [r['detections'] for r in load(observed)]
    assert [p.read_bytes() for p in paths] == before
    manifest = json.loads((directory/'manifest.json').read_text())
    assert manifest['status']=='COMPLETE' and manifest['frames']==16
    from dataclasses import asdict
    assert manifest['detector']==asdict(DetectorConfig.from_dict(json.loads(Path('config/spill_config.json').read_text())['detector']))
    assert (directory/'reference.png').read_bytes()==paths[0].read_bytes()
    assert (directory/'calibration.json').read_bytes()==paths[2].read_bytes()
    assert manifest['source_sha256'][str(paths[0])]==hashlib.sha256(before[0]).hexdigest()
    assert load(directory/'frames.jsonl')[0]['metrics']['component_count']==0
    assert cv2.imread(str(directory/'frame_000005_panels.png')).shape==(798,1278,3)
    assert (directory/'frame_000005_expected_background.png').is_file()
    assert all('contour' in c for c in load(directory/'frames.jsonl')[5]['metrics']['components'])
    if manifest['lossless_video_available']:
        cap = cv2.VideoCapture(str(directory/'capture.avi'))
        try:
            for index in range(16):
                ok,frame = cap.read()
                assert ok
                assert np.array_equal(frame,cv2.imread(str(directory/f'frame_{index:06d}_input.png')))
        finally:
            cap.release()
    # Existing run directories and input paths cannot be overwritten.
    assert main.main(args+['--diagnostic-dir',str(directory),'--output',str(tmp_path/'retry.jsonl')])==1
    assert [p.read_bytes() for p in paths] == before


def test_panels_require_diagnostic_directory(tmp_path):
    assert main.main(['detect-spills','--image','missing.png','--background','missing.png',
                      '--confirm-clean','--diagnostic-panels','--headless'])==1


def test_diagnostic_gui_cannot_reregister_background(tmp_path,monkeypatch):
    import vision.spill_cli as cli
    clean = np.full((180,240,3),180,np.uint8)
    bg = tmp_path/'baseline.png'
    BackgroundReference(clean,confirmed_clean=True).save(bg)
    before = bg.read_bytes(),Path(str(bg)+'.json').read_bytes()
    camera = Mock()
    camera.__enter__=Mock(return_value=camera)
    camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=[clean.copy() for _ in range(5)]
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    monkeypatch.setattr(cv2,'imshow',Mock())
    monkeypatch.setattr(cv2,'getWindowProperty',lambda *args:1)
    monkeypatch.setattr(cv2,'waitKey',Mock(side_effect=[ord('r'),ord('y'),ord('a'),ord('b'),ord('q')]))
    monkeypatch.setattr(cv2,'destroyAllWindows',Mock())
    assert main.main(['detect-spills','--camera','0','--background',str(bg),'--confirm-clean',
                      '--diagnostic-dir',str(tmp_path/'diagnostic'),'--output',str(tmp_path/'output.jsonl')])==0
    assert (bg.read_bytes(),Path(str(bg)+'.json').read_bytes())==before
    rows=[json.loads(line) for line in (tmp_path/'diagnostic/frames.jsonl').read_text().splitlines()]
    assert [r['operator_stage'] for r in rows]==[None,None,None,'A','B']
    assert json.loads((tmp_path/'diagnostic/manifest.json').read_text())['stage_markers']==[
        {'stage':'A','start_frame':3},{'stage':'B','start_frame':4}]


def test_distributed_image_fit_outside_trust_limits_preserves_real_targets():
    clean = np.random.default_rng(775).integers(45,200,(360,480,3),dtype=np.uint8)
    matrix = cv2.getRotationMatrix2D((240,180),.2,1.035)
    matrix[:,2] += [11,0]
    moved = cv2.warpAffine(clean,matrix,(480,360))
    reference = BackgroundReference(clean,confirmed_clean=True)
    detector = SpillDetector(reference)
    empty = detector.detect(moved)
    assert empty.geometry.status=='MOVED' and empty.alignment_applied
    assert empty.reference_to_current is None
    assert empty.background_alignment_transform is not None
    assert empty.detections==[]
    scene = moved.copy()
    cv2.circle(scene,(140,270),18,(20,20,230),-1)
    cv2.circle(scene,(320,270),18,(230,20,20),-1)
    tracker = SpillTracker()
    observed = [tracker.update(detector.detect(scene).detections) for _ in range(5)]
    assert {t.category.value for t in observed[-1]}=={'RED_SPILL','NON_RED_SPILL'}
    assert all(t.confirmed for t in observed[-1])
    assert [[t.target_id for t in row] for row in observed]==[[1,2]]*5
    assert tracker.update(detector.detect(moved).detections)==[]
    # Existing opt-out still leaves image alignment disabled.
    disabled = SpillDetector(reference,replace(DetectorConfig(),alignment_enabled=False)).detect(moved)
    assert not disabled.alignment_applied


def test_guard_never_exports_large_image_fit_as_physical_transform():
    image = np.random.default_rng(125).integers(45,200,(360,480,3),dtype=np.uint8)
    guard = FrameGeometryGuard(image)
    moved = cv2.warpAffine(image,np.float32([[1,0,15],[0,1,0]]),(480,360))
    assert guard.check(moved).status=='MOVED'
    assert guard.transform is None and guard.image_alignment_transform is not None
    assert guard.check(np.zeros_like(image)).status=='UNVERIFIED'
    assert guard.transform is None and guard.image_alignment_transform is None


def test_label_evaluator_reports_fp_delay_accuracy_duplicates_and_disappearance(tmp_path):
    def target(id,category='RED_SPILL'):
        return {'target_id':id,'category':category,'bounding_box':[10,10,20,20],'confirmed':True}
    rows=[]
    for i,targets in enumerate([[],[],[target(1)],[target(1),target(2,'NON_RED_SPILL')],[]]):
        rows.append({'frame_index':i,'elapsed_seconds':i*.1,'metrics':{'candidate_count':len(targets),
            'confirmed_count':len(targets),'physical_registration':{'status':'MATCHED'},
            'background_alignment_applied':False},'record':{'detections':targets}})
    (tmp_path/'frames.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    assert summarize(tmp_path)['false_positives'] is None
    labels={'schema_version':1,'coordinate_system':'original_image_pixels','segments':[
        {'stage':'A','start_frame':0,'end_frame':0,'objects':[]},
        {'stage':'B','start_frame':1,'end_frame':3,'objects':[
            {'id':'red-1','category':'RED_SPILL','bounding_box':[10,10,20,20]}]},
        {'stage':'C','start_frame':4,'end_frame':4,'objects':[]}]}
    report=summarize(tmp_path,labels)
    assert report['false_positives']==1
    assert report['classification_accuracy']==.5
    assert report['objects'][0]['delay_frames']==1
    assert report['objects'][0]['delay_seconds']==pytest.approx(.1)
    assert report['objects'][0]['duplicate_observations']==1
    assert report['disappearance'][-1]['first_zero_confirmed_frame']==4
    assert report['missing_stages']==['D','E','F']


def test_optional_actual_empty_arena_recording_regression():
    source=Path('data/module02_field_evidence/prechange_camera_attempt')
    if not source.is_dir():
        pytest.skip('Private real-camera recording is not distributed in Git')
    # Replay against the recording's immutable reference. Operators may register
    # a new live reference without invalidating this historical regression.
    recorded=Path('data/module02_field_evidence/after_empty')
    background=recorded/'reference.png'
    if not background.is_file():
        pytest.skip('Private recording reference is not distributed in Git')
    if hashlib.sha256(background.read_bytes()).hexdigest()!='f98dabf8ab08ec08b53c4613fe724d1cbbf30d68e1ec935bfa8de2e09700882f':
        pytest.skip('Operator has registered a different baseline')
    from vision.calibration import load_calibration
    from vision.spill_tracking import load_spill_settings
    dc,tc=load_spill_settings(recorded/'settings.json')
    detector=SpillDetector(BackgroundReference.load(background,confirmed_clean=True),dc,load_calibration(recorded/'calibration.json'))
    tracker=SpillTracker(tc)
    counts=[]
    for path in sorted(source.glob('*_input.png')):
        result=detector.detect(cv2.imread(str(path)))
        counts.append(sum(t.confirmed for t in tracker.update(result.detections)))
        assert result.reference_to_current is None
    assert counts==[0,0,0,0,1,1]  # Residual is deliberately not hidden with size/threshold changes.


def test_optional_recorded_real_red_blue_sequence_keeps_classes_and_ids():
    directory=Path('data/module02_field_evidence/live_sequence_01')
    labels=Path('data/module02_field_evidence/live_sequence_labels.json')
    if not directory.is_dir() or not labels.is_file():
        pytest.skip('Private operator-labeled physical-camera sequence is not distributed')
    from vision.calibration import load_calibration
    from vision.spill_tracking import load_spill_settings
    from tools.analyze_spill_diagnostics import iou
    dc,tc=load_spill_settings(directory/'settings.json')
    detector=SpillDetector(BackgroundReference.load(directory/'reference.png',confirmed_clean=True),
                           dc,load_calibration(directory/'calibration.json'))
    tracker=SpillTracker(tc)
    identities={}
    for segment in json.loads(labels.read_text())['segments']:
        for index in range(segment['start_frame'],segment['start_frame']+6):
            frame=cv2.imread(str(directory/f'frame_{index:06d}_input.png'))
            targets=tracker.update(detector.detect(frame).detections)
        confirmed=[target for target in targets if target.confirmed]
        if segment['stage']=='C':
            assert all(target.category.value!='RED_SPILL' for target in confirmed)
        for obj in segment['objects']:
            matched=[target for target in confirmed if iou(obj['bounding_box'],target.bounding_box)>=.25]
            assert len(matched)==1
            target=matched[0]
            assert target.category.value==obj['category']
            assert identities.setdefault(obj['id'],target.target_id)==target.target_id
