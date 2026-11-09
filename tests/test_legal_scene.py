"""Deterministic legal-baseline acceptance cases; no physical camera claims."""
from dataclasses import replace
import hashlib
import json
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import BackgroundReference, GroundCalibration, DualPlaneCalibration
from vision.frame_geometry import GeometryCheck
from vision.spill_detection import DetectorConfig, SpillDetector, SpillCategory, render_detections
from vision.spill_tracking import SpillTracker, TrackerConfig


@pytest.fixture
def cardboard():
    image = np.full((320,400,3),(95,145,185),np.uint8)
    cv2.rectangle(image,(35,35),(95,85),(15,15,220),-1)  # Legal red logo.
    for y in (40,120,210,290):
        cv2.putText(image,'LEGAL 42',(110,y),0,.5,(30,30,30),1)
    cv2.line(image,(15,180),(380,165),(65,90,110),2)  # Fold.
    cv2.rectangle(image,(310,50),(325,270),(165,185,200),-1)  # Tape.
    return image


def added(image, center=(85,240), color=(15,15,230), radius=19):
    frame = image.copy()
    cv2.circle(frame,center,radius,color,-1)
    return frame


def dual_planes():
    floor = GroundCalibration([[10,150],[390,150],[390,310],[10,310]],108.5,43,400,320)
    return DualPlaneCalibration.from_floor_and_wall_top(floor,[[10,10],[390,10]],108.5,29,'top')


def test_registered_cardboard_logo_text_tape_folds_all_legal(cardboard):
    detector = SpillDetector(BackgroundReference(cardboard,confirmed_clean=True))
    tracker = SpillTracker()
    for _ in range(8):
        result = detector.detect(cardboard)
        assert result.geometry.status == 'MATCHED'
        assert result.detections == tracker.update(result.detections) == []
        assert not result.foreground_mask.any()


@pytest.mark.parametrize('color,category', [
    ((15,15,230),SpillCategory.RED_SPILL),
    ((20,225,225),SpillCategory.NON_RED_SPILL),
    ((215,225,240),SpillCategory.NON_RED_SPILL),  # Cream, low saturation.
    ((230,30,20),SpillCategory.NON_RED_SPILL),
    ((30,30,30),SpillCategory.NON_RED_SPILL),
])
def test_new_red_yellow_cream_blue_dark_objects(cardboard,color,category):
    detector = SpillDetector(BackgroundReference(cardboard,confirmed_clean=True))
    tracker = SpillTracker()
    for index in range(3):
        result = detector.detect(added(cardboard,color=color))
        assert len(result.detections) == 1
        target = tracker.update(result.detections)[0]
        assert target.category == category
        assert target.confirmed == (index == 2)
        assert np.count_nonzero(result.mask_for(target)) == target.area_pixels


def test_new_object_beside_existing_red_logo_only_new_region_detected(cardboard):
    detector = SpillDetector(BackgroundReference(cardboard,confirmed_clean=True))
    result = detector.detect(added(cardboard,(110,70),(20,230,230),radius=10))
    assert len(result.detections) == 1
    assert result.detections[0].category == SpillCategory.NON_RED_SPILL
    assert result.detections[0].pixel_center == pytest.approx((110,70),abs=1)
    assert not result.foreground_mask[60,60]  # Old logo remains legal.


def test_foreground_pixels_not_bounding_box_drive_red_ratio(cardboard):
    # A sparse red ring has a large brown hole that must not enter the denominator.
    frame = cardboard.copy()
    cv2.circle(frame,(255,240),27,(10,10,230),8)
    result = SpillDetector(BackgroundReference(cardboard,confirmed_clean=True)).detect(frame)
    target = result.detections[0]
    assert target.category == SpillCategory.RED_SPILL
    assert result.mask_for(target)[240,255] == 0


def test_separate_floor_wall_objects_keep_ids_and_calibration(cardboard):
    calibration = dual_planes()
    original = calibration.to_dict()
    detector = SpillDetector(BackgroundReference(cardboard,confirmed_clean=True),calibration=calibration)
    frame = added(added(cardboard),(265,75),(20,230,230))
    tracker = SpillTracker()
    ids = None
    for _ in range(10):
        targets = tracker.update(detector.detect(frame).detections)
        assert len(targets) == 2
        current = {t.category:t.target_id for t in targets}
        assert len(set(current.values())) == 2
        assert ids is None or ids == current
        ids = current
    assert all(t.confirmed for t in targets)
    assert calibration.to_dict() == original


@pytest.mark.parametrize('change', ['noise','exposure','white_balance','shadow'])
def test_minor_photometric_changes_do_not_create_objects(cardboard,change):
    frame = cardboard.astype(np.int16)
    if change == 'noise':
        frame += np.random.default_rng(3).integers(-4,5,frame.shape,dtype=np.int16)
    elif change == 'exposure':
        frame += 13
    elif change == 'white_balance':
        frame += np.array([9,-8,6],dtype=np.int16)
    else:
        frame[220:270,45:110] -= 12
    result = SpillDetector(BackgroundReference(cardboard,confirmed_clean=True)).detect(np.clip(frame,0,255).astype(np.uint8))
    assert result.detections == []


def test_long_stationary_target_never_learns_then_removal_expires(cardboard):
    reference = BackgroundReference(cardboard,confirmed_clean=True)
    detector = SpillDetector(reference)
    tracker = SpillTracker(TrackerConfig(max_missed_frames=2))
    frame = added(cardboard)
    original = hashlib.sha256(detector._reference.tobytes()).hexdigest()
    ids = set()
    for index in range(65):
        target = tracker.update(detector.detect(frame).detections)[0]
        ids.add(target.target_id)
        assert target.confirmed == (index >= 2)
    assert len(ids) == 1
    assert hashlib.sha256(detector._reference.tobytes()).hexdigest() == original
    for _ in range(3):
        assert tracker.update(detector.detect(cardboard).detections) == []
    assert tracker._tracks == {}


def test_roi_outer_margin_and_manual_exclusions(cardboard):
    cal = dual_planes()
    detector = SpillDetector(BackgroundReference(cardboard,confirmed_clean=True),calibration=cal)
    assert detector.detect(added(cardboard,(3,250),radius=3)).detections == []
    assert not detector._roi[200,11] and detector._roi[200,15]
    mask = np.zeros(cardboard.shape[:2],np.uint8)
    cv2.circle(mask,(85,240),25,255,-1)
    assert detector.detect(added(cardboard),exclusion_mask=mask).detections == []


def test_unreliable_registration_warns_without_blocking_emission(cardboard,monkeypatch):
    detector = SpillDetector(BackgroundReference(cardboard,confirmed_clean=True))
    assert detector._geometry_guard.reference_verifiable
    monkeypatch.setattr(detector._geometry_guard,'check',lambda *args: GeometryCheck('UNVERIFIED','No consistent fit'))
    result = detector.detect(added(cardboard))
    assert not result.detection_paused
    assert result.geometry.status == 'UNVERIFIED'
    assert result.detections
    assert result.foreground_mask.any()  # Diagnostics retain the actual differences.


def test_low_texture_change_is_nonblocking_with_existing_area_filters():
    clean = np.full((240,320,3),180,np.uint8)
    detector = SpillDetector(BackgroundReference(clean,confirmed_clean=True))
    assert detector.detect(added(clean,(80,100))).detections
    changed = clean.copy()
    changed[:150] = (20,20,20)
    result = detector.detect(changed)
    assert result.geometry.status == 'UNVERIFIED' and not result.detection_paused
    assert result.detections == []


def test_small_feature_alignment_retains_original_pixel_coordinates():
    reference = np.random.default_rng(14).integers(40,190,(360,480,3),dtype=np.uint8)
    detector = SpillDetector(BackgroundReference(reference,confirmed_clean=True))
    matrix = np.float32([[1,0,1],[0,1,0]])
    shifted = cv2.warpAffine(reference,matrix,(480,360))
    result = detector.detect(shifted)
    assert result.geometry.status == 'MATCHED' and result.alignment_applied
    assert result.detections == []
    frame = added(shifted,(221,180))
    result = detector.detect(frame)
    assert len(result.detections) == 1
    target = result.detections[0]
    assert target.pixel_center == pytest.approx((221,180),abs=1)
    assert result.mask_for(target)[180,221] == 255


def test_operator_display_hides_pending_and_debug_reveals_them(cardboard):
    targets = SpillTracker().update(SpillDetector(BackgroundReference(cardboard,confirmed_clean=True)).detect(added(cardboard)).detections)
    assert np.array_equal(render_detections(cardboard,targets),cardboard)
    assert not np.array_equal(render_detections(cardboard,targets,debug=True),cardboard)
    assert not np.array_equal(render_detections(cardboard,[replace(targets[0],confirmed=True)]),cardboard)


def test_baseline_registration_refuses_silent_overwrite_and_preserves_metadata(cardboard,tmp_path):
    source, reference = tmp_path/'legal.png', tmp_path/'baseline.png'
    cv2.imwrite(str(source),cardboard)
    args = ['background-reference','--image',str(source),'--output',str(reference),'--confirm-clean']
    assert main.main(args) == 0
    before = reference.read_bytes()
    metadata = json.loads(reference.with_suffix('.png.json').read_text())
    assert metadata['purpose'] == 'initial_legal_scene'
    assert metadata['registered_at_utc'] and metadata['color_order'] == 'BGR'
    cv2.imwrite(str(source),added(cardboard))
    assert main.main(args) == 1
    assert reference.read_bytes() == before
    assert main.main(args+['--overwrite']) == 0
    assert reference.read_bytes() != before


def test_camera_baseline_defaults_to_fifteen_frames_and_no_implicit_initialization(cardboard,tmp_path,monkeypatch,caplog):
    import vision.spill_cli as cli
    camera = Mock()
    camera.__enter__ = Mock(return_value=camera)
    camera.__exit__ = Mock(return_value=False)
    camera.read.return_value = cardboard
    factory = Mock(return_value=camera)
    monkeypatch.setattr(cli,'CameraManager',factory)
    baseline = tmp_path/'baseline.png'
    assert main.main(['detect-spills','--camera','0','--background',str(baseline),'--confirm-clean','--headless']) == 1
    factory.assert_not_called()
    assert 'No registered baseline' in caplog.text
    assert main.main(['background-reference','--camera','0','--output',str(baseline),'--confirm-clean']) == 0
    assert camera.read.call_count == 15
    assert BackgroundReference.load(baseline,confirmed_clean=True).metadata['frame_count'] == 15
    camera.__exit__.assert_called_once()


def test_headless_normal_mode_skips_debug_rendering_and_outputs_json(cardboard,tmp_path,monkeypatch):
    import vision.spill_cli as cli
    baseline, image, output = (tmp_path/name for name in ('baseline.png','new.png','out.jsonl'))
    BackgroundReference(cardboard,confirmed_clean=True).save(baseline)
    cv2.imwrite(str(image),added(cardboard))
    monkeypatch.setattr(cli,'render_detections',Mock(side_effect=AssertionError('No unnecessary headless render')))
    assert main.main(['detect-spills','--image',str(image),'--background',str(baseline),
        '--confirm-clean','--headless','--confirmation-frames','1','--output',str(output)]) == 0
    record = json.loads(output.read_text())
    assert record['detections'][0]['confirmed']
    assert record['detections'][0]['category'] == 'RED_SPILL'
    assert not record['diagnostics']['detection_paused']


def test_normal_gui_has_no_pending_labels_or_debug_windows(cardboard,tmp_path,monkeypatch):
    baseline, image = tmp_path/'baseline.png', tmp_path/'new.png'
    BackgroundReference(cardboard,confirmed_clean=True).save(baseline)
    cv2.imwrite(str(image),added(cardboard))
    texts = []
    original = cv2.putText
    def capture(image,text,*args,**kwargs):
        texts.append(text)
        return original(image,text,*args,**kwargs)
    monkeypatch.setattr(cv2,'putText',capture)
    show, close = Mock(), Mock()
    monkeypatch.setattr(cv2,'imshow',show)
    monkeypatch.setattr(cv2,'destroyAllWindows',close)
    monkeypatch.setattr(cv2,'waitKey',lambda *args: ord('q'))
    assert main.main(['detect-spills','--image',str(image),'--background',str(baseline),
        '--confirm-clean','--output',str(tmp_path/'normal.jsonl')]) == 0
    assert show.call_count == 1
    assert any('confirmed 0' in text for text in texts)
    assert not any('pending' in text or 'score=' in text or '#1' in text for text in texts)
    close.assert_called_once()


def test_shift_larger_than_alignment_limit_warns_and_continues(cardboard):
    shifted = cv2.warpAffine(cardboard,np.float32([[1,0,4],[0,1,0]]),(400,320))
    detector = SpillDetector(BackgroundReference(cardboard,confirmed_clean=True))
    result=detector.detect(shifted)
    assert result.geometry.status=='MOVED' and not result.detection_paused
    # Image subtraction can align while physical registration remains MOVED.
    assert result.alignment_applied
    assert result.reference_to_current is None
    assert result.detections == []


@pytest.mark.parametrize('settings', [dict(roi_margin_pixels=-1),dict(alignment_enabled=1),
    dict(alignment_max_displacement_pixels=0),dict(unverified_max_foreground_fraction=2)])
def test_invalid_new_settings_rejected(settings):
    with pytest.raises(ValueError):
        DetectorConfig(**settings)


def test_declared_rgb_reference_metadata_is_not_silently_used_as_bgr(cardboard,tmp_path):
    reference = tmp_path/'reference.png'
    BackgroundReference(cardboard,confirmed_clean=True).save(reference)
    sidecar = reference.with_suffix('.png.json')
    metadata = json.loads(sidecar.read_text())
    metadata['color_order'] = 'RGB'
    sidecar.write_text(json.dumps(metadata))
    with pytest.raises(ValueError,match='metadata'):
        BackgroundReference.load(reference,confirmed_clean=True)
