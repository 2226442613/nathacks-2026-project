import json

import cv2
import numpy as np
import pytest

from tools.analyze_reflection_residuals import analyze, match_targets, residual_features
from vision.spill_detection import BackgroundReference, DetectorConfig, SpillDetector, DetectionError
from vision.spill_diagnostics import stage_metrics


def test_signed_residuals_use_only_component_pixels_without_uint8_wrap():
    predicted = np.full((2, 2, 3), 100, np.uint8)
    current = predicted.copy()
    current[0, 0] = [120, 130, 140]
    current[0, 1] = [80, 70, 60]
    current[1, :] = 255  # Outside component; must not bias its statistics.
    mask = np.array([[1, 1], [0, 0]], bool)
    result = residual_features(current, predicted, mask)
    assert result['pixels'] == 2
    assert result['signed_mean_bgr_levels'] == [0, 0, 0]
    assert result['chromatic_spread_median_levels'] == 20
    assert result['all_channels_brighter_fraction'] == .5
    assert result['all_channels_darker_fraction'] == .5
    with pytest.raises(ValueError, match='empty'):
        residual_features(current, predicted, np.zeros_like(mask))


def test_object_matching_does_not_count_duplicate_as_second_true_object():
    objects = [{'id': 'red', 'bounding_box': [0, 0, 10, 10]}]
    targets = [{'bounding_box': [0, 0, 10, 10]}, {'bounding_box': [1, 1, 10, 10]}]
    assert match_targets(objects, targets) == {0: 'red'}


def test_saved_masks_preserve_holes_and_analysis_cannot_overwrite(tmp_path):
    run = tmp_path/'run'
    run.mkdir()
    image = np.full((8, 8, 3), 100, np.uint8)
    mask = np.zeros((8, 8), np.uint8)
    mask[1:7, 1:7] = 255
    mask[3:5, 3:5] = 0
    current = image.copy()
    current[mask != 0] += 40
    for suffix, value in [('input', current), ('expected_background', image), ('foreground', mask)]:
        assert cv2.imwrite(str(run/f'frame_000000_{suffix}.png'), value)
    assert cv2.imwrite(str(run/'reference.png'), image)
    (run/'settings.json').write_text(json.dumps({'detector': {'blur_kernel': 1}}))
    row = {'frame_index': 0, 'record': {'detections': [{
        'confirmed': True, 'component_id': 1, 'area_pixels': 32,
        'bounding_box': [1, 1, 6, 6], 'target_id': 4, 'category': 'NON_RED_SPILL'}]}}
    (run/'frames.jsonl').write_text(json.dumps(row)+'\n')
    labels = {'schema_version': 1, 'coordinate_system': 'original_image_pixels',
              'segments': [{'start_frame': 0, 'end_frame': 0, 'stage': 'A', 'objects': []}]}
    output = tmp_path/'analysis'
    before = {p.name: p.read_bytes() for p in run.iterdir()}
    report = analyze(run, labels, output)
    assert report['summary']['A:unmatched']['ranges']['pixels'] == [32, 32]
    assert report['summary']['A:unmatched']['ranges']['neutral_mean_levels'] == [40, 40]
    assert {p.name: p.read_bytes() for p in run.iterdir()} == before
    with pytest.raises(FileExistsError):
        analyze(run, labels, output)


def test_experimental_policy_is_opt_in_and_preserves_red_blue_and_dark_targets():
    from dataclasses import replace
    clean = np.full((120, 240, 3), 100, np.uint8)
    current = clean.copy()
    for x, color in [(10, (160, 160, 160)), (65, (40, 40, 220)),
                     (120, (220, 50, 40)), (175, (40, 40, 40))]:
        current[40:70, x:x+30] = color
    config = DetectorConfig(geometry_check_enabled=False, illumination_correction=False)
    reference = BackgroundReference(clean, confirmed_clean=True)
    ordinary = SpillDetector(reference, config).detect(current)
    assert len(ordinary.detections) == 4
    enabled = replace(config, brightening_rejection_enabled=True)
    result = SpillDetector(reference, enabled).detect(current)
    assert len(result.detections) == 3
    assert [t.category.value for t in result.detections] == ['RED_SPILL', 'NON_RED_SPILL', 'NON_RED_SPILL']
    assert len(result.component_rejections) == 1
    assert np.array_equal(ordinary.foreground_mask, result.foreground_mask)
    metrics = stage_metrics(result, [], enabled)
    assert sum(c['decision'].startswith('experimental_brightening_rejection') for c in metrics['components']) == 1
    with pytest.raises(DetectionError, match='boolean'):
        replace(config, brightening_rejection_enabled=1)


def test_red_evidence_is_never_rejected_by_experimental_brightening_policy():
    clean = np.full((80, 100, 3), (30, 30, 140), np.uint8)
    current = clean.copy()
    current[20:50, 20:50] = [65, 65, 180]
    result = SpillDetector(BackgroundReference(clean, confirmed_clean=True),
        DetectorConfig(geometry_check_enabled=False, illumination_correction=False,
                       brightening_rejection_enabled=True)).detect(current)
    assert len(result.detections) == 1
    assert result.detections[0].category.value == 'RED_SPILL'
    assert not result.component_rejections


def test_optional_real_red_blue_windows_with_experimental_policy():
    from dataclasses import replace
    from pathlib import Path
    from vision.calibration import load_calibration
    from vision.spill_tracking import SpillTracker, load_spill_settings
    run = Path('data/module02_field_evidence/live_sequence_01')
    labels_path = run.parent/'live_sequence_labels.json'
    if not run.is_dir() or not labels_path.is_file():
        pytest.skip('Private physical-camera evidence is not distributed')
    config, tracker_config = load_spill_settings(run/'settings.json')
    detector = SpillDetector(BackgroundReference.load(run/'reference.png', confirmed_clean=True),
        replace(config, brightening_rejection_enabled=True), load_calibration(run/'calibration.json'))
    tracker = SpillTracker(tracker_config)
    identities = {}
    for segment in json.loads(labels_path.read_text())['segments']:
        for index in range(segment['start_frame'], segment['end_frame']+1):
            frame = cv2.imread(str(run/f'frame_{index:06d}_input.png'))
            result = detector.detect(frame)
            targets = [t.to_dict() for t in tracker.update(result.detections) if t.confirmed]
            # The sampled windows have gaps: allow ordinary confirmation startup.
            if index < segment['start_frame']+2:
                continue
            matched = match_targets(segment['objects'], targets)
            assert len(targets) == len(matched) == len(segment['objects'])
            for ti, object_id in matched.items():
                expected = next(o for o in segment['objects'] if o['id'] == object_id)
                assert targets[ti]['category'] == expected['category']
                assert identities.setdefault(object_id, targets[ti]['target_id']) == targets[ti]['target_id']
            assert result.reference_to_current is None  # Image policy grants no physical trust.
