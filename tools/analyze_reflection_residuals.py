"""Measure saved component residuals without changing detection or fitting a gate.

Uses a run's own predicted background, never the current registered reference.
Signed BGR residuals describe image changes, not a material/reflection diagnosis.
Run with --run RUN --labels LABELS --output NEW_DIRECTORY.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from tools.analyze_spill_diagnostics import iou


def residual_features(current, predicted, mask):
    if current.shape != predicted.shape or mask.shape != current.shape[:2]:
        raise ValueError('Current, predicted background and mask must share dimensions')
    pixels = mask != 0
    if not np.any(pixels):
        raise ValueError('Component mask is empty')
    # Predictions are saved as uint8; their quantization error is under one level.
    delta = current[pixels].astype(np.float32) - predicted[pixels].astype(np.float32)
    neutral = delta.mean(axis=1)
    chromatic = delta.max(axis=1) - delta.min(axis=1)
    return {
        'pixels': int(pixels.sum()),
        'signed_mean_bgr_levels': delta.mean(axis=0).tolist(),
        'neutral_mean_levels': float(neutral.mean()),
        'chromatic_spread_median_levels': float(np.median(chromatic)),
        'chromatic_spread_p95_levels': float(np.percentile(chromatic, 95)),
        'all_channels_brighter_fraction': float(np.mean(np.all(delta > 0, axis=1))),
        'all_channels_darker_fraction': float(np.mean(np.all(delta < 0, axis=1))),
    }


def match_targets(objects, targets):
    """Same greedy one-to-one IoU policy as the existing diagnostic evaluator."""
    pairs = sorted((iou(o['bounding_box'], t['bounding_box']), oi, ti)
                   for oi, o in enumerate(objects) for ti, t in enumerate(targets))
    used_objects, matched = set(), {}
    for overlap, oi, ti in reversed(pairs):
        if overlap >= .25 and oi not in used_objects and ti not in matched:
            used_objects.add(oi)
            matched[ti] = objects[oi]['id']
    return matched


def analyze(run, labels, output):
    run, output = Path(run), Path(output)
    if labels.get('schema_version') != 1 or labels.get('coordinate_system') != 'original_image_pixels':
        raise ValueError('Independent labels must use schema 1 and original image pixels')
    segments = labels['segments']
    windows = {}
    for segment in segments:
        start, end = segment['start_frame'], segment['end_frame']
        if start < 0 or end < start:
            raise ValueError('Invalid annotation window')
        for index in range(start, end + 1):
            if index in windows:
                raise ValueError('Overlapping annotation windows')
            windows[index] = segment
    output.mkdir(parents=True, exist_ok=False)
    blur = json.loads((run/'settings.json').read_text())['detector']['blur_kernel']
    observations, seen = [], set()
    with (run/'frames.jsonl').open() as source:
        for line in source:
            row = json.loads(line)
            index = row['frame_index']
            if index not in windows:
                continue
            seen.add(index)
            segment = windows[index]
            targets = [t for t in row['record']['detections'] if t['confirmed']]
            matched = match_targets(segment['objects'], targets)
            prefix = run/f'frame_{index:06d}'
            current = cv2.imread(str(prefix)+'_input.png')
            predicted = cv2.imread(str(prefix)+'_expected_background.png')
            foreground = cv2.imread(str(prefix)+'_foreground.png', cv2.IMREAD_GRAYSCALE)
            if current is None or predicted is None or foreground is None:
                raise ValueError(f'Missing diagnostic images for frame {index}')
            if blur > 1:
                current = cv2.GaussianBlur(current, (blur, blur), 0)
            _, components = cv2.connectedComponents(foreground, connectivity=8)
            for ti, target in enumerate(targets):
                mask = components == target['component_id']
                if int(mask.sum()) != target['area_pixels']:
                    raise ValueError(f'Component mismatch at frame {index}')
                observations.append(dict(frame_index=index, stage=segment['stage'],
                    target_id=target['target_id'], category=target['category'],
                    object_id=matched.get(ti), bounding_box=target['bounding_box'],
                    **residual_features(current, predicted, mask)))
    if seen != set(windows):
        raise ValueError('Annotation windows contain missing frame records')
    groups = {}
    for observation in observations:
        key = observation['stage']+':'+(observation['object_id'] or 'unmatched')
        groups.setdefault(key, []).append(observation)
    summary = {}
    for key, values in groups.items():
        summary[key] = {'observations': len(values), 'ranges': {
            name: [min(v[name] for v in values), max(v[name] for v in values)]
            for name in ('pixels', 'neutral_mean_levels', 'chromatic_spread_median_levels',
                         'all_channels_brighter_fraction', 'all_channels_darker_fraction')}}
    report = {'covered_frames': len(seen), 'summary': summary,
        'run': str(run.resolve()),
        'input_hashes': {name: hashlib.sha256((run/name).read_bytes()).hexdigest()
                         for name in ('reference.png', 'settings.json', 'frames.jsonl')},
        'labels_sha256': hashlib.sha256(json.dumps(labels, sort_keys=True).encode()).hexdigest(),
        'note': 'Read-only saved-output analysis. No suppression or material identification. '
                'Predicted PNG quantization contributes less than one level of residual error. '
                'Unmatched detections are false-positive observations only within supplied labeled windows.'}
    (output/'observations.json').write_text(json.dumps(observations, indent=2, allow_nan=False))
    (output/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--labels', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(json.dumps(analyze(args.run, json.loads(Path(args.labels).read_text()), args.output), indent=2))


if __name__ == '__main__':
    main()
