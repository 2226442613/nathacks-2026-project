"""Read-only evidence collection around the production spill detector.

Intensity residuals are 8-bit channel levels, not pixels or centimeters. These
measurements diagnose stages; they never change candidates or track state.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import time

import cv2
import numpy as np

from .serialization import json_line
from .spill_detection import DetectionError


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def expected_background(reference, result, config):
    """Reconstruct the blurred/warped photometric prediction for visualization."""
    image = reference.copy()
    if config.blur_kernel > 1:
        image = cv2.GaussianBlur(image, (config.blur_kernel, config.blur_kernel), 0)
    matrix = getattr(result, 'background_alignment_transform', None)
    if matrix is None and result.alignment_applied:
        matrix = result.reference_to_current
    if result.alignment_applied and matrix is not None:
        image = cv2.warpAffine(image, np.asarray(matrix)[:2], image.shape[1::-1])
    gain = np.asarray(result.illumination_gain_bgr)
    offset = np.asarray(result.illumination_offset_bgr)
    return np.clip(image.astype(np.float32)*gain+offset, 0, 255).astype(np.uint8)


def stage_metrics(result, targets, config):
    allowed = result.analysis_mask != 0
    pixels = int(allowed.sum())
    raw_mask = result.raw_foreground_mask
    components = []
    candidate_labels = {target.component_id for target in result.detections}
    tracked = {target.component_id: target for target in targets}
    count, labels, stats, _ = cv2.connectedComponentsWithStats(result.foreground_mask, connectivity=8)
    max_area = pixels*config.max_area_fraction
    if config.max_area_pixels:
        max_area = min(max_area, config.max_area_pixels)
    for label in range(1, count):
        x, y, w, h, area = map(int, stats[label])
        target = tracked.get(label)
        reason = (result.component_rejections[label] if label in result.component_rejections else
                  'accepted_candidate' if label in candidate_labels else
                  'below_min_area' if area < config.min_area_pixels else
                  'above_max_area' if area > max_area else 'not_emitted')
        contours, _ = cv2.findContours((labels[y:y+h, x:x+w] == label).astype(np.uint8),
                                      cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = max(contours, key=cv2.contourArea)[:, 0]+[x, y]
        components.append({'component_id':label, 'area_pixels':area, 'bounding_box':[x,y,w,h],
                           'contour':contour.tolist(), 'decision':reason,
                           'target_id':None if target is None else target.target_id,
                           'confirmed':False if target is None else target.confirmed})
    def residual(image):
        values = image[allowed] if image is not None else np.empty(0)
        return {'mean_levels':float(values.mean()) if len(values) else None,
                'p95_levels':float(np.percentile(values,95)) if len(values) else None,
                'above_threshold_pixels':int(np.count_nonzero(values >= config.difference_threshold))}
    return {'analysis_pixels':pixels, 'raw_residual':residual(result.raw_difference),
            'corrected_residual':residual(result.difference),
            'pre_morphology_pixels':int(np.count_nonzero(raw_mask)) if raw_mask is not None else None,
            'post_morphology_pixels':int(np.count_nonzero(result.foreground_mask)),
            'candidate_count':len(result.detections), 'confirmed_count':sum(t.confirmed for t in targets),
            'component_count':count-1, 'components':components,
            'illumination_offset_bgr_levels':list(result.illumination_offset_bgr),
            'illumination_gain_bgr':list(result.illumination_gain_bgr),
            'background_alignment_applied':result.alignment_applied,
            'physical_registration':None if result.geometry is None else result.geometry.to_dict()}


def diagnostic_panels(frame, reference, predicted, result, targets, metrics):
    candidates = frame.copy()
    for component in metrics['components']:
        contour = np.asarray(component['contour'], np.int32).reshape(-1,1,2)
        cv2.drawContours(candidates, [contour], -1, (0,200,255), 1)
    confirmed = frame.copy()
    for target in targets:
        if target.confirmed:
            contour = np.asarray(target.contour, np.int32).reshape(-1,1,2)
            color = (60,60,255) if target.category.value == 'RED_SPILL' else (255,180,60)
            cv2.drawContours(confirmed, [contour], -1, color, 2)
            x,y = map(int,target.pixel_center)
            cv2.putText(confirmed, f'#{target.target_id}', (x,y), 0, .5, color, 1)
    raw = result.raw_difference if result.raw_difference is not None else result.difference
    images = [('RAW CAMERA',frame), ('LEGAL BACKGROUND',reference),
              ('EXPECTED BG: ALIGN + GAIN/OFFSET',predicted),
              ('RAW DIFFERENCE: 0-255 LEVELS',cv2.applyColorMap(raw,cv2.COLORMAP_TURBO)),
              ('CORRECTED DIFFERENCE: 0-255 LEVELS',cv2.applyColorMap(result.difference,cv2.COLORMAP_TURBO)),
              ('FOREGROUND BEFORE MORPHOLOGY',result.raw_foreground_mask),
              ('FOREGROUND AFTER MORPHOLOGY',result.foreground_mask),
              (f'COMPONENTS {metrics["component_count"]} / CANDIDATES {len(result.detections)}',candidates),
              (f'CONFIRMED {metrics["confirmed_count"]}',confirmed)]
    cells = []
    for title,image in images:
        if image is None:
            image = np.zeros(frame.shape[:2],np.uint8)
        if image.ndim == 2:
            image = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        cell = np.zeros((266,426,3),np.uint8)
        cell[26:] = cv2.resize(image,(426,240),interpolation=cv2.INTER_AREA)
        cv2.putText(cell,title,(6,18),0,.37,(240,240,240),1)
        cells.append(cell)
    return np.vstack([np.hstack(cells[index:index+3]) for index in (0,3,6)])


class DiagnosticSession:
    """New directory per run; source files and registration are never written."""
    def __init__(self, directory, *, background, reference, detector_config, tracker_config,
                 inputs, panels=False, calibration=None, fps=30):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True,exist_ok=False)
        self.reference = reference.image
        self.config = detector_config
        self.panels = panels
        self.started = time.monotonic()
        self.frames = 0
        self.ids = set()
        self.previous_ids = set()
        self.video = None
        self.video_attempted = False
        self.fps = fps
        self.stage = None
        shutil.copyfile(background,self.directory/'reference.png')
        if calibration:
            shutil.copyfile(calibration,self.directory/'calibration.json')
        (self.directory/'reference_metadata.json').write_text(json_line(reference.metadata)+'\n')
        # Sidecar uses the original PNG bytes, so its checksum stays valid.
        (self.directory/'reference.png.json').write_text(json_line(reference.metadata)+'\n')
        (self.directory/'settings.json').write_text(json_line({'settings_version':1,
            'detector':asdict(detector_config),'tracker':asdict(tracker_config)})+'\n')
        sources = {str(path):file_hash(path) for path in inputs if path and Path(path).is_file()}
        self.manifest = {'diagnostic_schema_version':1, 'status':'STARTED',
            'started_at_utc':datetime.now(timezone.utc).isoformat(), 'source_sha256':sources,
            'background_snapshot_sha256':file_hash(self.directory/'reference.png'),
            'detector':asdict(detector_config), 'tracker':asdict(tracker_config),
            'exposure_and_white_balance':'Not locked or inferred; BGR frame statistics only',
            'residual_units':'8-bit per-channel intensity levels',
            'ground_truth':'Not provided; candidate/confirmed counts are not automatically false positives',
            'stage_markers':[],
            'input_metadata':{}, 'frames':0}
        self._save_manifest()
        self.stream = (self.directory/'frames.jsonl').open('x',encoding='utf-8')

    def _save_manifest(self):
        (self.directory/'manifest.json').write_text(json_line(self.manifest)+'\n')

    def __enter__(self):
        return self

    def __exit__(self, kind, error, traceback):
        self.stream.close()
        if self.video is not None:
            self.video.release()
        self.manifest.update(status='ERROR' if kind else 'COMPLETE',frames=self.frames,
                             error=None if error is None else str(error))
        self._save_manifest()

    def mark_capture(self):
        return {'elapsed_seconds':time.monotonic()-self.started,
                'captured_at_utc':datetime.now(timezone.utc).isoformat()}

    def mark_stage(self, stage, next_frame):
        if stage not in 'ABCDEF' or self.stage == stage:
            return
        self.stage = stage
        self.manifest['stage_markers'].append({'stage':stage,'start_frame':next_frame})
        self._save_manifest()

    def write(self, index, frame, result, targets, record, *, observation=None):
        if not self.video_attempted:
            self.video_attempted = True
            self.video = cv2.VideoWriter(str(self.directory/'capture.avi'),cv2.VideoWriter_fourcc(*'FFV1'),
                                         self.fps,frame.shape[1::-1])
            if not self.video.isOpened():
                self.video.release()
                self.video = None
            self.manifest['lossless_video_available'] = self.video is not None
            self.manifest['video_fps_note'] = 'Nominal requested FPS; use per-frame observation times for live delay measurements'
        if self.video is not None:
            self.video.write(frame)
        metrics = stage_metrics(result,targets,self.config)
        current = {target.target_id for target in targets}
        metrics['new_target_ids'] = sorted(current-self.ids)
        metrics['missing_target_ids'] = sorted(self.previous_ids-current)
        metrics['reappeared_target_ids'] = sorted((current & self.ids)-self.previous_ids)
        self.ids.update(current)
        self.previous_ids = current
        predicted = expected_background(self.reference,result,self.config)
        arrays = [('input',frame), ('expected_background',predicted), ('raw_difference',result.raw_difference),
                  ('difference',result.difference), ('raw_foreground',result.raw_foreground_mask),
                  ('foreground',result.foreground_mask), ('analysis',result.analysis_mask), ('red',result.red_mask)]
        panel = diagnostic_panels(frame,self.reference,predicted,result,targets,metrics) if self.panels else None
        if panel is not None:
            arrays.append(('panels',panel))
        for suffix,image in arrays:
            if image is not None and not cv2.imwrite(str(self.directory/f'frame_{index:06d}_{suffix}.png'),image):
                raise DetectionError(f'Cannot save diagnostic {suffix}')
        item = {'frame_index':index, **(observation or self.mark_capture()),
                'operator_stage':self.stage, 'metrics':metrics,'record':record}
        self.stream.write(json_line(item)+'\n')
        self.stream.flush()
        self.frames += 1
        return panel
