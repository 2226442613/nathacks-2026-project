"""Fixed-camera operator wizard; raw marker observations, existing plane fitter.

No chassis-reference input is offered: heading alignment has not been measured.
Receive timestamps are sampled immediately after synchronous CameraManager.read,
not synthesized frame times or claims about unavailable sensor exposure times.
"""
from collections import deque
from copy import deepcopy
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import textwrap
import time

import cv2
import numpy as np

from .calibration import load_calibration, DualPlaneCalibration
from .camera import CameraManager
from .robot_plane import MarkerPlaneMapping, MarkerPlaneValidationError
from .robot_tracking import MarkerObservation, RobotMarkerDetector, RobotTracker, draw_robot_pose
from .serialization import json_line

REFERENCE = 'MARKER_GROUND_PROJECTION'
TITLE = 'Hospital Guardian - robot plane calibration | Q exit'
SUGGESTIONS = {'training': [(25,18),(60,18),(95,18),(25,48),(95,48)],
               # The first three alone span only 12.5 cm², below the existing
               # 1% floor-area requirement. Add broad, independent hold-outs.
               'validation': [(60,35),(45,30),(80,40),(35,48),(85,22)]}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_new_json(path, data):
    """Publish complete JSON atomically, exclusively; never replace existing data."""
    return write_new_bytes(path, (json_line(data) + '\n').encode('utf-8'))


def write_new_bytes(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix='.'+path.name+'.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)  # Same filesystem; atomic and fails if destination exists.
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return path


class CaptureWindow:
    """Bounded genuine detections; loss/replay breaks the stationary window."""
    def __init__(self, duration=1.2, minimum=8, minimum_span=.3, maximum_spread=2.):
        self.duration, self.minimum = duration, minimum
        self.minimum_span, self.maximum_spread = minimum_span, maximum_spread
        self.samples = deque(maxlen=90)
        self.seen_frames = deque(maxlen=180)
        self.last_time = None
        self.current = False

    def clear(self):
        self.samples.clear()
        self.current = False

    def add(self, frame, observation, received_at, received_utc):
        self.current = False
        if not math.isfinite(received_at) or (self.last_time is not None and received_at <= self.last_time):
            self.clear()
            raise ValueError('Camera receive timestamps must strictly increase')
        self.last_time = received_at
        frame_id = hashlib.sha256(frame.tobytes()).hexdigest()
        if not isinstance(observation, MarkerObservation) or frame_id in self.seen_frames:
            self.clear()
            return  # Never accept RobotPose.last_known_pose, or replayed identical frames.
        self.seen_frames.append(frame_id)
        pixel = np.asarray(observation.center_pixels, float)
        if pixel.shape != (2,) or not np.isfinite(pixel).all():
            self.clear()
            return
        self.samples.append({'pixel': pixel.tolist(), 'received_at': received_at,
                             'received_utc': received_utc, 'frame_sha256': frame_id,
                             'observation_timestamp': observation.timestamp})
        while self.samples and received_at - self.samples[0]['received_at'] > self.duration:
            self.samples.popleft()
        self.current = True

    def representative(self, now):
        if not self.current or not self.samples or not 0 <= now-self.samples[-1]['received_at'] <= .25:
            raise ValueError('No fresh genuine marker observation; hold still and retry')
        if len(self.samples) < self.minimum:
            raise ValueError(f'Need {self.minimum} distinct detected frames; have {len(self.samples)}')
        if self.samples[-1]['received_at']-self.samples[0]['received_at'] < self.minimum_span:
            raise ValueError('Capture window too short; hold still for at least 0.3 seconds')
        pixels = np.asarray([p['pixel'] for p in self.samples])
        center = np.median(pixels, axis=0)
        distances = np.linalg.norm(pixels-center, axis=1)
        mad = np.median(np.abs(distances-np.median(distances)))
        keep = distances <= max(.5, np.median(distances)+3*1.4826*mad)
        if keep.sum() < self.minimum or keep.mean() < .8:
            raise ValueError('Unstable capture: too many outliers; retry after robot stops')
        good = pixels[keep]
        center = np.median(good, axis=0)
        spread = float(np.max(np.linalg.norm(good-center, axis=1)))
        # Check end-to-end movement too, so trimming cannot hide slow motion.
        drift = float(np.linalg.norm(np.median(pixels[:3], axis=0)-np.median(pixels[-3:], axis=0)))
        if spread > self.maximum_spread or drift > self.maximum_spread:
            raise ValueError(f'Unstable capture: spread {spread:.2f} px, drift {drift:.2f} px; limit {self.maximum_spread:g} px')
        quality = {'distinct_frames': len(pixels), 'inlier_frames': int(keep.sum()),
                   'outlier_frames': int((~keep).sum()), 'maximum_inlier_radius_px': spread,
                   'peak_to_peak_uv_px': np.ptp(good, axis=0).tolist(), 'drift_px': drift,
                   'start_received_utc': self.samples[0]['received_utc'],
                   'end_received_utc': self.samples[-1]['received_utc'],
                   'span_seconds': self.samples[-1]['received_at']-self.samples[0]['received_at'],
                   'timestamp_basis': 'host monotonic/UTC immediately after camera read; not sensor exposure',
                   'frames': list(self.samples)}
        return center.tolist(), quality


class CalibrationSession:
    def __init__(self, calibration_path, settings_path, *, height_cm=None, tolerance_cm=2., camera=0):
        from .robot_cli import load_robot_config
        self.inputs = {str(Path(p).resolve()): digest(p) for p in (calibration_path, settings_path)}
        self.calibration_path = Path(calibration_path)
        self.calibration = load_calibration(calibration_path)
        self.floor = self.calibration.floor if isinstance(self.calibration, DualPlaneCalibration) else self.calibration
        self.calibration_id = 'sha256:' + digest(calibration_path)
        self.config = load_robot_config(settings_path)
        height = self.config.mounting_height_cm if height_cm is None else height_cm
        if height is None or not math.isfinite(height) or height <= 0:
            raise ValueError('Enter a positive measured elevated-marker mounting height')
        if self.config.mounting_height_cm is not None and not math.isclose(height, self.config.mounting_height_cm, abs_tol=1e-6):
            raise ValueError('Marker height differs from robot settings; verify hardware before capture')
        self.config = replace(self.config, mounting_height_cm=height)
        if not math.isfinite(tolerance_cm) or tolerance_cm <= 0:
            raise ValueError('Validation tolerance must be positive and finite')
        if self.floor.source != camera:
            raise ValueError('Camera source differs from arena calibration')
        self.tolerance = tolerance_cm
        self.points = {'training': [], 'validation': []}
        self.captures = []
        self.attested = False
        self.mapping = None
        self.report = None
        self.saved_mapping = None
        self.revision = 0
        self.saved_revision = None
        self.last_points_path = None
        self.preview_tracker = None
        self.resume_source = None
        self.resume_backup = None
        self.loaded_metadata = {}

    def resume_points(self, path):
        """Validate a wizard snapshot before restoring any progress or observations."""
        if self.captures or any(self.points.values()):
            raise ValueError('Resume requires a new empty session')
        path = Path(path)
        if str(path.resolve()) in self.inputs:
            raise ValueError('Resume points must be distinct from calibration/settings inputs')
        content = path.read_bytes()
        try:
            data = json.loads(content)
            self._validate_resume(data)
        except (KeyError, TypeError, ValueError, AttributeError) as exc:
            raise ValueError(f'Cannot resume points: {exc}') from exc
        self.points = deepcopy({mode:data[mode] for mode in self.points})
        self.loaded_metadata = deepcopy(data['metadata'])
        self.captures = deepcopy(self.loaded_metadata['capture_quality'])
        self.attested = True
        # Serialized fits/pass flags never restore mapping or physical trust.
        self.mapping = self.report = self.saved_mapping = self.preview_tracker = None
        self.resume_source = path
        self.inputs[str(path.resolve())] = hashlib.sha256(content).hexdigest()
        self.last_points_path = path
        self.saved_revision = self.revision

    def _validate_resume(self, data):
        if not isinstance(data,dict) or type(data.get('schema_version',1)) is not int or data.get('schema_version',1) != 1:
            raise ValueError('Unsupported points schema')
        meta = data['metadata']
        if not isinstance(meta,dict) or meta['measurement_reference'] != REFERENCE or meta['attested_measured'] is not True:
            raise ValueError('Measured marker-ground-projection metadata required')
        if meta['calibration_id'] != self.calibration_id:
            raise ValueError('Calibration identity mismatch')
        frame_size = [self.floor.frame_width,self.floor.frame_height]
        if meta['frame_size'] != frame_size or any(type(v) is not int for v in meta['frame_size']):
            raise ValueError('Camera resolution mismatch')
        height = meta['mounting_height_cm']
        if isinstance(height,bool) or not isinstance(height,(int,float)) or not math.isfinite(height) or not math.isclose(height,self.config.mounting_height_cm,rel_tol=0,abs_tol=1e-6):
            raise ValueError('Marker height mismatch')
        from .robot_tracking import RobotConfig
        if not isinstance(meta['robot_configuration'],dict) or set(meta['robot_configuration']) != set(asdict(self.config)):
            raise ValueError('Incomplete robot configuration metadata')
        saved_config = RobotConfig(**meta['robot_configuration'])
        if asdict(saved_config) != asdict(self.config):
            raise ValueError('Robot configuration mismatch')
        if meta['heading_alignment_verified'] is not False or meta['chassis_reference_conversion_supported'] is not False:
            raise ValueError('Unsupported heading/chassis metadata')
        if not isinstance(meta['created_at_utc'],str) or not meta['created_at_utc']:
            raise ValueError('Missing creation metadata')

        def pair(value, bounds, *, pixels=False):
            if (not isinstance(value,list) or len(value) != 2 or
                any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in value)):
                raise ValueError('Invalid measured coordinate pair')
            if any(v < 0 or (v >= limit if pixels else v > limit) for v,limit in zip(value,bounds)):
                raise ValueError('Measured coordinates outside frame/floor')

        positions = []
        for mode in self.points:
            if not isinstance(data[mode],list):
                raise ValueError('Training/validation points must be lists')
            for point in data[mode]:
                pair(point['pixel'],frame_size,pixels=True)
                pair(point['ground_cm'],[self.floor.width_cm,self.floor.height_cm])
                if any(np.linalg.norm(np.array(point['ground_cm'])-p['ground_cm']) < .1 or
                       np.linalg.norm(np.array(point['pixel'])-p['pixel']) < .5 for p in positions):
                    raise ValueError('Duplicate physical/pixel position in saved points')
                positions.append(point)
        captures = meta['capture_quality']
        if not isinstance(captures,list) or len(captures) != len(positions):
            raise ValueError('Capture metadata must cover every saved point')
        indices = {'training':0,'validation':0}
        used_ids = set()
        for capture in captures:
            mode,index = capture['mode'],capture['index']
            if mode not in indices or type(index) is not int or index != indices[mode] or index >= len(data[mode]):
                raise ValueError('Invalid capture mode/index metadata')
            indices[mode] += 1
            ids,quality = capture['frame_ids'],capture['quality']
            frames = quality['frames']
            if (not isinstance(ids,list) or not ids or any(not isinstance(v,str) or len(v)!=64 or any(c not in '0123456789abcdef' for c in v) for v in ids)
                or len(set(ids)) != len(ids) or used_ids.intersection(ids) or not isinstance(frames,list)
                or len(frames) != len(ids) or {f['frame_sha256'] for f in frames} != set(ids)):
                raise ValueError('Invalid or reused saved frame observations')
            if type(quality['distinct_frames']) is not int or quality['distinct_frames'] != len(frames):
                raise ValueError('Invalid capture frame count')
            if (any(type(quality[k]) is not int or quality[k] < 0 for k in ('inlier_frames','outlier_frames'))
                or quality['inlier_frames']+quality['outlier_frames'] != len(frames)
                or any(isinstance(quality[k],bool) or not isinstance(quality[k],(int,float)) or
                       not math.isfinite(quality[k]) or quality[k] < 0
                       for k in ('maximum_inlier_radius_px','drift_px','span_seconds'))):
                raise ValueError('Invalid saved capture quality metadata')
            previous = None
            for observation in frames:
                pair(observation['pixel'],frame_size,pixels=True)
                timestamp = observation['received_at']
                if (isinstance(timestamp,bool) or not isinstance(timestamp,(int,float)) or not math.isfinite(timestamp)
                    or (previous is not None and timestamp <= previous)
                    or any(not isinstance(observation[k],str) or not observation[k] for k in ('received_utc','observation_timestamp'))):
                    raise ValueError('Invalid saved observation timestamps')
                previous = timestamp
            used_ids.update(ids)

    def check_inputs(self):
        if any(digest(p) != value for p, value in self.inputs.items()):
            raise ValueError('Calibration, robot settings or resume points changed during wizard; restart with the new inputs')

    def invalidate(self):
        self.mapping = self.report = self.saved_mapping = self.preview_tracker = None
        self.revision += 1

    def capture(self, mode, ground_cm, window, now):
        self.check_inputs()
        if not self.attested:
            raise ValueError('Press A to confirm measured MARKER ground projection, not chassis center')
        if mode not in self.points:
            raise ValueError('Select TRAINING or VALIDATION')
        ground = np.asarray(ground_cm, float)
        if ground.shape != (2,) or not np.isfinite(ground).all() or (ground < 0).any() or (ground > [self.floor.width_cm,self.floor.height_cm]).any():
            raise ValueError('Measured marker projection must be finite and inside the floor, in cm')
        pixel, quality = window.representative(now)
        if (np.asarray(pixel)<0).any() or (np.asarray(pixel)>=[self.floor.frame_width,self.floor.frame_height]).any():
            raise ValueError('Marker pixels outside calibrated frame')
        ids = {s['frame_sha256'] for s in quality['frames']}
        for capture in self.captures:
            if ids.intersection(capture['frame_ids']):
                raise ValueError('Observation already used; collect a fresh stationary window')
        for points in self.points.values():
            for p in points:
                if np.linalg.norm(ground-p['ground_cm']) < .1 or np.linalg.norm(np.array(pixel)-p['pixel']) < .5:
                    raise ValueError('Duplicate physical/pixel position; use a distinct independent location')
        self.points[mode].append({'pixel': pixel, 'ground_cm': ground.tolist()})
        self.captures.append({'mode': mode, 'index': len(self.points[mode])-1,
                              'frame_ids': sorted(ids), 'quality': quality})
        self.invalidate()
        window.clear()
        return quality

    def undo(self):
        if not self.captures:
            raise ValueError('No captured point to undo')
        capture = self.captures.pop()
        self.points[capture['mode']].pop()
        self.invalidate()

    def measurements(self):
        return {**self.points, 'metadata': {
            **self.loaded_metadata,
            'measurement_reference': REFERENCE, 'attested_measured': self.attested,
            'frame_size': [self.floor.frame_width,self.floor.frame_height],
            'calibration_id': self.calibration_id, 'robot_configuration': asdict(self.config),
            'mounting_height_cm': self.config.mounting_height_cm,
            'heading_alignment_verified': False, 'chassis_reference_conversion_supported': False,
            'capture_quality': self.captures, 'independent_validation': self.report,
            'created_at_utc': self.loaded_metadata.get('created_at_utc',datetime.now(timezone.utc).isoformat())}}

    def save_points(self, base_path):
        self.check_inputs()
        if not self.captures:
            raise ValueError('No measured points to save')
        if self.saved_revision == self.revision and self.last_points_path is not None:
            return self.last_points_path
        if self.resume_source is not None and self.resume_backup is None:
            content = self.resume_source.read_bytes()
            if hashlib.sha256(content).hexdigest() != self.inputs[str(self.resume_source.resolve())]:
                raise ValueError('Resume points changed before backup; refusing to save')
            base_backup = self.resume_source.with_name(self.resume_source.stem+'_resume_backup'+self.resume_source.suffix)
            index = 0
            while True:
                backup = base_backup if index == 0 else base_backup.with_name(f'{base_backup.stem}_{index:03d}{base_backup.suffix}')
                try:
                    write_new_bytes(backup,content)
                    break
                except FileExistsError:
                    index += 1
            if digest(backup) != self.inputs[str(self.resume_source.resolve())]:
                raise OSError('Resume backup verification failed')
            self.resume_backup = backup
            print(f'Resume source backup verified: {backup}; original points file remains unchanged')
        base = Path(base_path)
        # S/F/Q take new snapshots; never replace an earlier point collection.
        index = 0
        while True:
            candidate = base if index == 0 else base.with_name(f'{base.stem}_{index:03d}{base.suffix}')
            try:
                write_new_json(candidate, self.measurements())
                break
            except FileExistsError:
                index += 1
        self.last_points_path = candidate
        self.saved_revision = self.revision
        return candidate

    def fit(self):
        self.check_inputs()
        self.mapping = self.report = self.saved_mapping = self.preview_tracker = None
        kwargs = dict(calibration_id=self.calibration_id,
                      frame_size=(self.floor.frame_width,self.floor.frame_height),
                      floor_size_cm=(self.floor.width_cm,self.floor.height_cm),
                      height_cm=self.config.mounting_height_cm,tolerance_cm=self.tolerance,attested=self.attested)
        try:
            mapping = MarkerPlaneMapping.fit(self.points['training'],self.points['validation'],**kwargs)
        except MarkerPlaneValidationError as exc:
            self.report = self.validation_report(exc.report, exc.errors_cm)
            self.saved_revision = None
            raise
        errors = mapping.map([p['pixel'] for p in self.points['validation']])-np.array([p['ground_cm'] for p in self.points['validation']])
        self.report = self.validation_report(mapping.report, errors)
        self.mapping = mapping
        self.saved_revision = None
        return self.report

    def validation_report(self, report, errors):
        return {**report, 'training_count':len(self.points['training']), 'validation_count':len(self.points['validation']),
                'tolerance_satisfied': report['maximum_error_cm'] <= self.tolerance,
                'points': [{'index':i+1,'expected_cm':p['ground_cm'],'error_x_cm':float(e[0]),
                            'error_y_cm':float(e[1]),'euclidean_error_cm':float(np.linalg.norm(e))}
                           for i,(p,e) in enumerate(zip(self.points['validation'],errors))]}

    def save_mapping(self, path):
        self.check_inputs()
        if self.mapping is None or not self.report or not self.report['tolerance_satisfied']:
            raise ValueError('Fit must pass independent validation before saving a mapping')
        self.mapping.validate_context(self.calibration_id,self.config,(self.floor.frame_width,self.floor.frame_height))
        result = write_new_json(path,self.mapping.to_dict())
        self.saved_mapping = result
        self.preview_tracker = RobotTracker(self.calibration,self.config,calibration_id=self.calibration_id,marker_plane=self.mapping)
        return result

    def suggestion_clearance(self, ground):
        # Heading is unverified: bound every orientation by a circumscribed circle.
        c = self.config
        if c.body_length_cm is None:
            return False
        radius = math.hypot(c.body_length_cm/2,c.body_width_cm/2)+math.hypot(c.marker_offset_forward_cm,c.marker_offset_left_cm)
        return radius <= ground[0] <= self.floor.width_cm-radius and radius <= ground[1] <= self.floor.height_cm-radius


class WizardUI:
    """Keyboard coordinate editor, all on OpenCV's main GUI thread; no stdin."""
    def __init__(self, session, points_path, mapping_path):
        self.session, self.points_path, self.mapping_path = session, points_path, mapping_path
        self.mode = 'validation' if session.resume_source is not None and len(session.points['training']) >= 4 else 'training'
        self.fields = ['', '']
        self.focus = 0
        self.message = 'Measure marker center vertical projection with a plumb reference. Press A to confirm.'
        if session.resume_source is not None:
            self.message = (f'Resumed {len(session.points["training"])} training and {len(session.points["validation"])} validation points; '
                            f'next {self.mode} #{len(session.points[self.mode])+1}. Keep camera pose/height unchanged. '
                            'New captures save numbered snapshots; original remains unchanged.')
        self.preview = False
        self.report_page = 0

    def key(self, key, window, now):
        key = key & 255
        if key in (27, ord('q'), ord('Q')):
            if self.session.captures:
                path = self.session.save_points(self.points_path)
                print(f'Measurements preserved: {path}')
            return False
        try:
            if key in (ord('a'),ord('A')):
                self.session.attested = True
                self.message = 'Confirmed: measured MARKER vertical projection; chassis-center input is unsupported.'
            elif key in (ord('t'),ord('T'),ord('v'),ord('V')):
                self.mode = 'training' if key in (ord('t'),ord('T')) else 'validation'
                self.fields = ['', '']; self.focus = 0
                window.clear()
            elif key == 9:
                self.focus = 1-self.focus
            elif key in (8,127):
                self.fields[self.focus] = self.fields[self.focus][:-1]
            elif key == ord('c') or key == ord('C'):
                self.fields = ['', '']; self.focus = 0; window.clear()
            elif key in range(ord('0'),ord('9')+1) or key in (ord('.'),ord('-')):
                if len(self.fields[self.focus]) < 12:
                    self.fields[self.focus] += chr(key)
            elif key == ord(' '):
                if self.preview:
                    raise ValueError('Press P to leave read-only preview before capturing')
                quality = self.session.capture(self.mode,[float(f) for f in self.fields],window,now)
                self.message = f'Captured {self.mode}: {quality["inlier_frames"]} inlier frames, radius {quality["maximum_inlier_radius_px"]:.2f} px. Move to a NEW location.'
                if self.session.resume_source is not None:
                    self.message += f' Saved: {self.session.save_points(self.points_path)}'
            elif key in (ord('u'),ord('U')):
                self.session.undo(); window.clear(); self.preview = False
                self.message = 'Last point removed; fit invalidated. Collect fresh frames to retry.'
            elif key in (ord('s'),ord('S')):
                self.message = f'Measurements saved: {self.session.save_points(self.points_path)}'
            elif key in (ord('f'),ord('F')):
                self.preview = False
                self.session.save_points(self.points_path)  # Preserve even if fit fails.
                try:
                    report = self.session.fit()
                finally:
                    if self.session.report:
                        print('Independent validation:',json_line(self.session.report))
                        self.session.save_points(self.points_path)
                self.report_page = 0
                self.message = f'PASS: max {report["maximum_error_cm"]:.3f} cm. Review all validation rows, then M saves mapping.'
            elif key in (ord('m'),ord('M')):
                self.message = f'Mapping saved: {self.session.save_mapping(self.mapping_path)}. P opens read-only test.'
            elif key in (ord('p'),ord('P')):
                if self.session.saved_mapping is None:
                    raise ValueError('Save a passing mapping with M before live testing')
                self.preview = not self.preview
                window.clear()
            elif key in (ord('['),ord(']')):
                count = len(self.session.report['points']) if self.session.report else 0
                self.report_page = max(0,min(max(0,(count-1)//5),self.report_page+(1 if key == ord(']') else -1)))
        except (ValueError,OSError,cv2.error) as exc:
            self.message = str(exc)
            if self.session.report and not self.session.report['tolerance_satisfied']:
                self.message += '; inspect validation rows, ruler positions/height, camera pose. U undoes last point.'
        return True

    def render(self, frame, observation, window, now):
        view = frame.copy()
        if observation:
            cv2.polylines(view,[np.rint(observation.corners_pixels).astype(np.int32)],True,(0,255,0),2)
            cv2.circle(view,tuple(np.rint(observation.center_pixels).astype(int)),5,(0,255,255),-1)
        extra = []
        if self.preview and self.session.preview_tracker:
            pose = self.session.preview_tracker.update(frame)
            draw_robot_pose(view,pose)
            extra.append('READ-ONLY TEST: heading alignment NOT physically verified')
            if observation:
                marker = self.session.mapping.map([observation.center_pixels])[0]
                extra.append(f'Marker ground X={marker[0]:.2f} Y={marker[1]:.2f} cm')
            if pose.position_cm:
                extra.append(f'Chassis X={pose.position_cm["x"]:.2f} Y={pose.position_cm["y"]:.2f} cm; theta={pose.heading_deg:.1f}')
                c = self.session.config
                if c.body_length_cm is not None:
                    a = math.radians(pose.heading_deg)
                    forward=np.array([math.cos(a),math.sin(a)]);left=np.array([-math.sin(a),math.cos(a)])
                    center=np.array([pose.position_cm['x'],pose.position_cm['y']])
                    world=[center+s*c.body_length_cm/2*forward+t*c.body_width_cm/2*left for s,t in [(-1,-1),(1,-1),(1,1),(-1,1)]]
                    pixels=[self.session.floor.world_to_pixel(*p) for p in world]
                    cv2.polylines(view,[np.rint(pixels).astype(np.int32)],True,(255,200,0),2)
            extra.append(f'{pose.tracking_status} / {pose.localization_status}; current detection={pose.pose_quality["observation_current"]}')
        # A separate panel never covers source image corners. Resize affects display only.
        panel_width = 780
        canvas = np.zeros((max(frame.shape[0],760),frame.shape[1]+panel_width,3),np.uint8)
        canvas[:frame.shape[0],:frame.shape[1]] = view
        state = f'VISIBLE ID {observation.marker_id} pixel=({observation.center_pixels[0]:.2f},{observation.center_pixels[1]:.2f})' if observation else 'NO CURRENT VALID MARKER'
        counts = self.session.points
        lines = ['ROBOT MARKER-PLANE CALIBRATION',
                 'MEASURE: marker CENTER vertical projection onto FLOOR.',
                 'NOT chassis center. A = confirm ruler/plumb measurements.',
                 f'Height {self.session.config.mounting_height_cm:g} cm | marker size setting {self.session.config.marker_size_cm:g} cm',
                 f'Mode {self.mode.upper()} | next #{len(counts[self.mode])+1} | training {len(counts["training"])}/4+ validation {len(counts["validation"])}/3+',
                 f'{">" if self.focus==0 else " "} X cm: {self.fields[0]}   {">" if self.focus==1 else " "} Y cm: {self.fields[1]}   TAB selects X/Y',
                 state, f'Recent distinct detected frames: {len(window.samples)}; hold still >=0.3 s',
                 'Digits/dot/backspace edit | C clear input/retry | SPACE capture',
                 'T training | V validation | U undo | S save points snapshot',
                 'F fit/review | M save passing map | P live test | Q save points/quit',
                 '[ / ] validation pages | No robot movement is authorized.']
        try:
            _,quality=window.representative(now)
            lines.append(f'Stable: radius {quality["maximum_inlier_radius_px"]:.2f} px, drift {quality["drift_px"]:.2f} px; limit 2 px')
        except ValueError:
            lines.append('Capture not ready / moving / replay: SPACE explains; C discards window')
        suggestions = [p for p in SUGGESTIONS[self.mode] if self.session.suggestion_clearance(p)]
        lines += ['Suggestions only (not measurements), clearance for all headings:',str(suggestions) if suggestions else 'No suggested placements pass conservative full-body clearance.']
        if self.session.report:
            r = self.session.report
            lines.append(f'Independent validation: {"PASS" if r["tolerance_satisfied"] else "FAIL"} RMS {r["rms_error_cm"]:.3f} max {r["maximum_error_cm"]:.3f} cm <= {r["tolerance_cm"]:g}')
            lines.append(f'Errors = mapped minus measured; page {self.report_page+1}; [ / ] changes page')
            for p in r['points'][self.report_page*5:(self.report_page+1)*5]:
                lines.append(f'V{p["index"]}: dX {p["error_x_cm"]:+.3f} dY {p["error_y_cm"]:+.3f} distance {p["euclidean_error_cm"]:.3f} cm')
        lines += extra
        lines += [''] + textwrap.wrap(self.message,90)
        required_height = 24+len(lines)*23
        if required_height > canvas.shape[0]:
            canvas = cv2.copyMakeBorder(canvas,0,required_height-canvas.shape[0],0,0,cv2.BORDER_CONSTANT)
        for i,line in enumerate(lines):
            cv2.putText(canvas,line,(frame.shape[1]+12,24+i*23),cv2.FONT_HERSHEY_SIMPLEX,.48,(230,230,230),1,cv2.LINE_AA)
        return canvas


def run_wizard(args):
    session = CalibrationSession(args.calibration,args.robot_settings,height_cm=args.mounting_height_cm,
                                 tolerance_cm=args.tolerance_cm,camera=args.camera)
    outputs = [Path(args.points_output),Path(args.output)]
    if len({p.resolve() for p in outputs}) != 2 or any(str(p.resolve()) in session.inputs for p in outputs):
        raise ValueError('Points and mapping outputs must be distinct from each other and inputs')
    if outputs[1].exists() or (outputs[0].exists() and not args.resume):
        raise ValueError('Output exists; choose new --points-output and --output paths')
    if (args.width,args.height) != (session.floor.frame_width,session.floor.frame_height):
        raise ValueError('Requested frame dimensions differ from arena calibration')
    if args.resume:
        session.resume_points(args.points_output)
        print(f'Resumed {args.points_output}: training {len(session.points["training"])}, validation {len(session.points["validation"])}; source will not be overwritten')
    detector = RobotMarkerDetector(session.config)
    window = CaptureWindow()
    ui = WizardUI(session,args.points_output,args.output)
    cv2.namedWindow(TITLE,cv2.WINDOW_NORMAL)
    cv2.resizeWindow(TITLE,1440,760)
    try:
        with CameraManager(args.camera,args.width,args.height,args.fps) as camera:
            while True:
                frame = camera.read()
                received_at = time.monotonic()
                received_utc = datetime.now(timezone.utc).isoformat()
                if frame is None:
                    break
                if frame.shape[1::-1] != (session.floor.frame_width,session.floor.frame_height):
                    raise ValueError('Actual camera frame dimensions differ from arena calibration')
                observation = detector.detect(frame)
                window.add(frame,observation,received_at,received_utc)
                cv2.imshow(TITLE,ui.render(frame,observation,window,received_at))
                key = cv2.waitKey(1)
                if cv2.getWindowProperty(TITLE,cv2.WND_PROP_VISIBLE)<1:
                    break
                if key != -1 and not ui.key(key,window,time.monotonic()):
                    break
    finally:
        try:
            if session.captures:
                path=session.save_points(args.points_output)
                print(f'Measurements preserved: {path}')
        finally:
            try:
                cv2.destroyWindow(TITLE)
            except cv2.error:
                pass  # The operator may already have closed the native window.
