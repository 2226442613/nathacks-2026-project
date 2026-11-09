"""Offline-first Module 02 commands; native debug windows are optional."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import replace
import json
import logging
import hashlib
from pathlib import Path
import time

import cv2
import numpy as np

from .calibration import DualPlaneCalibration, load_calibration
from .camera import CameraManager
from .spill_detection import (BackgroundReference, DetectionError, DetectionResult,
                              SpillCategory, SpillDetector, render_detections, validate_image)
from .spill_tracking import SpillTracker, load_spill_settings
from .exclusion import load_exclusion_polygons
from .serialization import json_line
from .frame_geometry import GeometryCheck
from .framing_session import FramingSession

logger = logging.getLogger(__name__)


def add_spill_commands(commands) -> None:
    reference = commands.add_parser('background-reference', help='Explicitly register the initial legal scene')
    detection = commands.add_parser('detect-spills', help='Detect new regions relative to a registered legal scene')
    for parser in (reference, detection):
        sources = parser.add_mutually_exclusive_group(required=True)
        sources.add_argument('--image', help='Local BGR-compatible image file')
        sources.add_argument('--video', help='Local video file (no streaming)')
        sources.add_argument('--camera', type=int, help='macOS camera index, including iPhone Continuity Camera')
        parser.add_argument('--confirm-clean', action='store_true', required=True,
                            help='Explicitly trust everything in the baseline as legal; required')
        parser.add_argument('--width', type=int, default=1280, help='Requested camera width (default: 1280)')
        parser.add_argument('--height', type=int, default=720, help='Requested camera height (default: 720)')
        parser.add_argument('--fps', type=float, default=30, help='Requested camera FPS (default: 30)')
    reference.add_argument('--output', required=True, help='Destination lossless reference .png')
    reference.add_argument('--frame-index', type=int, default=0, help='Zero-based frame to capture from video/camera (default: 0)')
    reference.add_argument('--reference-frames', type=int, help='Aggregate 1-31 legal frames (default: 15 camera/video, 1 image)')
    reference.add_argument('--overwrite', action='store_true', help='Explicitly replace an existing registered baseline')
    reference.add_argument('--aggregation', choices=('median','mean'), default='median', help='Multi-frame aggregation method (default: median)')
    # Reference extraction has no GUI. Detection opts out explicitly for batch runs.
    reference.set_defaults(headless=True)
    detection.add_argument('--background', required=True, help='Registered legal baseline with identical frame geometry')
    detection.add_argument('--calibration', help='Module 01 JSON for pixel ROI; --localize additionally enables cm mapping')
    detection.add_argument('--exclusion-mask', help='Grayscale image: nonzero pixels ignore robot/irrelevant areas')
    detection.add_argument('--exclusion-polygons', help='Manual ignored pixel polygons JSON with frame_width/frame_height')
    detection.add_argument('--no-roi', action='store_true', help='Explicitly analyze the full image even if calibration polygons are supplied')
    detection.add_argument('--settings', default='config/spill_config.json', help='Detector/tracker threshold JSON')
    detection.add_argument('--confirmation-frames', type=int, help='Override configured confirmation streak; 1 for single-image inspection')
    detection.add_argument('--headless', action='store_true', help='Process without any OpenCV GUI calls')
    detection.add_argument('--debug', action='store_true', help='Show pending candidates, contours and detailed diagnostics')
    detection.add_argument('--show-masks', action='store_true', help='Display raw/corrected difference, raw/filtered foreground, and red masks')
    detection.add_argument('--no-polygons', action='store_true', help='Hide calibration polygon overlays (ROI still applies)')
    detection.add_argument('--output', help='Write one JSON record per frame (JSONL); otherwise stdout')
    detection.add_argument('--debug-dir', help='Save input/annotated frames, raw/corrected difference, raw/filtered foreground, red and analysis masks')
    detection.add_argument('--diagnostic-dir', help='Collect a read-only diagnostic run in a NEW directory; disables interactive baseline replacement')
    detection.add_argument('--diagnostic-panels', action='store_true', help='Save a nine-panel stage view with --diagnostic-dir; display it unless --headless')
    detection.add_argument('--max-frames', type=int, help='Optional positive processing limit for video/camera')
    detection.add_argument('--reregister-frames', type=int, default=15,
                           help='Median frames after interactive R then Y (3-31, default: 15)')
    detection.add_argument('--track-robot',action='store_true',help='Track one ArUco robot and dynamically exclude its approximate body region')
    detection.add_argument('--robot-settings',default='config/robot_config.json')
    detection.add_argument('--robot-marker-plane',help='Optional independent fixed-height marker-plane mapping')
    detection.add_argument('--localize',action='store_true',help='Enable Module 03 mask surface ownership and diagnostic cm coordinates')
    detection.add_argument('--localization-boundaries',action='store_true',help='Show original outlines and estimated physical dimensions/area with --localize')
    detection.add_argument('--localization-settings',default='config/localization_config.json',help='Module 03 threshold JSON')
    detection.add_argument('--localization-reference',help='Image explicitly declared to show the original calibration camera pose')
    detection.add_argument('--localization-validation',help='Independent measured-point report for this calibration/reference')
    detection.add_argument('--target-events',help='Append target events (default: OUTPUT.targets.jsonl, or data/localization_targets.jsonl)')
    validation = commands.add_parser('validate-localization',help='Report independently measured cm residuals without rewriting calibration')
    validation.add_argument('--calibration',required=True)
    validation.add_argument('--reference',required=True,help='Original calibration-view image')
    validation.add_argument('--points',required=True,help='Module 01 format [{surface,pixel,expected_cm}] independent measurements')
    validation.add_argument('--tolerance-cm',type=float,default=2.)
    validation.add_argument('--confirm-measured',action='store_true',required=True,help='Attest points were independently physically measured')
    validation.add_argument('--output',required=True)
    validation.set_defaults(headless=True)


def _source(args) -> int | str:
    return args.video if args.video is not None else args.camera


def _load_image(path: str):
    image = cv2.imread(path, cv2.IMREAD_COLOR)
    if image is None:
        raise DetectionError(f'Cannot read image: {path}')
    return image


def capture_background(args) -> None:
    if not args.confirm_clean:
        raise DetectionError('Explicit clean-reference confirmation is required')
    if args.frame_index < 0:
        raise DetectionError('frame-index must be nonnegative')
    count = args.reference_frames if args.reference_frames is not None else (1 if args.image else 15)
    if not 1 <= count <= 31:
        raise DetectionError('reference-frames must be between 1 and 31')
    path = Path(args.output)
    if not args.overwrite and (path.exists() or Path(str(path)+'.json').exists()):
        raise DetectionError('Baseline already exists; use a new path or --overwrite to explicitly re-register')
    if args.image:
        if args.frame_index != 0 or count != 1:
            raise DetectionError('frame-index and multi-frame aggregation only apply to video/camera')
        reference = BackgroundReference(_load_image(args.image), confirmed_clean=args.confirm_clean)
    else:
        with CameraManager(_source(args), args.width, args.height, args.fps) as camera:
            frames = []
            for index in range(args.frame_index + count):
                frame = camera.read()
                if frame is None:
                    raise DetectionError('Video ended before all requested legal-scene frames were captured')
                if index >= args.frame_index:
                    frames.append(frame.copy())
            reference = BackgroundReference.from_frames(frames, method=args.aggregation, confirmed_clean=args.confirm_clean)
    reference._metadata.update(source=args.image or _source(args), operator_registered=True,
                               requested_fps=args.fps, start_frame_index=args.frame_index)
    reference.save(args.output, overwrite=args.overwrite)
    logger.info('Legal baseline REGISTERED: %s, metadata: %s.json (%s)', args.output,args.output,reference.metadata)


def _visible(window: str) -> bool:
    try:
        return cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def _save_debug(directory: Path, index: int, annotated, result: DetectionResult, frame, record_json: str) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for suffix, image in (('input', frame), ('annotated', annotated), ('difference', result.difference),
                          ('raw_difference', result.raw_difference), ('raw_foreground', result.raw_foreground_mask),
                          ('foreground', result.foreground_mask), ('red', result.red_mask),
                          ('analysis', result.analysis_mask)):
        path = directory / f'frame_{index:06d}_{suffix}.png'
        if not cv2.imwrite(str(path), image):
            raise DetectionError(f'Cannot save debug image: {path}')
    (directory/f'frame_{index:06d}_diagnostics.json').write_text(record_json+'\n',encoding='utf-8')
    if result.component_labels is not None:
        np.savez_compressed(directory/f'frame_{index:06d}_components.npz',component_labels=result.component_labels)


def calibration_diagnostics(calibration, *, roi_enabled: bool) -> dict:
    """Describe loaded ROI/quality data without claiming verified physical accuracy."""
    if calibration is None:
        return {'status': 'NOT_LOADED', 'roi_mode': 'FULL_IMAGE',
                'quality_warnings': [], 'warnings_accepted': False}
    if isinstance(calibration, DualPlaneCalibration):
        quality = calibration.quality_report
        messages = quality['warnings']
        status = 'DUAL_PLANE_WITH_WARNINGS' if messages else 'DUAL_PLANE'
        accepted = quality['accepted_warnings']
    else:
        messages, accepted, status = [], False, 'FLOOR_ONLY'
    return {'status': status, 'roi_mode': 'CALIBRATED' if roi_enabled else 'FULL_IMAGE',
            'quality_warnings': messages, 'warnings_accepted': accepted}


def frame_diagnostics(frame, reference: BackgroundReference, calibration_status: dict,
                       detections, fps: float, *, input_mode: str, source: int | str) -> dict:
    """Machine-readable field diagnostics, also used by the native preview overlay."""
    counts = {category.value: 0 for category in SpillCategory}
    for target in detections:
        counts[target.category.value] += 1
    confirmed = sum(target.confirmed for target in detections)
    weak_reasons = ('weak color difference', 'unreliable shape', 'high edge density', 'ambiguous color')
    weak = sum(target.category == SpillCategory.UNKNOWN and any(reason in target.classification_reason for reason in weak_reasons)
               for target in detections)
    width, height = reference.resolution
    return {'input_mode': input_mode, 'source': source,
            'frame_resolution': {'width': frame.shape[1], 'height': frame.shape[0]},
            'processing_fps': float(fps),
            'background': {'status': 'READY', 'width': width, 'height': height,
                           'clean_confirmed_by_operator': reference.metadata['confirmed_clean']},
            'calibration': calibration_status, 'target_count': len(detections),
            'confirmed_count': int(confirmed), 'pending_count': len(detections) - int(confirmed),
            'weak_evidence_count': int(weak), 'category_counts': counts}


def draw_diagnostics(image, diagnostics: dict, geometry_status: str, *, debug: bool = True) -> None:
    resolution = diagnostics['frame_resolution']
    calibration = diagnostics['calibration']
    labels = {'NOT_LOADED': 'NONE', 'FLOOR_ONLY': 'FLOOR', 'DUAL_PLANE': 'DUAL', 'DUAL_PLANE_WITH_WARNINGS': 'DUAL+WARN'}
    categories = diagnostics['category_counts']
    lines = (
        f"{resolution['width']}x{resolution['height']} | FPS {diagnostics['processing_fps']:.1f} | {diagnostics['input_mode']}",
        f"BG READY | CAL {labels[calibration['status']]} | ROI {'ON' if calibration['roi_mode'] == 'CALIBRATED' else 'FULL'}",
        f"GEOMETRY {geometry_status} | targets {diagnostics['target_count']}",
        f"confirmed {diagnostics['confirmed_count']} | pending {diagnostics['pending_count']} | weak {diagnostics['weak_evidence_count']}",
        f"RED {categories['RED_SPILL']} | NONRED {categories['NON_RED_SPILL']} | UNKNOWN {categories['UNKNOWN']}")
    if not debug:
        lines = (lines[0], f"BASELINE REGISTERED | confirmed {diagnostics['confirmed_count']}",
                 'CALIBRATION: ESTIMATED | UNVERIFIED')
    for row, text in enumerate(lines):
        position = (8, 17 + row * 17)
        if position[1] >= image.shape[0]-2:
            break
        cv2.putText(image,text,position,0,.43,(0,0,0),3)
        cv2.putText(image,text,position,0,.43,(255,255,0),1)


def _paused_result(frame, reference, geometry) -> DetectionResult:
    validate_image(frame)
    if frame.shape[:2] != reference.resolution[::-1]:
        raise DetectionError('Frame resolution changed while awaiting operator; restart with compatible inputs')
    difference = cv2.absdiff(frame,reference.image).max(axis=2)
    empty = np.zeros(frame.shape[:2],np.uint8)
    return DetectionResult([],difference,empty,empty,empty,geometry,
                           raw_foreground_mask=empty,raw_difference=difference,detection_paused=True)


def run_detection(args) -> None:
    diagnostic_dir = getattr(args,'diagnostic_dir',None)
    diagnostic_panels = getattr(args,'diagnostic_panels',False)
    if diagnostic_panels and not diagnostic_dir:
        raise DetectionError('--diagnostic-panels requires --diagnostic-dir')
    if not args.confirm_clean:
        raise DetectionError('Explicit clean-reference confirmation is required')
    if args.max_frames is not None and args.max_frames < 1:
        raise DetectionError('max-frames must be positive')
    if not 3 <= args.reregister_frames <= 31:
        raise DetectionError('reregister-frames must be between 3 and 31')
    detector_config, tracker_config = load_spill_settings(args.settings)
    if args.no_roi:
        detector_config = replace(detector_config, roi_enabled=False)
    if args.confirmation_frames is not None:
        tracker_config = replace(tracker_config, confirmation_frames=args.confirmation_frames)
    if not Path(args.background).is_file():
        raise DetectionError('No registered baseline found; explicitly run background-reference --confirm-clean first')
    reference = BackgroundReference.load(args.background, confirmed_clean=args.confirm_clean)
    localization_error = None
    source_warning = None
    try:
        calibration = load_calibration(args.calibration) if args.calibration else None
    except ValueError as exc:
        if not args.localize:
            raise
        logger.error('Module 03 calibration invalid; visual detection continues: %s',exc)
        calibration,localization_error = None,str(exc)
    detector_calibration = calibration
    if calibration is not None:
        if args.camera is not None and calibration.source != args.camera:
            source_warning = 'Camera source index differs from saved calibration; operator-managed estimate'
            logger.debug(source_warning)
        plane = calibration.floor if isinstance(calibration,DualPlaneCalibration) else calibration
        if args.localize and (plane.frame_width,plane.frame_height)!=reference.resolution:
            localization_error = 'Frame dimensions differ from calibration'
            detector_calibration = None
        logger.warning('ROI polygons require the same camera pose, crop, and resolution as calibration; '
                       'matching dimensions alone cannot validate offline image geometry')
    exclusion = None
    if args.exclusion_mask:
        exclusion = cv2.imread(args.exclusion_mask, cv2.IMREAD_GRAYSCALE)
        if exclusion is None:
            raise DetectionError(f'Cannot read exclusion mask: {args.exclusion_mask}')
        if exclusion.shape != reference.resolution[::-1]:
            raise DetectionError('Exclusion mask resolution differs from background reference')
    if args.exclusion_polygons:
        polygon_mask = load_exclusion_polygons(args.exclusion_polygons, reference.resolution[::-1])
        exclusion = polygon_mask if exclusion is None else cv2.bitwise_or(exclusion, polygon_mask)
    detector = SpillDetector(reference, detector_config, detector_calibration, exclusion)
    calibration_status = calibration_diagnostics(detector_calibration, roi_enabled=detector_config.roi_enabled)
    logger.info('Legal baseline LOADED / Background READY: %sx%s; calibration %s; ROI %s', *reference.resolution,
                calibration_status['status'], calibration_status['roi_mode'])
    tracker = SpillTracker(tracker_config)
    session = FramingSession(capture_count=args.reregister_frames,
                             calibration_may_be_invalid=bool(source_warning) or reference.metadata.get('calibration_may_be_invalid',False) is True)
    interactive = not args.headless and args.camera is not None and not diagnostic_dir
    debug = args.debug or args.show_masks or bool(args.debug_dir) or bool(diagnostic_dir)
    last_geometry = GeometryCheck('UNVERIFIED','Waiting for current camera registration')
    localizer = bridge = event_store = None
    epoch = 0
    baseline_changed = False
    if args.localization_boundaries and not args.localize:
        raise DetectionError('--localization-boundaries requires --localize')
    if args.target_events and not args.localize:
        raise DetectionError('--target-events requires --localize')
    if args.localize:
        from .localization import SurfaceLocalizer, LocalizationConfig, CameraRegistration, utc_timestamp
        from .localization_runtime import LocalizationRegistration, TargetEventStore, draw_localizations
        settings = json.loads(Path(args.localization_settings).read_text(encoding='utf-8'))
        evidence = json.loads(Path(args.localization_validation).read_text(encoding='utf-8')) if args.localization_validation else None
        calibration_id = ('sha256:'+hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest()) if args.calibration and Path(args.calibration).is_file() else None
        localizer = SurfaceLocalizer(calibration,LocalizationConfig(**settings),calibration_id=calibration_id,
                                     physical_evidence=evidence,calibration_error=localization_error)
        if source_warning:
            localizer.quality_warnings.append(source_warning)
        view = cv2.imread(args.localization_reference,cv2.IMREAD_COLOR) if args.localization_reference else reference.image
        if view is not None:
            bridge = LocalizationRegistration(view,reference.image,declared=bool(args.localization_reference),
                baseline_unverified=session.calibration_may_be_invalid,config=detector_config)
        events_path = args.target_events or (args.output+'.targets.jsonl' if args.output else 'data/localization_targets.jsonl')
        protected = [args.image,args.video,args.background,args.background+'.json',args.calibration,args.settings,
                     args.localization_reference,args.localization_validation,args.localization_settings,args.output,
                     args.exclusion_mask,args.exclusion_polygons]
        if any(value and Path(value).resolve()==Path(events_path).resolve() for value in protected):
            raise DetectionError('Target events must not overwrite input files or detection output')
        if args.output and any(value and Path(value).resolve()==Path(args.output).resolve() for value in
                               [args.localization_reference,args.localization_validation,args.localization_settings]):
            raise DetectionError('Detection output must not overwrite localization inputs')
        event_store = TargetEventStore(events_path)
        logger.info('Module 03 enabled: calibration %s; target events append to %s; physical coordinates default UNVERIFIED',calibration_id,events_path)

    robot_tracker = robot_guard = None
    if args.track_robot:
        from .robot_tracking import RobotTracker, draw_robot_pose
        from .robot_cli import load_robot_config
        from .robot_plane import MarkerPlaneMapping
        from .localization import CameraRegistration
        from .frame_geometry import FrameGeometryGuard
        config = load_robot_config(args.robot_settings)
        robot_plane = MarkerPlaneMapping(json.loads(Path(args.robot_marker_plane).read_text())) if args.robot_marker_plane else None
        robot_id = 'sha256:'+hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest() if args.calibration and Path(args.calibration).is_file() else None
        robot_tracker = RobotTracker(calibration,config,calibration_id=robot_id,marker_plane=robot_plane)
        robot_guard = FrameGeometryGuard(reference.image)
        extra_inputs = [args.robot_settings,args.robot_marker_plane]
        robot_events_path = (args.target_events or (args.output+'.targets.jsonl' if args.output else 'data/localization_targets.jsonl')) if args.localize else None
        for output_path in (args.output,robot_events_path):
            if output_path and any(v and Path(v).resolve()==Path(output_path).resolve() for v in extra_inputs):
                raise DetectionError('Robot outputs must not overwrite settings or marker-plane input')

    def reset_tracking():
        nonlocal epoch
        tracker.reset()
        epoch += 1
        if event_store:
            event_store.deactivate_all('tracking_reset')
    if args.image and tracker_config.confirmation_frames > 1:
        logger.info('Single-image candidates remain pending; use --confirmation-frames 1 for explicit single-frame confirmation')
    with ExitStack() as stack:
        diagnostic = None
        if diagnostic_dir:
            from .spill_diagnostics import DiagnosticSession
            diagnostic = stack.enter_context(DiagnosticSession(diagnostic_dir,background=args.background,
                reference=reference,detector_config=detector_config,tracker_config=tracker_config,
                calibration=args.calibration, fps=args.fps,
                inputs=[args.background,args.background+'.json',args.calibration,args.settings,
                        args.exclusion_mask,args.exclusion_polygons,__file__,
                        Path(__file__).with_name('spill_detection.py'),Path(__file__).with_name('frame_geometry.py'),
                        Path(__file__).with_name('spill_tracking.py'),Path(__file__).with_name('red_classification.py')],
                panels=diagnostic_panels))
            diagnostic.manifest['input_metadata']={'camera':args.camera,'video':args.video,'image':args.image,
                'requested_width':args.width,'requested_height':args.height,'requested_fps':args.fps,
                'calibration':args.calibration,'roi_enabled':detector_config.roi_enabled}
            diagnostic._save_manifest()
        if event_store:
            stack.callback(event_store.deactivate_all,'session_exit')
        output = None
        if args.output:
            path = Path(args.output)
            # Avoid truncating an input if the user accidentally reuses its path.
            input_paths = [args.image, args.video, args.background, args.background+'.json', args.calibration,
                           args.exclusion_mask, args.exclusion_polygons, args.settings]
            if any(value and Path(value).resolve() == path.resolve() for value in input_paths):
                raise DetectionError('Detection output must not overwrite an input file')
            path.parent.mkdir(parents=True, exist_ok=True)
            output = stack.enter_context(path.open('w', encoding='utf-8'))
        if args.image:
            still_frame = _load_image(args.image)
            camera = None
        else:
            camera = stack.enter_context(CameraManager(_source(args), args.width, args.height, args.fps))
            still_frame = None
        index = 0
        previous = time.perf_counter()
        previous_geometry_status = None
        previous_pause = None
        while args.max_frames is None or index < args.max_frames:
            frame = still_frame if still_frame is not None else camera.read()
            if frame is None:
                logger.info('Video ended or decoding stopped')
                break
            observation = diagnostic.mark_capture() if diagnostic else None
            robot_pose = robot_mask = None
            if robot_tracker:
                check = robot_guard.check(frame)
                robot_reg = (CameraRegistration.from_reference_to_current(robot_guard.transform,frame.shape[1::-1])
                             if check.status=='MATCHED' else CameraRegistration(status=check.status,reason=check.reason))
                robot_pose = robot_tracker.update(frame,registration=robot_reg)
                robot_mask = robot_tracker.exclusion_mask(robot_pose,frame.shape[1::-1])
            if session.state == 'BASELINE_REREGISTERING':
                result = _paused_result(frame,reference,last_geometry)
                session.frames.append(frame.copy())
                if len(session.frames) == session.capture_count:
                    try:
                        fresh = BackgroundReference.from_frames(session.frames,method='median',confirmed_clean=True)
                        fresh._metadata.update(source=_source(args), operator_registered=True,
                                               registration_trigger='interactive_R_then_Y',
                                               calibration_may_be_invalid=True,
                                               physical_coordinates_verified=False)
                        fresh_detector = SpillDetector(fresh,detector_config,detector_calibration,exclusion)
                        fresh.save(args.background,overwrite=True)
                    except (OSError, ValueError, cv2.error) as exc:
                        logger.exception('Interactive baseline replacement failed; previous active reference retained')
                        session.registration_failed(str(exc))
                    else:
                        reference, detector = fresh, fresh_detector
                        reset_tracking()
                        baseline_changed = True
                        session.registered()
                        last_geometry = GeometryCheck('UNVERIFIED','New baseline registered; checking next frame')
                        result = _paused_result(frame,reference,last_geometry)
            else:
                options = {'color_diagnostics':True} if debug else {}
                if robot_mask is not None:
                    options['exclusion_mask'] = robot_mask
                result = detector.detect(frame,**options)
                last_geometry = result.geometry
                if result.geometry.status in ('MOVED','UNVERIFIED'):
                    session.mismatch(result.geometry.reason)
            if result.geometry.status != previous_geometry_status:
                if result.geometry.status == 'UNVERIFIED':
                    logger.warning('Camera geometry UNVERIFIED: %s', result.geometry.reason)
                else:
                    logger.info('Camera geometry: %s', result.geometry.status)
                previous_geometry_status = result.geometry.status
            detections = tracker.update(result.detections)
            localized, target_events = [],[]
            if localizer:
                registration = bridge.for_frame(frame,result,baseline_changed=baseline_changed,
                    override_active=session.override_active) if bridge else CameraRegistration(reason='Calibration-view image unavailable')
                localized = localizer.localize_detections(result,detections,frame_size=(frame.shape[1],frame.shape[0]),registration=registration)
                target_events = event_store.update(localized,epoch=epoch,timestamp=utc_timestamp())
            now = time.perf_counter()
            fps = 1 / max(now - previous, 1e-9)
            previous = now
            input_mode = 'CAMERA' if args.camera is not None else ('IMAGE' if args.image else 'VIDEO')
            diagnostics = frame_diagnostics(frame, reference, calibration_status, detections, fps,
                                            input_mode=input_mode, source=args.image or _source(args))
            diagnostics.update(detection_paused=result.detection_paused, alignment_applied=result.alignment_applied)
            diagnostics['framing'] = session.diagnostics()
            diagnostics['calibration'] = dict(calibration_status,physical_coordinates_verified=False,
                physical_coordinate_status=('UNVERIFIED_AFTER_FRAMING_CHANGE' if session.calibration_may_be_invalid
                                            else 'NOT_VERIFIED_BY_MODULE_02'), navigation_allowed=False)
            if result.detection_paused != previous_pause:
                if result.detection_paused:
                    logger.info('Detection paused for explicit baseline capture: geometry %s, state %s',
                                   result.geometry.status,session.state)
                elif previous_pause:
                    logger.info('Detection resumed: framing state %s, geometry %s',session.state,result.geometry.status)
                previous_pause = result.detection_paused
            try:
                record = {'frame_index': index, 'detections': [target.to_dict() for target in detections],
                          'geometry': result.geometry.to_dict(), 'illumination_offset_bgr': list(result.illumination_offset_bgr),
                          'illumination_gain_bgr': result.illumination_gain_bgr,
                          'diagnostics': diagnostics}
                if robot_pose is not None:
                    record['robot_pose'] = robot_pose.to_dict()
                    record['robot_exclusion'] = {'approximate':True,'current_marker_only':True,'pixel_count':int(np.count_nonzero(robot_mask))}
                if localizer:
                    record.update(localizations=[item.to_dict() for item in localized],target_events=target_events,target_message_schema_version=1)
                if debug:
                    record['color_diagnostics'] = [dict(result.color_diagnostics[target.component_id],
                        target_id=target.target_id,confirmed=target.confirmed,
                        classification_reason=target.classification_reason,
                        color_source='original_bgr_frame',coordinate_system='original_image_pixels')
                        for target in detections if target.component_id in result.color_diagnostics]
                line = json_line(record)
            except (ValueError, TypeError) as exc:
                logger.exception('Detection JSON serialization failed at frame %d, source %r, output %r',
                                 index, args.image or _source(args), args.output)
                raise DetectionError(f'Detection JSON failed at frame {index}, source {args.image or _source(args)!r}: {exc}') from exc
            if output is not None:
                output.write(line + '\n')
            else:
                print(line)
            panel = diagnostic.write(index,frame,result,detections,record,observation=observation) if diagnostic else None
            if not args.headless or args.debug_dir:
                annotated = render_detections(frame, detections, None if args.no_polygons else detector.polygons, debug=debug)
                draw_diagnostics(annotated, diagnostics, result.geometry.status, debug=debug)
                framing_bottom = session.draw(annotated,debug=debug)
                if localizer:
                    draw_localizations(annotated,localized,top=framing_bottom+10,boundaries=args.localization_boundaries)
                if robot_pose is not None:
                    draw_robot_pose(annotated,robot_pose,fps=fps,top=max(framing_bottom+10,frame.shape[0]-85))
                if args.debug_dir:
                    _save_debug(Path(args.debug_dir), index, annotated, result, frame,line)
            if not args.headless:
                title = 'Hospital Guardian - new objects | ' + ('A-F: label scene / Q: exit' if diagnostic else
                                                              'R: register / Q: exit' if interactive else 'Q exit')
                if diagnostic:
                    cv2.putText(annotated,f'DIAGNOSTIC STAGE {diagnostic.stage or "UNLABELED"} | A-F mark next frame | Q exit',
                                (8,frame.shape[0]-10),0,.43,(0,200,255),1)
                cv2.imshow(title, annotated)
                windows = [title]
                if panel is not None:
                    cv2.imshow('Hospital Guardian - diagnostic stages',panel)
                    windows.append('Hospital Guardian - diagnostic stages')
                if args.show_masks and session.state not in ('WAITING_FOR_OPERATOR','BASELINE_REREGISTERING'):
                    for name, mask in (('Raw difference', result.raw_difference), ('Difference', result.difference),
                                       ('Raw foreground', result.raw_foreground_mask),
                                       ('Foreground', result.foreground_mask), ('Red', result.red_mask)):
                        cv2.imshow(name, mask)
                        windows.append(name)
                while True:
                    key = cv2.waitKey(20 if still_frame is not None else 1) & 0xFF
                    if diagnostic and chr(key).upper() in 'ABCDEF':
                        diagnostic.mark_stage(chr(key).upper(),index+1)
                    action = session.key(key) if interactive else None
                    if action == 'quit' or key in (27, ord('q'), ord('Q')) or not all(_visible(name) for name in windows):
                        return
                    if action == 'confirm_registration':
                        reset_tracking()
                    if still_frame is None:
                        break
            index += 1
            if still_frame is not None:
                break


def run_spill_command(args) -> None:
    if args.command == 'background-reference':
        capture_background(args)
    elif args.command == 'validate-localization':
        from .localization_validation import run_validation
        run_validation(args)
    else:
        run_detection(args)
