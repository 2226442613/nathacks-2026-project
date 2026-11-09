"""Native OpenCV CLI. All HighGUI calls stay on the main thread."""
from __future__ import annotations

import argparse
from datetime import datetime
import logging
import math
from pathlib import Path
import threading
import time
import textwrap
from collections.abc import Callable
from uuid import uuid4

import cv2
import numpy as np

from vision import (CameraError, CameraManager, CalibrationError, GroundCalibration,
                    WallCalibration, DualPlaneCalibration, load_calibration)
from vision.calibration import CalibrationQualityThresholds, load_validation_points
from vision.spill_cli import add_spill_commands, run_spill_command
from vision.robot_cli import add_robot_commands, run_robot_command
from vision.cleaning_cli import add_cleaning_commands, run_cleaning_command
from robot_comm.cli import add_communication_commands, run_communication_command

logger = logging.getLogger(__name__)


def save_screenshot(frame, directory: str | Path) -> Path:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}.png"
    if not cv2.imwrite(str(path), frame):
        raise OSError(f"Could not save screenshot: {path}")
    logger.info("Saved screenshot: %s", path)
    return path


def visible(window: str) -> bool:
    try:
        return cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        # Some native backends remove a closed window before the next event-loop check.
        return False


def calibration_planes(calibration):
    if isinstance(calibration, DualPlaneCalibration):
        return [("Floor", calibration.floor, (0, 255, 0)),
                ("Wall", calibration.wall, (255, 0, 255))]
    return [("Floor", calibration, (0, 255, 0))]


def draw_plane_overlay(frame, name, plane, color) -> None:
    corners = plane.corners_pixels.astype('int32')
    cv2.polylines(frame, [corners], True, color, 2)
    for label, point in zip(("TL", "TR", "BR", "BL"), corners):
        cv2.circle(frame, tuple(point), 5, color, -1)
        cv2.putText(frame, f"{name} {label}", tuple(point), 0, .5, color, 2)


def show_rectified(calibration, frame, args) -> list[str]:
    windows = []
    for name, plane, _color in calibration_planes(calibration):
        title = f"{name} rectified (cm) - Q exit"
        cv2.imshow(title, plane.birdseye(frame, args.pixels_per_cm, grid=args.grid))
        windows.append(title)
    return windows


def run_preview(args, calibration: GroundCalibration | DualPlaneCalibration | None = None) -> None:
    source = args.video if args.video is not None else args.camera
    if calibration is not None:
        if source != calibration.source:
            raise CalibrationError("Input source differs from saved calibration; recalibrate this source")
        logger.warning("Resolution cannot detect camera movement. Verify the physical corners; "
                       "recalibrate if the camera, lens, zoom, crop or arena moved.")
    window = "Hospital Guardian - Q exit | S screenshot"
    with CameraManager(source, args.width, args.height, args.fps) as camera:
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        previous = time.perf_counter()
        while True:
            frame = camera.read()
            if frame is None:
                logger.info("Video ended or decoding stopped")
                break
            height, width = frame.shape[:2]
            if calibration:
                calibration.validate_resolution(width, height)
                rectified_windows = show_rectified(calibration, frame, args)
            now = time.perf_counter()
            fps = 1 / max(now - previous, 1e-9)
            previous = now
            display = frame.copy()
            if calibration:
                for name, plane, color in calibration_planes(calibration):
                    draw_plane_overlay(display, name, plane, color)
            for row, text in enumerate((f"Measured FPS: {fps:.1f} | {width}x{height}", f"Source: {source}")):
                cv2.putText(display, text, (12, 25 + row * 25), cv2.FONT_HERSHEY_SIMPLEX,
                            .6, (0, 255, 255), 2)
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27) or not visible(window):
                break
            if key in (ord('s'), ord('S')):
                save_screenshot(frame, args.screenshots)
            if calibration and not all(visible(title) for title in rectified_windows):
                break


def ask_dimension(name: str, supplied: float | None) -> float:
    if supplied is not None:
        return supplied
    while True:
        try:
            value = float(input(f"Arena {name} in centimeters: "))
            if 0 < value < float('inf'):
                return value
        except ValueError:
            pass
        print("Enter a positive finite number.")


def calibration_message(image: np.ndarray, text: str, *, error: bool = False) -> None:
    """Keep help/errors inside the frozen image, including on small offline frames."""
    lines = textwrap.wrap(text, width=max(15, image.shape[1] // 8))
    for row, line in enumerate(lines):
        position = (5, 18 + row * 18)
        cv2.putText(image, line, position, 0, .45, (0, 0, 0), 3)
        cv2.putText(image, line, position, 0, .45, (0, 100, 255) if error else (255, 255, 255), 1)


def draw_shared_geometry(frame: np.ndarray, floor: GroundCalibration, base: np.ndarray,
                         wall_top=None) -> np.ndarray:
    """Six distinct points, with combined labels on the two inherited endpoints."""
    display = frame.copy()
    cv2.polylines(display, [np.rint(floor.corners_pixels).astype('int32')], True, (0, 255, 0), 2)
    if wall_top is not None and len(wall_top) == 2:
        polygon = np.vstack((wall_top, base[1], base[0]))
        cv2.polylines(display, [np.rint(polygon).astype('int32')], True, (255, 0, 255), 2)
    cv2.line(display, tuple(np.rint(base[0]).astype(int)), tuple(np.rint(base[1]).astype(int)), (255, 255, 0), 4)
    for index, point in enumerate(floor.corners_pixels):
        label = f'F{index+1}'
        for wall_index, inherited in ((4, base[0]), (3, base[1])):
            if np.array_equal(point, inherited):
                label += f' / W{wall_index} inherited'
        pixel = tuple(np.rint(point).astype(int))
        cv2.circle(display, pixel, 6, (0, 255, 255), -1)
        cv2.putText(display, label, pixel, 0, .5, (0, 255, 255), 2)
    for index, point in enumerate(wall_top if wall_top is not None else []):
        pixel = tuple(np.rint(point).astype(int))
        cv2.circle(display, pixel, 6, (255, 0, 255), -1)
        cv2.putText(display, f'W{index+1}', pixel, 0, .6, (255, 0, 255), 2)
    return display


def select_corners(frame, surface: str, floor: GroundCalibration | None = None, *,
                   wall_base: np.ndarray | None = None, initial_points=None,
                   validator: Callable | None = None):
    """Select four floor corners or only two wall tops; inherited base points are locked."""
    points = list(initial_points) if initial_points is not None else []
    limit = 2 if wall_base is not None else 4
    order = "W1 TL, W2 TR only | F floor" if wall_base is not None else "TL TR BR BL"
    window = f"{surface} calibration - {order} | R reset | Enter confirm | Q cancel"
    error = ''
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

    def click(event, x, y, _flags, _param):
        nonlocal error
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < limit:
            points.append((x, y))
            error = ''

    cv2.setMouseCallback(window, click)
    while True:
        display = draw_shared_geometry(frame, floor, wall_base, points) if wall_base is not None else frame.copy()
        if floor is not None and wall_base is None:
            draw_plane_overlay(display, "Floor", floor, (0, 255, 0))
        for index, point in enumerate(points if wall_base is None else []):
            cv2.circle(display, point, 6, (0, 255, 255), -1)
            cv2.putText(display, ("TL", "TR", "BR", "BL")[index], point, 0, .7, (0, 255, 0), 2)
            if index:
                cv2.line(display, points[index - 1], point, (0, 255, 0), 2)
        if wall_base is None and len(points) == 4:
            cv2.line(display, points[-1], points[0], (0, 255, 0), 2)
        if wall_base is not None:
            calibration_message(display, error or 'W3/W4 inherited from floor (locked). Click only W1 TL, then W2 TR. R reset tops | F edit floor | Enter preview | Q cancel', error=bool(error))
        elif error:
            calibration_message(display, error, error=True)
        cv2.imshow(window, display)
        key = cv2.waitKey(20) & 0xFF
        if not visible(window):
            return None
        if key in (27, ord('q'), ord('Q')):
            cv2.destroyWindow(window)
            return None
        if key in (ord('r'), ord('R')):
            points.clear()
            error = ''
        if wall_base is not None and key in (ord('f'), ord('F')):
            cv2.destroyWindow(window)
            return 'floor'
        if key in (10, 13):
            if len(points) != limit:
                error = f'Select all {limit} corners before confirming'
                logger.error(error)
                continue
            if validator is not None:
                try:
                    validator(points)
                except CalibrationError as exc:
                    correction = 'R reselect | F edit floor | Q cancel' if wall_base is not None else 'R reselect | Q cancel'
                    error = f'{exc}. {correction}'
                    logger.error('%s', exc)
                    continue
            # Close the selection window before terminal input, so no GUI is left unresponsive.
            cv2.destroyWindow(window)
            return points


def review_dual_calibration(frame: np.ndarray, calibration: DualPlaneCalibration, args) -> str:
    """Explicit save confirmation; keep corrections in this calibration session."""
    quality = calibration.quality_report
    has_warnings = bool(quality['warnings'])
    save_control = 'A explicitly ACCEPT WARNINGS AND SAVE (Enter blocked)' if has_warnings else 'Enter SAVE'
    title = f'Review six points - {"A accept warnings/save" if has_warnings else "Enter save"} | F floor | W wall | R reset | D dimensions | Q cancel'
    base = DualPlaneCalibration.wall_base_corners(calibration.floor, calibration.floor_boundary,
                                                 calibration.wall_x_reversed)
    display = draw_shared_geometry(frame, calibration.floor, base, calibration.wall.corners_pixels[:2])
    calibration_message(display, f'Review: floor green, wall magenta, shared base cyan. {save_control} | F edit floor | W edit wall tops | R reset six points | D dimensions | Q cancel', error=has_warnings)
    cv2.imshow(title, display)
    windows = [title] + show_rectified(calibration, frame, args)
    edge = quality['shared_boundary']
    logger.info('Shared-boundary consistency: max %.6g px, RMS %.6g px', edge['max_error_px'], edge['rms_error_px'])
    if has_warnings or quality['physical_validation']['points']:
        physical = quality['physical_validation']
        physical_text = (f"Independent physical validation: {physical['status']}, max {physical['max_error_cm']:.4g} cm, RMS {physical['rms_error_cm']:.4g} cm. "
                         if physical['points'] else 'Independent physical accuracy NOT ASSESSED: no measured validation points. ')
        text = (f"Shared edge: max {edge['max_error_px']:.4g} px, RMS {edge['rms_error_px']:.4g} px; "
                f"warning above {calibration.quality_thresholds.shared_edge_warning_px:g} px. " + physical_text +
                ' '.join(quality['warnings']) + f' {save_control}. F/W correct points; Q cancel.')
        quality_window = 'Calibration quality - pixel consistency is not physical accuracy'
        panel = np.zeros((max(240, 24 * (len(text) // 100 + 2)), 900, 3), np.uint8)
        calibration_message(panel, text, error=has_warnings)
        cv2.imshow(quality_window, panel)
        windows.append(quality_window)
        for message in quality['warnings']:
            logger.warning('%s', message)
    actions = {10: 'save', 13: 'save', ord('f'): 'floor', ord('w'): 'wall', ord('r'): 'reset',
               ord('d'): 'dimensions', ord('q'): 'cancel', 27: 'cancel'}
    try:
        while all(visible(window) for window in windows):
            key = cv2.waitKey(20) & 0xFF
            if ord('A') <= key <= ord('Z'):
                key += ord('a') - ord('A')
            if has_warnings and key in (10, 13):
                logger.warning('Quality warnings are not accepted. Press A to explicitly accept and save, or F/W to correct.')
                continue
            if has_warnings and key == ord('a'):
                calibration.accept_quality_warnings()
                return 'save'
            if key in actions:
                return actions[key]
        return 'cancel'
    finally:
        for window in windows:
            if visible(window):
                cv2.destroyWindow(window)


def run_dual_calibration(frame: np.ndarray, args, source: int | str) -> None:
    """Six unique clicks, inherited endpoints, retryable validation, then explicit save."""
    height, width = frame.shape[:2]
    floor_points, wall_top_points = None, None
    dimensions = None
    use_cli_dimensions = True
    stage = 'floor'
    quality_thresholds = CalibrationQualityThresholds(args.shared_edge_warning_px, args.physical_good_cm, args.physical_warning_cm)
    validation_points = load_validation_points(args.validation_points) if args.validation_points else ()

    def dimension(name, supplied):
        if supplied is not None and (not math.isfinite(supplied) or supplied <= 0):
            logger.error('%s must be positive and finite; enter a corrected value', name)
            supplied = None
        return ask_dimension(name, supplied)

    while True:
        if dimensions is None:
            floor_width = dimension('floor width', args.arena_width_cm if use_cli_dimensions else None)
            floor_depth = dimension('floor depth', args.arena_height_cm if use_cli_dimensions else None)
            boundary_length = floor_width if args.floor_boundary in ('top', 'bottom') else floor_depth
            wall_width = dimension(f'wall width (shared edge {boundary_length:g} cm)', args.wall_width_cm if use_cli_dimensions else None)
            while not math.isclose(wall_width, boundary_length, rel_tol=1e-6, abs_tol=1e-6):
                logger.error('Wall width must equal the shared floor edge length: %g cm', boundary_length)
                wall_width = dimension('wall width', None)
            wall_height = dimension('wall height', args.wall_height_cm if use_cli_dimensions else None)
            dimensions = floor_width, floor_depth, wall_width, wall_height
        floor_width, floor_depth, wall_width, wall_height = dimensions

        def make_floor(points):
            return GroundCalibration(points, floor_width, floor_depth, width, height, source, args.fps)

        if stage == 'floor':
            selected = select_corners(frame, 'Floor', initial_points=floor_points, validator=make_floor)
            if selected is None:
                return
            try:
                floor = make_floor(selected)
            except CalibrationError as exc:
                logger.error('%s', exc)
                floor_points = None
                continue
            floor_points = selected
            stage = 'wall'

        def make_pair(points):
            return DualPlaneCalibration.from_floor_and_wall_top(floor, points, wall_width, wall_height,
                                                                args.floor_boundary, args.wall_x_reversed,
                                                                quality_thresholds=quality_thresholds, validation_points=validation_points)

        if stage == 'wall':
            base = DualPlaneCalibration.wall_base_corners(floor, args.floor_boundary, args.wall_x_reversed)
            selected = select_corners(frame, 'Wall', floor, wall_base=base,
                                       initial_points=wall_top_points, validator=make_pair)
            if selected is None:
                return
            if isinstance(selected, str) and selected == 'floor':
                stage = 'floor'
                continue
            try:
                calibration = make_pair(selected)
            except CalibrationError as exc:
                logger.error('%s', exc)
                wall_top_points = None
                continue
            wall_top_points = selected

        action = review_dual_calibration(frame, calibration, args)
        if action == 'save':
            calibration.save(args.config)
            logger.info('Saved validated six-point dual-plane calibration: %s', args.config)
            return
        if action == 'cancel':
            return
        if action == 'reset':
            floor_points, wall_top_points = None, None
            stage = 'floor'
        elif action == 'dimensions':
            dimensions = None
            use_cli_dimensions = False
            stage = 'floor'
        else:
            stage = action  # floor or wall; keep the previous selection visible for correction.


def run_calibration(args) -> None:
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise CalibrationError(f"Cannot read image: {args.image}")
        source = args.calibration_source if args.calibration_source is not None else args.image
    else:
        source = args.camera
        with CameraManager(source, args.width, args.height, args.fps) as camera:
            frame = camera.read()
    if frame is None:
        raise CameraError("No calibration frame available")
    if args.dual_plane:
        run_dual_calibration(frame, args, source)
        return
    height, width = frame.shape[:2]
    points = select_corners(frame, "Floor")
    if points is None:
        return
    arena_width = ask_dimension("floor width", args.arena_width_cm)
    arena_depth = ask_dimension("floor depth", args.arena_height_cm)
    floor = GroundCalibration(points, arena_width, arena_depth, width, height, source, args.fps)
    calibration = floor
    calibration.save(args.config)
    logger.info("Saved floor-only calibration: %s", args.config)
    windows = show_rectified(calibration, frame, args)
    while all(visible(title) for title in windows):
        if cv2.waitKey(20) & 0xFF in (27, ord('q'), ord('Q')):
            break


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Hospital Guardian: camera, planar calibration, and simulated spill detection")
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("preview", "calibrate", "verify-calibration"):
        sub = commands.add_parser(command, help={"preview": "Preview a camera or local video",
            "calibrate": "Select four corners on a frozen frame", "verify-calibration": "Verify saved calibration against live frames"}[command])
        sources = sub.add_mutually_exclusive_group()
        sources.add_argument("--camera", type=int, default=0, help="Physical camera index (default: 0)")
        if command == "calibrate":
            sources.add_argument("--image", help="Local calibration image; use same camera geometry for later live verification")
            sub.add_argument("--arena-width-cm", type=float, help="Physical width; prompted if omitted")
            sub.add_argument("--arena-height-cm", "--arena-depth-cm", dest="arena_height_cm", type=float,
                             help="Floor depth; prompted if omitted (--arena-height-cm is the legacy name)")
            sub.add_argument("--dual-plane", action="store_true", help="Six clicks: four floor corners, two wall tops; preview before saving (version 2)")
            sub.add_argument("--wall-width-cm", type=float, help="Wall width, must match shared floor edge; prompted if omitted")
            sub.add_argument("--wall-height-cm", type=float, help="Wall height above floor; prompted if omitted")
            sub.add_argument("--floor-boundary", choices=("top", "right", "bottom", "left"), default="top",
                             help="Floor edge coinciding with wall base (default: top)")
            sub.add_argument("--wall-x-reversed", action="store_true", help="Reverse wall X relative to floor boundary direction")
            sub.add_argument('--shared-edge-warning-px', type=float, default=3.0,
                             help='Warn above this shared-edge image discrepancy in pixels (default: 3); not a hard rejection')
            sub.add_argument('--physical-good-cm', type=float, default=2.0,
                             help='Independent measured-point error good up to this many cm (default: 2)')
            sub.add_argument('--physical-warning-cm', type=float, default=5.0,
                             help='Independent measured-point error warning up to this many cm; above is poor (default: 5)')
            sub.add_argument('--validation-points', help='Optional independent FLOOR/WALL measurement JSON; dual-plane only')
        else:
            sources.add_argument("--video", help="Local video file; exits at EOF")
            sub.add_argument("--screenshots", default="data/screenshots", help="Screenshot directory")
        sub.add_argument("--width", type=int, default=1280, help="Requested camera width (default: 1280)")
        sub.add_argument("--height", type=int, default=720, help="Requested camera height (default: 720)")
        sub.add_argument("--fps", type=float, default=30, help="Requested camera FPS (default: 30)")
        sub.add_argument("--config", default="config/camera_config.json", help="Calibration JSON path")
        sub.add_argument("--pixels-per-cm", type=float, default=5, help="Bird's-eye scale, capped to 2048 pixels per side")
        sub.add_argument("--grid", action="store_true", help="Show ten physical divisions per axis in rectified views")
    # Explicit association is needed for an offline image from the same physical camera.
    commands.choices["calibrate"].add_argument("--calibration-source", type=int,
        help="Camera index associated with --image; asserts identical camera geometry")
    add_spill_commands(commands)
    add_robot_commands(commands)
    from vision.live_test import add_test_vision_command
    add_test_vision_command(commands)
    add_cleaning_commands(commands)
    add_communication_commands(commands)
    from navigation.cli import add_navigation_commands
    add_navigation_commands(commands)
    from navigation.automatic import add_auto_command
    add_auto_command(commands)
    from navigation.quick_plan import add_quick_plan_command
    add_quick_plan_command(commands)
    from dashboard.cli import add_dashboard_command
    add_dashboard_command(commands)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    if threading.current_thread() is not threading.main_thread():
        logger.error("OpenCV GUI must run on the main thread")
        return 1
    try:
        if args.command in ('background-reference', 'detect-spills','validate-localization'):
            run_spill_command(args)
        elif args.command in ('generate-aruco','track-robot','calibrate-robot-plane','calibrate-robot-interactive'):
            run_robot_command(args)
        elif args.command == 'test-vision':
            from vision.live_test import run_test_vision
            run_test_vision(args)
        elif args.command in ('robot-simulator','robot-client'):
            run_communication_command(args)
        elif args.command == 'dashboard':
            from dashboard.cli import run_dashboard
            run_dashboard(args)
        elif args.command == 'plan-cleaning':
            run_cleaning_command(args)
        elif args.command == 'auto-clean':
            from navigation.automatic import run_auto
            run_auto(args)
        elif args.command == 'quick-plan':
            from navigation.quick_plan import run_quick_plan
            run_quick_plan(args)
        elif args.command in ('movement-calibration','simulate-navigation','navigate-task','navigate-test'):
            from navigation.cli import run_navigation
            run_navigation(args)
        elif args.command == "calibrate":
            run_calibration(args)
        else:
            calibration = load_calibration(args.config) if args.command == "verify-calibration" else None
            run_preview(args, calibration)
        return 0
    except KeyboardInterrupt:
        logger.info("Stopped")
        return 0
    except (CameraError, ValueError, OSError, cv2.error, RuntimeError, EOFError) as exc:
        logger.error("%s", exc)
        return 1
    except Exception:
        logger.exception('Unexpected failure in %s; exiting with error', args.command)
        return 1
    finally:
        if not getattr(args, 'headless', False):
            cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
