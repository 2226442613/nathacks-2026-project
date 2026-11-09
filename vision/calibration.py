"""Validated planar homography in centimeters, independent of capture and GUI."""
from __future__ import annotations

import json
import math
from enum import Enum
from dataclasses import asdict, dataclass
from pathlib import Path
import warnings

import cv2
import numpy as np


class CalibrationError(ValueError):
    """Invalid calibration, coordinates, persistence, or frame geometry."""


class SurfaceType(str, Enum):
    FLOOR = "FLOOR"
    WALL = "WALL"


@dataclass(frozen=True)
class CalibrationQualityThresholds:
    """Quality thresholds, with pixel diagnostics separate from ruler-validated cm errors."""

    shared_edge_warning_px: float = 3.0
    physical_good_cm: float = 2.0
    physical_warning_cm: float = 5.0

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise CalibrationError(f'{name} must be finite and nonnegative')
        if self.physical_warning_cm < self.physical_good_cm:
            raise CalibrationError('physical_warning_cm must be >= physical_good_cm')

    def physical_status(self, error_cm: float) -> str:
        if not math.isfinite(error_cm) or error_cm < 0:
            raise CalibrationError('Physical validation error must be finite and nonnegative (cm)')
        return 'GOOD' if error_cm <= self.physical_good_cm else ('WARNING' if error_cm <= self.physical_warning_cm else 'POOR')


@dataclass(frozen=True)
class PhysicalValidationPoint:
    """An independently measured point on a known plane; not used to fit either matrix."""

    surface: SurfaceType
    pixel: tuple[float, float]
    expected_cm: tuple[float, float]

    def __post_init__(self) -> None:
        try:
            object.__setattr__(self, 'surface', SurfaceType(self.surface))
            for name in ('pixel', 'expected_cm'):
                values = np.asarray(getattr(self, name), dtype=float)
                if values.shape != (2,) or not np.isfinite(values).all():
                    raise ValueError('expected a pair of finite numbers')
                object.__setattr__(self, name, tuple(float(value) for value in values))
        except (TypeError, ValueError) as exc:
            raise CalibrationError(f'Invalid independent validation point: {exc}') from exc


def load_validation_points(path: str | Path) -> tuple[PhysicalValidationPoint, ...]:
    """Load a JSON array of {surface, pixel, expected_cm} independent measurements."""
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(data, list) or not data:
            raise CalibrationError('Validation points must be a nonempty JSON array')
        return tuple(PhysicalValidationPoint(**record) for record in data)
    except (OSError, TypeError, ValueError) as exc:
        raise CalibrationError(f'Cannot load independent validation points {path}: {exc}') from exc


@dataclass(frozen=True)
class SurfaceCoordinates:
    """Known-surface geometry. For WALL, floor_xy_cm is only the vertical base projection."""

    surface: SurfaceType
    floor_xy_cm: tuple[float, float]
    wall_xh_cm: tuple[float, float] | None = None

    @property
    def height_above_floor_cm(self) -> float:
        return self.wall_xh_cm[1] if self.wall_xh_cm is not None else 0.0


@dataclass(frozen=True)
class RobotReachGeometry:
    """Future-module inputs only: no cleaning, reachability, or navigation decisions."""

    max_reachable_height_cm: float
    required_wall_distance_cm: float

    def __post_init__(self):
        for value in (self.max_reachable_height_cm, self.required_wall_distance_cm):
            if not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
                raise CalibrationError("Robot geometry must contain finite nonnegative distances")


class GroundCalibration:
    """Corners ordered TL, TR, BR, BL; world origin TL, X right, Y down (cm)."""

    surface = SurfaceType.FLOOR

    def __init__(self, corners_pixels, width_cm: float, height_cm: float,
                 frame_width: int, frame_height: int, source: int | str = 0,
                 requested_fps: float = 30):
        try:
            self.width_cm, self.height_cm = float(width_cm), float(height_cm)
            points = np.asarray(corners_pixels, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise CalibrationError("Dimensions and corners must be numeric") from exc
        if not all(math.isfinite(v) and v > 0 for v in (self.width_cm, self.height_cm)):
            raise CalibrationError("Arena dimensions must be positive and finite")
        if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0
               for v in (frame_width, frame_height)):
            raise CalibrationError("Frame dimensions must be positive integers")
        if isinstance(source, bool) or not isinstance(source, (int, str)) or (isinstance(source, int) and source < 0):
            raise CalibrationError("Invalid camera source")
        if not isinstance(requested_fps, (int, float)) or not math.isfinite(requested_fps) or requested_fps <= 0:
            raise CalibrationError("Requested FPS must be positive and finite")
        if points.shape != (4, 2) or not np.isfinite(points).all():
            raise CalibrationError("Select four finite pixel coordinates")
        if (points < 0).any() or (points[:, 0] >= frame_width).any() or (points[:, 1] >= frame_height).any():
            raise CalibrationError("Corners must lie inside the calibrated frame")
        # Strict convexity rejects duplicates, collinearity, concavity and crossing edges.
        edges = np.roll(points, -1, axis=0) - points
        cross = edges[:, 0] * np.roll(edges[:, 1], -1) - edges[:, 1] * np.roll(edges[:, 0], -1)
        if not (cross > 1e-6).all():
            raise CalibrationError("Corners must form a convex quadrilateral in TL, TR, BR, BL order")
        self.frame_width, self.frame_height = frame_width, frame_height
        self.source, self.requested_fps = source, float(requested_fps)
        self.corners_pixels = points.copy()
        world = np.array([[0, 0], [self.width_cm, 0],
                          [self.width_cm, self.height_cm], [0, self.height_cm]], dtype=np.float32)
        if self.surface == SurfaceType.WALL:
            world[:, 1] = self.height_cm - world[:, 1]
        try:
            matrix = cv2.getPerspectiveTransform(points.astype(np.float32), world)
            inverse = np.linalg.inv(matrix)
        except (cv2.error, np.linalg.LinAlgError) as exc:
            raise CalibrationError("Homography is not invertible") from exc
        if not np.isfinite(matrix).all() or not np.isfinite(inverse).all():
            raise CalibrationError("Invalid homography matrix")
        if not np.allclose(cv2.perspectiveTransform(points.reshape(1, 4, 2), matrix)[0], world,
                           rtol=1e-5, atol=1e-4):
            raise CalibrationError("Homography is numerically unstable")
        self.homography, self.inverse_homography = matrix, inverse

    @staticmethod
    def _transform(a: float, b: float, matrix: np.ndarray) -> tuple[float, float]:
        try:
            point = np.asarray([a, b, 1.0], dtype=float)
        except (TypeError, ValueError) as exc:
            raise CalibrationError("Coordinates must be numeric") from exc
        if not np.isfinite(point).all():
            raise CalibrationError("Coordinates must be finite")
        mapped = matrix @ point
        if not np.isfinite(mapped).all() or abs(mapped[2]) <= np.finfo(float).eps * max(1, np.linalg.norm(mapped)):
            raise CalibrationError("Coordinate lies on the homography horizon")
        result = mapped[:2] / mapped[2]
        return float(result[0]), float(result[1])

    def pixel_to_world(self, u: float, v: float) -> tuple[float, float]:
        return self._transform(u, v, self.homography)

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        return self._transform(x, y, self.inverse_homography)

    def validate_resolution(self, width: int, height: int) -> None:
        if (width, height) != (self.frame_width, self.frame_height):
            message = (f"Frame resolution {width}x{height} differs from calibrated "
                       f"{self.frame_width}x{self.frame_height}; recalibration required")
            warnings.warn(message, UserWarning, stacklevel=2)
            raise CalibrationError(message)

    def birdseye(self, frame: np.ndarray, pixels_per_cm: float = 5,
                 max_side: int = 2048, grid: bool = False) -> np.ndarray:
        """Rectify either plane; wall images are front-facing with +H upward."""
        self.validate_resolution(frame.shape[1], frame.shape[0])
        if not math.isfinite(pixels_per_cm) or pixels_per_cm <= 0:
            raise CalibrationError("Visualization scale must be positive and finite")
        if not isinstance(max_side, int) or not 2 <= max_side <= 4096:
            raise CalibrationError("Maximum visualization side must be between 2 and 4096")
        scale = min(pixels_per_cm, (max_side - 1) / max(self.width_cm, self.height_cm))
        width = max(2, int(round(self.width_cm * scale)) + 1)
        height = max(2, int(round(self.height_cm * scale)) + 1)
        # Endpoints land at width-1 / height-1, including every arena corner.
        display = np.diag([(width - 1) / self.width_cm, (height - 1) / self.height_cm, 1])
        wall = self.surface == SurfaceType.WALL
        if wall:
            display[1, 1] *= -1
            display[1, 2] = height - 1
        image = cv2.warpPerspective(frame, display @ self.homography, (width, height))
        if grid:
            # Ten evenly spaced physical divisions per axis, bounded regardless of arena size.
            for fraction in np.linspace(0, 1, 11):
                x, y = round(fraction * (width - 1)), round(fraction * (height - 1))
                cv2.line(image, (x, 0), (x, height - 1), (100, 100, 100), 1)
                cv2.line(image, (0, y), (width - 1, y), (100, 100, 100), 1)
        y0 = max(0, height - 25) if wall else min(25, height - 1)
        y_end = max(0, y0 - 60) if wall else min(height - 1, y0 + 60)
        x0 = min(5, width - 1)
        cv2.putText(image, "(0,0) cm", (x0, max(12, y0 - 8)), 0, .45, (0, 255, 255), 1)
        cv2.arrowedLine(image, (x0, y0), (min(width - 1, 85), y0), (0, 255, 255), 1)
        cv2.arrowedLine(image, (x0, y0), (x0, y_end), (0, 255, 255), 1)
        cv2.putText(image, "+X", (max(0, min(width - 25, 85)), max(12, y0 - 3)), 0, .45, (0, 255, 255), 1)
        cv2.putText(image, "+H" if wall else "+Y", (8, max(12, y_end)), 0, .45, (0, 255, 255), 1)
        return image

    def to_dict(self) -> dict:
        return {"calibration_version": 1, "surface": self.surface.value,
                "camera": {"source": self.source, "frame_width": self.frame_width,
                           "frame_height": self.frame_height, "requested_fps": self.requested_fps},
                "arena": {"width_cm": self.width_cm, "height_cm": self.height_cm},
                "corners_pixels": self.corners_pixels.tolist(), "homography": self.homography.tolist()}

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        # Write then replace so a failed serialization does not truncate the previous calibration.
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(self.to_dict(), indent=2, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(path)

    @classmethod
    def load(cls, path: str | Path) -> GroundCalibration:
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            return cls.from_dict(data)
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
            raise CalibrationError(f"Cannot load calibration {path}: {exc}") from exc

    @classmethod
    def from_dict(cls, data: dict) -> GroundCalibration:
        """Validate a single-plane record; older records without surface are floor-only."""
        try:
            if data["calibration_version"] != 1:
                raise CalibrationError("Unsupported calibration version")
            if data.get("surface", "FLOOR") != cls.surface.value:
                raise CalibrationError(f"Expected {cls.surface.value} calibration")
            camera, arena = data["camera"], data["arena"]
            result = cls(data["corners_pixels"], arena["width_cm"], arena["height_cm"],
                         camera["frame_width"], camera["frame_height"], camera["source"], camera["requested_fps"])
            stored = np.asarray(data["homography"], dtype=float)
            if stored.shape != (3, 3) or not np.isfinite(stored).all() or abs(stored[2, 2]) < 1e-15:
                raise CalibrationError("Invalid stored homography")
            if not np.allclose(stored / stored[2, 2], result.homography, rtol=1e-6, atol=1e-8):
                raise CalibrationError("Stored homography is inconsistent with calibration corners")
            return result
        except (ValueError, KeyError, TypeError, OverflowError, AttributeError) as exc:
            raise CalibrationError(f"Invalid {cls.surface.value} calibration: {exc}") from exc


class WallCalibration(GroundCalibration):
    """Same four-point solver; corners TL, TR, BR, BL map to (0,H),(W,H),(W,0),(0,0)."""

    surface = SurfaceType.WALL

    def pixel_to_wall(self, u: float, v: float) -> tuple[float, float]:
        return self.pixel_to_world(u, v)

    def wall_to_pixel(self, x: float, h: float) -> tuple[float, float]:
        return self.world_to_pixel(x, h)


class DualPlaneCalibration:
    """Two known planes sharing a complete floor edge, not arbitrary 3D reconstruction.

    Boundary directions: top/bottom +X, left/right +Y. wall_x_reversed flips
    the wall's X direction relative to that boundary. No navigation target is produced.
    """

    @staticmethod
    def wall_base_corners(floor: GroundCalibration, floor_boundary: str,
                          wall_x_reversed: bool = False) -> np.ndarray:
        """Return exact inherited [W4 (BL), W3 (BR)] pixels, without a matrix round trip."""
        if not isinstance(floor, GroundCalibration) or floor.surface != SurfaceType.FLOOR:
            raise CalibrationError("A valid FLOOR calibration is required")
        boundaries = {"top": (0, 1), "right": (1, 2), "bottom": (3, 2), "left": (0, 3)}
        if floor_boundary not in boundaries:
            raise CalibrationError("Floor boundary must be top, right, bottom or left")
        if not isinstance(wall_x_reversed, bool):
            raise CalibrationError("wall_x_reversed must be boolean")
        indices = boundaries[floor_boundary]
        if wall_x_reversed:
            indices = indices[::-1]
        return floor.corners_pixels[list(indices)].copy()

    @classmethod
    def from_floor_and_wall_top(cls, floor: GroundCalibration, wall_top_pixels,
                               wall_width_cm: float, wall_height_cm: float,
                               floor_boundary: str, wall_x_reversed: bool = False, *,
                               quality_thresholds: CalibrationQualityThresholds | None = None,
                               validation_points: tuple[PhysicalValidationPoint, ...] = ()) -> DualPlaneCalibration:
        """Build a six-click calibration; all four wall corners remain in the existing v2 schema."""
        base = cls.wall_base_corners(floor, floor_boundary, wall_x_reversed)
        try:
            top = np.asarray(wall_top_pixels, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise CalibrationError("Select two numeric wall top corners: W1 TL, W2 TR") from exc
        if top.shape != (2, 2) or not np.isfinite(top).all():
            raise CalibrationError("Select exactly two finite wall top corners: W1 TL, W2 TR")
        corners = np.vstack((top, base[1], base[0]))  # W1, W2, W3, W4
        wall = WallCalibration(corners, wall_width_cm, wall_height_cm,
                               floor.frame_width, floor.frame_height, floor.source, floor.requested_fps)
        if not np.array_equal(wall.corners_pixels[[3, 2]], base):
            raise CalibrationError('Inherited wall endpoints must match floor pixels exactly')
        return cls(floor, wall, floor_boundary, wall_x_reversed,
                   quality_thresholds=quality_thresholds, validation_points=validation_points)

    def __init__(self, floor: GroundCalibration, wall: WallCalibration,
                 floor_boundary: str, wall_x_reversed: bool = False, *,
                 quality_thresholds: CalibrationQualityThresholds | None = None,
                 validation_points: tuple[PhysicalValidationPoint, ...] = ()):
        if not isinstance(floor, GroundCalibration) or floor.surface != SurfaceType.FLOOR:
            raise CalibrationError("A valid FLOOR calibration is required")
        if not isinstance(wall, WallCalibration):
            raise CalibrationError("A valid WALL calibration is required")
        if floor_boundary not in ("top", "right", "bottom", "left"):
            raise CalibrationError("Floor boundary must be top, right, bottom or left")
        if not isinstance(wall_x_reversed, bool):
            raise CalibrationError("wall_x_reversed must be boolean")
        if (floor.frame_width, floor.frame_height, floor.source) != (wall.frame_width, wall.frame_height, wall.source):
            raise CalibrationError("Floor and wall must use the same camera source and resolution")
        length = floor.width_cm if floor_boundary in ("top", "bottom") else floor.height_cm
        if not math.isclose(wall.width_cm, length, rel_tol=1e-6, abs_tol=1e-6):
            raise CalibrationError("Wall width must equal the length of its shared floor boundary")
        self.floor, self.wall = floor, wall
        self.floor_boundary, self.wall_x_reversed = floor_boundary, wall_x_reversed
        # Shared corners must be the same image locations (allow 3 px clicking error).
        for x, point in ((0, wall.corners_pixels[3]), (wall.width_cm, wall.corners_pixels[2])):
            expected = floor.world_to_pixel(*self.wall_to_floor_base(x, 0))
            if np.linalg.norm(np.asarray(expected) - point) > 3:
                raise CalibrationError("Wall base corners do not match the selected floor boundary "
                                       "(3 pixel tolerance); check boundary and X direction")
        self.shared_boundary_errors_px = []
        for x in np.linspace(0, wall.width_cm, 5):
            floor_pixel = floor.world_to_pixel(*self.wall_to_floor_base(float(x), 0))
            wall_pixel = wall.wall_to_pixel(float(x), 0)
            error = float(np.linalg.norm(np.asarray(floor_pixel) - wall_pixel))
            if not math.isfinite(error):
                raise CalibrationError('Non-finite shared-boundary projection')
            self.shared_boundary_errors_px.append(error)
        # Both visible interiors must lie on opposite sides of the shared image edge.
        # Otherwise two overlapping patches could masquerade as a wall and floor.
        a, b = wall.corners_pixels[3], wall.corners_pixels[2]
        edge = b - a
        floor_center = floor.corners_pixels.mean(axis=0) - a
        wall_center = wall.corners_pixels.mean(axis=0) - a
        cross_floor = edge[0] * floor_center[1] - edge[1] * floor_center[0]
        cross_wall = edge[0] * wall_center[1] - edge[1] * wall_center[0]
        if cross_floor * cross_wall >= 0:
            raise CalibrationError("Visible floor and wall interiors must be on opposite sides of their shared edge")
        self.set_quality_validation(quality_thresholds, validation_points)

    def set_quality_validation(self, thresholds: CalibrationQualityThresholds | None = None,
                               points: tuple[PhysicalValidationPoint, ...] = ()) -> None:
        """Evaluate quality without refitting matrices; new evidence resets warning acceptance."""
        self.quality_thresholds = thresholds if thresholds is not None else CalibrationQualityThresholds()
        if not isinstance(self.quality_thresholds, CalibrationQualityThresholds):
            raise CalibrationError('Invalid calibration quality thresholds')
        self.validation_points = tuple(points)
        measurements = []
        for point in self.validation_points:
            if not isinstance(point, PhysicalValidationPoint):
                raise CalibrationError('Expected PhysicalValidationPoint values')
            plane = self.floor if point.surface == SurfaceType.FLOOR else self.wall
            if not (0 <= point.pixel[0] < plane.frame_width and 0 <= point.pixel[1] < plane.frame_height):
                raise CalibrationError('Validation pixel lies outside the calibrated image')
            if not (0 <= point.expected_cm[0] <= plane.width_cm and 0 <= point.expected_cm[1] <= plane.height_cm):
                raise CalibrationError('Independent expected coordinates lie outside their calibrated plane')
            predicted = plane.pixel_to_world(*point.pixel)
            error = float(math.dist(predicted, point.expected_cm))
            if not math.isfinite(error):
                raise CalibrationError('Non-finite independent physical validation error')
            measurements.append({'surface': point.surface.value, 'pixel': list(point.pixel),
                                 'expected_cm': list(point.expected_cm), 'predicted_cm': list(predicted),
                                 'error_cm': error, 'status': self.quality_thresholds.physical_status(error)})
        pixel_errors = self.shared_boundary_errors_px
        maximum_px = max(pixel_errors)
        physical_errors = [item['error_cm'] for item in measurements]
        maximum_cm = max(physical_errors) if physical_errors else None
        status = self.quality_thresholds.physical_status(maximum_cm) if maximum_cm is not None else 'NOT_ASSESSED'
        messages = []
        if maximum_px > self.quality_thresholds.shared_edge_warning_px:
            messages.append(f'Shared-boundary inconsistency: max {maximum_px:.4g} px, '
                            f'warning threshold {self.quality_thresholds.shared_edge_warning_px:g} px. '
                            'Independent homographies disagree along the edge; recheck corner selections.')
        if status in ('WARNING', 'POOR'):
            messages.append(f'{status} independent physical validation: max {maximum_cm:.4g} cm '
                            f'(good <= {self.quality_thresholds.physical_good_cm:g} cm; '
                            f'warning <= {self.quality_thresholds.physical_warning_cm:g} cm). Remeasure before use.')
        self._quality = {'quality_version': 1, 'thresholds': asdict(self.quality_thresholds),
                         'shared_boundary': {'sample_errors_px': list(pixel_errors), 'max_error_px': maximum_px,
                                             'rms_error_px': float(np.sqrt(np.mean(np.square(pixel_errors))))},
                         'physical_validation': {'status': status, 'points': measurements, 'max_error_cm': maximum_cm,
                                                 'rms_error_cm': float(np.sqrt(np.mean(np.square(physical_errors)))) if physical_errors else None},
                         'warnings': messages, 'accepted_warnings': False}

    @property
    def quality_report(self) -> dict:
        """Detached JSON-ready report; pixel consistency is not physical accuracy."""
        return json.loads(json.dumps(self._quality, allow_nan=False))

    def accept_quality_warnings(self) -> None:
        """Explicit caller acknowledgment of the current quality report only."""
        self._quality['accepted_warnings'] = bool(self._quality['warnings'])

    @property
    def source(self) -> int | str:
        return self.floor.source

    def validate_resolution(self, width: int, height: int) -> None:
        self.floor.validate_resolution(width, height)
        self.wall.validate_resolution(width, height)

    def pixel_to_floor(self, u: float, v: float) -> tuple[float, float]:
        return self.floor.pixel_to_world(u, v)

    def floor_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        return self.floor.world_to_pixel(x, y)

    def pixel_to_wall(self, u: float, v: float) -> tuple[float, float]:
        return self.wall.pixel_to_wall(u, v)

    def wall_to_pixel(self, x: float, h: float) -> tuple[float, float]:
        return self.wall.wall_to_pixel(x, h)

    def wall_to_floor_base(self, x: float, h: float) -> tuple[float, float]:
        """Vertical projection onto the wall base, NOT a reachable robot goal.

        H is retained by the caller for future mechanism-height checks; standoff and
        reachability decisions belong to future modules, not to this projection.
        """
        if not all(isinstance(v, (float, int)) and math.isfinite(v) for v in (x, h)):
            raise CalibrationError("Wall coordinates must be finite numbers")
        if not -1e-6 <= x <= self.wall.width_cm + 1e-6 or not -1e-6 <= h <= self.wall.height_cm + 1e-6:
            raise CalibrationError("Wall point lies outside the calibrated wall")
        x = min(self.wall.width_cm, max(0.0, x))
        distance = self.wall.width_cm - x if self.wall_x_reversed else x
        return {"top": (distance, 0.0), "bottom": (distance, self.floor.height_cm),
                "left": (0.0, distance), "right": (self.floor.width_cm, distance)}[self.floor_boundary]

    def localize(self, surface: SurfaceType | str, u: float, v: float) -> SurfaceCoordinates:
        """Caller supplies the known surface; this does not classify surfaces or recover arbitrary height."""
        try:
            surface = SurfaceType(surface)
        except ValueError as exc:
            raise CalibrationError("Specify FLOOR or WALL explicitly") from exc
        if surface == SurfaceType.FLOOR:
            return SurfaceCoordinates(surface, self.pixel_to_floor(u, v))
        x, h = self.pixel_to_wall(u, v)
        return SurfaceCoordinates(surface, self.wall_to_floor_base(x, h), (x, h))

    def to_dict(self) -> dict:
        def plane_data(plane):
            dimension = "depth_cm" if plane.surface == SurfaceType.FLOOR else "height_cm"
            return {"width_cm": plane.width_cm, dimension: plane.height_cm,
                    "corners_pixels": plane.corners_pixels.tolist(), "homography": plane.homography.tolist()}
        result = {"calibration_version": 2, "camera": self.floor.to_dict()["camera"],
                  "floor": plane_data(self.floor), "wall": plane_data(self.wall),
                  "shared_geometry": {"floor_boundary": self.floor_boundary,
                                      "wall_x_reversed": self.wall_x_reversed}}
        if self._quality['warnings'] or self.validation_points or self.quality_thresholds != CalibrationQualityThresholds():
            result['quality'] = self.quality_report  # Optional, backward-compatible v2 metadata.
        return result

    def save(self, path: str | Path) -> None:
        if self._quality['warnings'] and not self._quality['accepted_warnings']:
            raise CalibrationError('Quality warnings require explicit acceptance before saving; '
                                   'review quality_report and call accept_quality_warnings()')
        GroundCalibration.save(self, path)

    @classmethod
    def from_dict(cls, data: dict) -> DualPlaneCalibration:
        try:
            if data["calibration_version"] == 1:
                raise CalibrationError("Version 1 is floor-only; dual-plane calibration requires a wall")
            if data["calibration_version"] != 2:
                raise CalibrationError("Unsupported calibration version")
            camera = data["camera"]

            def parse_plane(name, plane_class, dimension):
                plane = data[name]
                record = {"calibration_version": 1, "surface": plane_class.surface.value,
                          "camera": camera, "arena": {"width_cm": plane["width_cm"], "height_cm": plane[dimension]},
                          "corners_pixels": plane["corners_pixels"], "homography": plane["homography"]}
                return plane_class.from_dict(record)

            floor = parse_plane("floor", GroundCalibration, "depth_cm")
            wall = parse_plane("wall", WallCalibration, "height_cm")
            geometry = data["shared_geometry"]
            thresholds = None
            points = ()
            quality = data.get('quality')
            if quality is not None:
                if not isinstance(quality, dict) or quality['quality_version'] != 1 or not isinstance(quality['accepted_warnings'], bool):
                    raise CalibrationError('Invalid quality metadata')
                json.dumps(quality, allow_nan=False)  # Reject non-finite stored quality data too.
                thresholds = CalibrationQualityThresholds(**quality['thresholds'])
                points = tuple(PhysicalValidationPoint(record['surface'], record['pixel'], record['expected_cm'])
                               for record in quality['physical_validation']['points'])
            result = cls(floor, wall, geometry["floor_boundary"], geometry["wall_x_reversed"],
                         quality_thresholds=thresholds, validation_points=points)
            if quality is not None and quality['accepted_warnings']:
                result.accept_quality_warnings()
            if result._quality['warnings']:
                state = 'acceptance recorded' if result._quality['accepted_warnings'] else 'not yet accepted for saving'
                warnings.warn('Calibration quality warning (' + state + '): ' + '; '.join(result._quality['warnings']),
                              UserWarning, stacklevel=2)
            return result
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
            raise CalibrationError(f"Invalid dual-plane calibration: {exc}") from exc

    @classmethod
    def load(cls, path: str | Path) -> DualPlaneCalibration:
        try:
            return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
        except (OSError, ValueError, TypeError) as exc:
            raise CalibrationError(f"Cannot load calibration {path}: {exc}") from exc


def load_calibration(path: str | Path) -> GroundCalibration | DualPlaneCalibration:
    """Load v1 explicitly as floor-only, or require a complete valid v2 pair."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if isinstance(data, dict) and data.get("calibration_version") == 1:
            calibration = GroundCalibration.from_dict(data)
            warnings.warn("Legacy floor-only calibration: WALL is unavailable. "
                          "Run calibrate --dual-plane to calibrate both surfaces.", UserWarning, stacklevel=2)
            return calibration
        return DualPlaneCalibration.from_dict(data)
    except (OSError, ValueError, TypeError) as exc:
        raise CalibrationError(f"Cannot load calibration {path}: {exc}") from exc
