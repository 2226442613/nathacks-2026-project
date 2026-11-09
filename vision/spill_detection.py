"""Immutable initial legal scene and deterministic new-region color classification."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from collections.abc import Iterable
from enum import Enum
import json
import math
import hashlib
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
from .serialization import json_native

from .calibration import DualPlaneCalibration, GroundCalibration
from .frame_geometry import FrameGeometryGuard, GeometryCheck
from .reference_store import save_reference_pair
from .red_classification import red_pixels, component_red_evidence


class DetectionError(ValueError):
    """Invalid image, reference, mask or detection configuration."""


class FramingMismatch(DetectionError):
    """Legacy compatibility exception; operator-managed detection no longer raises it."""

    def __init__(self, message: str, geometry: GeometryCheck):
        super().__init__(message)
        self.geometry = geometry


def validate_image(image: np.ndarray, name: str = "Image") -> None:
    if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) < 3:
        raise DetectionError(f"{name} must be a nonempty uint8 BGR image, at least 3x3")


def validate_mask(mask: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    if not isinstance(mask, np.ndarray) or mask.ndim != 2 or mask.shape != shape or mask.dtype not in (np.dtype('uint8'), np.dtype('bool')):
        raise DetectionError("Exclusion mask must be uint8/bool, single-channel and match reference resolution")
    return (mask != 0).astype(np.uint8) * 255


class BackgroundReference:
    """Operator-registered legal BGR scene. Never learns from incoming frames."""

    def __init__(self, image: np.ndarray, *, confirmed_clean: bool = False):
        if confirmed_clean is not True:
            raise DetectionError("Confirm the initial scene is legal (confirmed_clean=True)")
        validate_image(image, "Background reference")
        self._image = image.copy()
        self._image.setflags(write=False)
        self._metadata = {'background_version': 1, 'frame_width': image.shape[1], 'frame_height': image.shape[0],
                          'aggregation_method': 'single', 'frame_count': 1, 'confirmed_clean': True,
                          'registered_at_utc': datetime.now(timezone.utc).isoformat(),
                          'color_order': 'BGR', 'purpose': 'initial_legal_scene'}

    @property
    def image(self) -> np.ndarray:
        return self._image.copy()

    @property
    def resolution(self) -> tuple[int, int]:
        return self._image.shape[1], self._image.shape[0]

    @property
    def metadata(self) -> dict:
        return self._metadata.copy()

    @classmethod
    def from_frames(cls, frames: Iterable[np.ndarray], *, method: str = 'median',
                    confirmed_clean: bool = False) -> BackgroundReference:
        if confirmed_clean is not True:
            raise DetectionError('Confirm that everything in all reference frames is initially legal')
        if method not in ('median', 'mean'):
            raise DetectionError('Background aggregation must be median or mean')
        collected = []
        for frame in frames:
            validate_image(frame, 'Reference frame')
            if collected and frame.shape != collected[0].shape:
                raise DetectionError('Reference aggregation frames must have matching resolution')
            collected.append(frame.copy())
            if len(collected) > 31:
                raise DetectionError('Aggregate at most 31 reference frames to bound memory')
        if not collected:
            raise DetectionError('At least one clean reference frame is required')
        stacked = np.stack(collected)
        aggregated = np.median(stacked, axis=0) if method == 'median' else np.mean(stacked, axis=0)
        result = cls(np.rint(aggregated).astype(np.uint8), confirmed_clean=True)
        result._metadata.update(aggregation_method=method if len(collected) > 1 else 'single', frame_count=len(collected))
        return result

    @classmethod
    def load(cls, path: str | Path, *, confirmed_clean: bool = False) -> BackgroundReference:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise DetectionError(f"Cannot read clean reference image: {path}")
        result = cls(image, confirmed_clean=confirmed_clean)
        metadata_path = Path(str(path) + '.json')
        if metadata_path.exists():
            try:
                metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
                if (metadata['background_version'] != 1 or metadata['confirmed_clean'] is not True or
                    metadata.get('color_order', 'BGR') != 'BGR' or
                    (metadata['frame_width'], metadata['frame_height']) != result.resolution or
                    metadata['aggregation_method'] not in ('single','median','mean') or
                    isinstance(metadata['frame_count'], bool) or not isinstance(metadata['frame_count'], int) or
                    not 1 <= metadata['frame_count'] <= 31):
                    raise ValueError('Invalid background metadata or resolution mismatch')
                result._metadata = metadata
                if 'image_sha256' in metadata and hashlib.sha256(Path(path).read_bytes()).hexdigest() != metadata['image_sha256']:
                    raise ValueError('Baseline PNG/metadata checksum mismatch; explicitly re-register or restore the reference')
            except (OSError, ValueError, KeyError, TypeError) as exc:
                raise DetectionError(f'Cannot load reference metadata {metadata_path}: {exc}') from exc
        return result

    def save(self, path: str | Path, *, overwrite: bool = False) -> None:
        path = Path(path)
        if path.suffix.lower() != '.png':
            raise DetectionError("Save background references as lossless .png files")
        if not overwrite and (path.exists() or Path(str(path)+'.json').exists()):
            raise DetectionError('Baseline already exists; choose a new path or explicitly allow overwrite')
        path.parent.mkdir(parents=True, exist_ok=True)
        self._metadata = save_reference_pair(path, self._image, self._metadata)


@dataclass(frozen=True)
class DetectorConfig:
    roi_enabled: bool = True
    geometry_check_enabled: bool = True
    illumination_correction: bool = True
    illumination_gain_correction: bool = True
    illumination_max_gain_change: float = 0.15
    roi_margin_pixels: int = 3
    alignment_enabled: bool = True
    alignment_max_displacement_pixels: float = 2.0
    unverified_max_foreground_fraction: float = 0.15
    difference_threshold: int = 30
    illumination_max_shift: int = 25
    blur_kernel: int = 3
    opening_kernel: int = 3
    closing_kernel: int = 5
    min_area_pixels: int = 80
    max_area_pixels: int = 0  # 0 means no absolute cap; max_area_fraction still applies.
    max_area_fraction: float = 0.35
    # Legacy settings retained for file/API compatibility; shape/material gates
    # no longer affect the baseline-relative binary classification.
    max_aspect_ratio: float = 5.0
    min_fill_ratio: float = 0.25
    min_solidity: float = 0.55
    min_mean_color_difference: float = 45.0
    red_hue_low_max: int = 10
    red_hue_high_min: int = 170
    red_min_saturation: int = 100
    red_min_value: int = 50
    red_shadow_enabled: bool = True
    red_shadow_min_saturation: int = 65
    red_shadow_min_value: int = 25
    red_shadow_min_normalized_excess: float = 0.08
    red_interior_erosion_pixels: int = 1
    red_interior_min_pixels: int = 25
    red_interior_min_fraction: float = 0.5
    red_interior_min_full_ratio: float = 0.5
    red_fraction_threshold: float = 0.70
    non_red_max_red_fraction: float = 0.15
    color_min_saturation: int = 55
    color_min_value: int = 40
    min_color_fraction: float = 0.65
    max_edge_density: float = 0.35
    exclusion_padding_pixels: int = 3
    boundary_fraction: float = 0.05
    boundary_margin_pixels: int = 3
    geometry_max_shift_pixels: float = 8.0
    geometry_max_rotation_degrees: float = 2.0
    geometry_max_scale_change: float = 0.03
    geometry_min_matches: int = 12
    geometry_min_inlier_ratio: float = 0.6
    # Experimental restricted-scene policy. Can hide real pale/bright objects;
    # keep disabled for general baseline-relative detection.
    brightening_rejection_enabled: bool = False

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if name in ('roi_enabled', 'geometry_check_enabled', 'illumination_correction', 'illumination_gain_correction', 'alignment_enabled','red_shadow_enabled','brightening_rejection_enabled'):
                if not isinstance(value, bool):
                    raise DetectionError(f'{name} must be boolean')
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise DetectionError(f"{name} must be a finite number")
        for name in ('blur_kernel', 'opening_kernel', 'closing_kernel'):
            value = getattr(self, name)
            if not isinstance(value, int) or value < 1 or value > 31 or value % 2 != 1:
                raise DetectionError(f"{name} must be an odd integer from 1 to 31 (1 disables)")
        for name in ('difference_threshold', 'illumination_max_shift', 'red_min_saturation', 'red_min_value',
                     'color_min_saturation', 'color_min_value','red_shadow_min_saturation','red_shadow_min_value'):
            value = getattr(self, name)
            if not isinstance(value, int) or not 0 <= value <= 255:
                raise DetectionError(f"{name} must be an integer in [0,255]")
        if self.difference_threshold == 0:
            raise DetectionError("difference_threshold must be positive")
        for name in ('red_shadow_min_normalized_excess','red_interior_min_fraction','red_interior_min_full_ratio'):
            if not 0 < getattr(self,name) <= 1:
                raise DetectionError(f'{name} must be in (0,1]')
        if not isinstance(self.red_interior_min_pixels,int) or self.red_interior_min_pixels < 1:
            raise DetectionError('red_interior_min_pixels must be a positive integer')
        if not isinstance(self.red_interior_erosion_pixels,int) or not 0 <= self.red_interior_erosion_pixels <= 3:
            raise DetectionError('red_interior_erosion_pixels must be an integer in [0,3]')
        if not 0 <= self.illumination_max_gain_change < 1:
            raise DetectionError('illumination_max_gain_change must be in [0,1)')
        for name in ('red_hue_low_max', 'red_hue_high_min'):
            value = getattr(self, name)
            if not isinstance(value, int) or not 0 <= value <= 179:
                raise DetectionError(f"{name} must be an OpenCV hue integer in [0,179]")
        if not self.red_hue_low_max < self.red_hue_high_min:
            raise DetectionError("Red hue bands must not overlap")
        for name in ('max_area_fraction', 'min_fill_ratio', 'min_solidity', 'red_fraction_threshold',
                     'non_red_max_red_fraction', 'min_color_fraction', 'max_edge_density', 'boundary_fraction'):
            if not 0 < getattr(self, name) <= 1:
                raise DetectionError(f"{name} must be in (0,1]")
        if self.non_red_max_red_fraction >= self.red_fraction_threshold:
            raise DetectionError("Red/non-red classification thresholds must leave an ambiguous interval")
        if not isinstance(self.min_area_pixels, int) or self.min_area_pixels < 1:
            raise DetectionError("min_area_pixels must be a positive integer")
        if not isinstance(self.max_area_pixels, int) or self.max_area_pixels < 0 or (self.max_area_pixels and self.max_area_pixels < self.min_area_pixels):
            raise DetectionError('max_area_pixels must be 0 or an integer >= min_area_pixels')
        for name in ('exclusion_padding_pixels', 'boundary_margin_pixels', 'roi_margin_pixels'):
            if not isinstance(getattr(self, name), int) or not 0 <= getattr(self, name) <= 100:
                raise DetectionError(f"{name} must be an integer from 0 to 100")
        if self.max_aspect_ratio < 1 or not 0 <= self.min_mean_color_difference <= 255:
            raise DetectionError("Invalid aspect-ratio or mean color-difference threshold")
        if self.alignment_max_displacement_pixels <= 0 or not 0 < self.unverified_max_foreground_fraction <= 1:
            raise DetectionError('Invalid alignment displacement or unverified foreground fraction')
        if not isinstance(self.geometry_min_matches, int) or self.geometry_min_matches < 4:
            raise DetectionError('geometry_min_matches must be an integer >= 4')
        if any(getattr(self,name) <= 0 for name in ('geometry_max_shift_pixels','geometry_max_rotation_degrees','geometry_max_scale_change')) or not 0 < self.geometry_min_inlier_ratio <= 1:
            raise DetectionError('Invalid image-geometry thresholds')

    @classmethod
    def from_dict(cls, values: dict) -> DetectorConfig:
        try:
            return cls(**values)
        except (TypeError, ValueError) as exc:
            raise DetectionError(f"Invalid detector settings: {exc}") from exc


class SpillCategory(str, Enum):
    RED_SPILL = 'RED_SPILL'
    NON_RED_SPILL = 'NON_RED_SPILL'
    UNKNOWN = 'UNKNOWN'


@dataclass(frozen=True)
class SpillDetection:
    category: SpillCategory
    pixel_center: tuple[float, float]
    bounding_box: tuple[int, int, int, int]
    contour: tuple[tuple[int, int], ...]
    area_pixels: int
    detection_score: float
    classification_reason: str
    boundary_overlap: bool = False
    target_id: int | None = None
    confirmed: bool = False
    consecutive_frames: int = 1
    first_seen: float | None = None  # UTC Unix seconds; populated by SpillTracker.
    last_seen: float | None = None
    component_id: int = 0  # Per-frame connected-component label, not a persistent ID.

    def to_dict(self) -> dict:
        result = asdict(self)
        result['category'] = self.category.value
        # JSON-compatible lists, including deeply nested contour points.
        return json_native(result)


@dataclass
class DetectionResult:
    detections: list[SpillDetection]
    difference: np.ndarray  # max per-channel absolute difference after Gaussian blur, uint8
    foreground_mask: np.ndarray
    red_mask: np.ndarray
    analysis_mask: np.ndarray
    geometry: GeometryCheck | None = None
    illumination_offset_bgr: tuple[float, float, float] = (0.0, 0.0, 0.0)
    component_labels: np.ndarray | None = None
    raw_foreground_mask: np.ndarray | None = None
    raw_difference: np.ndarray | None = None
    illumination_gain_bgr: tuple[float, float, float] = (1.0, 1.0, 1.0)
    detection_paused: bool = False
    alignment_applied: bool = False
    color_diagnostics: dict[int,dict] = field(default_factory=dict)
    reference_to_current: np.ndarray | None = None  # Image registration only; never a cm transform.
    background_alignment_transform: np.ndarray | None = None  # Subtraction only; not localization authorization.
    component_rejections: dict[int, str] = field(default_factory=dict)

    def mask_for(self, target: SpillDetection) -> np.ndarray:
        """Exact full-frame component mask, preserving holes for later localization."""
        if self.component_labels is None or target.component_id < 1:
            raise DetectionError('This target has no component mask in the current result')
        return np.where(self.component_labels == target.component_id, 255, 0).astype(np.uint8)


class SpillDetector:
    """Detect candidate image regions; caller controls temporal filtering separately.

    Calibration supplies only pixel polygons/resolution. Neither homographies nor
    physical coordinates are accessed. Exclusion masks use nonzero = ignore.
    """

    def __init__(self, reference: BackgroundReference, config: DetectorConfig | None = None,
                 calibration: GroundCalibration | DualPlaneCalibration | None = None,
                 exclusion_mask: np.ndarray | None = None):
        if not isinstance(reference, BackgroundReference):
            raise DetectionError("A confirmed BackgroundReference is required")
        self.config = config or DetectorConfig()
        self._reference = reference.image
        self.shape = self._reference.shape[:2]
        height, width = self.shape
        self.polygons: dict[str, np.ndarray] = {}
        self._plane_masks: dict[str, np.ndarray] = {}
        if calibration is not None:
            calibration.validate_resolution(width, height)
            planes = [('FLOOR', calibration.floor), ('WALL', calibration.wall)] if isinstance(calibration, DualPlaneCalibration) else [(calibration.surface.value, calibration)]
            for name, plane in planes:
                polygon = np.rint(plane.corners_pixels).astype(np.int32)
                self.polygons[name] = polygon
                mask = np.zeros(self.shape, np.uint8)
                cv2.fillPoly(mask, [polygon], 255)
                self._plane_masks[name] = mask
            self._roi = np.maximum.reduce(list(self._plane_masks.values()))
        else:
            self._roi = np.full(self.shape, 255, np.uint8)
        if not self.config.roi_enabled:
            self._roi[:] = 255
        elif calibration is not None and self.config.roi_margin_pixels:
            size = 2*self.config.roi_margin_pixels+1
            # Preserve the shared interior edge so one object cannot be split into two.
            self._roi = cv2.erode(self._roi, np.ones((size,size),np.uint8),
                                 borderType=cv2.BORDER_CONSTANT, borderValue=0)
        self._exclusion = np.zeros(self.shape, np.uint8) if exclusion_mask is None else validate_mask(exclusion_mask, self.shape)
        self._blurred_reference = self._blur(self._reference)
        self._geometry_guard = FrameGeometryGuard(self._reference,
            max_shift_pixels=self.config.geometry_max_shift_pixels,
            max_rotation_degrees=self.config.geometry_max_rotation_degrees,
            max_scale_change=self.config.geometry_max_scale_change,
            min_matches=self.config.geometry_min_matches,
            min_inlier_ratio=self.config.geometry_min_inlier_ratio) if self.config.geometry_check_enabled else None
        self._shared_band = np.zeros(self.shape, bool)
        if 'FLOOR' in self._plane_masks and 'WALL' in self._plane_masks:
            size = 2 * self.config.boundary_margin_pixels + 1
            kernel = np.ones((size, size), np.uint8)
            self._shared_band = ((cv2.dilate(self._plane_masks['FLOOR'], kernel) != 0) &
                                 (cv2.dilate(self._plane_masks['WALL'], kernel) != 0))

    def _blur(self, image: np.ndarray) -> np.ndarray:
        size = self.config.blur_kernel
        return cv2.GaussianBlur(image, (size, size), 0) if size > 1 else image

    def detect(self, frame: np.ndarray, *, exclusion_mask: np.ndarray | None = None,
               framing_override: bool = False, color_diagnostics: bool = False) -> DetectionResult:
        """Return current candidates/debug masks; does not mutate the clean reference.

        framing_override is retained for API compatibility; framing never gates
        detection and reliable bounded alignment remains enabled.
        Per-frame exclusions combine with the configured static mask. Supply the
        moving object's current silhouette plus margin; robot tracking is external.
        """
        validate_image(frame)
        if not isinstance(framing_override, bool):
            raise DetectionError('framing_override must be an explicit boolean')
        if not isinstance(color_diagnostics,bool):
            raise DetectionError('color_diagnostics must be boolean')
        if frame.shape != self._reference.shape:
            raise DetectionError(f"Frame resolution {frame.shape[1]}x{frame.shape[0]} differs from background "
                                 f"{self.shape[1]}x{self.shape[0]}; capture a matching clean reference")
        cfg = self.config
        exclusion = self._exclusion.copy()
        if exclusion_mask is not None:
            exclusion = cv2.bitwise_or(exclusion, validate_mask(exclusion_mask, self.shape))
        if cfg.exclusion_padding_pixels:
            size = 2 * cfg.exclusion_padding_pixels + 1
            exclusion = cv2.dilate(exclusion, np.ones((size, size), np.uint8))
        allowed = cv2.bitwise_and(self._roi, cv2.bitwise_not(exclusion))
        geometry = self._geometry_guard.check(frame, exclusion) if self._geometry_guard else GeometryCheck('DISABLED', 'Image geometry checking explicitly disabled')
        reference_to_current = None
        if geometry.status == 'MATCHED' and self._geometry_guard.transform is not None:
            reference_to_current = np.vstack((self._geometry_guard.transform,[0.,0.,1.]))
        reference = self._blurred_reference
        aligned = False
        background_transform = None
        image_fit = (getattr(self._geometry_guard,'image_alignment_transform',self._geometry_guard.transform)
                     if self._geometry_guard is not None else None)
        if cfg.alignment_enabled and geometry.status in ('MATCHED','MOVED') and image_fit is not None:
            matrix = image_fit
            height, width = self.shape
            corners = np.float64([[0,0,1],[width-1,0,1],[width-1,height-1,1],[0,height-1,1]])
            displacement = float(np.linalg.norm(corners @ matrix.T-corners[:,:2],axis=1).max())
            if geometry.status == 'MATCHED' and displacement > cfg.alignment_max_displacement_pixels:
                geometry = GeometryCheck('MOVED', 'Camera framing changed beyond safe small-shift alignment',
                                         geometry.inlier_count, geometry.center_shift_pixels,
                                         geometry.rotation_degrees, geometry.scale_change)
            # Physical small-motion limits still diagnose MOVED and prevent reuse
            # for localization. A finite distributed RANSAC image fit can still
            # align the immutable background for subtraction, in current pixels.
            if displacement > .1:
                reference = cv2.warpAffine(reference, matrix, (width,height), flags=cv2.INTER_LINEAR)
                valid = cv2.warpAffine(np.full(self.shape,255,np.uint8),matrix,(width,height),flags=cv2.INTER_NEAREST)
                valid = cv2.erode(valid,np.ones((3,3),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)
                allowed = cv2.bitwise_and(allowed,valid)
                aligned = True
                background_transform = np.vstack((matrix,[0.,0.,1.]))
        delta = self._blur(frame).astype(np.int16) - reference.astype(np.int16)
        raw_difference = np.abs(delta).max(axis=2).astype(np.uint8)
        offset = np.zeros(3, dtype=float)
        gain = np.ones(3, dtype=float)
        if cfg.illumination_correction:
            # Only raw sub-threshold pixels estimate global brightness; candidate targets are excluded.
            stable = (allowed != 0) & (raw_difference <= min(cfg.illumination_max_shift, cfg.difference_threshold-1))
            if np.count_nonzero(stable) >= max(16, .3 * np.count_nonzero(allowed)):
                offset = np.median(delta[stable], axis=0)
                if cfg.illumination_gain_correction:
                    # Fit only sub-threshold background samples, never candidate pixels.
                    # Flat channels cannot constrain gain: retain median-offset correction.
                    # Bound fitting cost independently of camera resolution.
                    indices = np.flatnonzero(stable.ravel())
                    indices = indices[::max(1, (len(indices)+4095)//4096)]
                    reference_samples = reference.reshape(-1,3)[indices].astype(np.float32)
                    current_samples = reference_samples + delta.reshape(-1,3)[indices]
                    for channel in range(3):
                        x, y = reference_samples[:, channel], current_samples[:, channel]
                        if np.std(x) < 20:
                            continue
                        keep = np.ones(len(x), dtype=bool)
                        slope, intercept = 1.0, offset[channel]
                        for _ in range(3):
                            if np.count_nonzero(keep) < 16 or np.std(x[keep]) < 20:
                                break
                            xc, yc = x[keep], y[keep]
                            slope = float(np.mean((xc-xc.mean())*(yc-yc.mean())) / np.var(xc))
                            intercept = float(np.median(yc - slope*xc))
                            keep = np.abs(y - (slope*x + intercept)) <= max(3, cfg.difference_threshold*.25)
                        if (abs(slope-1) <= cfg.illumination_max_gain_change and
                                abs(intercept) <= cfg.illumination_max_shift):
                            gain[channel], offset[channel] = slope, intercept
        corrected = delta.astype(np.float32) - (gain.astype(np.float32)-1)*reference - offset.astype(np.float32)
        difference = np.clip(np.abs(corrected).max(axis=2), 0, 255).astype(np.uint8)
        foreground = np.where(difference >= cfg.difference_threshold, 255, 0).astype(np.uint8)
        foreground = cv2.bitwise_and(foreground, allowed)
        raw_foreground = foreground  # Subsequent OpenCV operations allocate their output.
        for operation, size in ((cv2.MORPH_OPEN, cfg.opening_kernel), (cv2.MORPH_CLOSE, cfg.closing_kernel)):
            if size > 1:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
                foreground = cv2.morphologyEx(foreground, operation, kernel)
        foreground = cv2.bitwise_and(foreground, allowed)  # Morphology may not grow back into excluded pixels.
        selected_foreground = foreground != 0
        red_map = np.zeros(self.shape, dtype=bool)
        if np.any(selected_foreground):
            # Classify only actual component pixels, never cardboard inside a box.
            red_map[selected_foreground] = red_pixels(frame[selected_foreground],cfg)
        red_mask = red_map.astype(np.uint8)*255
        eroded = cv2.erode(allowed, np.ones((3, 3), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=0)
        roi_edge = (allowed != 0) & (eroded == 0)
        count, labels, stats, centers = cv2.connectedComponentsWithStats(foreground, connectivity=8)
        detections = []
        color_details = {}
        component_rejections = {}
        # Operator-managed demo policy: registration diagnoses framing only.
        # Keep immutable-baseline segmentation running even when framing is unsure.
        paused = False
        max_area = np.count_nonzero(allowed) * cfg.max_area_fraction
        if cfg.max_area_pixels:
            max_area = min(max_area, cfg.max_area_pixels)
        for label in range(1, count):
            if paused:
                break
            x, y, width, height, area = (int(value) for value in stats[label])
            if area < cfg.min_area_pixels or area > max_area:
                continue
            component = labels[y:y+height, x:x+width] == label
            component_image = component.astype(np.uint8) * 255
            contours, _ = cv2.findContours(component_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour = max(contours, key=cv2.contourArea)
            contour = contour + np.array([[[x, y]]], np.int32)
            region = np.s_[y:y+height, x:x+width]
            contrast = float(difference[region][component].mean())
            evidence = component_red_evidence(frame[region],component,red_map[region],cfg,diagnostics=color_diagnostics)
            if cfg.brightening_rejection_enabled and not evidence['is_red']:
                # A restricted-scene hypothesis, not a reflection/material test:
                # overwhelmingly positive, nearly common-channel residuals.
                # Operates before tracking so rejected components cannot confirm.
                residual = corrected[region][component]
                brighter_fraction = float(np.mean(np.all(residual > 0, axis=1)))
                chromatic_spread = float(np.median(np.ptp(residual, axis=1)))
                if brighter_fraction >= .9 and chromatic_spread <= cfg.difference_threshold:
                    component_rejections[label] = (
                        'experimental_brightening_rejection; '
                        f'all_channels_brighter_fraction={brighter_fraction:.6f}; '
                        f'median_chromatic_spread_levels={chromatic_spread:.6f}')
                    continue
            crosses_boundary = False
            if 'FLOOR' in self._plane_masks and 'WALL' in self._plane_masks:
                floor = self._plane_masks['FLOOR'][region] != 0
                wall = self._plane_masks['WALL'][region] != 0
                floor_fraction = np.count_nonzero(component & floor & ~wall) / area
                wall_fraction = np.count_nonzero(component & wall & ~floor) / area
                crosses_boundary = bool(min(floor_fraction, wall_fraction) >= cfg.boundary_fraction or
                                        np.any(component & self._shared_band[region]))
            reasons = []
            if crosses_boundary:
                reasons.append('overlaps floor-wall boundary')
            if np.any(roi_edge[region] & component):
                reasons.append('truncated by image, ROI or exclusion boundary')
            category = SpillCategory.RED_SPILL if evidence['is_red'] else SpillCategory.NON_RED_SPILL
            reasons.insert(0, 'new baseline-relative region; '+evidence['reason'])
            if color_diagnostics:
                memberships = {name:int(np.count_nonzero(component & (mask[region]!=0)))
                               for name,mask in self._plane_masks.items()}
                surface = ('BOUNDARY' if crosses_boundary else
                           max(memberships,key=memberships.get) if memberships and max(memberships.values()) else 'UNKNOWN')
                color_details[label] = dict(evidence,component_id=label,area_pixels=area,surface_roi=surface,
                                           surface_pixel_counts=memberships,category=category.value,
                                           bounding_box=[x,y,width,height],pixel_center=[float(v) for v in centers[label]])
            score = min(1., contrast / 255)  # Difference strength, never a material probability.
            detections.append(SpillDetection(
                category=category, pixel_center=tuple(float(v) for v in centers[label]),
                bounding_box=(x, y, width, height), contour=tuple(tuple(int(v) for v in point) for point in contour[:, 0]),
                area_pixels=area, detection_score=round(float(score), 6),
                classification_reason='; '.join(reasons),
                boundary_overlap=crosses_boundary, component_id=label))
        detections.sort(key=lambda item: (item.pixel_center[0], item.pixel_center[1]))
        return DetectionResult(detections, difference, foreground, red_mask, allowed, geometry,
                               tuple(float(value) for value in offset), labels, raw_foreground,
                               raw_difference, tuple(float(v) for v in gain), paused, aligned,color_details,
                               reference_to_current if geometry.status == 'MATCHED' else None,
                               background_transform, component_rejections)


def render_detections(frame: np.ndarray, detections: list[SpillDetection],
                      polygons: dict[str, np.ndarray] | None = None, *, debug: bool = False) -> np.ndarray:
    """Operator boxes are confirmed-only; debug adds pending regions and evidence."""
    image = frame.copy()
    for name, polygon in (polygons or {}).items():
        color = (0, 255, 0) if name == 'FLOOR' else (255, 0, 255)
        cv2.polylines(image, [polygon], True, color, 2)
        cv2.putText(image, name, tuple(polygon[0]), 0, .5, color, 1)
    colors = {SpillCategory.RED_SPILL: (0, 0, 255), SpillCategory.NON_RED_SPILL: (0, 220, 255),
              SpillCategory.UNKNOWN: (180, 180, 180)}
    label_boxes = []
    visible = detections if debug else [target for target in detections if target.confirmed]
    for index, target in enumerate(sorted(visible, key=lambda target: target.bounding_box[1])):
        color = colors[target.category] if target.confirmed else (255, 180, 60)
        x, y, width, height = target.bounding_box
        cv2.rectangle(image, (x, y), (x+width-1, y+height-1), color, 2)
        if debug:
            cv2.polylines(image, [np.asarray(target.contour, np.int32)], True, color, 1)
        state = 'confirmed' if target.confirmed else f'pending:{target.consecutive_frames}'
        lines = (f'#{target.target_id} {target.category.value}',)
        if debug:
            lines += (f'{state} area={target.area_pixels}px score={target.detection_score:.2f}',)
        text_width = max(cv2.getTextSize(text, 0, .45, 1)[0][0] for text in lines)
        label_x = max(0, min(x, image.shape[1] - text_width - 2))
        # Try above/below the region so neighboring candidates do not obscure each other's labels.
        for proposed_y in (y - 34, y + height + 4, y - 68, 26 + index * 34):
            top = max(0, min(proposed_y, image.shape[0] - 32))
            box = (label_x, top, label_x + text_width, top + 30)
            if not any(box[0] < old[2] and box[2] > old[0] and box[1] < old[3] and box[3] > old[1]
                       for old in label_boxes):
                break
        label_boxes.append(box)
        for row, text in enumerate(lines):
            label_y = min(image.shape[0] - 2, top + 12 + row * 14)
            cv2.putText(image, text, (label_x, label_y), 0, .45, (0, 0, 0), 3)
            cv2.putText(image, text, (label_x, label_y), 0, .45, color, 1)
    return image
