"""Small one-to-one centroid/box association with stationary multi-frame confirmation."""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import math
import time
from pathlib import Path

from .spill_detection import DetectionError, DetectorConfig, SpillDetection


@dataclass(frozen=True)
class TrackerConfig:
    confirmation_frames: int = 3
    max_missed_frames: int = 2
    max_match_distance_pixels: float = 35.0
    stationary_distance_pixels: float = 4.0
    max_area_ratio: float = 2.5
    confirmation_min_iou: float = 0.6
    confirmation_max_area_ratio: float = 1.35

    def __post_init__(self) -> None:
        for name in ('confirmation_frames', 'max_missed_frames'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < (1 if name == 'confirmation_frames' else 0):
                raise DetectionError(f"{name} must be an integer >= {1 if name == 'confirmation_frames' else 0}")
        for name in ('max_match_distance_pixels', 'stationary_distance_pixels', 'max_area_ratio'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise DetectionError(f"{name} must be positive and finite")
        if self.max_area_ratio < 1 or self.stationary_distance_pixels > self.max_match_distance_pixels:
            raise DetectionError("Invalid tracking area ratio or stationary distance")
        for name in ('confirmation_min_iou', 'confirmation_max_area_ratio'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise DetectionError(f'{name} must be finite')
        if not 0 <= self.confirmation_min_iou <= 1 or self.confirmation_max_area_ratio < 1:
            raise DetectionError('Invalid confirmation IoU or area ratio')

    @classmethod
    def from_dict(cls, values: dict) -> TrackerConfig:
        try:
            return cls(**values)
        except (TypeError, ValueError) as exc:
            raise DetectionError(f"Invalid tracker settings: {exc}") from exc


@dataclass
class _Track:
    detection: SpillDetection
    anchor: tuple[float, float]
    first_seen: float
    consecutive: int = 1
    missed: int = 0


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])


def _iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[0]+a[2], b[0]+b[2]), min(a[1]+a[3], b[1]+b[3])
    intersection = max(0, x1-x0) * max(0, y1-y0)
    union = a[2]*a[3] + b[2]*b[3] - intersection
    return intersection / union if union else 0.0


class SpillTracker:
    """IDs remain stable for basic noncrossing video scenes; not a robot tracker.

    Return only current observations, including pending candidates. Missing tracks
    are retained privately for a bounded time but are never emitted as stale events.
    Confirmation requires consecutive, category-consistent, spatially stationary
    observations. Movement, gaps, or category changes reset that streak.
    """

    def __init__(self, config: TrackerConfig | None = None):
        self.config = config or TrackerConfig()
        self._tracks: dict[int, _Track] = {}
        self._next_id = 1
        self._last_timestamp: float | None = None

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1
        self._last_timestamp = None

    def update(self, detections: list[SpillDetection], *, timestamp: float | None = None) -> list[SpillDetection]:
        """timestamp is UTC Unix seconds of observation, not recorded-video presentation time."""
        if timestamp is None:
            timestamp = max(time.time(), self._last_timestamp or 0)
        if isinstance(timestamp,bool) or not isinstance(timestamp,(int,float)) or not math.isfinite(timestamp) or timestamp < 0:
            raise DetectionError('Observation timestamp must be finite nonnegative Unix seconds')
        if self._last_timestamp is not None and timestamp < self._last_timestamp:
            raise DetectionError('Observation timestamps must be nondecreasing')
        self._last_timestamp = float(timestamp)
        cfg = self.config
        matches = []
        for target_id, track in self._tracks.items():
            for index, detection in enumerate(detections):
                distance = _distance(track.detection.pixel_center, detection.pixel_center)
                areas = (track.detection.area_pixels, detection.area_pixels)
                ratio = max(areas) / max(1, min(areas))
                if distance <= cfg.max_match_distance_pixels and ratio <= cfg.max_area_ratio:
                    cost = distance + .25 * cfg.max_match_distance_pixels * (1 - _iou(track.detection.bounding_box, detection.bounding_box))
                    matches.append((cost, target_id, index))
        used_tracks, used_detections = set(), set()
        associations = {}
        for _cost, target_id, index in sorted(matches):
            if target_id not in used_tracks and index not in used_detections:
                used_tracks.add(target_id)
                used_detections.add(index)
                associations[index] = target_id
        for target_id in list(self._tracks):
            if target_id not in used_tracks:
                self._tracks[target_id].missed += 1
                self._tracks[target_id].consecutive = 0
                if self._tracks[target_id].missed > cfg.max_missed_frames:
                    del self._tracks[target_id]
        output = []
        for index, detection in enumerate(detections):
            if index in associations:
                target_id = associations[index]
                track = self._tracks[target_id]
                stable = (track.missed == 0 and track.detection.category == detection.category and
                          _distance(track.anchor, detection.pixel_center) <= cfg.stationary_distance_pixels and
                          _iou(track.detection.bounding_box, detection.bounding_box) >= cfg.confirmation_min_iou and
                          max(track.detection.area_pixels, detection.area_pixels) /
                          max(1, min(track.detection.area_pixels, detection.area_pixels)) <= cfg.confirmation_max_area_ratio)
                track.consecutive = track.consecutive + 1 if stable else 1
                if not stable:
                    track.anchor = detection.pixel_center
                track.missed = 0
            else:
                target_id = self._next_id
                self._next_id += 1
                track = _Track(detection, detection.pixel_center, float(timestamp))
                self._tracks[target_id] = track
            observed = replace(detection, target_id=target_id, consecutive_frames=track.consecutive,
                               confirmed=track.consecutive >= cfg.confirmation_frames,
                               first_seen=track.first_seen, last_seen=float(timestamp))
            track.detection = observed
            output.append(observed)
        return output


def load_spill_settings(path: str | Path) -> tuple[DetectorConfig, TrackerConfig]:
    """Strict JSON settings prevent misspelled threshold names from being ignored."""
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(data, dict) or set(data) != {'settings_version', 'detector', 'tracker'} or data['settings_version'] != 1:
            raise DetectionError("Settings require version 1, detector, and tracker objects")
        return DetectorConfig.from_dict(data['detector']), TrackerConfig.from_dict(data['tracker'])
    except (OSError, ValueError, TypeError) as exc:
        raise DetectionError(f"Cannot load spill settings {path}: {exc}") from exc


def save_spill_settings(path: str | Path, detector: DetectorConfig, tracker: TrackerConfig) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'settings_version': 1, 'detector': asdict(detector),
                                'tracker': asdict(tracker)}, indent=2, allow_nan=False) + '\n', encoding='utf-8')
