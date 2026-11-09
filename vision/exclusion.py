"""Manual pixel-space polygons for static ignored regions; no surface localization."""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from .spill_detection import DetectionError


def exclusion_mask_from_polygons(shape: tuple[int, int], polygons) -> np.ndarray:
    """Rasterize simple polygons; shape=(height,width), nonzero pixels are excluded."""
    mask = np.zeros(shape, np.uint8)
    try:
        if not isinstance(polygons, (list, tuple)):
            raise ValueError('polygons must be a list')
        for polygon in polygons:
            points = np.asarray(polygon, dtype=float)
            if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.isfinite(points).all():
                raise ValueError('each polygon needs at least three finite [u,v] pairs')
            if (points < 0).any() or (points[:,0] >= shape[1]).any() or (points[:,1] >= shape[0]).any():
                raise ValueError('polygon coordinates must lie inside the reference image')
            points = np.rint(points).astype(np.int32)
            if abs(cv2.contourArea(points)) < 1:
                raise ValueError('exclusion polygon has zero pixel area')
            cv2.fillPoly(mask, [points], 255)
    except (TypeError, ValueError) as exc:
        raise DetectionError(f'Invalid manual exclusion polygons: {exc}') from exc
    return mask


def load_exclusion_polygons(path: str | Path, shape: tuple[int, int]) -> np.ndarray:
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(data, dict) or set(data) != {'frame_width','frame_height','polygons'}:
            raise ValueError('require frame_width, frame_height, polygons')
        if (data['frame_height'], data['frame_width']) != shape:
            raise ValueError('exclusion polygon resolution differs from reference')
        return exclusion_mask_from_polygons(shape, data['polygons'])
    except (OSError, TypeError, ValueError) as exc:
        raise DetectionError(f'Cannot load manual exclusion polygons {path}: {exc}') from exc
