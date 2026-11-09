"""Image-only stationary-camera registration; no physical coordinates."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class GeometryCheck:
    status: str  # MATCHED, MOVED, UNVERIFIED, DISABLED
    reason: str
    inlier_count: int = 0
    center_shift_pixels: float | None = None
    rotation_degrees: float | None = None
    scale_change: float | None = None

    def to_dict(self) -> dict:
        return asdict(self)


class FrameGeometryGuard:
    """ORB + RANSAC similarity fit diagnoses framing changes when stable texture exists.

    Low texture/ambiguous matching is UNVERIFIED, never a claim that the camera is
    stationary. Fit parameters are image-space diagnostics, not robot/world geometry.
    """

    def __init__(self, reference: np.ndarray, *, max_shift_pixels: float = 8,
                 max_rotation_degrees: float = 2, max_scale_change: float = .03,
                 min_matches: int = 12, min_inlier_ratio: float = .6):
        self.shape = reference.shape[:2]
        self.max_shift_pixels = max_shift_pixels
        self.max_rotation_degrees = max_rotation_degrees
        self.max_scale_change = max_scale_change
        self.min_matches = min_matches
        self.min_inlier_ratio = min_inlier_ratio
        self._orb = cv2.ORB_create(nfeatures=1000)
        gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
        self._keypoints, self._descriptors = self._orb.detectAndCompute(gray, None)
        self.transform = None  # Reliable reference-to-current similarity, reset on every check.
        # Separate image matching from the unchanged small-motion physical trust
        # limits. Consumers must not use this image-only fit as a verified pose.
        self.image_alignment_transform = None
        # Repeated/localized texture may never establish global registration,
        # even against itself. Preserve that low-information diagnostic separately.
        self.reference_verifiable = self.check(reference).status == 'MATCHED'

    def check(self, frame: np.ndarray, exclusion_mask: np.ndarray | None = None) -> GeometryCheck:
        self.transform = None
        self.image_alignment_transform = None
        if self._descriptors is None or len(self._keypoints) < self.min_matches:
            return GeometryCheck('UNVERIFIED', 'Reference has insufficient stable texture; manually verify camera framing')
        mask = cv2.bitwise_not(exclusion_mask) if exclusion_mask is not None else None
        keypoints, descriptors = self._orb.detectAndCompute(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), mask)
        if descriptors is None or len(keypoints) < self.min_matches:
            return GeometryCheck('UNVERIFIED', 'Current frame has insufficient unmasked texture')
        pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(self._descriptors, descriptors, k=2)
        matches = []
        used_current = set()
        for neighbors in pairs:
            if len(neighbors) != 2:
                continue
            best, runner_up = neighbors
            if best.distance >= .75 * runner_up.distance or best.trainIdx in used_current:
                continue
            x, y = self._keypoints[best.queryIdx].pt
            if exclusion_mask is not None and exclusion_mask[int(y), int(x)] != 0:
                continue
            matches.append(best)
            used_current.add(best.trainIdx)
        if len(matches) < self.min_matches:
            return GeometryCheck('UNVERIFIED', 'Too few background correspondences; camera movement or occlusion is possible')
        source = np.float32([self._keypoints[item.queryIdx].pt for item in matches])
        destination = np.float32([keypoints[item.trainIdx].pt for item in matches])
        matrix, inliers = cv2.estimateAffinePartial2D(source, destination, method=cv2.RANSAC,
                                                    ransacReprojThreshold=2.0, maxIters=1500)
        if matrix is None or inliers is None or not np.isfinite(matrix).all():
            return GeometryCheck('UNVERIFIED', 'No finite, consistent image-space fit')
        selected = inliers.ravel() != 0
        count = int(selected.sum())
        if count < self.min_matches or count / len(matches) < self.min_inlier_ratio:
            return GeometryCheck('UNVERIFIED', 'Background matches do not agree on framing', count)
        span = np.ptp(source[selected], axis=0)
        if float(span[0] * span[1]) < .1 * self.shape[0] * self.shape[1]:
            return GeometryCheck('UNVERIFIED', 'Matches are too localized to establish global framing', count)
        center = np.array([self.shape[1]/2, self.shape[0]/2, 1.0])
        shift = float(np.linalg.norm(matrix @ center - center[:2]))
        rotation = float(math.degrees(math.atan2(matrix[1, 0], matrix[0, 0])))
        scale_change = abs(float(math.hypot(matrix[0, 0], matrix[1, 0])) - 1)
        if abs(float(np.linalg.det(matrix[:,:2]))) <= np.finfo(float).eps:
            return GeometryCheck('UNVERIFIED', 'Image-space fit is singular', count)
        self.image_alignment_transform = matrix.copy()
        moved = shift > self.max_shift_pixels or abs(rotation) > self.max_rotation_degrees or scale_change > self.max_scale_change
        if not moved:
            self.transform = matrix
        return GeometryCheck('MOVED' if moved else 'MATCHED',
            'Image framing exceeds configured motion limits' if moved else 'Distributed background matches support stable framing',
            count, shift, rotation, scale_change)
