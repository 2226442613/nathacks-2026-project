"""macOS-exposed camera devices and local video files; no transport/GUI assumptions."""
from __future__ import annotations

import logging
import math
import platform
from pathlib import Path

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class CameraError(RuntimeError):
    """Capture initialization or physical camera read failed."""


class CameraManager:
    """Read BGR frames. File EOF returns None; camera failures raise CameraError."""

    def __init__(self, source: int | str | Path = 0, width: int = 1280,
                 height: int = 720, fps: float = 30):
        if isinstance(source, bool) or not isinstance(source, (int, str, Path)):
            raise ValueError("Source must be a camera index or local video path")
        if isinstance(source, int) and source < 0:
            raise ValueError("Camera index must be nonnegative")
        if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in (width, height)):
            raise ValueError("Requested width and height must be positive integers")
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Requested FPS must be positive and finite")
        self.source = str(source) if isinstance(source, Path) else source
        self.width, self.height, self.fps = width, height, float(fps)
        self._capture = None

    @property
    def is_file(self) -> bool:
        return isinstance(self.source, str)

    def open(self) -> CameraManager:
        self.release()
        if self.is_file and not Path(self.source).is_file():
            raise CameraError(f"Local video does not exist: {self.source}")
        backends = ([cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]
                    if not self.is_file and platform.system() == "Darwin" else [cv2.CAP_ANY])
        for backend in backends:
            capture = cv2.VideoCapture(self.source, backend)
            if capture.isOpened():
                self._capture = capture
                break
            capture.release()
        if self._capture is None:
            raise CameraError(f"Cannot open {self.source!r}. Camera may be disconnected, busy, "
                              "or unavailable; check macOS camera permission and device index.")
        try:
            if not self.is_file:
                for prop, value in ((cv2.CAP_PROP_FRAME_WIDTH, self.width),
                                    (cv2.CAP_PROP_FRAME_HEIGHT, self.height),
                                    (cv2.CAP_PROP_FPS, self.fps)):
                    if not self._capture.set(prop, value):
                        logger.debug("Capture did not accept property %s", prop)
                # Best effort: AVFoundation may ignore this. Synchronous reads avoid a Python queue.
                self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            logger.info("Opened %r: %s", self.source, self.settings)
        except Exception:
            self.release()
            raise
        return self

    @property
    def settings(self) -> dict[str, float | None]:
        if self._capture is None:
            raise CameraError("Camera is not open")
        result = {}
        for name, prop in (("width", cv2.CAP_PROP_FRAME_WIDTH),
                           ("height", cv2.CAP_PROP_FRAME_HEIGHT), ("fps", cv2.CAP_PROP_FPS)):
            value = float(self._capture.get(prop))
            result[name] = value if math.isfinite(value) and value > 0 else None
        return result

    def read(self) -> np.ndarray | None:
        if self._capture is None:
            raise CameraError("Camera is not open")
        ok, frame = self._capture.read()
        if not ok or frame is None or frame.size == 0:
            if self.is_file:
                return None  # OpenCV cannot reliably distinguish EOF from a file decode failure.
            raise CameraError("Camera frame read failed; the device may be disconnected or Continuity Camera "
                              "interrupted. Check the selected macOS camera and its permission/availability")
        return frame

    def release(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def __enter__(self) -> CameraManager:
        return self.open()

    def __exit__(self, *_: object) -> None:
        self.release()
