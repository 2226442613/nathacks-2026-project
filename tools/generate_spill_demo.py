"""Generate deterministic offline engineering scenes, not an accuracy benchmark."""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def write_image(path: Path, image: np.ndarray) -> None:
    if not cv2.imwrite(str(path), image):
        raise OSError(f'Cannot write {path}')


def generate_demo(directory: str | Path) -> Path:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    clean = np.full((320, 400, 3), 200, np.uint8)
    scene = clean.copy()
    cv2.circle(scene, (100, 150), 24, (20, 20, 220), -1)
    cv2.ellipse(scene, (260, 160), (32, 20), 15, 0, 360, (20, 180, 180), -1)
    cv2.circle(scene, (250, 260), 18, (45, 45, 45), -1)  # New dark NON_RED foreground.
    exclusion = np.zeros(clean.shape[:2], np.uint8)
    exclusion[:65] = 255  # Exclude a moving-object lane, not automatically detected robot geometry.
    write_image(directory / 'clean.png', clean)
    write_image(directory / 'spills.png', scene)
    write_image(directory / 'exclude.png', exclusion)
    path = directory / 'spills.avi'
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 10, (400, 320))
    if not writer.isOpened():
        raise OSError('MJPG video encoder unavailable')
    try:
        for index in range(18):
            frame = clean.copy() if index < 3 else scene.copy()
            cv2.circle(frame, (30 + index*15, 32), 12, (220, 40, 20), -1)
            if index in (6, 7):
                cv2.circle(frame, (70, 260), 12, (220, 40, 20), -1)
            writer.write(frame)
    finally:
        writer.release()
    return directory


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='data/spill_demo')
    arguments = parser.parse_args()
    print(generate_demo(arguments.output_dir))
