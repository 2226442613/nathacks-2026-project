"""Stage PNG/metadata before replacement; roll back ordinary commit failures."""
import hashlib
import os
from pathlib import Path
import tempfile

import cv2

from .serialization import json_line


def _stage(path: Path, data: bytes) -> Path:
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f'.{path.name}.', delete=False) as stream:
        staged = Path(stream.name)
        try:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            staged.unlink(missing_ok=True)
            raise
    return staged


def save_reference_pair(path: Path, image, metadata: dict) -> dict:
    """Each rename is atomic. Hash detects a process crash between the two renames.

    Commit metadata first, then PNG; an ordinary PNG replacement failure restores
    previous metadata. A power/process interruption between renames fails closed
    at load rather than treating mismatched image/metadata as a registered pair.
    """
    ok, encoded = cv2.imencode('.png', image)
    if not ok:
        raise OSError(f'Cannot encode baseline PNG: {path}')
    payload = encoded.tobytes()
    metadata = dict(metadata, image_sha256=hashlib.sha256(payload).hexdigest())
    sidecar = Path(str(path)+'.json')
    staged = []
    try:
        png = _stage(path, payload)
        staged.append(png)
        info = _stage(sidecar, (json_line(metadata)+'\n').encode('utf-8'))
        staged.append(info)
        backup = _stage(sidecar, sidecar.read_bytes()) if sidecar.exists() else None
        if backup is not None:
            staged.append(backup)
        os.replace(info, sidecar)
        try:
            os.replace(png, path)
        except BaseException:
            if backup is None:
                sidecar.unlink(missing_ok=True)
            else:
                os.replace(backup, sidecar)
            raise
        return metadata
    finally:
        for temporary in staged:
            temporary.unlink(missing_ok=True)
