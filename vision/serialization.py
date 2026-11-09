"""Strict JSON conversion for detector and diagnostic NumPy values."""
import json
import math

import numpy as np


def json_native(value, path='$'):
    """Preserve supported data; reject unsupported or non-finite values with a path."""
    if isinstance(value, np.ndarray):
        return json_native(value.tolist(), path)
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        value = float(value)
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f'Non-finite JSON number at {path}')
        return value
    if isinstance(value, dict):
        if any(not isinstance(key, str) for key in value):
            raise TypeError(f'JSON object keys must be strings at {path}')
        return {key: json_native(item, f'{path}.{key}') for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_native(item, f'{path}[{index}]') for index, item in enumerate(value)]
    raise TypeError(f'Unsupported JSON value {type(value).__name__} at {path}')


def json_line(value):
    return json.dumps(json_native(value), allow_nan=False)
