"""Strict JSON helpers that stay importable without numpy.

Process-level failure capsules are written by host ``python3`` after Isaac
exits. That interpreter may lack numpy, so serialization must not require it.
When numpy is present, array / numpy scalar values are still coerced safely.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

try:
    import numpy as np
except ImportError:  # pragma: no cover - exercised on bare host python3
    np = None  # type: ignore[assignment]


def finite_or_none(value: Any) -> float | None:
    """Return a finite float, else None (never emit NaN into reports)."""
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def json_safe(value: Any) -> Any:
    """Recursively replace non-finite floats with null for strict JSON."""
    if isinstance(value, Mapping):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if np is not None:
        if isinstance(value, np.ndarray):
            return json_safe(value.tolist())
        if isinstance(value, np.floating):
            return finite_or_none(float(value))
        if isinstance(value, np.integer):
            return int(value)
        if isinstance(value, np.bool_):
            return bool(value)
    if isinstance(value, float):
        return finite_or_none(value)
    if isinstance(value, (bool, int, str)) or value is None:
        return value
    return value


def dumps_strict_json(
    payload: Any, *, indent: int = 2, sort_keys: bool = True
) -> str:
    return (
        json.dumps(
            json_safe(payload),
            indent=indent,
            sort_keys=sort_keys,
            allow_nan=False,
        )
        + "\n"
    )


def write_strict_json(path: Path, payload: Any, *, sort_keys: bool = True) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        dumps_strict_json(payload, sort_keys=sort_keys), encoding="utf-8"
    )
