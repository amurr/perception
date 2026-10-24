#!/usr/bin/env python3
"""Merge bounded native-sensor capture batches into an authoritative summary.

Union full-profile single-session attach is a reproducible GPU device-lost
failure. Deterministic bounded batches are the supported alternative; this
merger is the only path that may report full-profile green, and only after
proving exact disjoint coverage of every declared logical sensor plus
per-sensor semantic success.

Requires a numpy-bearing interpreter — use Isaac Sim's
``${ISAACSIM_PATH}/python.sh`` (same as capture). Bare host ``python3`` is
not sufficient when numpy is absent from the system site-packages.
"""

from __future__ import annotations

import argparse
import json
import runpy
import sys
from pathlib import Path
from typing import Any


def _load_batch(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Batch summary must be an object: {path}")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--batch-summary",
        action="append",
        type=Path,
        required=True,
        help="Path to capture_summary.json from one batch (repeatable).",
    )
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    tooling = Path(
        __import__("os").environ.get(
            "ISAACSIM6_TOOLING_ROOT",
            str(Path(__file__).resolve().parents[1]),
        )
    )
    ensure_tooling = runpy.run_path(str(tooling / "lib" / "bootstrap.py"))[
        "ensure_tooling_on_path"
    ]
    ensure_tooling(tooling)

    from lib.sensor_capture import (
        logical_names_for_model,
        merge_capture_batch_summaries,
        write_strict_json,
    )

    batches = [_load_batch(path.resolve()) for path in args.batch_summary]
    declared = logical_names_for_model(args.model)
    merged = merge_capture_batch_summaries(
        batches,
        declared_logical_names=declared,
        model=args.model,
    )
    merged["batch_summary_paths"] = [str(path.resolve()) for path in args.batch_summary]
    write_strict_json(args.report.resolve(), merged)
    return 0 if merged.get("ok") else 3


if __name__ == "__main__":
    sys.exit(main())
