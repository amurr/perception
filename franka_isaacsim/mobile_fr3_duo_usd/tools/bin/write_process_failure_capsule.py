#!/usr/bin/env python3
"""Write a process-level failure capsule when Isaac capture aborts externally.

GPU device-lost / hard aborts never reach the in-process ``except`` path in
``capture_native_sensors.py``. The launcher calls this after a non-zero child
exit so provenance remains reachable under the evidence directory.

Runnable under bare host ``python3`` (no numpy / Isaac site-packages required).
"""

from __future__ import annotations

import argparse
import json
import runpy
import sys
import traceback
from pathlib import Path


def _emergency_write(
    evidence_dir: Path,
    *,
    exit_code: int,
    error: str,
    model: str | None,
    owned_root_pid: int | None,
    owned_pgid: int | None,
) -> Path | None:
    """Last-resort capsule when the normal writer path fails mid-flight."""
    try:
        evidence_dir.mkdir(parents=True, exist_ok=True)
        capsule_path = evidence_dir / "failure_capsule.json"
        payload = {
            "ok": False,
            "provenance": "process_level",
            "where": "process_abort_writer_failure",
            "error": error,
            "exit_code": int(exit_code),
            "model": model,
            "evidence_dir": str(evidence_dir),
            "owned_root_pid": owned_root_pid,
            "owned_pgid": owned_pgid,
            "log_excerpt": None,
            "preexisting_kit_note": "Do not signal foreign Kit PIDs; owned teardown only.",
        }
        capsule_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        return capsule_path
    except Exception as emergency_error:  # noqa: BLE001 - must report, never silent
        print(
            f"ERROR: emergency failure_capsule write failed: {emergency_error}",
            file=sys.stderr,
        )
        return None


def _run(args: argparse.Namespace) -> int:
    tooling = Path(__file__).resolve().parents[1]
    ensure_tooling = runpy.run_path(str(tooling / "lib" / "bootstrap.py"))[
        "ensure_tooling_on_path"
    ]
    ensure_tooling(tooling)

    from lib.process_failure_capsule import build_process_failure_capsule
    from lib.strict_json import write_strict_json

    evidence_dir = args.evidence_dir.resolve()
    evidence_dir.mkdir(parents=True, exist_ok=True)
    capsule_path = evidence_dir / "failure_capsule.json"
    if capsule_path.is_file() and not args.force:
        existing = capsule_path.read_text(encoding="utf-8")
        if existing.strip():
            print(f"failure_capsule_exists={capsule_path}")
            return 0

    log_excerpt = None
    log_file = args.log_file
    if log_file is None:
        candidate = evidence_dir / "capture_run.log"
        if candidate.is_file():
            log_file = candidate
    if log_file is not None and log_file.is_file():
        lines = log_file.read_text(encoding="utf-8", errors="replace").splitlines()
        log_excerpt = "\n".join(lines[-80:])

    capsule = build_process_failure_capsule(
        exit_code=int(args.exit_code),
        model=args.model,
        evidence_dir=str(evidence_dir),
        owned_root_pid=args.owned_root_pid,
        owned_pgid=args.owned_pgid,
        log_excerpt=log_excerpt,
    )
    write_strict_json(capsule_path, capsule)
    print(f"failure_capsule_written={capsule_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--exit-code", type=int, required=True)
    parser.add_argument("--model", default=None)
    parser.add_argument("--owned-root-pid", type=int, default=None)
    parser.add_argument("--owned-pgid", type=int, default=None)
    parser.add_argument("--log-file", type=Path, default=None)
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite an existing failure_capsule.json",
    )
    args = parser.parse_args()

    try:
        return _run(args)
    except Exception as exc:  # noqa: BLE001 - capsule writer must not fail silently
        print(
            f"ERROR: write_process_failure_capsule failed: {exc}",
            file=sys.stderr,
        )
        traceback.print_exc(file=sys.stderr)
        emergency = _emergency_write(
            Path(args.evidence_dir),
            exit_code=int(args.exit_code),
            error=f"capsule_writer_failed: {exc}",
            model=args.model,
            owned_root_pid=args.owned_root_pid,
            owned_pgid=args.owned_pgid,
        )
        if emergency is not None:
            print(f"failure_capsule_emergency_written={emergency}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
