#!/usr/bin/env python3
"""External post-gate Isaac/Kit teardown verification (survives SimulationApp os._exit)."""

from __future__ import annotations

import runpy
from pathlib import Path

_TOOLING = runpy.run_path(
    str(Path(__file__).resolve().parents[1] / "lib" / "bootstrap.py")
)["ensure_tooling_on_path"]()

import argparse
import json
import sys


from lib.teardown import verify_after_gate


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--default-evidence", type=Path, default=None)
    parser.add_argument("--natural-wait-s", type=float, default=25.0)
    parser.add_argument("--owned-root-pid", type=int, default=None)
    parser.add_argument("--owned-pgid", type=int, default=None)
    args = parser.parse_args()

    report = verify_after_gate(
        request_path=args.request,
        default_evidence_path=args.default_evidence,
        natural_wait_s=args.natural_wait_s,
        owned_pgid=args.owned_pgid,
        owned_root_pid=args.owned_root_pid,
    )
    evidence = None
    if args.request.is_file():
        try:
            evidence = json.loads(args.request.read_text(encoding="utf-8")).get("evidence_path")
        except (OSError, json.JSONDecodeError):
            evidence = None
    if not evidence:
        evidence = str(args.default_evidence) if args.default_evidence is not None else None
    # Prefer the path actually written by verify_after_gate when request was honored.
    written = None
    if evidence and Path(evidence).is_file():
        written = evidence
    elif args.default_evidence is not None and args.default_evidence.is_file():
        written = str(args.default_evidence)
    summary = {
        "clean": report["clean"],
        "evidence_path": written or evidence,
        "after_count": len(report.get("after") or []),
        "owned_after_count": len(report.get("owned_after") or []),
        "foreign_after_count": len(report.get("foreign_after") or []),
        "signaled": report.get("signaled"),
        "permission_denied": report.get("permission_denied"),
        "owned_pgid": report.get("owned_pgid"),
        "owned_root_pid": report.get("owned_root_pid"),
        "gate": report.get("gate"),
    }
    print(json.dumps(summary, sort_keys=True))
    return 0 if report["clean"] else 3


if __name__ == "__main__":
    sys.exit(main())
