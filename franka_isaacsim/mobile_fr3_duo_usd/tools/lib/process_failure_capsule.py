"""Process-level failure capsule builder (stdlib-only).

Hard aborts never reach in-process ``except``; the launcher writes this capsule
under the evidence directory via host ``python3``.
"""

from __future__ import annotations

from typing import Any


def build_process_failure_capsule(
    *,
    exit_code: int,
    model: str | None = None,
    evidence_dir: str | None = None,
    owned_root_pid: int | None = None,
    owned_pgid: int | None = None,
    log_excerpt: str | None = None,
    where: str = "process_abort",
    error: str | None = None,
) -> dict[str, Any]:
    """Capsule for externally fatal runs that never reach in-process except."""
    return {
        "ok": False,
        "provenance": "process_level",
        "where": where,
        "error": error
        or (
            "Capture process exited before writing an in-process failure capsule "
            f"(exit_code={exit_code})"
        ),
        "exit_code": int(exit_code),
        "model": model,
        "evidence_dir": evidence_dir,
        "owned_root_pid": owned_root_pid,
        "owned_pgid": owned_pgid,
        "log_excerpt": log_excerpt,
        "preexisting_kit_note": "Do not signal foreign Kit PIDs; owned teardown only.",
    }
