"""Ownership-scoped Isaac / Kit process teardown helpers.

In-process verification after SimulationApp.close() is unreliable: Kit may call
os._exit and skip the rest of the finally block. Scripts register an evidence path;
bin/run_isaac_script.sh verifies teardown in a fresh process after the child exits.

Signaling is limited to the wrapper-owned process group (and explicit ancestry
fallback). Foreign Isaac-like leftovers are reported but never signaled.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any

_LOG = logging.getLogger(__name__)


# Avoid bare "kit" / "carb." — they false-positive on rtkit-daemon / polkitd.
# Avoid bare "isaacsim" — argv paths under this repo can contain that substring.
ISAAC_PROCESS_PATTERN = (
    r"(isaac-sim|/isaacsim/|/kit/kit|omni\.kit\.|omni\.telemetry\.transmitter|"
    r"SimulationApp|isaacsim\.exp\.|apps/isaacsim)"
)

TEARDOWN_REQUEST_ENV = "ISAACSIM6_TEARDOWN_REQUEST"
OWNED_ROOT_PID_ENV = "ISAACSIM6_OWNED_ROOT_PID"
OWNED_PGID_ENV = "ISAACSIM6_OWNED_PGID"

_HELPER_COMMAND_MARKERS = (
    "pgrep",
    "teardown_isaac.sh",
    "verify_teardown",
    "run_isaac_script.sh",
)


def close_simulation_app(simulation_app) -> None:
    try:
        import omni.timeline

        omni.timeline.get_timeline_interface().stop()
    except Exception as error:
        _LOG.warning("timeline stop failed during SimulationApp close: %s", error)
    try:
        simulation_app.close()
    except Exception as error:
        _LOG.warning("SimulationApp.close() failed: %s", error)


def list_matching_processes(pattern: str = ISAAC_PROCESS_PATTERN) -> list[dict[str, Any]]:
    try:
        result = subprocess.run(
            ["pgrep", "-af", pattern],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return []
    rows: list[dict[str, Any]] = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        if any(marker in line for marker in _HELPER_COMMAND_MARKERS):
            continue
        parts = line.split(None, 1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        rows.append({"pid": pid, "command": parts[1] if len(parts) > 1 else ""})
    return rows


def register_external_teardown(evidence_path: Path, gate: str = "") -> None:
    """Record where the wrapper should write teardown evidence after this process exits."""
    request_path = os.environ.get(TEARDOWN_REQUEST_ENV)
    try:
        pgid = os.getpgid(0)
    except OSError:
        pgid = None
    payload = {
        "evidence_path": str(evidence_path),
        "gate": gate,
        "registered_at_unix_s": time.time(),
        "pid": os.getpid(),
        "pgid": pgid,
    }
    if request_path:
        path = Path(request_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    # Also drop a sibling marker so audits can see registration even if the wrapper
    # request file is absent (direct python.sh invocation).
    marker = evidence_path.with_name(evidence_path.stem + "_request.json")
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_report(evidence_path: Path | None, report: dict[str, Any]) -> None:
    if evidence_path is None:
        return
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def read_process_pgid(pid: int) -> int | None:
    try:
        return os.getpgid(pid)
    except (ProcessLookupError, PermissionError, OSError):
        return None


def read_process_ppid(pid: int) -> int | None:
    stat_path = Path(f"/proc/{pid}/stat")
    try:
        text = stat_path.read_text(encoding="utf-8")
    except (FileNotFoundError, PermissionError, OSError):
        return None
    close_paren = text.rfind(")")
    if close_paren < 0:
        return None
    fields = text[close_paren + 2 :].split()
    if len(fields) < 2:
        return None
    try:
        return int(fields[1])
    except ValueError:
        return None


def is_owned_process(
    pid: int,
    *,
    owned_pgid: int | None,
    owned_root_pid: int | None,
) -> bool:
    if pid <= 0:
        return False
    if owned_root_pid is not None and pid == owned_root_pid:
        return True
    if owned_pgid is not None:
        pgid = read_process_pgid(pid)
        if pgid is not None and pgid == owned_pgid:
            return True
    if owned_root_pid is None:
        return False
    seen: set[int] = set()
    current = pid
    while current > 1 and current not in seen:
        seen.add(current)
        parent = read_process_ppid(current)
        if parent is None:
            return False
        if parent == owned_root_pid:
            return True
        current = parent
    return False


def classify_owned_processes(
    rows: list[dict[str, Any]],
    *,
    owned_pgid: int | None,
    owned_root_pid: int | None,
    self_pid: int | None = None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    self_pid = os.getpid() if self_pid is None else self_pid
    owned: list[dict[str, Any]] = []
    foreign: list[dict[str, Any]] = []
    for row in rows:
        pid = int(row["pid"])
        if pid == self_pid:
            continue
        if is_owned_process(pid, owned_pgid=owned_pgid, owned_root_pid=owned_root_pid):
            owned.append(row)
        else:
            foreign.append(row)
    return owned, foreign


def _signal_pid(pid: int, sig: signal.Signals, report: dict[str, Any]) -> None:
    try:
        os.kill(pid, sig)
        report["signaled"].append({"pid": pid, "signal": sig.name, "scope": "pid"})
    except ProcessLookupError:
        pass
    except PermissionError:
        report["permission_denied"].append({"pid": pid, "signal": sig.name, "scope": "pid"})


def _caller_pgid() -> int | None:
    try:
        return os.getpgid(0)
    except OSError:
        return None


def _signal_process_group(pgid: int, sig: signal.Signals, report: dict[str, Any]) -> bool:
    """Signal a process group unless it is the caller's own group.

    Returns True when killpg was attempted (even if the target vanished).
    Returns False when refused because owned_pgid equals the caller's pgid.
    """
    caller_pgid = _caller_pgid()
    if caller_pgid is not None and caller_pgid == pgid:
        report.setdefault("pgid_signal_refused", []).append(
            {
                "pgid": pgid,
                "signal": sig.name,
                "reason": "owned_pgid_is_caller_process_group",
            }
        )
        report["pgid_signal_refused_reason"] = "owned_pgid_is_caller_process_group"
        return False
    try:
        os.killpg(pgid, sig)
        report["signaled"].append({"pgid": pgid, "signal": sig.name, "scope": "pgid"})
    except ProcessLookupError:
        pass
    except PermissionError:
        report["permission_denied"].append({"pgid": pgid, "signal": sig.name, "scope": "pgid"})
    return True


def ensure_no_isaac_processes(
    evidence_path: Path | None = None,
    grace_s: float = 2.0,
    force: bool = True,
    natural_wait_s: float = 25.0,
    poll_s: float = 1.0,
    owned_pgid: int | None = None,
    owned_root_pid: int | None = None,
) -> dict[str, Any]:
    """Clear owned Isaac/Kit leftovers; report foreign matches without signaling them.

    Without ownership coordinates, force signaling is refused. Foreign leftovers
    never fail the clean verdict; remaining owned processes do.
    """
    self_pid = os.getpid()
    report: dict[str, Any] = {
        "before": [],
        "owned_before": [],
        "foreign_before": [],
        "signaled": [],
        "permission_denied": [],
        "after": [],
        "owned_after": [],
        "foreign_after": [],
        "after_natural_wait": [],
        "clean": False,
        "self_pid": self_pid,
        "natural_wait_s": natural_wait_s,
        "external": True,
        "owned_pgid": owned_pgid,
        "owned_root_pid": owned_root_pid,
        "ownership_required_for_signal": True,
        "signal_attempted": False,
    }
    try:
        before = list_matching_processes()
        report["before"] = before
        owned, foreign = classify_owned_processes(
            before,
            owned_pgid=owned_pgid,
            owned_root_pid=owned_root_pid,
            self_pid=self_pid,
        )
        report["owned_before"] = owned
        report["foreign_before"] = foreign

        leftovers = list(owned)
        deadline = time.time() + max(0.0, natural_wait_s)
        while leftovers and time.time() < deadline:
            time.sleep(max(0.1, poll_s))
            current = list_matching_processes()
            leftovers, _ = classify_owned_processes(
                current,
                owned_pgid=owned_pgid,
                owned_root_pid=owned_root_pid,
                self_pid=self_pid,
            )
        report["after_natural_wait"] = list(leftovers)

        can_signal = force and (owned_pgid is not None or owned_root_pid is not None)
        if leftovers and force and not can_signal:
            report["signal_refused_reason"] = "missing_ownership"
        elif leftovers and can_signal:
            report["signal_attempted"] = True
            if owned_pgid is not None:
                _signal_process_group(owned_pgid, signal.SIGTERM, report)
            for row in leftovers:
                _signal_pid(int(row["pid"]), signal.SIGTERM, report)
            time.sleep(grace_s)
            still_rows = list_matching_processes()
            still, _ = classify_owned_processes(
                still_rows,
                owned_pgid=owned_pgid,
                owned_root_pid=owned_root_pid,
                self_pid=self_pid,
            )
            if still and owned_pgid is not None:
                _signal_process_group(owned_pgid, signal.SIGKILL, report)
            for row in still:
                _signal_pid(int(row["pid"]), signal.SIGKILL, report)
            time.sleep(0.5)

        after = list_matching_processes()
        owned_after, foreign_after = classify_owned_processes(
            after,
            owned_pgid=owned_pgid,
            owned_root_pid=owned_root_pid,
            self_pid=self_pid,
        )
        report["after"] = after
        report["owned_after"] = owned_after
        report["foreign_after"] = foreign_after
        has_ownership = owned_pgid is not None or owned_root_pid is not None
        if has_ownership:
            report["clean"] = len(owned_after) == 0
        else:
            # No ownership scope: never signal, and only claim clean when nothing matches.
            report["clean"] = len(after) == 0
            if not report["clean"] and "signal_refused_reason" not in report:
                report["signal_refused_reason"] = "missing_ownership"
    finally:
        _write_report(evidence_path, report)
    return report


def _coerce_optional_int(value: Any) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def resolve_ownership(
    *,
    request_payload: dict[str, Any] | None = None,
    owned_pgid: int | None = None,
    owned_root_pid: int | None = None,
) -> tuple[int | None, int | None]:
    payload = request_payload or {}
    if owned_pgid is None:
        owned_pgid = _coerce_optional_int(payload.get("owned_pgid"))
    if owned_pgid is None:
        owned_pgid = _coerce_optional_int(os.environ.get(OWNED_PGID_ENV))
    if owned_pgid is None:
        owned_pgid = _coerce_optional_int(payload.get("pgid"))

    if owned_root_pid is None:
        owned_root_pid = _coerce_optional_int(payload.get("owned_root_pid"))
    if owned_root_pid is None:
        owned_root_pid = _coerce_optional_int(os.environ.get(OWNED_ROOT_PID_ENV))
    if owned_root_pid is None:
        owned_root_pid = _coerce_optional_int(payload.get("pid"))
    return owned_pgid, owned_root_pid


def verify_after_gate(
    request_path: Path,
    default_evidence_path: Path | None = None,
    natural_wait_s: float = 25.0,
    owned_pgid: int | None = None,
    owned_root_pid: int | None = None,
) -> dict[str, Any]:
    """Post-child teardown verification used by bin/run_isaac_script.sh."""
    evidence_path = default_evidence_path
    gate = ""
    payload: dict[str, Any] = {}
    if request_path.is_file():
        try:
            loaded = json.loads(request_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                payload = loaded
                registered = payload.get("evidence_path")
                if registered:
                    evidence_path = Path(registered)
                gate = str(payload.get("gate") or "")
        except (OSError, json.JSONDecodeError):
            pass
    owned_pgid, owned_root_pid = resolve_ownership(
        request_payload=payload,
        owned_pgid=owned_pgid,
        owned_root_pid=owned_root_pid,
    )
    if evidence_path is None:
        handle, temporary_name = tempfile.mkstemp(
            prefix="isaacsim6_teardown_unscoped_",
            suffix=".json",
        )
        os.close(handle)
        evidence_path = Path(temporary_name)
    report = ensure_no_isaac_processes(
        evidence_path=evidence_path,
        natural_wait_s=natural_wait_s,
        force=True,
        owned_pgid=owned_pgid,
        owned_root_pid=owned_root_pid,
    )
    report["gate"] = gate
    report["request_path"] = str(request_path)
    _write_report(evidence_path, report)
    return report
