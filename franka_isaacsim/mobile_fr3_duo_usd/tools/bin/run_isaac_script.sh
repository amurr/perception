#!/usr/bin/env bash
# Launch a tooling script with Isaac Sim's python.sh in an owned process group,
# then verify teardown for that ownership scope only (child may os._exit on close).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLING_ROOT="${ISAACSIM6_TOOLING_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

if [[ -z "${ISAACSIM_PATH:-}" ]]; then
  echo "ERROR: ISAACSIM_PATH is required (path to Isaac Sim installation)" >&2
  exit 2
fi
ISAAC_ROOT="${ISAACSIM_PATH}"
PYTHON_SH="${ISAAC_ROOT}/python.sh"

if [[ ! -x "${PYTHON_SH}" ]]; then
  echo "ERROR: Isaac python.sh not found/executable: ${PYTHON_SH}" >&2
  exit 2
fi

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <script-name.py|path> [args...]" >&2
  exit 2
fi

SCRIPT_ARG="$1"
shift

if [[ "${SCRIPT_ARG}" == /* ]]; then
  SCRIPT_PATH="${SCRIPT_ARG}"
else
  SCRIPT_PATH="${TOOLING_ROOT}/scripts/${SCRIPT_ARG}"
fi

if [[ ! -f "${SCRIPT_PATH}" ]]; then
  echo "ERROR: script not found: ${SCRIPT_PATH}" >&2
  exit 2
fi

export ISAACSIM6_TOOLING_ROOT="${TOOLING_ROOT}"

# Optional capture coordinates from CLI (used for process-level failure capsules).
EVIDENCE_DIR="${ISAACSIM6_EVIDENCE_DIR:-}"
CAPTURE_MODEL=""
PREV_ARG=""
for ARG in "$@"; do
  if [[ "${PREV_ARG}" == "--evidence-dir" ]]; then
    EVIDENCE_DIR="${ARG}"
  elif [[ "${PREV_ARG}" == "--model" ]]; then
    CAPTURE_MODEL="${ARG}"
  fi
  PREV_ARG="${ARG}"
done

REQUEST_FILE="$(mktemp "${TMPDIR:-/tmp}/isaacsim6_teardown_request.XXXXXX")"
if [[ -n "${ISAACSIM6_TEARDOWN_EVIDENCE:-}" ]]; then
  DEFAULT_TEARDOWN="${ISAACSIM6_TEARDOWN_EVIDENCE}"
else
  # Collision-safe fallback when the child never registers an evidence path.
  DEFAULT_TEARDOWN="$(mktemp "${TMPDIR:-/tmp}/isaacsim6_teardown_unscoped.XXXXXX.json")"
fi
export ISAACSIM6_TEARDOWN_REQUEST="${REQUEST_FILE}"
trap 'rm -f "${REQUEST_FILE}"' EXIT

CAPTURE_LOG=""
if [[ -n "${EVIDENCE_DIR}" ]]; then
  mkdir -p "${EVIDENCE_DIR}"
  CAPTURE_LOG="${EVIDENCE_DIR}/capture_run.log"
fi

# New session/process group so post-exit teardown can signal only this tree.
set +e
if [[ -n "${CAPTURE_LOG}" ]]; then
  setsid "${PYTHON_SH}" "${SCRIPT_PATH}" "$@" > >(tee -a "${CAPTURE_LOG}") 2>&1 &
else
  setsid "${PYTHON_SH}" "${SCRIPT_PATH}" "$@" &
fi
CHILD_PID=$!
OWNED_PGID="$(ps -o pgid= -p "${CHILD_PID}" 2>/dev/null | tr -d ' ' || true)"
if [[ -z "${OWNED_PGID}" ]]; then
  OWNED_PGID="${CHILD_PID}"
fi
wait "${CHILD_PID}"
SCRIPT_EXIT=$?
set -e

set +e
python3 "${TOOLING_ROOT}/bin/verify_teardown.py" \
  --request "${REQUEST_FILE}" \
  --default-evidence "${DEFAULT_TEARDOWN}" \
  --natural-wait-s "${ISAACSIM6_TEARDOWN_NATURAL_WAIT_S:-25}" \
  --owned-root-pid "${CHILD_PID}" \
  --owned-pgid "${OWNED_PGID}"
TEARDOWN_EXIT=$?
set -e

echo "SCRIPT_EXIT=${SCRIPT_EXIT} TEARDOWN_EXIT=${TEARDOWN_EXIT} OWNED_ROOT_PID=${CHILD_PID} OWNED_PGID=${OWNED_PGID}"

# Hard aborts (GPU device-lost, segfault) never reach in-process except and
# leave no capture_summary.json. Exit 2/3 are in-process selection outcomes.
# Capsule writer runs under host python3 after Isaac exits (numpy-free path).
if [[ "${SCRIPT_EXIT}" -ne 0 && -n "${EVIDENCE_DIR}" ]]; then
  if [[ ! -f "${EVIDENCE_DIR}/failure_capsule.json" && ! -f "${EVIDENCE_DIR}/capture_summary.json" ]]; then
    set +e
    CAPSULE_ARGS=(
      --evidence-dir "${EVIDENCE_DIR}"
      --exit-code "${SCRIPT_EXIT}"
      --owned-root-pid "${CHILD_PID}"
      --owned-pgid "${OWNED_PGID}"
    )
    if [[ -n "${CAPTURE_MODEL}" ]]; then
      CAPSULE_ARGS+=(--model "${CAPTURE_MODEL}")
    fi
    if [[ -n "${CAPTURE_LOG}" && -f "${CAPTURE_LOG}" ]]; then
      CAPSULE_ARGS+=(--log-file "${CAPTURE_LOG}")
    fi
    python3 "${TOOLING_ROOT}/bin/write_process_failure_capsule.py" "${CAPSULE_ARGS[@]}"
    CAPSULE_EXIT=$?
    set -e
    if [[ "${CAPSULE_EXIT}" -ne 0 ]]; then
      echo "ERROR: process failure capsule writer failed (exit=${CAPSULE_EXIT}); child status preserved as ${SCRIPT_EXIT}" >&2
    fi
    if [[ ! -f "${EVIDENCE_DIR}/failure_capsule.json" ]]; then
      echo "ERROR: failure_capsule.json missing after hard-abort capsule attempt under ${EVIDENCE_DIR}" >&2
    fi
  fi
fi

if [[ "${SCRIPT_EXIT}" -ne 0 ]]; then
  exit "${SCRIPT_EXIT}"
fi
if [[ "${TEARDOWN_EXIT}" -ne 0 ]]; then
  exit "${TEARDOWN_EXIT}"
fi
exit 0
