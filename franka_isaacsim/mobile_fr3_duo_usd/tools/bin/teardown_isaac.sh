#!/usr/bin/env bash
# Ownership-scoped Isaac/Kit teardown entrypoint.
# Refuses host-wide pgrep→kill. Requires the launcher's owned process group.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLING_ROOT="${ISAACSIM6_TOOLING_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

OWNED_PGID="${ISAACSIM6_OWNED_PGID:-}"
OWNED_ROOT_PID="${ISAACSIM6_OWNED_ROOT_PID:-}"
if [[ $# -ge 1 && -z "${OWNED_PGID}" ]]; then
  OWNED_PGID="$1"
fi
if [[ $# -ge 2 && -z "${OWNED_ROOT_PID}" ]]; then
  OWNED_ROOT_PID="$2"
fi

if [[ -n "${ISAACSIM6_TEARDOWN_EVIDENCE:-}" ]]; then
  EVIDENCE="${ISAACSIM6_TEARDOWN_EVIDENCE}"
else
  EVIDENCE="$(mktemp "${TMPDIR:-/tmp}/isaacsim6_teardown_owned.XXXXXX.json")"
fi

if [[ -z "${OWNED_PGID}" && -z "${OWNED_ROOT_PID}" ]]; then
  echo "ERROR: refusing host-wide Isaac teardown." >&2
  echo "Usage: ISAACSIM6_OWNED_PGID=<pgid> [ISAACSIM6_OWNED_ROOT_PID=<pid>] $0" >&2
  echo "   or: $0 <owned-pgid> [owned-root-pid]" >&2
  exit 2
fi

REQUEST="${ISAACSIM6_TEARDOWN_REQUEST:-}"
CLEANUP_REQUEST=0
if [[ -z "${REQUEST}" ]]; then
  REQUEST="$(mktemp "${TMPDIR:-/tmp}/isaacsim6_teardown_request.XXXXXX")"
  printf '{}\n' >"${REQUEST}"
  CLEANUP_REQUEST=1
fi
if [[ "${CLEANUP_REQUEST}" -eq 1 ]]; then
  trap 'rm -f "${REQUEST}"' EXIT
fi

ARGS=(
  --request "${REQUEST}"
  --default-evidence "${EVIDENCE}"
  --natural-wait-s "${ISAACSIM6_TEARDOWN_NATURAL_WAIT_S:-0}"
)
if [[ -n "${OWNED_PGID}" ]]; then
  ARGS+=(--owned-pgid "${OWNED_PGID}")
fi
if [[ -n "${OWNED_ROOT_PID}" ]]; then
  ARGS+=(--owned-root-pid "${OWNED_ROOT_PID}")
fi

python3 "${TOOLING_ROOT}/bin/verify_teardown.py" "${ARGS[@]}"
exit $?
