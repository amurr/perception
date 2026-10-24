#!/usr/bin/env bash
# Launch Isaac Sim 6.0.1 with an optional USD stage (GUI).
# Without a USD path, Isaac Sim starts with an empty stage.
#
#   ./isaacsim_launch.sh
#   DISPLAY=:10 ./isaacsim_launch.sh
#   ./isaacsim_launch.sh /path/to/world.usda
#   ISAAC_SIM_ROOT=/path/to/isaacsim ./isaacsim_launch.sh --reset-user
#
# Extra arguments are forwarded to isaac-sim.sh / Kit.
set -euo pipefail

ISAAC="${ISAAC_SIM_ROOT:-/workspace/isaacsim}"
LAUNCHER="${ISAAC}/isaac-sim.sh"

if [[ ! -x "$LAUNCHER" ]]; then
  echo "error: Isaac Sim launcher not found: $LAUNCHER" >&2
  echo "Set ISAAC_SIM_ROOT to the 6.0.1 install directory." >&2
  exit 1
fi

# xrdp is usually :10, VNC is :1. Override with DISPLAY=:N if needed.
if [[ -z "${DISPLAY:-}" ]]; then
  if [[ -S /tmp/.X11-unix/X10 ]]; then
    DISPLAY=:10
  elif [[ -S /tmp/.X11-unix/X1 ]]; then
    DISPLAY=:1
  else
    echo "error: no X display (start xrdp or VNC, or set DISPLAY=:N)" >&2
    exit 1
  fi
fi
export DISPLAY

# Kit will not start without an accepted EULA. Root needs an extra flag on this pod.
export ACCEPT_EULA="${ACCEPT_EULA:-Y}"
export PRIVACY_CONSENT="${PRIVACY_CONSENT:-Y}"
export OMNI_KIT_ACCEPT_EULA=YES
export OMNI_KIT_ALLOW_ROOT=1
export NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all,graphics,utility,compute,display,video}"
export OMNI_SERVER="${OMNI_SERVER:-https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0}"

# Copy the desktop user's X cookie so root can draw on that display.
if [[ -z "${XAUTHORITY:-}" ]]; then
  if [[ -f /home/ubuntu/.Xauthority ]]; then
    cp -f /home/ubuntu/.Xauthority /root/.Xauthority
    chmod 600 /root/.Xauthority
    export XAUTHORITY=/root/.Xauthority
  elif [[ -f "${HOME}/.Xauthority" ]]; then
    export XAUTHORITY="${HOME}/.Xauthority"
  fi
fi
if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

# Kit looks for renderD128; some GPUs expose a different render node.
chmod a+rw /dev/dri/card* /dev/dri/renderD* 2>/dev/null || true
if [[ ! -e /dev/dri/renderD128 ]]; then
  for n in 130 135 129 131 132 133 134; do
    if [[ -e "/dev/dri/renderD${n}" ]]; then
      ln -sf "/dev/dri/renderD${n}" /dev/dri/renderD128
      break
    fi
  done
fi

# One GPU, one Kit. Skip our own process tree so this script is not killed.
echo "Stopping any previous Isaac Kit processes..."
mine=()
pid=$$
while [[ -n "$pid" && "$pid" != 0 && "$pid" != 1 ]]; do
  mine+=("$pid")
  pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')"
done
for pattern in "${ISAAC}/kit/kit " isaac-sim.sh; do
  for victim in $(pgrep -f -- "$pattern" 2>/dev/null || true); do
    [[ " ${mine[*]} " == *" $victim "* ]] && continue
    kill -9 "$victim" 2>/dev/null || true
  done
done
sleep 1
rm -f /dev/shm/sem.carbonite-sharedmemory /dev/shm/carb-RStringInternals-* 2>/dev/null || true

kit_args=("$@")
if [[ "${kit_args[0]:-}" != --* && "${kit_args[0]:-}" =~ \.(usd|usda|usdc|usdz)$ ]]; then
  stage_path="${kit_args[0]}"
  if [[ ! -f "$stage_path" ]]; then
    echo "error: USD stage not found: $stage_path" >&2
    exit 1
  fi
  # isaacsim.app.setup unconditionally blows away the opened stage with a blank
  # template on startup unless this is disabled (isaac.startup.create_new_stage
  # defaults to true in isaacsim.app.setup/config/extension.toml).
  kit_args=("--/app/file/filename=$stage_path" "--/isaac/startup/create_new_stage=false" "--/app/viewport/cameraPath=/World/OverviewCamera" "${kit_args[@]:1}")
fi

echo "Isaac Sim 6.0.1 GUI on DISPLAY=${DISPLAY}"
if [[ "${kit_args[0]:-}" == --/app/file/filename=* ]]; then
  echo "Opening USD stage: ${kit_args[0]#--/app/file/filename=}"
else
  echo "Empty stage — Create > Ground Plane, then drag a robot from Isaac Assets."
fi

cmd=("$LAUNCHER" --allow-root "${kit_args[@]}")
if command -v vglrun >/dev/null 2>&1; then
  cmd=(vglrun -d egl "${cmd[@]}")
fi

exec "${cmd[@]}"
