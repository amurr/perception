#!/usr/bin/env bash
# Install XFCE + xrdp and start RDP on 127.0.0.1:3389.
# Target: the ROS 2 Jazzy container (osrf/ros:jazzy-desktop-full, Ubuntu 24.04,
# root, no systemd).
# Do not publish 3389 — SSH-tunnel it, same as VNC.
#
#   ./start_xrdp.sh              # install if needed, start
#   ./start_xrdp.sh status
#   ./start_xrdp.sh reconnect    # after a dropped Remmina: keep :10, drop extras
#   ./start_xrdp.sh stop
#
# Optional: XRDP_PASSWORD=secret ./start_xrdp.sh
#
# login: username= root, password= xrdp
set -euo pipefail

PORT=3389
SESMAN_PORT=3350
# Local side of the SSH tunnel. 3389 is often already taken on Windows PCs.
LOCAL_PORT="${XRDP_LOCAL_PORT:-13389}"
USER_NAME="$(id -un)"
PASSWORD="${XRDP_PASSWORD:-xrdp}"

is_up() {
  ss -lnt 2>/dev/null | grep -qE "127\\.0\\.0\\.1:${PORT}\\b"
}

# xrdp always dials IPv4 127.0.0.1:3350. With --enable-ipv6, ListenAddress=127.0.0.1
# binds [::1] only, so login dies with "error connecting to sesman".
sesman_ipv4_ok() {
  python3 -c "import socket; s=socket.socket(); s.settimeout(1); s.connect(('127.0.0.1', ${SESMAN_PORT}))" 2>/dev/null
}

xrdp_desktop_running() {
  ps -eo args= | grep -qE '[X](vnc|tigervnc) :1[0-9]( |$)'
}

systemd_live() {
  command -v systemctl >/dev/null 2>&1 && systemctl is-system-running >/dev/null 2>&1
}

# Dead Remmina links otherwise sit "connected" for hours (Linux default
# keepalive is 2h). Then the next login creates :11 and Isaac stays on :10.
# /proc/sys is read-only in unprivileged containers; then set these with
# docker run --sysctl net.ipv4.tcp_keepalive_time=30 ... instead.
short_tcp_keepalive() {
  sysctl -w net.ipv4.tcp_keepalive_time=30 >/dev/null 2>&1 || return 1
  sysctl -w net.ipv4.tcp_keepalive_intvl=5 >/dev/null 2>&1
  sysctl -w net.ipv4.tcp_keepalive_probes=3 >/dev/null 2>&1
}

# Keep Xvnc :10 (Isaac) and the optional VNC helper on :1. Kill :11, :12, …
kill_extra_desktops() {
  local pid args
  while read -r pid args; do
    [[ "$args" =~ (Xvnc|Xtigervnc)\ :10(\ |$) ]] && continue
    [[ "$args" =~ (Xvnc|Xtigervnc)\ :1(\ |$) ]] && continue
    echo "stopping extra desktop: $args"
    kill "$pid" 2>/dev/null || true
  done < <(ps -eo pid=,args= | grep -E '[X]vnc :' || true)
}

# Drop stale RDP clients, keep the listener and sesman (so :10 can be reused).
disconnect_stale_clients() {
  local main kid
  main="$(pgrep -xo xrdp || true)"
  [[ -n "$main" ]] || return 0
  for kid in $(pgrep -x xrdp || true); do
    [[ "$kid" == "$main" ]] && continue
    kill "$kid" 2>/dev/null || true
  done
}

ensure_tls_readable() {
  getent group ssl-cert >/dev/null 2>&1 && usermod -aG ssl-cert xrdp 2>/dev/null || true
  if ! id xrdp >/dev/null 2>&1; then
    return 0
  fi
  # runuser (util-linux) instead of sudo: the container has no sudo.
  if runuser -u xrdp -- test -r /etc/xrdp/key.pem 2>/dev/null; then
    return 0
  fi
  # systemd unit runs as user xrdp; snakeoil key is 640 root:ssl-cert.
  rm -f /etc/xrdp/key.pem /etc/xrdp/cert.pem
  openssl req -x509 -newkey rsa:2048 -nodes \
    -keyout /etc/xrdp/key.pem -out /etc/xrdp/cert.pem -days 3650 \
    -subj "/CN=xrdp" >/dev/null 2>&1
  chown xrdp:xrdp /etc/xrdp/key.pem /etc/xrdp/cert.pem
  chmod 640 /etc/xrdp/key.pem
  chmod 644 /etc/xrdp/cert.pem
}

print_connect_hint() {
  echo "xrdp on 127.0.0.1:${PORT}  (not published — do not RDP to the pod public IP)"
  echo "  user     ${USER_NAME}"
  echo "  pass     ${PASSWORD}"
  echo "  session  Xvnc"
  echo "From your PC, keep this tunnel open, then RDP to 127.0.0.1:${LOCAL_PORT}:"
  if [[ -n "${RUNPOD_PUBLIC_IP:-}" ]]; then
    echo "  ssh -N -L ${LOCAL_PORT}:127.0.0.1:${PORT} -p ${RUNPOD_TCP_PORT_22:-22} ${USER_NAME}@${RUNPOD_PUBLIC_IP}"
  else
    echo "  ssh -N -L ${LOCAL_PORT}:127.0.0.1:${PORT} -p PORT ${USER_NAME}@HOST"
  fi
  echo "Local ${LOCAL_PORT} avoids Windows' own listener on 3389."
  echo "After a dropped connection: one Remmina window, or: $0 reconnect"
}

prepare_apt_sources() {
  local source_file
  for source_file in /etc/apt/sources.list /etc/apt/sources.list.d/*; do
    [[ -f "$source_file" ]] || continue
    case "$source_file" in
      *.list|*.sources|/etc/apt/sources.list)
        sed -i \
          -e 's|http://archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g' \
          -e 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' \
          "$source_file"
        ;;
    esac
  done
}

case "${1:-start}" in
start)
  need_install=false
  for cmd in xrdp startxfce4 Xtigervnc ss pgrep openssl xrandr xrdb; do
    command -v "$cmd" >/dev/null || need_install=true
  done
  if $need_install; then
    echo "Installing XFCE + xrdp..."
    export DEBIAN_FRONTEND=noninteractive
    prepare_apt_sources
    apt-get \
      -o Acquire::Retries=2 \
      -o Acquire::http::Timeout=10 \
      -o Acquire::https::Timeout=10 \
      update
    apt-get install -y --no-install-recommends \
      xrdp xfce4 xfce4-terminal dbus-x11 \
      tigervnc-standalone-server \
      x11-xserver-utils iproute2 procps openssl ssl-cert ca-certificates
  fi

  # sesman calls "Xvnc"; Ubuntu 22.04/24.04 only ship Xtigervnc.
  command -v Xvnc >/dev/null || ln -sf "$(command -v Xtigervnc)" /usr/local/bin/Xvnc

  # xfwm4 default easy_click=Alt steals Alt+Left/Right from Isaac Sim orbit.
  xml="$HOME/.config/xfce4/xfconf/xfce-perchannel-xml/xfwm4.xml"
  mkdir -p "$(dirname "$xml")"
  if [[ -f "$xml" ]] && grep -q 'name="easy_click"' "$xml"; then
    sed -i 's/\(name="easy_click" type="string" value="\)[^"]*"/\1None"/' "$xml"
  elif [[ -f "$xml" ]] && grep -q 'name="general"' "$xml"; then
    sed -i '/name="general"/a\    <property name="easy_click" type="string" value="None"/>' "$xml"
  else
    cat >"$xml" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<channel name="xfwm4" version="1.0">
  <property name="general" type="empty">
    <property name="easy_click" type="string" value="None"/>
  </property>
</channel>
XML
  fi

  # Each RDP login should start XFCE at a normal size.
  # Remmina often connects at the tiny window size (e.g. 636x396); the client
  # then scales that up and every icon looks huge.
  GEOMETRY="${XRDP_GEOMETRY:-1920x1080}"
  # sesman starts the desktop with a clean environment, so the Docker ENV
  # (RMW, Fast DDS profile, domain) would be missing in XFCE terminals and in
  # Isaac Sim launched from there. Bake the current values in.
  cat > /etc/xrdp/startwm.sh <<EOF
#!/bin/sh
GEOMETRY=${GEOMETRY}
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/root/.ros/fastdds.xml}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
EOF
  cat >> /etc/xrdp/startwm.sh <<'EOF'
unset DBUS_SESSION_BUS_ADDRESS
unset SESSION_MANAGER
export GDK_SCALE=1
export GDK_DPI_SCALE=1
export QT_SCALE_FACTOR=1
command -v xrandr >/dev/null && xrandr -s "$GEOMETRY" >/dev/null 2>&1
echo "Xft.dpi: 96" | xrdb -merge 2>/dev/null

# Give Alt+Left/Right/Middle click to Isaac Sim, not xfwm4 window move/resize.
(
  i=0
  while [ "$i" -lt 50 ]; do
    if pgrep -x xfwm4 >/dev/null 2>&1 && command -v xfconf-query >/dev/null 2>&1; then
      xfconf-query -c xfwm4 -p /general/easy_click -n -t string -s None \
        >/dev/null 2>&1 && break
    fi
    i=$((i + 1))
    sleep 0.2
  done
) &

exec startxfce4
EOF
  chmod +x /etc/xrdp/startwm.sh
  echo startxfce4 > "$HOME/.xsession"

  # First port= in xrdp.ini is the listen port. Keep it on localhost.
  sed -i "0,/^port=/ s|^port=.*|port=tcp://127.0.0.1:${PORT}|" /etc/xrdp/xrdp.ini
  # 0.0.0.0 → dual-stack *:3350. 127.0.0.1 → [::1] only on this xrdp build.
  sed -i 's/^ListenAddress=.*/ListenAddress=0.0.0.0/' /etc/xrdp/sesman.ini
  sed -i 's/^AllowRootLogin=.*/AllowRootLogin=true/' /etc/xrdp/sesman.ini
  # Same user + color depth reuses the existing desktop (do not use UBD:
  # a different Remmina window size would start :11).
  sed -i 's/^Policy=.*/Policy=Default/' /etc/xrdp/sesman.ini
  sed -i 's/^KillDisconnected=.*/KillDisconnected=false/' /etc/xrdp/sesman.ini
  sed -i 's/^tcp_keepalive=.*/tcp_keepalive=true/' /etc/xrdp/xrdp.ini

  # The RDP client defaults to the first session, [Xorg]. This image has no
  # /usr/lib/xorg/Xorg, which is the blue screen. Point that slot at Xvnc.
  sed -i \
    -e 's/^lib=libxup.so/lib=libvnc.so/' \
    -e '/^code=20/d' \
    /etc/xrdp/xrdp.ini

  echo "${USER_NAME}:${PASSWORD}" | chpasswd
  short_tcp_keepalive || true
  ensure_tls_readable

  mkdir -p /var/run/xrdp /run/xrdp/sockdir
  # Do not restart sesman while an xrdp desktop (:10+) exists — that forgets
  # :10 and the next login creates :11 with Isaac still on the old display.
  # VNC on :1 is a different server; do not treat it as an xrdp session.
  if ! sesman_ipv4_ok; then
    echo "sesman not on 127.0.0.1:${SESMAN_PORT} — restarting (IPv6-only bind)"
    pkill -x xrdp-sesman 2>/dev/null || true
    sleep 0.2
    /usr/sbin/xrdp-sesman
  elif xrdp_desktop_running; then
    echo "sesman already has a desktop — leaving it running"
  elif systemd_live; then
    systemctl enable --now xrdp
    systemctl start xrdp
  else
    pgrep -x xrdp-sesman >/dev/null || /usr/sbin/xrdp-sesman
  fi
  pgrep -x xrdp >/dev/null || /usr/sbin/xrdp
  # xrdp takes ~2 s to bind after forking.
  for _ in $(seq 1 20); do is_up && break; sleep 0.5; done

  if ! sesman_ipv4_ok; then
    echo "error: sesman still not reachable on 127.0.0.1:${SESMAN_PORT}" >&2
    ss -lntp | grep 3350 >&2 || true
    exit 1
  fi

  print_connect_hint
  ;;

reconnect)
  short_tcp_keepalive || true
  kill_extra_desktops
  disconnect_stale_clients
  echo "kept Xvnc :10 if it was running; extra desktops dropped"
  echo "connect Remmina once to 127.0.0.1:${LOCAL_PORT}  (${USER_NAME} / ${PASSWORD})"
  ;;

stop)
  if systemd_live; then
    systemctl stop xrdp xrdp-sesman 2>/dev/null || true
  fi
  pkill -x xrdp 2>/dev/null || true
  pkill -x xrdp-sesman 2>/dev/null || true
  echo "stopped xrdp"
  ;;

status)
  if is_up; then
    echo "up  127.0.0.1:${PORT}"
  else
    echo "down  xrdp :${PORT}"
    exit 1
  fi
  if sesman_ipv4_ok; then
    echo "up  sesman 127.0.0.1:${SESMAN_PORT}"
  else
    echo "down sesman 127.0.0.1:${SESMAN_PORT} (login will fail)"
    ss -lnt | grep 3350 || true
    exit 1
  fi
  ps -eo args= | grep -E '[X]vnc :' || true
  ;;

*)
  echo "usage: $0 [start|stop|status|reconnect]" >&2
  exit 2
  ;;
esac
