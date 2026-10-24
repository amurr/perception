#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "error: ROS 2 Humble requires Ubuntu 22.04 (Jammy)." >&2
  echo "       Detected: ${PRETTY_NAME:-unknown}" >&2
  echo "       Use an Ubuntu 22.04 environment, or choose the ROS release matching this OS." >&2
  exit 1
fi

# Remove the APT cache override used by the previous workspace-persistent script.
$SUDO rm -f /etc/apt/apt.conf.d/01workspace-persist

echo "Installing prerequisites..."
$SUDO apt update
$SUDO apt install -y \
  locales \
  software-properties-common \
  curl \
  gnupg \
  lsb-release \
  ufw

$SUDO locale-gen en_US en_US.UTF-8
$SUDO update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8

echo "Enabling the Ubuntu Universe repository..."
$SUDO add-apt-repository universe -y

echo "Adding the ROS 2 APT repository..."
$SUDO curl -fsSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
  | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "Installing ROS 2 Humble and dependencies..."
$SUDO apt update
$SUDO apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  ros-humble-foxglove-bridge \
  ros-humble-vision-msgs \
  ros-humble-ackermann-msgs \
  ros-humble-control-msgs \
  ros-humble-realtime-tools \
  ros-humble-xacro \
  ros-humble-tf2-tools \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rmw-fastrtps-cpp

BASHRC="${HOME}/.bashrc"
ROS_SOURCE="source /opt/ros/humble/setup.bash"
if ! grep -Fq "$ROS_SOURCE" "$BASHRC"; then
  printf '\n%s\n' "$ROS_SOURCE" >> "$BASHRC"
fi

echo "ROS 2 Humble, Foxglove Bridge, and UFW are installed."
echo "Run: source ~/.bashrc"
