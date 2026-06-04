#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"
ROS2_SETUP="/opt/ros/humble/setup.bash"

echo "[INFO] Building camera_bridge (ROS2)"

if [[ -f "${ROS2_SETUP}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${ROS2_SETUP}"
  set -u
else
  echo "[ERROR] ROS2 setup file not found: ${ROS2_SETUP}"
  exit 1
fi

cd "${WS_DIR}"
colcon build --packages-select camera_bridge

echo "[INFO] Build finished"
echo "[INFO] To use the package in a new shell:"
echo "       source ${ROS2_SETUP}"
echo "       source ${WS_DIR}/install/setup.bash"
