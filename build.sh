#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-}"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"

if [[ -z "${MODE}" ]]; then
  echo "Usage: ./build.sh ROS1|ROS2"
  exit 1
fi

MODE_UPPER="$(echo "${MODE}" | tr '[:lower:]' '[:upper:]')"

case "${MODE_UPPER}" in
  ROS1)
    echo "[INFO] Build mode: ROS1"
    if [[ -f "${WS_DIR}/src/CMakeLists.txt" ]] && command -v catkin_make >/dev/null 2>&1; then
      echo "[INFO] Using catkin_make in workspace: ${WS_DIR}"
      cd "${WS_DIR}"
      catkin_make --pkg camera_bridge --cmake-args -DROS_VERSION=ROS1
    else
      echo "[WARN] catkin workspace or catkin_make not found, fallback to standalone CMake"
      cd "${PKG_DIR}"
      cmake -S . -B build_ros1 -DROS_VERSION=ROS1 -DCMAKE_BUILD_TYPE=Release
      cmake --build build_ros1 -j"$(nproc)"
    fi
    ;;

  ROS2)
    echo "[INFO] Build mode: ROS2"
    cd "${PKG_DIR}"
    cmake -S . -B build_ros2 -DROS_VERSION=ROS2 -DCMAKE_BUILD_TYPE=Release
    cmake --build build_ros2 -j"$(nproc)"
    ;;

  *)
    echo "[ERROR] Invalid mode: ${MODE}"
    echo "Usage: ./build.sh ROS1|ROS2"
    exit 1
    ;;
esac


echo "[INFO] Build finished: ${MODE_UPPER}"