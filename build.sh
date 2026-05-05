#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-}"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"
ROS1_SETUP="/opt/ros/noetic/setup.bash"
ROS2_SETUP="/opt/ros/humble/setup.bash"

if [[ -z "${MODE}" ]]; then
  echo "Usage: ./build.sh ROS1|ROS2"
  exit 1
fi

MODE_UPPER="$(echo "${MODE}" | tr '[:lower:]' '[:upper:]')"

case "${MODE_UPPER}" in
  ROS1)
    echo "[INFO] Build mode: ROS1"
    if [[ -f "${ROS1_SETUP}" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "${ROS1_SETUP}"
      set -u
    else
      echo "[ERROR] ROS1 setup file not found: ${ROS1_SETUP}"
      exit 1
    fi

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
    colcon build --packages-select camera_bridge --cmake-args -DROS_VERSION=ROS2
    ;;

  *)
    echo "[ERROR] Invalid mode: ${MODE}"
    echo "Usage: ./build.sh ROS1|ROS2"
    exit 1
    ;;
esac

echo "[INFO] Build finished: ${MODE_UPPER}"

if [[ "${MODE_UPPER}" == "ROS2" ]]; then
  echo "[INFO] To use the package in a new shell:"
  echo "       source ${ROS2_SETUP}"
  echo "       source ${WS_DIR}/install/setup.bash"
elif [[ "${MODE_UPPER}" == "ROS1" ]]; then
  echo "[INFO] To use the package in a new shell:"
  echo "       source ${ROS1_SETUP}"
  echo "       source ${WS_DIR}/devel/setup.bash"
fi
