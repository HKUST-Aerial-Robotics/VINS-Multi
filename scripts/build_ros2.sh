#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ROS2_SETUP}" ]]; then
  echo "[build_ros2] ROS2 setup file not found: ${ROS2_SETUP}" >&2
  echo "Set ROS2_SETUP to your desired ROS2 setup.bash path." >&2
  exit 1
fi

source "${ROS2_SETUP}"

CORE_DIR="${REPO_ROOT}/core"
ROS2_WS="${REPO_ROOT}/ros2"

BUILD_TYPE="${BUILD_TYPE:-Release}"

echo "[build_ros2] Building core library (${BUILD_TYPE})"
cmake -S "${CORE_DIR}" -B "${CORE_DIR}/build" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build "${CORE_DIR}/build" -- -j"${NPROC:-$(nproc)}"
cmake --install "${CORE_DIR}/build" --prefix "${CORE_DIR}/install"

pushd "${ROS2_WS}" > /dev/null

echo "[build_ros2] Running colcon build"
colcon build \
  --base-paths src \
  --packages-select vins_estimator_ros2 \
  --build-base build_ros2 \
  --install-base install_ros2 \
  --log-base log_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DCMAKE_PREFIX_PATH="${CORE_DIR}/install"

popd > /dev/null

echo "[build_ros2] Completed successfully"
