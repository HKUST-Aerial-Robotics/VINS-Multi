#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS1_SETUP="${ROS1_SETUP:-/opt/ros/noetic/setup.bash}"
if [[ ! -f "${ROS1_SETUP}" ]]; then
  echo "[build_ros1] ROS1 setup file not found: ${ROS1_SETUP}" >&2
  echo "Set ROS1_SETUP to your desired ROS1 setup.bash path." >&2
  exit 1
fi

source "${ROS1_SETUP}"

CORE_DIR="${REPO_ROOT}/core"
ROS1_WS="${REPO_ROOT}/ros1"

BUILD_TYPE="${BUILD_TYPE:-Release}"

echo "[build_ros1] Building core library (${BUILD_TYPE})"
cmake -S "${CORE_DIR}" -B "${CORE_DIR}/build" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build "${CORE_DIR}/build" -- -j"${NPROC:-$(nproc)}"
cmake --install "${CORE_DIR}/build" --prefix "${CORE_DIR}/install"

export CMAKE_PREFIX_PATH="${CORE_DIR}/install:${CMAKE_PREFIX_PATH:-}"

echo "[build_ros1] Building catkin workspace"
catkin_make -C "${ROS1_WS}" --pkg vins_estimator_ros1

echo "[build_ros1] Completed successfully"
