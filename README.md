# Build Guide

This workspace is split into a pure-CMake “core” plus two wrappers:

- ROS1 (catkin): `ros1/src/vins_estimator_ros1`
- ROS2 (ament): `ros2/src/vins_estimator_ros2`

Unless otherwise noted, run the commands below from the repository root.

## Quick Start Scripts

From the repository root:

```bash
bash scripts/build_ros1.sh  # builds core + ROS1 workspace
bash scripts/build_ros2.sh  # builds core + ROS2 workspace
```

Set `ROS1_SETUP` or `ROS2_SETUP` environment variables before running the scripts if your ROS installations live elsewhere (defaults: `/opt/ros/noetic` and `/opt/ros/humble`). Both scripts accept an optional `BUILD_TYPE` (default `Release`).

## 1. Build & Install the Core Libraries (manual)

```bash
cd /path/to/cloned/repo
cmake -S core -B core/build -DCMAKE_BUILD_TYPE=Release
cmake --build core/build -j$(nproc)
cmake --install core/build --prefix core/install
```

This installs the static libraries (`camera_models`, `vins_estimator`) plus their CMake package files under `core/install/`.

## 2. Build the ROS1 Wrapper (catkin)

```bash
cd ros1
source /opt/ros/noetic/setup.bash
export CMAKE_PREFIX_PATH="$(pwd)/../core/install:${CMAKE_PREFIX_PATH:-}"
catkin_make --pkg vins_estimator_ros1
```

If you previously built in another workspace layout, remove any existing `build/` and `devel/` directories under `ros1/` first.

The command above whitelists the ROS¹ package (`vins_estimator_ros1`) so that the ROS² sources in `src/vins_estimator_ros2` are ignored by catkin. The resulting binaries (node, nodelet, etc.) link against the core library installed in step 1.

## 3. Build the ROS2 Wrapper (ament)

ROS² sources live under `ros2/src`. Build them with `colcon`, pointing to the same core installation:

```bash
# Ensure catkin_pkg is available (ament parses package.xml via catkin_pkg)
python3 -m pip install --user catkin_pkg  # or: sudo apt install python3-catkin-pkg

cd ros2
source /opt/ros/humble/setup.bash  # or your ROS2 distro
colcon build \
  --packages-select vins_estimator_ros2 \
  --build-base build_ros2 \
  --install-base install_ros2 \
  --cmake-args -DCMAKE_PREFIX_PATH="$(pwd)/../core/install"
```

```
colcon build --base-paths src --packages-select vins_estimator_ros2 --build-base build_ros2 --install-base install_ros2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_PREFIX_PATH="$(pwd)/../core/install"  -DCMAKE_CXX_FLAGS="-O0 -g3"

```

Launch example:

```bash
source ros2/install_ros2/setup.bash  # from the colcon build above
ros2 launch vins_estimator_ros2 vins_estimator.launch.py
```

## 4. Use the Catkin Workspace

```bash
source ros1/devel/setup.bash
# Launch/rosrun as needed, e.g.
roslaunch vins_estimator_ros1 multi_l515_d435.launch
```

## Notes

- Any time the core sources change, repeat **Step 1** before rebuilding the ROS workspace.
- For ROS¹ builds, keep using `catkin_make --pkg vins_estimator_ros1` (or set `CATKIN_WHITELIST_PACKAGES=vins_estimator_ros1`) so that the ROS² package is ignored by catkin.
- The ROS2 build expects the same `core/install` path; adjust the `--cmake-args -DCMAKE_PREFIX_PATH=...` flag if it changes.
- The example ROS² build places artefacts in `ros2/build_ros2/`, `ros2/install_ros2/`, and `ros2/log_ros2/` to avoid clashing with catkin’s `build/` and `devel/` directories; tweak as needed.
- When switching between ROS¹ (catkin) and ROS² (colcon) builds, clean the corresponding output directories if you see stale artefacts (`rm -rf ros1/build ros1/devel` for catkin, `rm -rf ros2/build_ros2 ros2/install_ros2 ros2/log_ros2` for colcon).
