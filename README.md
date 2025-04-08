This is the branch for ROS2; use the [ros1](https://github.com/isri-aist/BaselineFootstepPlanner/tree/ros1) branch for ROS1.

# [BaselineFootstepPlanner](https://github.com/isri-aist/BaselineFootstepPlanner)
Humanoid footstep planner based on baseline methods with graph search

[![CI](https://github.com/isri-aist/BaselineFootstepPlanner/actions/workflows/ci-standalone.yaml/badge.svg)](https://github.com/isri-aist/BaselineFootstepPlanner/actions/workflows/ci-standalone.yaml)
[![CI](https://github.com/isri-aist/BaselineFootstepPlanner/actions/workflows/ci-colcon.yaml/badge.svg)](https://github.com/isri-aist/BaselineFootstepPlanner/actions/workflows/ci-colcon.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/BaselineFootstepPlanner/)

## Install

### Requirements
- Compiler supporting C++17
- Tested on `Ubuntu 22.04 / ROS Humble`

### Dependencies
This package depends on
- [SBPL](https://github.com/sbpl/sbpl)

### Installation procedure
It is assumed that ROS is installed.

1. Setup colcon workspace.
```bash
$ mkdir -p ~/ros/ws_bfp/src
$ cd ~/ros/ws_bfp
$ wstool init src
$ wstool set -t src isri-aist/BaselineFootstepPlanner git@github.com:isri-aist/BaselineFootstepPlanner.git --git -y
$ wstool update -t src
```

2. Install dependent packages.
```bash
$ source /opt/ros/${ROS_DISTRO}/setup.bash
$ rosdep install -y -r --from-paths src --ignore-src
```

3. Build a package.
```bash
$ colcon build --packages-select baseline_footstep_planner --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAMENT_CMAKE_UNINSTALL_TARGET=OFF -DUSE_ROS2=ON 
$ colcon test --packages-select baseline_footstep_planner
$ colcon test-result --all --verbose
```

## Examples

### Interactive planning
```
$ ros2 launch baseline_footstep_planner footstep_planner.launch.py
```
https://user-images.githubusercontent.com/6636600/187672008-fb93fb0e-5ec0-4054-a31d-68ce6c884005.mp4

### Standalone planning
```
$ ros2 run baseline_footstep_planner TestFootstepPlanner
```

## Integration into controller
The footstep planner in this library is available in the humanoid controller [BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController).
