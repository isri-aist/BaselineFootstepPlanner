# [BaselineFootstepPlanner](https://github.com/isri-aist/BaselineFootstepPlanner)
Humanoid footstep planner based on baseline methods with graph search

[![CI](https://github.com/isri-aist/BaselineFootstepPlanner/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/BaselineFootstepPlanner/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/BaselineFootstepPlanner/)

## Install

### Requirements
- Compiler supporting C++17
- Tested on `Ubuntu 20.04 / ROS Noetic` and `Ubuntu 18.04 / ROS Melodic`

### Dependencies
This package depends on
- [SBPL](https://github.com/sbpl/sbpl) (automatically installed by `rosdep install`)

### Installation procedure
It is assumed that ROS is installed.

1. Setup catkin workspace.
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
$ catkin build baseline_footstep_planner -DCMAKE_BUILD_TYPE=RelWithDebInfo --catkin-make-args all tests
```

## Examples
Make sure that it is built with `--catkin-make-args tests` option.

### Interactive planning
```
$ roslaunch baseline_footstep_planner footstep_planner.launch
```

### Standalone planning
```
$ rosrun baseline_footstep_planner TestFootstepPlanner
```
