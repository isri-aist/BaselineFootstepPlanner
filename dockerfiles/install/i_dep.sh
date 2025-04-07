#!/bin/sh
set -e

apt-get update
apt-mark hold ros-*
apt-get upgrade -y

basic_dep="git \
           curl \
           nano \
           vim \
           python3-pip \
           python3-colcon-common-extensions \
           wget \
           ninja-build \
           x11-apps"

apt install -y python3-colcon-clean doxygen
apt-get install -y $basic_dep
apt-get update
apt-get upgrade -y

mkdir /root/catkin_ws
cd /root/catkin_ws

# Clean up
apt-get autoremove -y
apt-get clean -y