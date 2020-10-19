#!/usr/bin/env bash
set -e

# setup ros environment
. /opt/ros/$ROS_DISTRO/setup.bash
. /joy_ws/devel/setup.bash
X=${1:-0}
Y=${2:-0}
Z=${3:-0}
YAW=${4:-0}
PITCH=${5:-0}
ROLL=${6:-0}

# Twist from keyboard
roslaunch teleop_twist_joy teleop.launch