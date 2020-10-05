#!/usr/bin/env bash
set -e

# setup ros environment
. /opt/ros/$ROS_DISTRO/setup.bash

X=${1:-0}
Y=${2:-0}
Z=${3:-0}
YAW=${4:-0}
PITCH=${5:-0}
ROLL=${6:-0}

# Twist from keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
