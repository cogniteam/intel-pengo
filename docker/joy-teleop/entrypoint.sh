#!/usr/bin/env bash
set -e

# setup ros environment
. /opt/ros/$ROS_DISTRO/setup.bash
. /joy_ws/devel/setup.bash

# Twist from keyboard
exec roslaunch teleop_twist_joy teleop.launch
