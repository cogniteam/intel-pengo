#!/usr/bin/env bash
set -e

# setup ros environment
. /opt/ros/$ROS_DISTRO/setup.bash

CMD_VEL=${1:-cmd_vel}

# Twist from keyboard
exec rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=${CMD_VEL}
