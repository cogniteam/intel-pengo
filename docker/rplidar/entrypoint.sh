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

rosrun tf2_ros static_transform_publisher ${X} ${Y} ${Z} ${YAW} ${PITCH} ${ROLL} base_link laser &

roslaunch rplidar_ros rplidar.launch 