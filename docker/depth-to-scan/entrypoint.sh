#!/usr/bin/env bash
set -e

# setup ros environment
. /opt/ros/$ROS_DISTRO/setup.bash

CAMERA_NAME=${1:-front}
X=${2:-0}
Y=${3:-0}
Z=${4:-0}
YAW=${5:-0}
PITCH=${6:-0}
ROLL=${7:-0}

rosrun tf2_ros static_transform_publisher ${X} ${Y} ${Z} ${YAW} ${PITCH} ${ROLL} base_link laser &

exec roslaunch d2s.launch camera_name:=${CAMERA_NAME}
