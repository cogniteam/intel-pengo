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
SERIAL_PORT=${7:-/dev/ttyUSB0}
FRAME_ID=${8:-laser}

# adding transform publisher from the base_link to the choosen frame for the lidar
rosrun tf2_ros static_transform_publisher ${X} ${Y} ${Z} ${YAW} ${PITCH} ${ROLL} base_link ${FRAME_ID} &

#executing the custom rplidar launch modified to receive arguments
exec roslaunch rplidar.launch serial_port:=${SERIAL_PORT} frame_id:=${FRAME_ID}