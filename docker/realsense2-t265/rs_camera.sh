#!/usr/bin/env bash
set -e

# setup ros environment
. /opt/ros/$ROS_DISTRO/setup.bash

CAMERA_NAME="${1:-camera}"
SERIAL_NO="${2}"

X=${3:-0}
Y=${4:-0}
Z=${5:-0}
YAW=${6:-0}
PITCH=${7:-0}
ROLL=${8:-0}

rosrun tf2_ros static_transform_publisher ${X} ${Y} ${Z} ${YAW} ${PITCH} ${ROLL} base_link ${CAMERA_NAME}_link &
rosrun tf2_ros static_transform_publisher ${X} ${Y} ${Z} ${YAW} ${PITCH} ${ROLL} odom ${CAMERA_NAME}_odom_frame &

exec roslaunch realsense2_camera rs_t265.launch enable_fisheye1:=true enable_fisheye2:=true camera:=${CAMERA_NAME} serial_no:=${SERIAL_NO} publish_odom_tf:=false
