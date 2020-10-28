#!/usr/bin/env bash
set -e

# setup ros environment
. /opt/ros/$ROS_DISTRO/setup.bash

CAMERA_NAME="${1:-camera}"
SERIAL_NO="${2}"

# Inverse
X=${3:-0}
Y=${4:-0}
Z=${5:-0}
YAW=${6:-0}
PITCH=${7:-0}
ROLL=${8:-0}

X_INVERSE=$(bc <<< "-${3:-0}")
Y_INVERSE=$(bc <<< "-${4:-0}")
Z_INVERSE=$(bc <<< "-${5:-0}")
YAW_INVERSE=$(bc <<< "-${6:-0}")
PITCH_INVERSE=$(bc <<< "-${7:-0}")
ROLL_INVERSE=$(bc <<< "-${8:-0}")

rosrun tf2_ros static_transform_publisher ${X_INVERSE} ${Y_INVERSE} ${Z_INVERSE} ${YAW_INVERSE} ${PITCH_INVERSE} ${ROLL_INVERSE} ${CAMERA_NAME}_pose_frame base_footprint &
rosrun tf2_ros static_transform_publisher ${X} ${Y} ${Z} ${YAW} ${PITCH} ${ROLL} odom t265_odom_frame &
rosrun topic_tools relay /t265/odom/sample /odom &

exec roslaunch realsense2_camera rs_t265_pengo.launch enable_fisheye1:=true enable_fisheye2:=true camera:=${CAMERA_NAME} serial_no:=${SERIAL_NO} publish_odom_tf:=true
