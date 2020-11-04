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

exec roslaunch realsense2_camera opensource_tracking.launch camera:=${CAMERA_NAME} serial_no:=${SERIAL_NO}
