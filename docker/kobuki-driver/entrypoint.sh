#!/bin/sh

TURTLEBOT_SERIAL_PORT=${1:-/dev/ttyUSB0}
. /catkin_ws/devel/setup.sh
roslaunch kobuki_launch kobuki.launch serialport:=${TURTLEBOT_SERIAL_PORT} base:=kobuki