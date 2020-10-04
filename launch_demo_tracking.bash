#!/bin/bash

set -e 

SLEEP_TIME=20

roscore &
sleep 10

#
# OpenVino Myriad
#
docker run -d -e ROS_MASTER_URI -e ROS_IP --net=host --privileged --rm -v /dev:/dev cogniteam/openvino bash -ic "roslaunch vino_launch pengo_detection.launch myriad:=true camera_name:=cam1"
sleep ${SLEEP_TIME}

#
# D435 front camera
#
docker run -d -e ROS_MASTER_URI -e ROS_IP --privileged --net=host --rm cogniteam/realsense2 /rs_camera.sh cam1 939622072996 0.17 0 0.27 0 0 0
# sleep ${SLEEP_TIME}
sleep 10

#
# T265
#
# docker run -d -v /dev:/dev -e ROS_MASTER_URI -e ROS_IP --privileged --net=host --rm cogniteam/realsense2-t265 cam5 11622110757 0.17 0 0.27 0 0 0

#
# Kobuki
#
docker run -d -e ROS_MASTER_URI -e ROS_IP --net=host --privileged --rm cogniteam/kobuki-driver:latest /dev/ttyUSB1
sleep 1

docker run -d -e ROS_MASTER_URI -e ROS_IP --net=host cogniteam/kobuki-navigation:latest
sleep 1

#
# Person follower
#

docker run -d -e ROS_MASTER_URI -e ROS_IP --net=host cogniteam/person-follower:latest
sleep 1

#
# ROS slam-toolbox
#

docker run -d -e ROS_MASTER_URI -e ROS_IP --net=host cogniteam/slam-toolbox:latest
sleep 1

#
# RP Lidar
#

docker run -d -e ROS_MASTER_URI -e ROS_IP --rm -it --net=host --privileged cogniteam/rplidar 0 0 0.22 -1.5707 0 3.1415