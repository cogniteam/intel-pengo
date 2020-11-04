#!/bin/sh
. /rover_ws/devel/setup.sh
rosrun message_to_tf message_to_tf /rr_openrover_driver/odom_encoder &
exec roslaunch rr_openrover_driver starterkit_bringup.launch 
