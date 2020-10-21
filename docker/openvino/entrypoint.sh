#!/bin/bash

echo "entrypoint"

# setup ros environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /root/catkin_ws/devel/setup.bash
# source /opt/intel/computer_vision_sdk/bin/setupvars.sh && echo "Succedded"
export LD_LIBRARY_PATH=/root/catkin_ws/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/opt/intel//computer_vision_sdk_2018.5.455/opencv/lib:/opt/intel/opencl:/opt/intel//computer_vision_sdk_2018.5.455/deployment_tools/inference_engine/external/hddl/lib:/opt/intel//computer_vision_sdk_2018.5.455/deployment_tools/inference_engine/external/gna/lib:/opt/intel//computer_vision_sdk_2018.5.455/deployment_tools/inference_engine/external/mkltiny_lnx/lib:/opt/intel//computer_vision_sdk_2018.5.455/deployment_tools/inference_engine/external/omp/lib:/opt/intel//computer_vision_sdk_2018.5.455/deployment_tools/inference_engine/lib/ubuntu_16.04/intel64:/opt/intel//computer_vision_sdk_2018.5.455/deployment_tools/model_optimizer/bin:/opt/intel//computer_vision_sdk_2018.5.455/openvx/lib:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib


exec $@
