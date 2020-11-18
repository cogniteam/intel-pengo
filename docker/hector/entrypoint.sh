#!/usr/bin/env bash
set -e

# setup ros environment
. /opt/ros/$ROS_DISTRO/setup.bash

MAP_RESOLUTION=${1:-0.05}

exec roslaunch hector_mapping.launch map_resolution:=${MAP_RESOLUTION}
