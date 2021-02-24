FROM cogniteam/realsense2-rtabmap

LABEL maintainer="nedim.hadzic@intel.com"
LABEL build_date="2021-20-16"
LABEL description="Realsense Multi cmaera RTABMAP"

SHELL ["/bin/bash", "-xo", "pipefail", "-c"]

RUN apt remove ros-$ROS_DISTRO-rtabmap-ros -y \
    && apt remove ros-$ROS_DISTRO-rtabmap -y


RUN cd ~                                                         \
    && git clone https://github.com/introlab/rtabmap.git rtabmap \
    && cd rtabmap/build                                          \
    && cmake ..                                                  \
    && make -j4                                                  \
    && sudo make install

RUN mkdir ~/catkin_ws                                                        \
    && cd ~/catkin_ws                                                        \
    && git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
    && source /opt/ros/$ROS_DISTRO/setup.bash                                \
    && catkin_make -j4 -DRTABMAP_SYNC_MULTI_RGBD=ON

ADD multi_camera_rtabmap.launch /root/catkin_ws/src/rtabmap_ros/launch/

