FROM cogniteam/realsense2
RUN apt update && apt install -y ros-${ROS_DISTRO}-tf-conversions ros-$ROS_DISTRO-dual-quaternions-ros bc && rm /var/lib/apt/lists/* -rf
COPY ./sensor_odom_manager /catkin_ws/src/sensor_odom_manager
RUN cd /catkin_ws && \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        catkin_make
COPY ./rs_camera.sh /
RUN chmod +x /rs_camera.sh
COPY ./rs_t265_pengo.launch /opt/ros/${ROS_DISTRO}/share/realsense2_camera/launch/rs_t265_pengo.launch
CMD /rs_camera.sh t265 0 0 0 0 0 0 0