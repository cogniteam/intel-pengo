FROM cogniteam/realsense2

RUN apt update && apt install -y ros-${ROS_DISTRO}-imu-filter-madgwick ros-${ROS_DISTRO}-rtabmap-ros ros-${ROS_DISTRO}-robot-localization && rm /var/lib/apt/lists/* -rf
COPY ./rs_camera.sh /
RUN chmod +x /rs_camera.sh
CMD /rs_camera.sh cam1 0 0 0 0 0 0 0