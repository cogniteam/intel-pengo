ARG ROS_DISTRO=melodic
FROM ros:${ROS_DISTRO}
RUN apt -q -qq update && DEBIAN_FRONTEND=noninteractive apt -y install ros-${ROS_DISTRO}-realsense2-camera ros-${ROS_DISTRO}-image-transport-plugins && rm -rf /var/lib/apt/lists/*
COPY ./rs_camera.sh /
RUN chmod +x /rs_camera.sh

CMD /rs_camera.sh cam1 0 0 0 0 0 0 0