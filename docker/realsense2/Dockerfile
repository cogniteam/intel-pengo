ARG ROS_DISTRO=melodic
FROM ros:${ROS_DISTRO}
RUN apt -q -qq update && DEBIAN_FRONTEND=noninteractive apt -y install ros-${ROS_DISTRO}-realsense2-camera  && rm -rf /var/lib/apt/lists/*
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]