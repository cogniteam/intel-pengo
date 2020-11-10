FROM ros:melodic

RUN apt-get update && apt install ros-melodic-message-to-tf ros-melodic-rr-openrover-driver ros-melodic-rr-control-input-manager -y && rm /var/lib/apt/lists/* -rf

COPY starterkit_bringup.launch /opt/ros/${ROS_DISTRO}/share/rr_openrover_driver/launch/starterkit_bringup.launch

CMD roslaunch rr_openrover_driver starterkit_bringup.launch
