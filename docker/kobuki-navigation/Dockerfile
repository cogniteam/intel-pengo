FROM intelpengo/kobuki-driver

RUN apt update && apt install -y ros-${ROS_DISTRO}-move-base ros-${ROS_DISTRO}-dwa-local-planner && rm /var/lib/apt/lists/* -rf

COPY turtlebot_navigation /catkin_ws/src/turtlebot_navigation

WORKDIR /
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x ./entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch turtlebot_navigation move_base.launch.xml