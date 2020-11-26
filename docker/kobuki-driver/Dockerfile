FROM ros:melodic

COPY ./kobuki /catkin_ws/src/kobuki
WORKDIR /catkin_ws

RUN apt update && apt install -y ros-${ROS_DISTRO}-robot-state-publisher && rosdep update && rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY ./turtlebot_description /catkin_ws/src/turtlebot_description

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch kobuki_launch kobuki.launch serialport:=/dev/ttyUSB0 base:=kobuki kobuki_publish_tf:=true
 

