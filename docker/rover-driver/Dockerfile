FROM ros:melodic

RUN apt-get update && apt install ros-melodic-tf2-geometry-msgs ros-melodic-robot-localization ros-melodic-twist-mux -y && rm /var/lib/apt/lists/* -rf

RUN mkdir -p /rover_ws/src/ && cd /rover_ws/src/ && \
        git clone https://github.com/RoverRobotics/rr_openrover_stack.git && \
        cd .. && \
        . /opt/ros/melodic/setup.sh && catkin_make


WORKDIR /
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x ./entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
