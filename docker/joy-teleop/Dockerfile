FROM ros:melodic

RUN apt-get update && apt install libusb-dev libx11-dev libspnav-dev libbluetooth-dev libcwiid-dev ros-melodic-roslint ros-melodic-joy ros-melodic-joy-teleop -y && rm /var/lib/apt/lists/* -rf

RUN mkdir -p /joy_ws/src/ && cd /joy_ws/src/ && \
        git clone https://github.com/ros-drivers/joystick_drivers.git && \
        git clone https://github.com/ros-teleop/teleop_tools.git && \
        git clone https://github.com/ros-teleop/teleop_twist_joy.git && \
        cd .. && \
        . /opt/ros/melodic/setup.sh && catkin_make


WORKDIR /
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x ./entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
