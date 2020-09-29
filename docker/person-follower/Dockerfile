FROM ros:melodic

COPY ./hupster /catkin_ws/src/pengo
WORKDIR /catkin_ws

RUN apt update && rosdep update && rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch hupster_detection intel.launch
