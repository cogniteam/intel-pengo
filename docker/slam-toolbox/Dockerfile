FROM ros:melodic

RUN apt update && apt install ros-melodic-slam-toolbox -y && rm /var/lib/apt/lists/* -rf

CMD roslaunch slam_toolbox online_sync.launch

