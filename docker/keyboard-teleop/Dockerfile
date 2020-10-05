FROM ros:melodic

RUN apt update && apt install ros-melodic-teleop-twist-keyboard -y && rm /var/lib/apt/lists/* -rf

WORKDIR /
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x ./entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
