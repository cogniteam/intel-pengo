FROM ros:melodic

RUN apt-get update && apt install ros-melodic-depthimage-to-laserscan -y && rm /var/lib/apt/lists/* -rf

WORKDIR /

COPY entrypoint.sh /entrypoint.sh
COPY d2s.launch /d2s.launch

RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

