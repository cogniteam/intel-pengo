FROM ros:melodic

RUN apt update && apt install ros-melodic-hector* -y && rm /var/lib/apt/lists/* -rf


WORKDIR /

COPY hector_mapping.launch /hector_mapping.launch
COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]



