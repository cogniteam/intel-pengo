FROM cogniteam/realsense2
RUN apt update && apt install -y bc && rm /var/lib/apt/lists/* -rf
COPY ./rs_camera.sh /
RUN chmod +x /rs_camera.sh
COPY ./rs_t265_pengo.launch /opt/ros/${ROS_DISTRO}/share/realsense2_camera/launch/rs_t265_pengo.launch
CMD /rs_camera.sh t265 0 0 0 0 0 0 0