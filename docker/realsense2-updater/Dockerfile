FROM intelpengo/realsense2
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
        echo "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" > /etc/apt/sources.list.d/realsense-sdk.list && \
        apt -q -qq update && DEBIAN_FRONTEND=noninteractive apt -y install wget unzip librealsense2-utils ros-${ROS_DISTRO}-realsense2-camera  && rm -rf /var/lib/apt/lists/* && \
        rm -rf /var/lib/apt/lists/*

WORKDIR /

RUN wget https://www.intelrealsense.com/wp-content/uploads/2020/08/D400_Series_Production_FW_5_12_7_100.zip && \
        unzip D400_Series_Production_FW_5_12_7_100.zip

#CMD rs-fw-update -f Signed_Image_UVC_5_12_7_100.bin

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]       


