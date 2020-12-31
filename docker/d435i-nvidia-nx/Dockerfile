FROM nvcr.io/nvidia/l4t-base:r32.4.2
RUN apt-get update -y
RUN apt install -y wget software-properties-common apt-utils  curl git
RUN /bin/bash  -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE ||  apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
RUN apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE ||  apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
RUN add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
RUN apt-get update -y
ENV TZ America/Los_Angeles
RUN DEBIAN_FRONTEND="noninteractive" apt-get install -y tzdata git curl cmake wget mc mlocate 
RUN DEBIAN_FRONTEND="noninteractive" apt install -y ros-melodic-desktop clang-format ros-melodic-desktop-full python-pip python3-pip  python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential 
RUN apt update -y
#RUN rm -rf /var/lib/apt/lists
RUN  apt-get install -y librealsense2-dev librealsense2-utils
RUN apt install -y ros-melodic-ddynamic-reconfigure
RUN dpkg-reconfigure --frontend noninteractive tzdata
RUN rm -rf /var/lib/apt/lists
WORKDIR /home/nvidia/ws
RUN mkdir src && cd src &&  git clone https://github.com/intel-ros/realsense.git -b development
RUN rosdep init && rosdep update


RUN /bin/bash -c '. /opt/ros/melodic/setup.bash;  cd /home/nvidia/ws; rosdep install --from-paths src --ignore-src; catkin_make'
COPY ./rs_camera.sh /
RUN chmod +x /rs_camera.sh

CMD /rs_camera.sh cam1 0 0 0 0 0 0 0