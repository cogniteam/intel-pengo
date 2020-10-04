
![docker-build:slam-toolbox](https://github.com/cogniteam/intel-pengo/workflows/docker-build:slam-toolbox/badge.svg)
![docker-build:rplidar](https://github.com/cogniteam/intel-pengo/workflows/docker-build:rplidar/badge.svg)
![docker-build:realsense2](https://github.com/cogniteam/intel-pengo/workflows/docker-build:realsense2/badge.svg)
![docker-build:person-follower](https://github.com/cogniteam/intel-pengo/workflows/docker-build:person-follower/badge.svg)
![docker-build:openvino](https://github.com/cogniteam/intel-pengo/workflows/docker-build:openvino/badge.svg)
![docker-build:kobuki](https://github.com/cogniteam/intel-pengo/workflows/docker-build:kobuki/badge.svg)


## Pengo platform docker images

### Realsense cameras:

Front D435 camera:
```
docker run -e ROS_MASTER_URI -e ROS_IP --privileged --net=host --rm -it cogniteam/realsense2 /rs_camera.sh cam1 939622072996 0.17 0 0.27 0 0 0
```

Left D435 camera:
```
docker run -e ROS_MASTER_URI -e ROS_IP --privileged --net=host --rm -it cogniteam/realsense2 /rs_camera.sh cam2 947122070333 0 0.17 0.27 -4.7123889803 0 0
```

Rear D435 camera:
```
docker run -e ROS_MASTER_URI -e ROS_IP --privileged --net=host --rm -it cogniteam/realsense2 /rs_camera.sh cam3 944622073450 -0.17 0 0.27 -3.14159265 0 0
```

Right D435 camera:
```
docker run -e ROS_MASTER_URI -e ROS_IP --privileged --net=host --rm -it cogniteam/realsense2 /rs_camera.sh cam4 944622074845 0 -0.17 0.27 -1.570796326 0 0
```

Front T265 camera:
```
docker run -v /dev:/dev -e ROS_MASTER_URI -e ROS_IP --privileged --net=host --rm -it cogniteam/realsense2-t265 cam5 11622110757 0.17 0 0.27 0 0 0
```

Other serials:
 - 11622110838

### OpenVINO Detection

ROS OpenVINo MobileNet SSD detection on Myriad:
```
docker run -e ROS_MASTER_URI -e ROS_IP --net=host --privileged --rm -v /dev:/dev -it cogniteam/openvino bash -ic "roslaunch vino_launch pengo_detection.launch myriad:=true camera_name:=cam1"
```

### Kobuki driver
```
docker run -e ROS_MASTER_URI -e ROS_IP --net=host --privileged --rm -it cogniteam/kobuki-driver:latest /dev/ttyUSB1
```

### RPlidar driver
```
docker run -e ROS_MASTER_URI -e ROS_IP --rm -it --net=host --privileged cogniteam/rplidar 0 0 0.22 -1.5707 0 3.1415
```

### Navigation 
```
docker run -e ROS_MASTER_URI -e ROS_IP --net=host -it cogniteam/kobuki-navigation:latest
```

### VNC setup on pengo

Install tigervnc and xfce4 desktop:
```
sudo apt update
sudo apt install xfce4 xfce4-goodies tigervnc-standalone-server -t

# Ubuntu 16.04 Only 
# Install tigervnc using tar
# Download https://bintray.com/tigervnc/stable/download_file?file_path=tigervnc-1.11.0.x86_64.tar.gz
# Unpack contents to /

vnc4server # enter password
sudo ufw allow from any to any port 5901 proto tcp
```

Edit `~/.vnc/config`:
```
geometry=1920x1080
dpi=96
session=xfce
alwaysshared
```

Edit `/etc/tigervnc/vncserver.users` and map display to user:
```
:1=hamster
```

Enable and run systemd service:
```
sudo systemctl daemon-reload
sudo systemctl enable vncserver@:1.service
sudo systemctl start vncserver@:1.service
```