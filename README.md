
![docker-build:openvino](https://github.com/blackpc/pengo-docker/workflows/docker-build:openvino/badge.svg) 
![docker-build:realsense2](https://github.com/blackpc/pengo-docker/workflows/docker-build:realsense2/badge.svg)

### OpenVINO docker image

Add docker permission to access your local xserver:
```
xhost +local:docker
```

Run container with GUI
```
docker run --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --privileged -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -it cogniteam/openvino
```

People OSS launch
```
docker run --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --privileged -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -it cogniteam/openvino /bin/bash -ic 'roslaunch vino_launch pipeline_people_oss.launch'
```