
![Docker Image CI](https://github.com/blackpc/pengo-docker/workflows/Docker%20Image%20CI/badge.svg)


### OpenVINO docker image

Add docker permission to access your local xserver:
```
xhost +local:docker
```

Run container with GUI
```
docker run --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --privileged -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -it intelpengo/openvino
```

People OSS launch
```
docker run --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --privileged -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -it intelpengo/openvino /bin/bash -ic 'roslaunch vino_launch pipeline_people_oss.launch'
```