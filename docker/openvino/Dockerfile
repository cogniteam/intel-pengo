FROM ros:kinetic

#
# Set default shell to bash (following setup.sh is a bash script)
#
SHELL [ "/bin/bash", "-c" ]

#
# Base apt dependencies
#
RUN apt update && apt install sudo cpio lsb-release wget software-properties-common python3-yaml python3-requests -y

#
# Download OpenVINO 2018 R5
#
WORKDIR /
RUN wget -c http://registrationcenter-download.intel.com/akdlm/irc_nas/15078/l_openvino_toolkit_p_2018.5.455.tgz
RUN mkdir /openvino_install
RUN tar -xzf l_openvino_toolkit_p_2018.5.455.tgz && mv /l_openvino_toolkit_p_2018.5.455 /openvino_install
WORKDIR /openvino_install/l_openvino_toolkit_p_2018.5.455
COPY ./silent.cfg /openvino_install/l_openvino_toolkit_p_2018.5.455

#
# OpenVINO install
#
RUN ./install_cv_sdk_dependencies.sh && ./install.sh --silent silent.cfg

#
# Build samples
#
RUN . /opt/intel/computer_vision_sdk/bin/setupvars.sh && \
        cd /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/ && \
        mkdir build && cd build && \
        cmake .. && \
        make

RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib" >> /root/.bashrc
RUN echo "source /opt/intel/computer_vision_sdk/bin/setupvars.sh" >> /root/.bashrc

#
# OpenCV
#
WORKDIR /
RUN mkdir /opencv_build/ && cd /opencv_build && \
        apt-get install build-essential && \
        apt-get install -y cmake git python3-numpy libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev && \
        git clone https://github.com/opencv/opencv.git -b 3.4.2 --depth 1 && \
        git clone https://github.com/opencv/opencv_contrib.git -b 3.4.2 --depth 1 && \
        cd opencv && \
        mkdir build && cd build && \
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/opencv_build/opencv_contrib/modules/ .. && \
        make -j8 && \
        sudo make install



#
# DLDT
#
WORKDIR /opt/openvino_toolkit
RUN git clone https://github.com/opencv/dldt.git -b 2018_R4 --recurse-submodules
WORKDIR /opt/openvino_toolkit/dldt/inference-engine
RUN ./install_dependencies.sh
WORKDIR /opt/openvino_toolkit/dldt/inference-engine/build
RUN . /opt/intel/computer_vision_sdk/bin/setupvars.sh && \
        cmake -DCMAKE_BUILD_TYPE=Release .. && make -j8

#
# Open Model Zoo
#
COPY ./open_model_zoo /opt/openvino_toolkit/open_model_zoo
WORKDIR /opt/openvino_toolkit/open_model_zoo/demos/build
RUN source /opt/intel/computer_vision_sdk/bin/setupvars.sh && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j8

#
# 
#
ENV InferenceEngine_DIR /opt/openvino_toolkit/dldt/inference-engine/build/
ENV CPU_EXTENSION_LIB /opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libcpu_extension.so
ENV GFLAGS_LIB /opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libgflags_nothreads.a
ENV LD_LIBRARY_PATH /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib


#
# Intel realsense2 sdk
#
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo `lsb_release -cs` main" -u -y
RUN apt-get install librealsense2-dev ros-kinetic-realsense2-camera -y

#
# Catkin make
#
COPY ./catkin_ws /root/catkin_ws
WORKDIR /root/catkin_ws
RUN . /opt/ros/kinetic/setup.bash && \
        rosdep update && \
        rosdep install --from-path src --ignore-src -y && \
        catkin_make

RUN ln -s /root/catkin_ws/src/ros_openvino_toolkit /opt/openvino_toolkit/ros_openvino_toolkit

#
# Add ROS workspace setup.bash source to .bashrc
#
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

#
# Download trained models and install labels
#
WORKDIR /opt/openvino_toolkit/open_model_zoo/model_downloader

RUN python3 ./downloader.py --name mobilenet-ssd && \
        python3 downloader.py --name face-detection-adas-0001 && \
        python3 downloader.py --name age-gender-recognition-retail-0013 && \
        python3 downloader.py --name emotions-recognition-retail-0003 && \
        python3 downloader.py --name head-pose-estimation-adas-0001 && \
        python3 downloader.py --name person-detection-retail-0013 && \
        python3 downloader.py --name person-reidentification-retail-0076


#
# Install python 3.6
# 
RUN add-apt-repository ppa:deadsnakes/ppa -y && \
        apt-get update && \
        apt-get install python3.6 -y

#
# Install PIP 3.6
#
RUN curl https://bootstrap.pypa.io/get-pip.py | python3.6

#
# Install python dependencies for TF model optimizer
#
WORKDIR /opt/openvino_toolkit/dldt/model-optimizer
RUN python3.6 -m pip install -r requirements.txt
RUN python3.6 -m pip install networkx==1.11

COPY ./mask_rcnn_inception_v2_coco_2018_01_28 /opt/models/mask_rcnn_inception_v2_coco_2018_01_28
WORKDIR /opt/models/mask_rcnn_inception_v2_coco_2018_01_28

#
# Convert TF model to OpenVINO (?)
#
RUN python3.6 /opt/openvino_toolkit/dldt/model-optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/openvino_toolkit/dldt/model-optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --output_dir ./output/

#
# Optimize model
#
WORKDIR /opt/openvino_toolkit/open_model_zoo/model_downloader

# FP32 precision mode
RUN python3.6 /opt/openvino_toolkit/dldt/model-optimizer/mo.py --input_model /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
# FP16 precision mode
RUN python3.6 /opt/openvino_toolkit/dldt/model-optimizer/mo.py --input_model /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16 --data_type=FP16 --mean_values [127.5,127.5,127.5] --scale_values [127.5]


#
# Copy labels
#
RUN cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/open_model_zoo/model_downloader/Retail/object_attributes/emotions_recognition/0003/dldt && \
        cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/open_model_zoo/model_downloader/Transportation/object_detection/face/pruned_mobilenet_reduced_ssd_shared_weights/dldt && \
        cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32 && \
        cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16 && \
        cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/models/mask_rcnn_inception_v2_coco_2018_01_28/output

#
# Install rviz
#
RUN apt-get install ros-kinetic-rviz -y

#
# Copy example video
#
COPY ./video.webm /root/

#
# Ready !
#