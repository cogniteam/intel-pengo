FROM ros:kinetic

#
# Set default shell to bash (following setup.sh is a bash script)
#
SHELL [ "/bin/bash", "-c" ]

#
# Copy local build dep files
#
ADD ./silent.cfg /openvino_install/l_openvino_toolkit_p_2018.5.455/
COPY ./open_model_zoo /opt/openvino_toolkit/open_model_zoo
COPY ./mask_rcnn_inception_v2_coco_2018_01_28 /opt/models/mask_rcnn_inception_v2_coco_2018_01_28
COPY ./video.webm /root/

#
# 
#
ENV InferenceEngine_DIR /opt/openvino_toolkit/dldt/inference-engine/build/
ENV CPU_EXTENSION_LIB /opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libcpu_extension.so
ENV GFLAGS_LIB /opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libgflags_nothreads.a
ENV LD_LIBRARY_PATH /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib

#
# Base apt dependencies
#
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
        apt update && apt install build-essential cmake git python3-numpy libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev sudo cpio lsb-release wget software-properties-common ros-kinetic-rviz python3-yaml python3-requests -y && \
        add-apt-repository ppa:deadsnakes/ppa -y && \
        add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo `lsb_release -cs` main" -y && \
        apt update && \
        apt install librealsense2-dev ros-kinetic-realsense2-camera python3.6 autoconf libtool unzip -y && \
        curl https://bootstrap.pypa.io/get-pip.py | python3.6 && \
        rm /var/lib/apt/lists/* -rf

#
# https://docs.openvinotoolkit.org/2018_R5/_docs_install_guides_installing_openvino_docker.html
#
RUN cd /tmp/ && \
   wget https://github.com/libusb/libusb/archive/v1.0.22.zip && \
   unzip v1.0.22.zip && cd libusb-1.0.22 && \
   ./bootstrap.sh && \
   ./configure --disable-udev --enable-shared && \
   make -j4 && make install && \
   rm -rf /tmp/*

#
# 1. Download OpenVINO 2018 R5 
# 2. Install it
# 3. Build samples
#
WORKDIR /
RUN wget -nv -c https://cloud.cogni.io/index.php/s/hwIJbGZAnl0xXWP/download -O l_openvino_toolkit_p_2018.5.455.tgz && \
        mkdir /openvino_install -p && \
        tar -xzf l_openvino_toolkit_p_2018.5.455.tgz && \ 
        rm /l_openvino_toolkit_p_2018.5.455/silent.cfg -rf && \
        mv /l_openvino_toolkit_p_2018.5.455/* /openvino_install/l_openvino_toolkit_p_2018.5.455/ && \
        rm /l_openvino_toolkit_p_2018.5.455 -rf && \
        cd /openvino_install/l_openvino_toolkit_p_2018.5.455 && \
        ./install_cv_sdk_dependencies.sh && ./install.sh --silent silent.cfg && \
        . /opt/intel/computer_vision_sdk/bin/setupvars.sh && \
        cd /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/ && \
        mkdir build -p && cd build && \
        cmake .. && \
        make && \
        echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib" >> /root/.bashrc && \
        echo "source /opt/intel/computer_vision_sdk/bin/setupvars.sh" >> /root/.bashrc && \
        rm /openvino_install -rf && rm /l_openvino_toolkit_p_2018.5.455.tgz -rf && \
        rm /var/lib/apt/lists/* -rf

#
# OpenCV
#
WORKDIR /
RUN mkdir /opencv_build/ && cd /opencv_build && \
        git clone https://github.com/opencv/opencv.git -b 3.4.2 --depth 1 && \
        git clone https://github.com/opencv/opencv_contrib.git -b 3.4.2 --depth 1 && \
        cd opencv && \
        mkdir build && cd build && \
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/opencv_build/opencv_contrib/modules/ .. && \
        make -j8 && \
        make install && \
        cd / && rm /opencv_build -rf



#
# DLDT & Open Model Zoo
#
COPY ./catkin_ws /root/catkin_ws
WORKDIR /opt/openvino_toolkit
RUN git clone https://github.com/opencv/dldt.git -b 2018_R4 --recurse-submodules && \
        cd /opt/openvino_toolkit/dldt/inference-engine && \
        ./install_dependencies.sh && \
        mkdir build -p && \
        cd /opt/openvino_toolkit/dldt/inference-engine/build && \
        . /opt/intel/computer_vision_sdk/bin/setupvars.sh && \
        cmake -DCMAKE_BUILD_TYPE=Release .. && make -j8 && \
        mkdir /opt/openvino_toolkit/open_model_zoo/demos/build -p && \
        cd /opt/openvino_toolkit/open_model_zoo/demos/build && \
        . /opt/intel/computer_vision_sdk/bin/setupvars.sh && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j8 && \
        #
        # Catkin make
        #
        cd /root/catkin_ws && \
        . /opt/ros/kinetic/setup.bash && \
        rosdep update && \
        rosdep install --from-path src --ignore-src -y && \
        catkin_make && \
        ln -s /root/catkin_ws/src/ros_openvino_toolkit /opt/openvino_toolkit/ros_openvino_toolkit && \
        echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
        rm /root/catkin_ws/build -rf && \
        rm /var/lib/apt/lists/* -rf

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
# Install python dependencies for TF model optimizer
#
WORKDIR /opt/openvino_toolkit/dldt/model-optimizer
RUN python3.6 -m pip install -r requirements.txt && \
        python3.6 -m pip install networkx==1.11 && \
        cd /opt/models/mask_rcnn_inception_v2_coco_2018_01_28 && \
        #
        # Convert TF model to OpenVINO (?)
        #
        python3.6 /opt/openvino_toolkit/dldt/model-optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/openvino_toolkit/dldt/model-optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --output_dir ./output/ && \
        python3.6 /opt/openvino_toolkit/dldt/model-optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/openvino_toolkit/dldt/model-optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --data_type=FP16 --reverse_input_channels --output_dir ./output_fp16/ && \
        #
        # Optimize model
        #
        cd /opt/openvino_toolkit/open_model_zoo/model_downloader && \
        # FP32 precision mode
        python3.6 /opt/openvino_toolkit/dldt/model-optimizer/mo.py --input_model /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32 --mean_values [127.5,127.5,127.5] --scale_values [127.5] && \
        # FP16 precision mode
        python3.6 /opt/openvino_toolkit/dldt/model-optimizer/mo.py --input_model /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16 --data_type=FP16 --mean_values [127.5,127.5,127.5] --scale_values [127.5] && \
        #
        # Copy labels
        #
        cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/open_model_zoo/model_downloader/Retail/object_attributes/emotions_recognition/0003/dldt && \
        cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/open_model_zoo/model_downloader/Transportation/object_detection/face/pruned_mobilenet_reduced_ssd_shared_weights/dldt && \
        cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32 && \
        cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/open_model_zoo/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16 && \
        cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/models/mask_rcnn_inception_v2_coco_2018_01_28/output


#
# Ready !
#

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

