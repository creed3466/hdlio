# base.Dockerfile — ROS1 Noetic base for all LIO baselines
# Shared deps: PCL, Eigen, OpenCV, catkin tools, rosbags utils
# Author: Euntae Hong

FROM ros:noetic-perception

ENV DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS=/root/catkin_ws

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget curl nano vim unzip \
    cmake ninja-build build-essential \
    libeigen3-dev libpcl-dev libopencv-dev \
    libboost-all-dev libyaml-cpp-dev \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    libtbb-dev \
    python3-pip python3-wstool python3-catkin-tools \
    ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-ros \
    ros-noetic-tf-conversions \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions \
    ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-rviz ros-noetic-jsk-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*

# rosbags Python lib (ROS1↔ROS2 bag utilities, consistent with paper_figures)
RUN pip3 install --no-cache-dir rosbags numpy scipy

RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Create livox_ros_driver workspace (most algos need this)
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git /opt/Livox-SDK && \
    cd /opt/Livox-SDK && \
    mkdir -p build && cd build && \
    cmake .. && make -j"$(nproc)" && make install && \
    rm -rf /opt/Livox-SDK

CMD ["/bin/bash"]
