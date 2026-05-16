# Author: Euntae Hong
# email: euntae.hong@lge.com

FROM osrf/ros:humble-desktop-full

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    libatlas-base-dev \
    libeigen3-dev \
    libpcl-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libsuitesparse-dev \
    libglew-dev \
    libcairo2-dev \
    libabsl-dev \
    ninja-build \
    wget unzip git vim python3-pip \
    protobuf-compiler \
    ros-humble-tf2 \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-navigation2 \
    ros-humble-backward-ros \
  && rm -rf /var/lib/apt/lists/*

# Install Ceres Solver 2.1.x explicitly for QuaternionManifold support.
RUN git clone --depth 1 --branch 2.1.0 https://github.com/ceres-solver/ceres-solver.git /tmp/ceres-solver \
  && cmake -S /tmp/ceres-solver -B /tmp/ceres-solver/build \
       -G Ninja \
       -DBUILD_TESTING=OFF \
       -DBUILD_EXAMPLES=OFF \
       -DBUILD_BENCHMARKS=OFF \
       -DMINIGLOG=OFF \
  && cmake --build /tmp/ceres-solver/build -j"$(nproc)" \
  && cmake --install /tmp/ceres-solver/build \
  && rm -rf /tmp/ceres-solver

WORKDIR /root/ros2_ws

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
