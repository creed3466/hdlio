# Dockerfile for Surfel-LIO (non-ROS, standalone C++)
# Reads pre-parsed PLY+CSV data, outputs TUM trajectory.
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DOCKER_CONTAINER=1

RUN apt-get update && apt-get install -y \
    cmake build-essential \
    libeigen3-dev \
    libgl1-mesa-dev libglu1-mesa-dev libglew-dev \
    libyaml-cpp-dev \
    python3 python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy algorithm source
COPY algorithms/surfel_lio /opt/surfel_lio

# Build Surfel-LIO
WORKDIR /opt/surfel_lio
RUN bash build.sh

WORKDIR /opt/surfel_lio/build
