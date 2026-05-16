# faster_lio.Dockerfile — Faster-LIO (gaoxiang12/faster-lio), prebuilt workspace.
#
# Prebuilt pattern: Faster-LIO is copied into the catkin workspace and compiled
# at image-build time. Runtime containers only overlay the per-seq config onto
# the baked config/<basename>.yaml, roslaunch, rosbag record+play, extract.
#
# Faster-LIO bundles a messages-only livox_ros_driver stub under thirdparty/
# and references it via add_subdirectory(). We remove that stub and the
# add_subdirectory line so the externally cloned livox_ros_driver (which
# provides the same CustomMsg/CustomPoint messages) is used instead.
# This avoids a duplicate-package conflict in the catkin workspace and
# ensures the republisher sidecar (for Mid-360) can link against the
# same message definitions.

FROM baselines-base:ros1

ENV CATKIN_WS=/root/catkin_ws
ENV FASTER_LIO_DIR=/root/catkin_ws/src/faster_lio
WORKDIR ${CATKIN_WS}

# Livox driver v1 (provides CustomMsg/CustomPoint messages for all Livox algos)
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git ${CATKIN_WS}/src/livox_ros_driver

# Bake Faster-LIO source
COPY algorithms/faster_lio ${FASTER_LIO_DIR}

# Remove the bundled livox_ros_driver stub so it doesn't conflict with the
# externally cloned one, and strip the add_subdirectory reference from CMakeLists.
RUN rm -rf ${FASTER_LIO_DIR}/thirdparty/livox_ros_driver && \
    sed -i '/add_subdirectory(thirdparty\/livox_ros_driver)/d' ${FASTER_LIO_DIR}/CMakeLists.txt

# Install TBB (required by Faster-LIO's iVox)
RUN apt-get update && apt-get install -y --no-install-recommends \
        libtbb-dev libgflags-dev libgoogle-glog-dev libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Two-pass build: generate messages first (Pose6D.h + CustomMsg.h), then full build.
# This avoids the race where laserMapping.cc compiles before message headers exist.
RUN bash -c "source /opt/ros/noetic/setup.bash && \
             cd ${CATKIN_WS} && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release \
               faster_lio_generate_messages_cpp \
               livox_ros_driver_generate_messages_cpp && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release"

CMD ["/bin/bash"]
