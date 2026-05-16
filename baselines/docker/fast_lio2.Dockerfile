# fast_lio2.Dockerfile — FAST-LIO2 (hku-mars/FAST_LIO), prebuilt workspace.
#
# Prebuilt pattern: FAST-LIO2 is copied into the catkin workspace and compiled
# at image-build time. Runtime containers only overlay the per-seq config onto
# the baked `config/<basename>.yaml`, roslaunch, rosbag record+play, extract.

FROM baselines-base:ros1

ENV CATKIN_WS=/root/catkin_ws
ENV FAST_LIO_DIR=/root/catkin_ws/src/FAST_LIO
WORKDIR ${CATKIN_WS}

# Livox drivers (v1 for Avia, v2 optional, guarded with || true)
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git ${CATKIN_WS}/src/livox_ros_driver
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git ${CATKIN_WS}/src/livox_ros_driver2 \
    && (cd ${CATKIN_WS}/src/livox_ros_driver2 && sed -i 's/ROS2/ROS1/g' build.sh 2>/dev/null || true) \
    || true

# Bake FAST-LIO2 source
COPY algorithms/fast_lio2 ${FAST_LIO_DIR}

# Messages pre-pass: fastlio_mapping depends on BOTH fast_lio/Pose6D.h and
# livox_ros_driver/CustomMsg.h. Under `-j2` scheduling, laserMapping.cpp can
# start compiling before either generator finishes → fatal: Pose6D.h not found.
# Build both *_generate_messages_cpp targets explicitly first.
RUN bash -c "source /opt/ros/noetic/setup.bash && \
             cd ${CATKIN_WS} && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release \
               fast_lio_generate_messages_cpp \
               livox_ros_driver_generate_messages_cpp && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release"

CMD ["/bin/bash"]
