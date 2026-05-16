# point_lio.Dockerfile — Point-LIO (hku-mars/Point-LIO), prebuilt workspace.
#
# Point-LIO has a known missing explicit dep on `livox_ros_driver_generate_messages_cpp`,
# which races with the LIO target when built in parallel. We do the messages
# pre-pass explicitly at image build time.

FROM baselines-base:ros1

ENV CATKIN_WS=/root/catkin_ws
ENV POINT_LIO_DIR=/root/catkin_ws/src/Point-LIO
WORKDIR ${CATKIN_WS}

RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git ${CATKIN_WS}/src/livox_ros_driver
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git ${CATKIN_WS}/src/livox_ros_driver2 \
    || true

# Bake Point-LIO source
COPY algorithms/point_lio ${POINT_LIO_DIR}

# Messages pre-pass (workaround for missing dep), then full build
RUN bash -c "source /opt/ros/noetic/setup.bash && \
             cd ${CATKIN_WS} && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release livox_ros_driver_generate_messages_cpp && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release"

CMD ["/bin/bash"]
