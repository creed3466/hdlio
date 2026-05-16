# ig_lio.Dockerfile — iG-LIO (zijiechenrobotics/ig_lio), prebuilt workspace.
#
# Upstream iG-LIO launch files spawn rviz with required="true"; we bake a
# headless launch (loaded at runtime against the overlaid config).

FROM baselines-base:ros1

ENV CATKIN_WS=/root/catkin_ws
ENV IG_DIR=/root/catkin_ws/src/ig_lio
WORKDIR ${CATKIN_WS}

RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git ${CATKIN_WS}/src/livox_ros_driver
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git ${CATKIN_WS}/src/livox_ros_driver2 \
    || true

# Bake iG-LIO source
COPY algorithms/ig_lio ${IG_DIR}

# Headless launch asset: the config filename (avia.yaml / ncd.yaml) is chosen
# at runtime via IG_LAUNCH_CFG env substitution inside the generated launch.
# We stage a parameterized launch that reads $(env IG_LAUNCH_CFG) so a single
# launch file works for both Avia (avia.yaml) and NTU (ncd.yaml) datasets.
COPY docker/ig_lio_headless.launch ${IG_DIR}/launch/headless.launch

# Build workspace
RUN bash -c "source /opt/ros/noetic/setup.bash && \
             cd ${CATKIN_WS} && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release"

CMD ["/bin/bash"]
