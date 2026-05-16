# dlio.Dockerfile — DLIO (vectr-ucla/direct_lidar_inertial_odometry),
# feature/livox-support branch, prebuilt workspace.
#
# This branch natively subscribes to livox_ros_driver/CustomMsg via the
# livox_topic launch arg. It converts CustomMsg to PointCloud2 internally
# and feeds into its GICP pipeline. No external converter is needed.
#
# Patched: upstream uses livox_ros_driver2 but we use livox_ros_driver (v1)
# since the message types have identical MD5 checksums and the v1 package
# is simpler to build in ROS Noetic.
#
# Build context: baselines/ directory (same as other algo Dockerfiles).

FROM baselines-base:ros1

ENV CATKIN_WS=/root/catkin_ws
ENV DLIO_DIR=/root/catkin_ws/src/direct_lidar_inertial_odometry
WORKDIR ${CATKIN_WS}

# Livox ROS driver v1 (message definitions for CustomMsg)
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git \
    ${CATKIN_WS}/src/livox_ros_driver

# Bake DLIO source (patched to use livox_ros_driver v1)
COPY algorithms/dlio ${DLIO_DIR}

# Messages pre-pass: build livox_ros_driver messages first so
# livox_ros_driver/CustomMsg.h is generated before DLIO compilation.
# Then full build (DLIO + its save_pcd.srv service).
RUN bash -c "source /opt/ros/noetic/setup.bash && \
             cd ${CATKIN_WS} && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release \
               livox_ros_driver_generate_messages_cpp && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release"

CMD ["/bin/bash"]
