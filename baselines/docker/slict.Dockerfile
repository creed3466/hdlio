# slict.Dockerfile — SLICT (brytsknguyen/slict)
# Deps: Ceres 2.1.0, Sophus 1.22.10, UFOMap (devel_surfel branch), custom livox_ros_driver
# Paper validation: NTU VIRAL (author's own dataset) — run_ntuviral.launch provided
# Surfel-based continuous-time LIO → direct comparison with TofSLAM's surfel map

FROM baselines-base:ros1

ENV CATKIN_WS=/root/catkin_ws
ENV CERES_VERSION="2.1.0"
ENV LIBRARY=/root/library

RUN mkdir -p $LIBRARY

# Ceres 2.1.0
RUN cd $LIBRARY && \
    git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && git checkout ${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    make -j"$(nproc)" install && \
    cd $LIBRARY && rm -rf ceres-solver

# Sophus 1.22.10
RUN cd $LIBRARY && \
    git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git && \
    cd Sophus && mkdir build && cd build && \
    cmake .. -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF && \
    make -j"$(nproc)" install && \
    cd $LIBRARY && rm -rf Sophus

# UFOMap (devel_surfel branch — SLICT's fork)
RUN git clone https://github.com/brytsknguyen/ufomap $CATKIN_WS/src/ufomap && \
    cd $CATKIN_WS/src/ufomap && git checkout devel_surfel

# Custom livox_ros_driver (SLICT's fork)
RUN git clone https://github.com/brytsknguyen/livox_ros_driver $CATKIN_WS/src/livox_ros_driver

# Build ufomap + livox_driver first (SLICT depends on them)
WORKDIR $CATKIN_WS
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && cd $CATKIN_WS && catkin build'

# SLICT source mounted at run time -> src/slict
CMD ["/bin/bash"]
