#!/usr/bin/env bash
# In-container entrypoint for SLICT (pinned to `noetic` tag — ROS1 catkin build).
#
# Mounts:
#   /algo_src (SLICT source), /config/params.yaml, /bag/input.bag, /out
# Env:
#   DATASET = ntu (only supported path for now)
set -euo pipefail

source /opt/ros/noetic/setup.bash
export CATKIN_WS=/root/catkin_ws

DATASET="${DATASET:-ntu}"
case "${DATASET}" in
    ntu) CFG_BASENAME="ntuviral.yaml"; LAUNCH_NAME="run_ntuviral.launch" ;;
    *) echo "[ERROR] SLICT ${DATASET} not configured yet"; exit 1 ;;
esac

SLICT_DIR="${CATKIN_WS}/src/slict"
rm -rf "${SLICT_DIR}"
cp -r /algo_src "${SLICT_DIR}"
cp /config/params.yaml "${SLICT_DIR}/config/${CFG_BASENAME}"
echo "[INFO] Using config → ${SLICT_DIR}/config/${CFG_BASENAME}"

# Also need custom livox_ros_driver (from brytsknguyen fork) for the custom
# msg types. If /opt/livox_ros_driver exists (built by Dockerfile), include it.
if [[ -d /opt/livox_ros_driver_slict ]]; then
    cp -r /opt/livox_ros_driver_slict "${CATKIN_WS}/src/livox_ros_driver" || true
fi

cd "${CATKIN_WS}"
# SLICT Dockerfile initialized the workspace with catkin_tools (catkin build),
# so we must use catkin build here.
#
# brytsknguyen/ufomap @ devel_surfel ships ufomap_msgs / ufomap_ros with
# `buildtool_depend=ament_cmake` (ROS2). The C++ code inside is ROS1-agnostic,
# so we rewrite their package.xml + CMakeLists.txt as catkin packages.
UFO_ROOT=${CATKIN_WS}/src/ufomap/ufomap_ros
if [[ -d "${UFO_ROOT}/ufomap_msgs" ]]; then
    cat > "${UFO_ROOT}/ufomap_msgs/package.xml" <<'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>ufomap_msgs</name>
  <version>1.0.0</version>
  <description>UFOMap ROS1 messages (rewritten from ament_cmake for Noetic).</description>
  <maintainer email="dev@example.com">local</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>std_msgs</build_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
EOF
    cat > "${UFO_ROOT}/ufomap_msgs/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(ufomap_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES UFOMap.msg UFOMapStamped.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
EOF
fi
if [[ -d "${UFO_ROOT}/ufomap_ros" ]]; then
    cat > "${UFO_ROOT}/ufomap_ros/package.xml" <<'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>ufomap_ros</name>
  <version>1.0.0</version>
  <description>UFOMap ROS1 bridge (rewritten from ament_cmake for Noetic).</description>
  <maintainer email="dev@example.com">local</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>sensor_msgs</depend>
  <depend>ufomap_msgs</depend>
</package>
EOF
    cat > "${UFO_ROOT}/ufomap_ros/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(ufomap_ros)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs ufomap_msgs)
find_package(ufomap REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs ufomap_msgs
  DEPENDS ufomap
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/conversions.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} UFO::Map)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
EOF
fi

catkin build slict -j3 --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -60
source "${CATKIN_WS}/devel/setup.bash"

ROSCORE_PORT=11311
export ROS_MASTER_URI="http://localhost:${ROSCORE_PORT}"
export ROS_HOSTNAME=localhost
roscore -p ${ROSCORE_PORT} &
ROSCORE_PID=$!
sleep 3

# Upstream run_ntuviral.launch spawns its own rosbag player + rviz. Strip those.
# Simpler: write a minimal launch that only starts sensorsync + estimator.
cat > "${SLICT_DIR}/launch/headless.launch" <<EOF
<launch>
    <rosparam file="\$(find slict)/config/${CFG_BASENAME}" command="load"/>
    <param name="/autoexit"  type="int"    value="0"/>
    <param name="/loop_en"   type="int"    value="0"/>
    <param name="/log_dir"   type="string" value="/out/slict_logs"/>
    <node pkg="slict" type="slict_sensorsync" name="slict_sensorsync" respawn="false" output="log" required="true"/>
    <node pkg="slict" required="true" type="slict_estimator" name="slict_estimator" respawn="false" output="screen"/>
</launch>
EOF

mkdir -p /out/slict_logs

roslaunch slict headless.launch &
LIO_PID=$!
sleep 8

# SLICT main pose topic: /opt_odom
rosbag record -O /out/odom.bag /opt_odom /opt_odom_high_freq /sw_ctr_pose &
REC_PID=$!
sleep 2

set +e
rosbag play /bag/input.bag --clock -r 1.0 -d 2.0
PLAY_EXIT=$?
set -e
echo "[INFO] rosbag play exited with ${PLAY_EXIT}"
sleep 10   # SLICT needs time to finalize last optimization window

kill -INT ${REC_PID} 2>/dev/null || true
wait ${REC_PID} 2>/dev/null || true

python3 /baselines_scripts/extract_tum_from_odom_bag.py \
    /out/odom.bag /opt_odom /out/traj.csv \
    || echo "[WARN] trajectory extraction failed"

kill -INT ${LIO_PID} 2>/dev/null || true
sleep 2
kill -INT ${ROSCORE_PID} 2>/dev/null || true
wait 2>/dev/null || true

echo "[DONE] SLICT run complete (${DATASET}/${SEQ:-?})"
exit 0
