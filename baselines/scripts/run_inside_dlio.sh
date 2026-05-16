#!/usr/bin/env bash
# In-container entrypoint for DLIO (feature/livox-support branch, prebuilt).
#
# DLIO's feature/livox-support branch natively subscribes to
# livox_ros_driver2/CustomMsg via the livox_topic launch arg. It converts
# CustomMsg to PointCloud2 internally and publishes on the pointcloud topic
# for its own GICP pipeline. No external converter is needed.
#
# For NTU VIRAL (Ouster OS1-16), the standard PointCloud2 path is used via
# the pointcloud_topic launch arg. The sensor type (Ouster vs Livox) is
# auto-detected from point cloud field names.
#
# M3DGR Avia bags record livox_ros_driver/CustomMsg (v1) and Mid-360 bags
# record livox_ros_driver2/CustomMsg (v2), but these have identical MD5
# sums so DLIO's livox subscriber works with both without modification.
#
# The per-dataset config (dlio.yaml) is overlaid from /config/params.yaml
# onto the baked cfg/dlio.yaml. The upstream params.yaml (odom/map params)
# is kept as-is from the build.
set -euo pipefail

source /opt/ros/noetic/setup.bash
CATKIN_WS="${CATKIN_WS:-/root/catkin_ws}"
DLIO_DIR="${CATKIN_WS}/src/direct_lidar_inertial_odometry"

DATASET="${DATASET:-avia}"

# Topic and launch arg configuration per dataset.
# Livox datasets use the livox_topic arg (CustomMsg subscriber).
# NTU VIRAL uses the pointcloud_topic arg (PointCloud2 subscriber).
USE_LIVOX=false
case "${DATASET}" in
    avia)
        LIVOX_TOPIC="/livox/avia/lidar"
        IMU_TOPIC="/livox/avia/imu"
        USE_LIVOX=true
        ;;
    mid360|indoor)
        LIVOX_TOPIC="/livox/mid360/lidar"
        IMU_TOPIC="/livox/mid360/imu"
        USE_LIVOX=true
        ;;
    ntu)
        PC_TOPIC="/os1_cloud_node1/points"
        IMU_TOPIC="/imu/imu"
        ;;
    *)
        echo "[ERROR] Unsupported DATASET: ${DATASET}"; exit 1 ;;
esac

# Sanity: prebuilt devel space must exist
if [[ ! -f "${CATKIN_WS}/devel/setup.bash" ]]; then
    echo "[ERROR] Prebuilt devel space missing — rebuild via build_algo.sh dlio"
    exit 1
fi

# Overlay per-dataset extrinsics config
if [[ ! -f /config/params.yaml ]]; then
    echo "[ERROR] /config/params.yaml not mounted"; exit 1
fi
cp /config/params.yaml "${DLIO_DIR}/cfg/dlio.yaml"
echo "[INFO] Using config → ${DLIO_DIR}/cfg/dlio.yaml"

source "${CATKIN_WS}/devel/setup.bash"

ROSCORE_PORT=11311
export ROS_MASTER_URI="http://localhost:${ROSCORE_PORT}"
export ROS_HOSTNAME=localhost
roscore -p ${ROSCORE_PORT} &
ROSCORE_PID=$!
sleep 3

# Launch DLIO with correct topic remappings.
# For Livox: use livox_topic (CustomMsg subscriber converts internally).
# For NTU: use pointcloud_topic (standard PointCloud2).
if [[ "${USE_LIVOX}" == "true" ]]; then
    echo "[INFO] Livox mode: livox_topic=${LIVOX_TOPIC}, imu=${IMU_TOPIC}"
    roslaunch direct_lidar_inertial_odometry dlio.launch \
        livox_topic:="${LIVOX_TOPIC}" \
        imu_topic:="${IMU_TOPIC}" \
        rviz:=false &
else
    echo "[INFO] PointCloud2 mode: pointcloud_topic=${PC_TOPIC}, imu=${IMU_TOPIC}"
    roslaunch direct_lidar_inertial_odometry dlio.launch \
        pointcloud_topic:="${PC_TOPIC}" \
        imu_topic:="${IMU_TOPIC}" \
        rviz:=false &
fi
LIO_PID=$!
sleep 5

# Record odometry output for trajectory extraction.
# DLIO publishes nav_msgs/Odometry on /robot/dlio/odom_node/odom
# (the robot_namespace default is "robot").
rosbag record -O /out/odom.bag /robot/dlio/odom_node/odom /robot/dlio/odom_node/path &
REC_PID=$!
sleep 2

# Play bag
set +e
rosbag play /bag/input.bag --clock -r 1.0 -d 2.0
PLAY_EXIT=$?
set -e
echo "[INFO] rosbag play exited with ${PLAY_EXIT}"
sleep 5

# Stop recording
kill -INT ${REC_PID} 2>/dev/null || true
wait ${REC_PID} 2>/dev/null || true

# Extract TUM-format trajectory
python3 /baselines_scripts/extract_tum_from_odom_bag.py \
    /out/odom.bag /robot/dlio/odom_node/odom /out/traj.csv \
    || echo "[WARN] trajectory extraction failed"

# Cleanup
kill -INT ${LIO_PID} 2>/dev/null || true
sleep 2
kill -INT ${ROSCORE_PID} 2>/dev/null || true
wait 2>/dev/null || true

echo "[DONE] DLIO run complete (${DATASET}/${SEQ:-?})"
exit 0
