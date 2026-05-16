#!/usr/bin/env bash
# In-container entrypoint for LIO-SAM (brytsknguyen fork, NTU VIRAL path).
#
# Prebuilt image: LIO-SAM catkin workspace is already compiled at image-build
# time (see baselines/docker/lio_sam.Dockerfile). This script skips the compile
# step — runtime cost is only roscore + roslaunch + rosbag play + extract.
#
# brytsknguyen/LIO-SAM adds R_W2NED = diag(1,-1,-1) in utility.h and is the
# canonical NTU VIRAL LIO-SAM fork (authored by NTU VIRAL dataset lead).
# Only NTU VIRAL is supported (9-axis IMU requirement).
set -euo pipefail

source /opt/ros/noetic/setup.bash

CATKIN_WS="${CATKIN_WS:-/root/catkin_ws}"
LIOSAM_DIR="${CATKIN_WS}/src/LIO-SAM"

DATASET="${DATASET:-ntu}"
case "${DATASET}" in
    ntu) : ;;
    *) echo "[ERROR] LIO-SAM excluded from ${DATASET} (requires 9-axis IMU)"; exit 1 ;;
esac

# Overlay per-seq config onto the baked workspace.
if [[ ! -f /config/params.yaml ]]; then
    echo "[ERROR] /config/params.yaml not mounted"; exit 1
fi
cp /config/params.yaml "${LIOSAM_DIR}/config/params.yaml"
echo "[INFO] Using config → ${LIOSAM_DIR}/config/params.yaml (overlay)"

# Sanity: devel space must exist (proves prebuilt image is in use).
if [[ ! -f "${CATKIN_WS}/devel/setup.bash" ]]; then
    echo "[ERROR] Prebuilt devel space missing — wrong image? Rebuild via build_algo.sh lio_sam"
    exit 1
fi
source "${CATKIN_WS}/devel/setup.bash"

# Ensure libmetis-gtsam.so is loadable (ldconfig was baked into image but
# LD_LIBRARY_PATH export is re-applied here to survive CMD overrides).
ldconfig /usr/local/lib 2>/dev/null || true
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

ROSCORE_PORT=11311
export ROS_MASTER_URI="http://localhost:${ROSCORE_PORT}"
export ROS_HOSTNAME=localhost
roscore -p ${ROSCORE_PORT} &
ROSCORE_PID=$!
sleep 3

roslaunch lio_sam run.launch &
LIO_PID=$!
sleep 8

# LIO-SAM main pose output: /lio_sam/mapping/odometry
rosbag record -O /out/odom.bag /lio_sam/mapping/odometry /lio_sam/mapping/path &
REC_PID=$!
sleep 2

set +e
rosbag play /bag/input.bag --clock -r 1.0 -d 2.0
PLAY_EXIT=$?
set -e
echo "[INFO] rosbag play exited with ${PLAY_EXIT}"
sleep 5

kill -INT ${REC_PID} 2>/dev/null || true
wait ${REC_PID} 2>/dev/null || true

python3 /baselines_scripts/extract_tum_from_odom_bag.py \
    /out/odom.bag /lio_sam/mapping/odometry /out/traj.csv \
    || echo "[WARN] trajectory extraction failed"

kill -INT ${LIO_PID} 2>/dev/null || true
sleep 2
kill -INT ${ROSCORE_PID} 2>/dev/null || true
wait 2>/dev/null || true

echo "[DONE] LIO-SAM run complete (${DATASET}/${SEQ:-?})"
exit 0
