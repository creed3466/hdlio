#!/usr/bin/env bash
# In-container entrypoint for FAST-LIO2 (prebuilt image variant).
#
# The workspace was compiled at image-build time. Runtime does only:
#   1. Overlay per-seq config onto baked config/<basename>.yaml
#   2. roscore + roslaunch (headless) + rosbag record + rosbag play
#   3. Extract TUM trajectory from the recorded bag
set -euo pipefail

source /opt/ros/noetic/setup.bash
CATKIN_WS="${CATKIN_WS:-/root/catkin_ws}"
FAST_LIO_DIR="${CATKIN_WS}/src/FAST_LIO"

DATASET="${DATASET:-avia}"
case "${DATASET}" in
    avia)         LAUNCH_FILE="mapping_avia.launch";      CFG_BASENAME="avia.yaml" ;;
    mid360)       LAUNCH_FILE="mapping_mid360.launch";    CFG_BASENAME="mid360.yaml" ;;
    ntu)          LAUNCH_FILE="mapping_ouster64.launch";  CFG_BASENAME="ouster64.yaml" ;;
    *) echo "[ERROR] Unsupported DATASET: ${DATASET}"; exit 1 ;;
esac

# Sanity: prebuilt devel space must exist
if [[ ! -f "${CATKIN_WS}/devel/setup.bash" ]]; then
    echo "[ERROR] Prebuilt devel space missing — rebuild via build_algo.sh fast_lio2"
    exit 1
fi

if [[ ! -f /config/params.yaml ]]; then
    echo "[ERROR] /config/params.yaml not mounted"; exit 1
fi
cp /config/params.yaml "${FAST_LIO_DIR}/config/${CFG_BASENAME}"
echo "[INFO] Using config → ${FAST_LIO_DIR}/config/${CFG_BASENAME}"

# Copy custom launch files from mounted algo_src if they exist
if [[ -d /algo_src/launch ]]; then
    cp /algo_src/launch/*.launch "${FAST_LIO_DIR}/launch/" 2>/dev/null || true
fi

source "${CATKIN_WS}/devel/setup.bash"

ROSCORE_PORT=11311
export ROS_MASTER_URI="http://localhost:${ROSCORE_PORT}"
export ROS_HOSTNAME=localhost
roscore -p ${ROSCORE_PORT} &
ROSCORE_PID=$!
sleep 3

# Mid-360 needs the v2→v1 namespace republisher. Source the helper so it
# can export REPUB_PID into our shell for later shutdown.
if [[ "${DATASET}" == "mid360" ]]; then
    # shellcheck disable=SC1091
    source /baselines_scripts/setup_mid360_republisher.sh
fi

roslaunch fast_lio "${LAUNCH_FILE}" rviz:=false &
LIO_PID=$!
sleep 5

rosbag record -O /out/odom.bag /Odometry /path &
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
    /out/odom.bag /Odometry /out/traj.csv \
    || echo "[WARN] trajectory extraction failed"

kill -INT ${LIO_PID} 2>/dev/null || true
sleep 2
if [[ "${DATASET}" == "mid360" && -n "${REPUB_PID:-}" ]]; then
    kill -INT "${REPUB_PID}" 2>/dev/null || true
fi
kill -INT ${ROSCORE_PID} 2>/dev/null || true
wait 2>/dev/null || true

echo "[DONE] FAST-LIO2 run complete (${DATASET}/${SEQ:-?})"
exit 0
