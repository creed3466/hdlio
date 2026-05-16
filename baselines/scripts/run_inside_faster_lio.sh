#!/usr/bin/env bash
# In-container entrypoint for Faster-LIO (prebuilt image variant).
#
# The workspace was compiled at image-build time. Runtime does only:
#   1. Overlay per-seq config onto baked config/<basename>.yaml
#   2. roscore + roslaunch (headless) + rosbag play
#   3. Copy the internally saved TUM trajectory to /out/traj.csv
set -euo pipefail

source /opt/ros/noetic/setup.bash
CATKIN_WS="${CATKIN_WS:-/root/catkin_ws}"
FASTER_LIO_DIR="${CATKIN_WS}/src/faster_lio"

DATASET="${DATASET:-avia}"
case "${DATASET}" in
    avia)          LAUNCH_FILE="mapping_avia.launch";      CFG_BASENAME="avia.yaml" ;;
    mid360|indoor) LAUNCH_FILE="mapping_mid360.launch";    CFG_BASENAME="mid360.yaml" ;;
    ntu)           LAUNCH_FILE="mapping_ouster64.launch";  CFG_BASENAME="ouster64.yaml" ;;
    *) echo "[ERROR] Unsupported DATASET: ${DATASET}"; exit 1 ;;
esac

# Sanity: prebuilt devel space must exist
if [[ ! -f "${CATKIN_WS}/devel/setup.bash" ]]; then
    echo "[ERROR] Prebuilt devel space missing — rebuild via build_algo.sh faster_lio"
    exit 1
fi

if [[ ! -f /config/params.yaml ]]; then
    echo "[ERROR] /config/params.yaml not mounted"; exit 1
fi
cp /config/params.yaml "${FASTER_LIO_DIR}/config/${CFG_BASENAME}"
echo "[INFO] Using config → ${FASTER_LIO_DIR}/config/${CFG_BASENAME}"

# Copy custom launch files from mounted algo_src if they exist
if [[ -d /algo_src/launch ]]; then
    cp /algo_src/launch/*.launch "${FASTER_LIO_DIR}/launch/" 2>/dev/null || true
fi

source "${CATKIN_WS}/devel/setup.bash"

# Faster-LIO saves trajectory to ./Log/traj.txt via gflags default.
# Create the directory so the internal saver can write there.
mkdir -p /out/Log

ROSCORE_PORT=11311
export ROS_MASTER_URI="http://localhost:${ROSCORE_PORT}"
export ROS_HOSTNAME=localhost
roscore -p ${ROSCORE_PORT} &
ROSCORE_PID=$!
sleep 3

# Mid-360/Indoor needs the v2→v1 namespace republisher.
if [[ "${DATASET}" == "mid360" || "${DATASET}" == "indoor" ]]; then
    # shellcheck disable=SC1091
    source /baselines_scripts/setup_mid360_republisher.sh
fi

# Launch from /out so that ./Log/traj.txt lands in /out/Log/traj.txt
cd /out
roslaunch faster_lio "${LAUNCH_FILE}" rviz:=false &
LIO_PID=$!
sleep 5

set +e
rosbag play /bag/input.bag --clock -r 1.0 -d 2.0
PLAY_EXIT=$?
set -e
echo "[INFO] rosbag play exited with ${PLAY_EXIT}"
sleep 5

# Signal Faster-LIO to shutdown gracefully (triggers Savetrajectory)
kill -INT ${LIO_PID} 2>/dev/null || true
sleep 3
wait ${LIO_PID} 2>/dev/null || true

# Copy internally saved trajectory to the expected output path
if [[ -f /out/Log/traj.txt ]]; then
    cp /out/Log/traj.txt /out/traj.csv
    TRAJ_LINES=$(wc -l < /out/traj.csv)
    echo "[OK] Trajectory: ${TRAJ_LINES} lines → /out/traj.csv"
else
    echo "[WARN] /out/Log/traj.txt not found — trajectory save failed"
fi

if [[ ( "${DATASET}" == "mid360" || "${DATASET}" == "indoor" ) && -n "${REPUB_PID:-}" ]]; then
    kill -INT "${REPUB_PID}" 2>/dev/null || true
fi
kill -INT ${ROSCORE_PID} 2>/dev/null || true
wait 2>/dev/null || true

echo "[DONE] Faster-LIO run complete (${DATASET}/${SEQ:-?})"
exit 0
