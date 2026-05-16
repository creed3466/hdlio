#!/usr/bin/env bash
# In-container entrypoint for iG-LIO (prebuilt image variant).
#
# The baked headless.launch reads config filename from $IG_LAUNCH_CFG env
# (avia.yaml for Avia, ncd.yaml for NTU).
set -euo pipefail

source /opt/ros/noetic/setup.bash
CATKIN_WS="${CATKIN_WS:-/root/catkin_ws}"
IG_DIR="${CATKIN_WS}/src/ig_lio"

DATASET="${DATASET:-avia}"
case "${DATASET}" in
    avia)   CFG_BASENAME="avia.yaml" ;;
    mid360)       CFG_BASENAME="avia.yaml" ;;
    ntu)          CFG_BASENAME="ncd.yaml"  ;;
    *) echo "[ERROR] Unsupported DATASET: ${DATASET}"; exit 1 ;;
esac
export IG_LAUNCH_CFG="${CFG_BASENAME}"

if [[ ! -f "${CATKIN_WS}/devel/setup.bash" ]]; then
    echo "[ERROR] Prebuilt devel space missing — rebuild via build_algo.sh ig_lio"
    exit 1
fi

if [[ ! -f /config/params.yaml ]]; then
    echo "[ERROR] /config/params.yaml not mounted"; exit 1
fi
cp /config/params.yaml "${IG_DIR}/config/${CFG_BASENAME}"
echo "[INFO] Using config → ${IG_DIR}/config/${CFG_BASENAME}"

source "${CATKIN_WS}/devel/setup.bash"

ROSCORE_PORT=11311
export ROS_MASTER_URI="http://localhost:${ROSCORE_PORT}"
export ROS_HOSTNAME=localhost
roscore -p ${ROSCORE_PORT} &
ROSCORE_PID=$!
sleep 3

if [[ "${DATASET}" == "mid360" ]]; then
    # shellcheck disable=SC1091
    source /baselines_scripts/setup_mid360_republisher.sh
fi

roslaunch ig_lio headless.launch &
LIO_PID=$!
sleep 5

rosbag record -O /out/odom.bag /lio_odom /path &
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
    /out/odom.bag /lio_odom /out/traj.csv \
    || echo "[WARN] trajectory extraction failed"

kill -INT ${LIO_PID} 2>/dev/null || true
sleep 2
if [[ "${DATASET}" == "mid360" && -n "${REPUB_PID:-}" ]]; then
    kill -INT "${REPUB_PID}" 2>/dev/null || true
fi
kill -INT ${ROSCORE_PID} 2>/dev/null || true
wait 2>/dev/null || true

echo "[DONE] iG-LIO run complete (${DATASET}/${SEQ:-?})"
exit 0
