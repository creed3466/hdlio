#!/bin/bash
# run_avia_exp.sh — Container-internal: run single Avia sequence with given config
# Usage: run_avia_exp.sh <CONFIG> <SEQ> <OUT_DIR> [PORT] [RATE]
#
# Example: run_avia_exp.sh exp_avia_pko.yaml Dark01 /root/catkin_ws/dump/Dark01 11311 1.0
# If TOFSLAM_DEBUG_DETERMINISM=1 is exported, debug CSVs are copied into OUT_DIR.

set -e

CONFIG="${1:?Usage: run_avia_exp.sh <CONFIG> <SEQ> <OUT_DIR> [PORT] [RATE]}"
SEQ="${2:?Missing SEQ}"
OUT_DIR="${3:?Missing OUT_DIR}"
PORT="${4:-11311}"
RATE="${5:-3.0}"

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
export ROS_MASTER_URI="http://localhost:${PORT}"

DATA_DIR="/root/catkin_ws/data/m3dgr_surfel"
GT_DIR="${DATA_DIR}/ground_truth"

mkdir -p "${OUT_DIR}"

# Cleanup — kill all ROS processes, then verify port is free.
# Note: fuser/ss/lsof/netstat are NOT available in the container.
# Use repeated killall + sufficient sleep for port release.
killall -9 rosmaster roscore roslaunch tofslam_node rosbag rosout 2>/dev/null || true
sleep 3
# Second sweep: catch any zombies or respawned processes
killall -9 rosmaster roscore roslaunch tofslam_node rosbag rosout 2>/dev/null || true
pkill -9 -f "rosmaster\|roscore\|tofslam_node" 2>/dev/null || true
sleep 3
# Port-level kill (container has no fuser — try /proc/net/tcp fallback)
PORT_HEX=$(printf '%04X' ${PORT})
for _pid in $(awk -v ph="${PORT_HEX}" '$2 ~ ":"ph"$" {print $10}' /proc/net/tcp 2>/dev/null | cut -d/ -f1 | sort -u); do
  [ -n "$_pid" ] && [ "$_pid" != "0" ] && kill -9 "$_pid" 2>/dev/null || true
done
sleep 2

rm -f /root/tofslam_traj.csv /root/tofslam_timing.csv /root/tofslam_diagnostics.csv \
      /root/tofslam_debug_imu.csv /root/tofslam_debug_state.csv \
      /root/char_*.csv

echo "========================================="
echo "  Config: ${CONFIG}"
echo "  Seq:    ${SEQ}"
echo "  Port:   ${PORT}"
echo "========================================="

# Start roscore
roscore -p ${PORT} &
ROSCORE_PID=$!
sleep 6

# Verify master
if ! rostopic list >/dev/null 2>&1; then
    echo "WARN: roscore not ready, retrying..."
    kill -9 $ROSCORE_PID 2>/dev/null || true
    fuser -k ${PORT}/tcp 2>/dev/null || true
    sleep 2
    roscore -p ${PORT} &
    ROSCORE_PID=$!
    sleep 8
fi

rosparam set /use_sim_time true 2>/dev/null || true

# Launch SLAM node
roslaunch tof_slam tofslam_m3dgr.launch config:=${CONFIG} &
LAUNCH_PID=$!
sleep 3

# --- Fix B v2: subscriber-ready gate + bag-play delay ---------------------
# Step 1: wait until tofslam_node is registered as a subscriber on both
#         topics (master-level visibility).
# Step 2: start rosbag play with --delay 3.0, which forces a 3-second wait
#         between publisher advertise and actual publication.  This
#         guarantees the ROS TCP handshake between bag player (publisher)
#         and tofslam_node (subscriber) has fully completed before any
#         IMU/LiDAR message is emitted — eliminating the startup message-loss
#         race that v1 could not catch.  See dump/avia_debug_dark01_fixAB_*
#         (Fix B v1 failed because rostopic-info visibility ≠ TCP flow).
CFG_FILE="/root/catkin_ws/src/tof_slam/config/${CONFIG}"
IMU_TOPIC=$(awk '/^imu_topic:/ {gsub(/["'"'"']/, "", $2); print $2; exit}' "${CFG_FILE}")
LIDAR_TOPIC=$(awk '/^lidar_topic:/ {gsub(/["'"'"']/, "", $2); print $2; exit}' "${CFG_FILE}")
echo "[ready-gate] waiting for subscribers on ${IMU_TOPIC} and ${LIDAR_TOPIC}..."
for i in $(seq 1 60); do
  imu_sub=$(rostopic info "${IMU_TOPIC}" 2>/dev/null | \
    awk '/Subscribers:/{f=1;next} /Publishers:/{f=0} f && /\*/ {print}')
  lidar_sub=$(rostopic info "${LIDAR_TOPIC}" 2>/dev/null | \
    awk '/Subscribers:/{f=1;next} /Publishers:/{f=0} f && /\*/ {print}')
  if [ -n "$imu_sub" ] && [ -n "$lidar_sub" ]; then
    echo "[ready-gate] subscribers attached after ${i} polls"
    break
  fi
  sleep 0.2
done
# --------------------------------------------------------------------------

# Play bag — --delay 3.0 ensures TCP handshake completes before publish begins.
echo "Playing ${SEQ} at rate ${RATE} (delay=3.0s)..."
rosbag play "${DATA_DIR}/${SEQ}.bag" --clock -r ${RATE} --delay 3.0 2>&1 | tail -3
sleep 5

# Shutdown
rosnode kill -a 2>/dev/null || true
sleep 2
kill -9 $LAUNCH_PID 2>/dev/null || true
kill -9 $ROSCORE_PID 2>/dev/null || true

# Explicit grandchild reaping (prevents container hang from tofslam_node
# orphan holding stdout/stderr fd open after roslaunch parent dies).
killall -9 rosmaster roscore roslaunch tofslam_node rosbag rosout 2>/dev/null || true
pkill -9 -f "tofslam_node|roslaunch|rosbag play|roscore|rosmaster|rosout" 2>/dev/null || true
sleep 1

# Collect results
if [ -f /root/tofslam_traj.csv ] && [ $(wc -l < /root/tofslam_traj.csv) -gt 10 ]; then
    cp /root/tofslam_traj.csv "${OUT_DIR}/traj.csv"
    [ -f /root/tofslam_timing.csv ] && cp /root/tofslam_timing.csv "${OUT_DIR}/timing.csv"
    [ -f /root/tofslam_diagnostics.csv ] && cp /root/tofslam_diagnostics.csv "${OUT_DIR}/diagnostics.csv"
    [ -f /root/tofslam_debug_imu.csv ]   && cp /root/tofslam_debug_imu.csv   "${OUT_DIR}/debug_imu.csv"
    [ -f /root/tofslam_debug_state.csv ] && cp /root/tofslam_debug_state.csv "${OUT_DIR}/debug_state.csv"
    # Characterization CSVs (Proposal 0 instrumentation)
    for cf in /root/char_*.csv; do
      [ -f "$cf" ] && cp "$cf" "${OUT_DIR}/"
    done

    NLINES=$(wc -l < "${OUT_DIR}/traj.csv")
    echo "Trajectory: ${NLINES} lines"

    # Run ATE evaluation
    pip3 install scipy numpy -q 2>/dev/null || true
    python3 /root/catkin_ws/docker/eval_ate_m3dgr.py \
        "${OUT_DIR}/traj.csv" \
        "${GT_DIR}/${SEQ}.txt" \
        --output_dir "${OUT_DIR}/" 2>&1
else
    echo "ERROR: No trajectory for ${SEQ}!"
fi

echo "=== Done: ${CONFIG} / ${SEQ} ==="
