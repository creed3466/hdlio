#!/bin/bash
# run_ntu_exp.sh — Container-internal: run single NTU VIRAL sequence with given config
# Usage: run_ntu_exp.sh <CONFIG> <SEQ> <OUT_DIR> [PORT] [RATE]
#
# Example: run_ntu_exp.sh ros1_ntu_viral.yaml eee_01 /root/catkin_ws/dump/eee_01 11311 1.0

set -e

CONFIG="${1:?Usage: run_ntu_exp.sh <CONFIG> <SEQ> <OUT_DIR> [PORT] [RATE]}"
SEQ="${2:?Missing SEQ}"
OUT_DIR="${3:?Missing OUT_DIR}"
PORT="${4:-11311}"
RATE="${5:-1.0}"

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
export ROS_MASTER_URI="http://localhost:${PORT}"

DATA_DIR="/root/catkin_ws/data/ntu_viral"

mkdir -p "${OUT_DIR}"

# Cleanup
# Cleanup — kill all ROS processes, then verify port is free.
# Note: fuser/ss/lsof/netstat are NOT available in the container.
# Use repeated killall + /proc/net/tcp fallback for port release.
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

rm -f /root/tofslam_traj.csv /root/tofslam_timing.csv /root/tofslam_diagnostics.csv

echo "========================================="
echo "  Config: ${CONFIG}"
echo "  Seq:    ${SEQ}"
echo "  Port:   ${PORT}"
echo "========================================="

# Start roscore
roscore -p ${PORT} &
ROSCORE_PID=$!
sleep 6

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

# Launch SLAM node with NTU launch file
roslaunch tof_slam tofslam_ntu_viral.launch config:=${CONFIG} &
LAUNCH_PID=$!
sleep 3

# Subscriber-ready gate
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

# Play bag
BAG="${DATA_DIR}/${SEQ}/${SEQ}.bag"
echo "Playing ${SEQ} at rate ${RATE} (delay=3.0s)..."
rosbag play "${BAG}" --clock -r ${RATE} --delay 3.0 2>&1 | tail -3
sleep 5

# Shutdown
rosnode kill -a 2>/dev/null || true
sleep 2
kill -9 $LAUNCH_PID 2>/dev/null || true
kill -9 $ROSCORE_PID 2>/dev/null || true

# Collect results
if [ -f /root/tofslam_traj.csv ] && [ $(wc -l < /root/tofslam_traj.csv) -gt 10 ]; then
    cp /root/tofslam_traj.csv "${OUT_DIR}/traj_est.csv"
    [ -f /root/tofslam_timing.csv ] && cp /root/tofslam_timing.csv "${OUT_DIR}/timing.csv"

    NLINES=$(wc -l < "${OUT_DIR}/traj_est.csv")
    echo "Trajectory: ${NLINES} lines"

    # Run ATE evaluation
    GT="${DATA_DIR}/${SEQ}/ground_truth.csv"
    pip3 install scipy numpy -q 2>/dev/null || true
    python3 /root/catkin_ws/docker/eval_ate_ntu_viral.py \
        "${OUT_DIR}/traj_est.csv" \
        "${GT}" \
        --output_dir "${OUT_DIR}/" 2>&1
else
    echo "ERROR: No trajectory for ${SEQ}!"
fi

echo "=== Done: ${CONFIG} / ${SEQ} ==="
