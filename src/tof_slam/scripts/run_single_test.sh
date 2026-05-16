#!/usr/bin/env bash
# Usage: run_single_test.sh <seq_name> <bag_path> <enable_lc>
set -eo pipefail

SEQ=$1
BAG=$2
LC_ENABLE=${3:-true}

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

DUMP="/root/ros2_ws/dump/lc_${SEQ}_${LC_ENABLE}"
mkdir -p "$DUMP"

pkill -f tofslam_node 2>/dev/null || true
pkill -f static_transform_publisher 2>/dev/null || true
sleep 1

# TF
ros2 run tf2_ros static_transform_publisher \
  0.169950 -0.000090 0.060050 -0.500040 0.499920 -0.500022 0.500019 \
  base_link front_spot_pcl &>/dev/null &
TF_PID=$!

# Node
CONFIG=/root/ros2_ws/install/tof_slam/share/tof_slam/config/nrx_chj.yaml
ros2 run tof_slam tofslam_node --ros-args \
  --params-file "$CONFIG" \
  -p dump_path:="$DUMP" \
  -p trajectory_csv_path:="$DUMP/traj.csv" \
  -p use_sim_time:=true \
  -p enable_loop_closure:="${LC_ENABLE}" \
  -p lc_enable_debug_log:=true &>"$DUMP/node.log" &
NODE_PID=$!

sleep 3

# Verify node is running
if ! kill -0 $NODE_PID 2>/dev/null; then
    echo "ERROR: Node crashed on startup"
    cat "$DUMP/node.log"
    exit 1
fi

# Play bag
ros2 bag play "$BAG" --rate 0.5 --clock --read-ahead-queue-size 1000 2>/dev/null

# Wait for drain
sleep 15

# Graceful shutdown
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true
kill $TF_PID 2>/dev/null || true
wait $TF_PID 2>/dev/null || true

# Results
echo "=== [$SEQ lc=$LC_ENABLE] S2E ==="
python3 /root/ros2_ws/compute_s2e.py "$DUMP/traj.csv" 2>&1 || echo "FAILED"

echo ""
echo "=== GICP Debug ==="
if [ -f "$DUMP/gicp_debug.csv" ]; then
    LINES=$(wc -l < "$DUMP/gicp_debug.csv")
    echo "Entries: $((LINES - 1))"
    cat "$DUMP/gicp_debug.csv"
else
    echo "No GICP debug CSV"
fi

echo ""
echo "=== Node Log (last 10 lines with LC) ==="
grep -i "\[LC\]" "$DUMP/node.log" 2>/dev/null | tail -20 || echo "No LC logs"
