#!/usr/bin/env bash
# Map save validation: run se_1 with LC enabled, then call save_map service
set -eo pipefail

source /opt/ros/humble/setup.bash || true
source /root/ros2_ws/install/setup.bash || true

CONFIG=/root/ros2_ws/install/tof_slam/share/tof_slam/config/nrx_chj.yaml
DUMP="/root/ros2_ws/dump/map_save_test"
BAG="/root/dataset/test_0327_se_1/rest_20260327_053316"
TF_ARGS="0.169950 -0.000090 0.060050 -0.500040 0.499920 -0.500022 0.500019 base_link front_spot_pcl"

echo "=========================================="
echo "  Map Save Validation Test (se_1)"
echo "=========================================="

# Clean kill any leftover processes
pkill -9 -f tofslam_node 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f "ros2 bag" 2>/dev/null || true
sleep 2

mkdir -p "$DUMP"

# Start TF publisher
ros2 run tf2_ros static_transform_publisher $TF_ARGS &>/dev/null &
TF_PID=$!

# Start SLAM node with LC enabled
ros2 run tof_slam tofslam_node --ros-args \
  --params-file "$CONFIG" \
  -p dump_path:="$DUMP" \
  -p trajectory_csv_path:="$DUMP/traj.csv" \
  -p use_sim_time:=true \
  -p enable_loop_closure:=true \
  -p lc_enable_debug_log:=true &>"$DUMP/node.log" &
NODE_PID=$!

sleep 4

# Verify node started
if ! kill -0 $NODE_PID 2>/dev/null; then
    echo "ERROR: Node crashed on startup"
    cat "$DUMP/node.log"
    kill $TF_PID 2>/dev/null || true
    exit 1
fi

echo "Node started (PID=$NODE_PID). Playing bag..."

# Play bag
ros2 bag play "$BAG" --rate 0.5 --clock --read-ahead-queue-size 1000 2>/dev/null

echo "Bag playback done. Waiting for processing..."
sleep 15

# Check service availability
echo ""
echo "--- Checking save_map service ---"
ros2 service list 2>/dev/null | grep save_map || echo "WARNING: save_map service not found"

# Call save_map service
echo ""
echo "--- Calling save_map service ---"
ros2 service call /tofslam_node/save_map std_srvs/srv/Trigger "{}" 2>&1 | tee "$DUMP/save_map_response.txt"

sleep 3

# Check results
echo ""
echo "--- Save Results ---"
if [ -d "$DUMP/saved_map" ]; then
    echo "saved_map directory exists"
    ls -la "$DUMP/saved_map/"
    echo ""
    if [ -f "$DUMP/saved_map/global_map.pcd" ]; then
        SIZE=$(stat -c%s "$DUMP/saved_map/global_map.pcd" 2>/dev/null || echo "0")
        echo "global_map.pcd: ${SIZE} bytes"
    fi
    if [ -f "$DUMP/saved_map/pose_graph.bin" ]; then
        SIZE=$(stat -c%s "$DUMP/saved_map/pose_graph.bin" 2>/dev/null || echo "0")
        echo "pose_graph.bin: ${SIZE} bytes"
    fi
    if [ -f "$DUMP/saved_map/metadata.yaml" ]; then
        echo ""
        echo "--- metadata.yaml ---"
        cat "$DUMP/saved_map/metadata.yaml"
    fi
    if [ -d "$DUMP/saved_map/submaps" ]; then
        echo ""
        echo "--- Submaps ---"
        ls -la "$DUMP/saved_map/submaps/"
    fi
else
    echo "ERROR: saved_map directory not created"
fi

# S2E
echo ""
echo "--- S2E ---"
python3 /root/ros2_ws/compute_s2e.py "$DUMP/traj.csv" 2>&1 || echo "S2E FAILED"

# Graceful shutdown
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true
kill $TF_PID 2>/dev/null || true
wait $TF_PID 2>/dev/null || true

echo ""
echo "=== DONE ==="
