#!/usr/bin/env bash
# OGM validation: run se_1 with LC+OGM, verify /map topic + map save PNG/YAML
set -eo pipefail

source /opt/ros/humble/setup.bash || true
source /root/ros2_ws/install/setup.bash || true

CONFIG=/root/ros2_ws/install/tof_slam/share/tof_slam/config/nrx_chj.yaml
DUMP="/root/ros2_ws/dump/ogm_test"
BAG="/root/dataset/test_0327_se_1/rest_20260327_053316"
TF_ARGS="0.169950 -0.000090 0.060050 -0.500040 0.499920 -0.500022 0.500019 base_link front_spot_pcl"

echo "=========================================="
echo "  OGM Validation Test (se_1)"
echo "=========================================="

pkill -9 -f tofslam_node 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f "ros2 bag" 2>/dev/null || true
sleep 2

mkdir -p "$DUMP"

# Start TF
ros2 run tf2_ros static_transform_publisher $TF_ARGS &>/dev/null &
TF_PID=$!

# Start SLAM node
ros2 run tof_slam tofslam_node --ros-args \
  --params-file "$CONFIG" \
  -p dump_path:="$DUMP" \
  -p trajectory_csv_path:="$DUMP/traj.csv" \
  -p use_sim_time:=true \
  -p enable_loop_closure:=true \
  -p ogm_enable:=true \
  -p lc_enable_debug_log:=false &>"$DUMP/node.log" &
NODE_PID=$!

sleep 4

if ! kill -0 $NODE_PID 2>/dev/null; then
    echo "ERROR: Node crashed"
    cat "$DUMP/node.log" | tail -30
    kill $TF_PID 2>/dev/null || true
    exit 1
fi

echo "Node started. Playing bag..."

# Play bag
ros2 bag play "$BAG" --rate 0.5 --clock --read-ahead-queue-size 1000 2>/dev/null

echo "Bag done. Waiting for processing..."
sleep 15

# Check /map topic
echo ""
echo "--- /map topic check ---"
ros2 topic info /map 2>/dev/null || echo "WARNING: /map topic not found"
echo ""
echo "--- /map echo (1 msg, 2s timeout) ---"
timeout 3 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once 2>/dev/null | head -20 || echo "(no message received or timeout)"

# Call save_map
echo ""
echo "--- Calling save_map service ---"
ros2 service call /tofslam_node/save_map std_srvs/srv/Trigger "{}" 2>&1

sleep 2

# Check OGM files
echo ""
echo "--- Saved Map Files ---"
if [ -d "$DUMP/saved_map" ]; then
    ls -la "$DUMP/saved_map/"
    echo ""
    if [ -f "$DUMP/saved_map/map.png" ]; then
        SIZE=$(stat -c%s "$DUMP/saved_map/map.png" 2>/dev/null || echo "0")
        echo "map.png: ${SIZE} bytes"
        # Get image dimensions with python
        python3 -c "
from PIL import Image
im = Image.open('$DUMP/saved_map/map.png')
print(f'  Dimensions: {im.size[0]}x{im.size[1]}')
print(f'  Mode: {im.mode}')
" 2>/dev/null || echo "  (PIL not available for dimension check)"
    else
        echo "ERROR: map.png NOT found"
    fi
    if [ -f "$DUMP/saved_map/map.yaml" ]; then
        echo ""
        echo "--- map.yaml (Nav2 format) ---"
        cat "$DUMP/saved_map/map.yaml"
    else
        echo "ERROR: map.yaml NOT found"
    fi
else
    echo "ERROR: saved_map directory not created"
fi

# OGM publish count from logs
echo ""
echo "--- OGM Log Summary ---"
grep -c "\[OGM\]" "$DUMP/node.log" 2>/dev/null | xargs -I{} echo "OGM log lines: {}" || echo "No OGM logs"
grep "\[OGM\]" "$DUMP/node.log" 2>/dev/null | tail -5 || true

# S2E
echo ""
echo "--- S2E ---"
python3 /root/ros2_ws/compute_s2e.py "$DUMP/traj.csv" 2>&1 || echo "S2E FAILED"

# Shutdown
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true
kill $TF_PID 2>/dev/null || true
wait $TF_PID 2>/dev/null || true

echo ""
echo "=== DONE ==="
