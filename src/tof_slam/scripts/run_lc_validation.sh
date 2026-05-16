#!/usr/bin/env bash
# Loop closure validation: run all 5 sequences with LC enabled
# Collects S2E, GICP debug, and LC logs into a single result file
set -eo pipefail

source /opt/ros/humble/setup.bash || true
source /root/ros2_ws/install/setup.bash || true

CONFIG=/root/ros2_ws/install/tof_slam/share/tof_slam/config/nrx_chj.yaml
RESULT_FILE="/root/ros2_ws/dump/lc_validation_results.txt"
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')

echo "=== Loop Closure Validation Results ===" > $RESULT_FILE
echo "Date: $(date)" >> $RESULT_FILE
echo "Config: updated yaml (submap=15, gap=1/15, max_rot=1.5, kf_trans=0.2)" >> $RESULT_FILE

# TF params
TF_ARGS="0.169950 -0.000090 0.060050 -0.500040 0.499920 -0.500022 0.500019 base_link front_spot_pcl"

declare -A BAGS=(
  [se1]="/root/dataset/test_0327_se_1/rest_20260327_053316"
  [se2]="/root/dataset/test_0327_se_2/rest_20260327_053435"
  [se3]="/root/dataset/test_0327_se_3/rest_20260327_053736"
  [se4]="/root/dataset/test_0327_se_4/rest_20260327_053918"
  [se5]="/root/dataset/test_0327_se_5/rest_20260327_054519"
)

run_one() {
    local SEQ=$1
    local BAG=$2
    local DUMP="/root/ros2_ws/dump/lc_val_${SEQ}"

    echo ""
    echo "=========================================="
    echo "  Running: $SEQ"
    echo "=========================================="

    # Clean kill any leftover processes
    pkill -9 -f tofslam_node 2>/dev/null || true
    pkill -9 -f static_transform_publisher 2>/dev/null || true
    pkill -9 -f "ros2 bag" 2>/dev/null || true
    sleep 3

    mkdir -p "$DUMP"

    # Start TF publisher
    ros2 run tf2_ros static_transform_publisher $TF_ARGS &>/dev/null &
    TF_PID=$!

    # Start SLAM node (uses yaml config defaults + override dump path)
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
        echo "[$SEQ] ERROR: Node crashed on startup" | tee -a $RESULT_FILE
        cat "$DUMP/node.log" >> $RESULT_FILE
        kill $TF_PID 2>/dev/null || true
        return
    fi

    # Play bag
    ros2 bag play "$BAG" --rate 0.5 --clock --read-ahead-queue-size 1000 2>/dev/null

    # Wait for processing to finish
    sleep 20

    # Graceful shutdown
    kill $NODE_PID 2>/dev/null || true
    wait $NODE_PID 2>/dev/null || true
    kill $TF_PID 2>/dev/null || true
    wait $TF_PID 2>/dev/null || true
    sleep 2

    # Collect results
    echo "" >> $RESULT_FILE
    echo "=== $SEQ ===" >> $RESULT_FILE

    # S2E
    echo "--- S2E ---" >> $RESULT_FILE
    python3 /root/ros2_ws/compute_s2e.py "$DUMP/traj.csv" >> $RESULT_FILE 2>&1 || echo "S2E FAILED" >> $RESULT_FILE

    # GICP Debug
    echo "--- GICP Debug ---" >> $RESULT_FILE
    if [ -f "$DUMP/gicp_debug.csv" ]; then
        LINES=$(wc -l < "$DUMP/gicp_debug.csv")
        echo "Entries: $((LINES - 1))" >> $RESULT_FILE
        cat "$DUMP/gicp_debug.csv" >> $RESULT_FILE
    else
        echo "No GICP CSV" >> $RESULT_FILE
    fi

    # LC Logs (summary)
    echo "--- LC Summary ---" >> $RESULT_FILE
    grep -c "LOOP ACCEPTED" "$DUMP/node.log" 2>/dev/null | xargs -I{} echo "Loops accepted: {}" >> $RESULT_FILE
    grep -c "LOOP REJECTED" "$DUMP/node.log" 2>/dev/null | xargs -I{} echo "Loops rejected: {}" >> $RESULT_FILE
    grep -c "PGO" "$DUMP/node.log" 2>/dev/null | xargs -I{} echo "PGO lines: {}" >> $RESULT_FILE
    grep "\[LC\]" "$DUMP/node.log" 2>/dev/null | tail -30 >> $RESULT_FILE || echo "No LC logs" >> $RESULT_FILE

    echo "[$SEQ] Done"
}

for seq in se1 se2 se3 se4 se5; do
    run_one "$seq" "${BAGS[$seq]}"
done

echo "" >> $RESULT_FILE
echo "=== ALL DONE ===" >> $RESULT_FILE
echo "DONE" > /root/ros2_ws/dump/lc_val_done.flag
echo ""
echo "Results written to: $RESULT_FILE"
