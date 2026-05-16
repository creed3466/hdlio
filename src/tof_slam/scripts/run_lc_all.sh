#!/bin/bash
# Run loop closure test on all 5 sequences
# Results written to /root/ros2_ws/dump/lc_results.txt

source /opt/ros/humble/setup.bash || true
source /root/ros2_ws/install/setup.bash || true

RESULT_FILE="/root/ros2_ws/dump/lc_results.txt"
echo "=== Loop Closure Test Results ===" > $RESULT_FILE
echo "Date: $(date)" >> $RESULT_FILE

CONFIG=/root/ros2_ws/install/tof_slam/share/tof_slam/config/nrx_chj.yaml

run_one() {
    local SEQ=$1
    local BAG=$2

    local DUMP="/root/ros2_ws/dump/lc_${SEQ}"
    mkdir -p "$DUMP"

    pkill -f tofslam_node 2>/dev/null || true
    pkill -f static_transform_publisher 2>/dev/null || true
    sleep 2

    # TF
    ros2 run tf2_ros static_transform_publisher \
      0.169950 -0.000090 0.060050 -0.500040 0.499920 -0.500022 0.500019 \
      base_link front_spot_pcl &>/dev/null &

    # Node
    ros2 run tof_slam tofslam_node --ros-args \
      --params-file "$CONFIG" \
      -p dump_path:="$DUMP" \
      -p trajectory_csv_path:="$DUMP/traj.csv" \
      -p use_sim_time:=true \
      -p enable_loop_closure:=true \
      -p lc_submap_keyframe_count:=15 \
      -p lc_loop_min_submap_gap:=1 \
      -p lc_loop_min_keyframe_gap:=15 \
      -p lc_keyframe_trans_thresh:=0.2 \
      -p lc_enable_debug_log:=true &>"$DUMP/node.log" &
    NODE_PID=$!

    sleep 3

    # Play bag
    ros2 bag play "$BAG" --rate 0.5 --clock --read-ahead-queue-size 1000 2>/dev/null

    sleep 15

    kill $NODE_PID 2>/dev/null || true
    wait $NODE_PID 2>/dev/null || true
    pkill -f static_transform_publisher 2>/dev/null || true
    sleep 1

    echo "" >> $RESULT_FILE
    echo "=== $SEQ ===" >> $RESULT_FILE
    python3 /root/ros2_ws/compute_s2e.py "$DUMP/traj.csv" >> $RESULT_FILE 2>&1 || echo "FAILED" >> $RESULT_FILE

    echo "--- GICP Debug ---" >> $RESULT_FILE
    if [ -f "$DUMP/gicp_debug.csv" ]; then
        cat "$DUMP/gicp_debug.csv" >> $RESULT_FILE
    else
        echo "No GICP CSV" >> $RESULT_FILE
    fi

    echo "--- LC Logs ---" >> $RESULT_FILE
    grep "\[LC\]" "$DUMP/node.log" >> $RESULT_FILE 2>/dev/null || echo "No LC logs" >> $RESULT_FILE
}

run_one se1 /root/dataset/test_0327_se_1/rest_20260327_053316
run_one se2 /root/dataset/test_0327_se_2/rest_20260327_053435
run_one se3 /root/dataset/test_0327_se_3/rest_20260327_053736
run_one se4 /root/dataset/test_0327_se_4/rest_20260327_053918
run_one se5 /root/dataset/test_0327_se_5/rest_20260327_054519

echo "" >> $RESULT_FILE
echo "=== ALL DONE ===" >> $RESULT_FILE
echo "DONE" > /root/ros2_ws/dump/lc_done.flag
