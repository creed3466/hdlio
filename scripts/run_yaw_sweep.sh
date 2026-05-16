#!/usr/bin/env bash
# run_yaw_sweep.sh — Run yaw sweep experiments on sbs_03
set -eo pipefail

SEQ="sbs_03"
BAG="/root/catkin_ws/data/ntu_viral/${SEQ}/${SEQ}.bag"
GT="/root/catkin_ws/data/ntu_viral/${SEQ}/ground_truth.csv"
BASE_DUMP="/root/catkin_ws/dump/yaw_sweep_sbs03"
RATE="${1:-3.0}"

# Yaw configs to test: label config_file
declare -A CONFIGS
CONFIGS[yawn2p0]="ros1_ntu_viral_v12_yawn2p0.yaml"
CONFIGS[yawn1p0]="ros1_ntu_viral_v12_yawn1p0.yaml"
CONFIGS[yawn0p5]="ros1_ntu_viral_v12_yawn0p5.yaml"
CONFIGS[yaw0p0]="ros1_ntu_viral_v12.yaml"
CONFIGS[yaw0p5]="ros1_ntu_viral_v12_yaw0p5.yaml"
CONFIGS[yaw1p0]="ros1_ntu_viral_v12_yaw1p0.yaml"
CONFIGS[yaw2p0]="ros1_ntu_viral_v12_yaw2p0.yaml"

# Order
ORDER=(yawn2p0 yawn1p0 yawn0p5 yaw0p0 yaw0p5 yaw1p0 yaw2p0)

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

for LABEL in "${ORDER[@]}"; do
    CONFIG="${CONFIGS[$LABEL]}"
    DUMP_DIR="${BASE_DUMP}/${LABEL}"
    TRAJ="${DUMP_DIR}/traj_est.csv"

    # Skip if already done
    if [ -f "${DUMP_DIR}/ate_result.txt" ]; then
        echo "=== SKIP $LABEL (already done) ==="
        continue
    fi

    echo "=== Running $LABEL (config=$CONFIG) ==="
    mkdir -p "$DUMP_DIR"

    # Kill everything
    killall -9 rosmaster roscore roslaunch rosout tofslam_node rosbag 2>/dev/null || true
    sleep 3

    # Start roscore
    roscore &
    sleep 2

    # Set sim time
    rosparam set /use_sim_time true

    # Set dump_path and trajectory_csv_path
    rosparam set /tofslam_node/dump_path "$DUMP_DIR"
    rosparam set /tofslam_node/trajectory_csv_path "$TRAJ"

    # Load config
    rosparam load "/root/catkin_ws/src/tof_slam/config/$CONFIG" /tofslam_node

    # Override dump_path and trajectory_csv_path (after config load)
    rosparam set /tofslam_node/dump_path "$DUMP_DIR"
    rosparam set /tofslam_node/trajectory_csv_path "$TRAJ"

    # Launch node
    rosrun tof_slam tofslam_node __name:=tofslam_node &>/dev/null &
    NODE_PID=$!
    sleep 5

    if ! kill -0 $NODE_PID 2>/dev/null; then
        echo "ERROR: Node crashed for $LABEL"
        continue
    fi

    # Play bag
    rosbag play --clock --rate "$RATE" "$BAG" 2>/dev/null

    # Wait for processing
    sleep 10

    # Kill node gracefully
    kill $NODE_PID 2>/dev/null || true
    wait $NODE_PID 2>/dev/null || true
    sleep 2

    # Check trajectory
    if [ ! -f "$TRAJ" ]; then
        echo "ERROR: No trajectory for $LABEL"
        # Try default path
        if [ -f "/root/tofslam_traj.csv" ]; then
            cp /root/tofslam_traj.csv "$TRAJ"
            echo "Copied from default path"
        fi
    fi

    # Evaluate ATE
    if [ -f "$TRAJ" ]; then
        python3 /root/catkin_ws/docker/eval_ate_ntu_viral.py "$TRAJ" "$GT" \
            --lever-arm > "${DUMP_DIR}/ate_result.txt" 2>&1 || echo "Eval failed for $LABEL"
        echo "=== $LABEL result ==="
        cat "${DUMP_DIR}/ate_result.txt"
    fi

    # Cleanup
    killall -9 rosmaster roscore roslaunch rosout tofslam_node rosbag 2>/dev/null || true
    sleep 2
done

echo ""
echo "=== SUMMARY ==="
for LABEL in "${ORDER[@]}"; do
    DUMP_DIR="${BASE_DUMP}/${LABEL}"
    if [ -f "${DUMP_DIR}/ate_result.txt" ]; then
        ATE=$(grep "rmse:" "${DUMP_DIR}/ate_result.txt" | awk '{print $2}')
        echo "$LABEL: ATE=$ATE"
    else
        echo "$LABEL: NO RESULT"
    fi
done
