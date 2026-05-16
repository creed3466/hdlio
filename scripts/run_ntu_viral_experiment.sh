#!/usr/bin/env bash
# run_ntu_viral_experiment.sh — Run TofSLAM on all NTU VIRAL sequences and evaluate ATE
# Usage: bash scripts/run_ntu_viral_experiment.sh <config_name> <experiment_name>
# Example: bash scripts/run_ntu_viral_experiment.sh ros1_ntu_viral_v22.yaml ntu_viral_v22

set -euo pipefail

CONFIG="${1:?Usage: $0 <config_name> <experiment_name>}"
EXPERIMENT="${2:?Usage: $0 <config_name> <experiment_name>}"

CONTAINER="tofslam_ros1"
DATA_ROOT="/root/catkin_ws/data/ntu_viral"
DUMP_HOST="/home/euntae/Project/TofSLAM_v1.0/dump/${EXPERIMENT}"
DUMP_CONTAINER="/root/catkin_ws/dump/${EXPERIMENT}"
EVAL_SCRIPT="/home/euntae/Project/TofSLAM_v1.0/docker/eval_ate_ntu_viral.py"

SEQUENCES="eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03"

# Create output directory
mkdir -p "${DUMP_HOST}"

cleanup_ros() {
  docker exec ${CONTAINER} bash -c 'killall -9 rosmaster roscore roslaunch rosout tofslam_node rosbag 2>/dev/null || true'
  sleep 2
}

run_sequence() {
  local seq="$1"
  local seq_dir="${DUMP_HOST}/${seq}"
  local seq_dir_container="${DUMP_CONTAINER}/${seq}"

  echo "======================================================================"
  echo "[$(date +%H:%M:%S)] Running sequence: ${seq}"
  echo "======================================================================"

  mkdir -p "${seq_dir}"

  # Clean up
  cleanup_ros

  # Start roscore
  docker exec -d ${CONTAINER} bash -c 'source /opt/ros/noetic/setup.bash && roscore'
  sleep 3

  # Start TofSLAM node
  docker exec -d ${CONTAINER} bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch tof_slam tofslam_ntu_viral.launch config:=${CONFIG}"
  sleep 5

  # Verify node is running
  if ! docker exec ${CONTAINER} bash -c 'pgrep -f tofslam_node > /dev/null 2>&1'; then
    echo "[ERROR] tofslam_node failed to start for ${seq}"
    return 1
  fi
  echo "[OK] tofslam_node started"

  # Play rosbag (rate=3.0 per requirements.md)
  echo "[PLAY] ${seq}.bag (rate=3.0)"
  docker exec ${CONTAINER} bash -c "source /opt/ros/noetic/setup.bash && rosbag play --clock --rate 3.0 ${DATA_ROOT}/${seq}/${seq}.bag --topics /imu/imu /os1_cloud_node1/points" 2>&1 | tail -5

  # Wait for node to finish processing
  echo "[WAIT] Waiting for processing to complete..."
  sleep 10

  # Copy trajectory
  docker exec ${CONTAINER} bash -c "cp /root/tofslam_traj.csv ${seq_dir_container}/traj_est.csv 2>/dev/null || true"

  # Check if trajectory was generated
  if [ ! -f "${seq_dir}/traj_est.csv" ]; then
    echo "[ERROR] No trajectory file for ${seq}"
    cleanup_ros
    return 1
  fi

  local nlines=$(wc -l < "${seq_dir}/traj_est.csv")
  echo "[OK] Trajectory: ${nlines} lines"

  # Evaluate ATE
  echo "[EVAL] Computing ATE..."
  docker exec ${CONTAINER} bash -c "python3 /root/catkin_ws/docker/eval_ate_ntu_viral.py ${seq_dir_container}/traj_est.csv ${DATA_ROOT}/${seq}/ground_truth.csv --output_dir ${seq_dir_container}" 2>&1

  # Cleanup
  cleanup_ros

  echo "[DONE] ${seq} complete"
  echo ""
}

# Summary generation
generate_summary() {
  echo "Generating summary..."
  local summary="${DUMP_HOST}/summary.csv"
  echo "sequence,ate_rmse,ate_mean,ate_median,ate_std,ate_max,matches" > "${summary}"

  for seq in ${SEQUENCES}; do
    local ate_file="${DUMP_HOST}/${seq}/ate_result.txt"
    if [ -f "${ate_file}" ]; then
      local rmse=$(grep "^rmse:" "${ate_file}" | awk '{print $2}')
      local mean=$(grep "^mean:" "${ate_file}" | awk '{print $2}')
      local median=$(grep "^median:" "${ate_file}" | awk '{print $2}')
      local std=$(grep "^std:" "${ate_file}" | awk '{print $2}')
      local max=$(grep "^max:" "${ate_file}" | awk '{print $2}')
      local matches=$(grep "^n_matches:" "${ate_file}" | awk '{print $2}')
      echo "${seq},${rmse},${mean},${median},${std},${max},${matches}" >> "${summary}"
    else
      echo "${seq},NA,NA,NA,NA,NA,0" >> "${summary}"
    fi
  done

  echo ""
  echo "======================================================================"
  echo "SUMMARY: ${EXPERIMENT}"
  echo "======================================================================"
  cat "${summary}"
  echo ""

  # Compute average ATE
  python3 -c "
import csv
with open('${summary}') as f:
    reader = csv.DictReader(f)
    vals = [float(r['ate_rmse']) for r in reader if r['ate_rmse'] != 'NA']
if vals:
    print(f'Average ATE RMSE: {sum(vals)/len(vals):.4f}m ({len(vals)}/9 sequences)')
    print(f'Min: {min(vals):.4f}m  Max: {max(vals):.4f}m')
else:
    print('No valid results')
"
}

# Main
echo "======================================================================"
echo "TofSLAM NTU VIRAL Experiment: ${EXPERIMENT}"
echo "Config: ${CONFIG}"
echo "Started: $(date)"
echo "======================================================================"

# Initial cleanup
cleanup_ros

# Run all sequences
for seq in ${SEQUENCES}; do
  run_sequence "${seq}" || echo "[WARN] ${seq} failed, continuing..."
done

# Generate summary
generate_summary

# Mark complete
touch "${DUMP_HOST}/DONE"
echo ""
echo "[ALL DONE] Results in: ${DUMP_HOST}"
