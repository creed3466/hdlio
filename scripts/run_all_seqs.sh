#!/usr/bin/env bash
# Run inside the container: docker exec tofslam_ros1 bash /root/catkin_ws/scripts/run_all_seqs.sh <config> <experiment>
set -euo pipefail

CONFIG="${1:?Usage: $0 <config_name> <experiment_name> [sequences]}"
EXPERIMENT="${2:?Usage: $0 <config_name> <experiment_name> [sequences]}"
SEQUENCES="${3:-eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03}"

DATA_ROOT="/root/catkin_ws/data/ntu_viral"
DUMP_ROOT="/root/catkin_ws/dump/${EXPERIMENT}"
EVAL_SCRIPT="/root/catkin_ws/docker/eval_ate_ntu_viral.py"

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

cleanup() {
  killall -9 rosmaster roscore roslaunch rosout tofslam_node rosbag 2>/dev/null || true
  sleep 2
}

run_seq() {
  local seq="$1"
  local out_dir="${DUMP_ROOT}/${seq}"
  mkdir -p "${out_dir}"

  echo "====== [$(date +%H:%M:%S)] ${seq} ======"

  cleanup

  # Start roscore
  roscore &
  sleep 3

  # Start node
  roslaunch tof_slam tofslam_ntu_viral.launch config:="${CONFIG}" &
  sleep 5

  # Verify
  if ! pgrep -f tofslam_node > /dev/null 2>&1; then
    echo "[ERROR] tofslam_node not running for ${seq}"
    cleanup
    return 1
  fi

  # Play bag
  echo "[PLAY] ${seq}"
  rosbag play --clock "${DATA_ROOT}/${seq}/${seq}.bag" 2>&1 | tail -1

  sleep 10

  # Copy trajectory
  cp /root/tofslam_traj.csv "${out_dir}/traj_est.csv" 2>/dev/null || {
    echo "[ERROR] No trajectory for ${seq}"
    cleanup
    return 1
  }

  local nlines=$(wc -l < "${out_dir}/traj_est.csv")
  echo "[TRAJ] ${nlines} lines"

  # Evaluate
  python3 "${EVAL_SCRIPT}" "${out_dir}/traj_est.csv" "${DATA_ROOT}/${seq}/ground_truth.csv" --output_dir "${out_dir}" || echo "[WARN] eval failed for ${seq}"

  cleanup
}

# Main
echo "Experiment: ${EXPERIMENT}  Config: ${CONFIG}"
mkdir -p "${DUMP_ROOT}"

for seq in ${SEQUENCES}; do
  run_seq "${seq}" || echo "[FAIL] ${seq}, continuing..."
done

# Summary
echo ""
echo "====== SUMMARY ======"
SUMMARY="${DUMP_ROOT}/summary.csv"
echo "sequence,ate_rmse,ate_mean,ate_median,ate_std,ate_max,matches" > "${SUMMARY}"
for seq in ${SEQUENCES}; do
  ate="${DUMP_ROOT}/${seq}/ate_result.txt"
  if [ -f "${ate}" ]; then
    rmse=$(grep "^rmse:" "${ate}" | awk '{print $2}')
    mean=$(grep "^mean:" "${ate}" | awk '{print $2}')
    median=$(grep "^median:" "${ate}" | awk '{print $2}')
    std=$(grep "^std:" "${ate}" | awk '{print $2}')
    max=$(grep "^max:" "${ate}" | awk '{print $2}')
    matches=$(grep "^n_matches:" "${ate}" | awk '{print $2}')
    echo "${seq},${rmse},${mean},${median},${std},${max},${matches}"
    echo "${seq},${rmse},${mean},${median},${std},${max},${matches}" >> "${SUMMARY}"
  else
    echo "${seq},NA,NA,NA,NA,NA,0"
    echo "${seq},NA,NA,NA,NA,NA,0" >> "${SUMMARY}"
  fi
done

touch "${DUMP_ROOT}/DONE"
echo ""
echo "[DONE] Results in ${DUMP_ROOT}"
