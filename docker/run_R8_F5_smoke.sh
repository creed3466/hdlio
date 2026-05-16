#!/bin/bash
# run_R8_F5_smoke.sh — T5.4-R8 F.5 single-seq smoke (3 critical seqs in parallel)
# Tests:
#   VI03 → STAGE_A LOCK class=CLASS_D rho_1≈0.9140  ATE ≤ 0.70m
#   DK01 → STAGE_A DEFER rho_1≈0.79, STAGE_B LOCK class=CLEAN_DENSE  ATE ≤ 0.18m
#   DY03 → STAGE_A DEFER rho_1, STAGE_B LOCK class=CLEAN_OUT  ATE ≤ 0.20m
# Frame-1 boundary at rho=0.80 (R-A risk if DK01 exceeds).
# Uses rate=3.0 (smoke/screening); F.6 uses rate=1.0 for canonical.
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-R8_F5_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
RATE="3.0"
IMAGE="tofslam:ros1"
CONFIG="avia_outdoor.yaml"
TIMEOUT_S=900  # 15min/seq @ rate=3.0

CONTAINERS=(tofslam_R8F5_1 tofslam_R8F5_2 tofslam_R8F5_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
SEQS=(Varying-illu03 Dark01 Dynamic03)

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

log "==== T5.4-R8 F.5 SMOKE ($(date)) ===="
log "Seqs: ${SEQS[*]} | rate=${RATE} | config=${CONFIG}"
log "Expectations:"
log "  Varying-illu03: STAGE_A LOCK CLASS_D rho_1≈0.914  ATE ≤ 0.70m"
log "  Dark01:         STAGE_A DEFER, STAGE_B LOCK CLEAN_DENSE  ATE ≤ 0.18m"
log "  Dynamic03:      STAGE_A DEFER, STAGE_B LOCK CLEAN_OUT  ATE ≤ 0.20m"

# Pre-build 3 containers
cleanup; sleep 1
for i in 0 1 2; do
  docker run -d --rm --init --name "${CONTAINERS[$i]}" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump:rw" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    "${IMAGE}" bash -lc "sleep infinity" > /dev/null
done
sleep 2

log "[Build] 3 containers..."
for i in 0 1 2; do
  docker exec "${CONTAINERS[$i]}" bash -lc \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
     catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
done
wait
log "[Build] Done."

# Launch 3 seqs in parallel
PIDS=()
for i in 0 1 2; do
  seq="${SEQS[$i]}"
  port="${PORTS[$i]}"
  out_dir="dump/${seq}"
  log "  Run ${seq} on port ${port}..."
  timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
    "TOFSLAM_DEBUG_DETERMINISM=0 bash /root/catkin_ws/docker/run_avia_exp.sh \
     ${CONFIG} ${seq} ${out_dir} ${port} ${RATE} 2>&1 | \
     tee /root/catkin_ws/${out_dir}/run.log" \
    > "${OUT_ROOT}/${seq}_stdout.log" 2>&1 &
  PIDS+=($!)
done

# Wait for all
for pid in "${PIDS[@]}"; do wait "$pid" || true; done
log "[Run] All 3 finished."

# Cleanup ROS processes
for c in "${CONTAINERS[@]}"; do
  docker exec "$c" bash -lc \
    "killall -9 tofslam_node rosbag rosout rosmaster roscore 2>/dev/null || true; \
     pkill -9 -f tofslam_node 2>/dev/null || true" || true
done

# Evaluate ATE
log ""
log "==== EVAL ATE ===="
for seq in "${SEQS[@]}"; do
  out_dir="${OUT_ROOT}/${seq}"
  if [[ -f "${out_dir}/trajectory.txt" ]]; then
    gt="/home/euntae/Project/dataset/ros1/surfel_data/ground_truth/${seq}.txt"
    if [[ -f "$gt" ]]; then
      ate=$(python3 docker/eval_ate_m3dgr.py --gt "$gt" --traj "${out_dir}/trajectory.txt" 2>&1 | grep -oE "ATE.*= [0-9.]+" | grep -oE "[0-9.]+" | head -1)
      log "  ${seq}: ATE=${ate}"
    else
      log "  ${seq}: no GT at $gt"
    fi
  else
    log "  ${seq}: NO TRAJECTORY (run failed)"
  fi
done

# Grep classifier SPDLOG events
log ""
log "==== CLASSIFIER EVENTS ===="
for seq in "${SEQS[@]}"; do
  stdout_log="${OUT_ROOT}/${seq}_stdout.log"
  if [[ -f "$stdout_log" ]]; then
    log "  ${seq}:"
    grep -E "STAGE_A LOCK|STAGE_A DEFER|STAGE_B LOCK|\[classifier\]" "$stdout_log" | head -5 | sed 's/^/    /' | tee -a "${LOG}"
  fi
done

log ""
log "==== DONE $(date) ===="
log "Output: ${OUT_ROOT}/"
