#!/bin/bash
# run_R8_F5_RA_measure.sh — R-A measurement: collect frame=1 (degen, rho_1) for 6 outdoor seqs
# Already have VI03/DK01/DY03 from prior F.5 run.
# Now measure DK02, DY04, OC03, OC04, VI04, VI05.
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-R8_F5_RA_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
RATE="3.0"
IMAGE="tofslam:ros1"
CONFIG="avia_outdoor.yaml"
TIMEOUT_S=900

CONTAINERS=(tofslam_R8RA_1 tofslam_R8RA_2 tofslam_R8RA_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

BATCH1=(Dark02 Dynamic04 Occlusion03)
BATCH2=(Occlusion04 Varying-illu04 Varying-illu05)

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

log "==== T5.4-R8 F.5-RA MEASURE ($(date)) ===="
log "Batches: ${BATCH1[*]} / ${BATCH2[*]} | rate=${RATE}"

run_batch() {
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}
  cleanup; sleep 1
  for i in $(seq 0 $((n-1))); do
    docker run -d --rm --init --name "${CONTAINERS[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump:rw" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 2
  log "  [Build] ${n} containers..."
  for i in $(seq 0 $((n-1))); do
    docker exec "${CONTAINERS[$i]}" bash -lc \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
  done
  wait
  log "  [Build] Done."

  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    seq="${batch_seqs[$i]}"
    port="${PORTS[$i]}"
    out_dir="dump/${seq}"
    log "    Run ${seq} on port ${port}..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "TOFSLAM_DEBUG_DETERMINISM=0 bash /root/catkin_ws/docker/run_avia_exp.sh \
       ${CONFIG} ${seq} ${out_dir} ${port} ${RATE} 2>&1 | \
       tee /root/catkin_ws/${out_dir}/run.log" \
      > "${OUT_ROOT}/${seq}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done
  for c in "${CONTAINERS[@]}"; do
    docker exec "$c" bash -lc \
      "killall -9 tofslam_node rosbag rosout rosmaster roscore 2>/dev/null || true; \
       pkill -9 -f tofslam_node 2>/dev/null || true" 2>/dev/null || true
  done
}

run_batch "${BATCH1[@]}"
run_batch "${BATCH2[@]}"

log ""
log "==== FRAME=1 RUNTIME SIGNALS ===="
log "seq, degen_dirs, rho_1, corrs, l1, ATE"
ALL_SEQS=(Dark02 Dynamic04 Occlusion03 Occlusion04 Varying-illu04 Varying-illu05)
for seq in "${ALL_SEQS[@]}"; do
  csv="${OUT_ROOT}/${seq}/diagnostics.csv"
  ate_file="${OUT_ROOT}/${seq}/ate_result.txt"
  if [[ -f "$csv" ]]; then
    row=$(head -2 "$csv" | tail -1)
    degen=$(echo "$row" | awk -F, '{print $8}')
    corrs=$(echo "$row" | awk -F, '{print $9}')
    l1=$(echo "$row" | awk -F, '{print $13}')
    rho=$(python3 -c "print(round($corrs/$l1, 4)) if $l1>0 else print('NaN')")
    ate=$([[ -f "$ate_file" ]] && grep -oE "^rmse: [0-9.]+" "$ate_file" | awk '{print $2}' || echo "NaN")
    log "  $seq, degen=$degen, rho_1=$rho, corrs=$corrs, l1=$l1, ATE=$ate"
  fi
done

log ""
log "==== CLASSIFIER EVENTS ===="
for seq in "${ALL_SEQS[@]}"; do
  stdout_log="${OUT_ROOT}/${seq}_stdout.log"
  if [[ -f "$stdout_log" ]]; then
    log "  ${seq}:"
    grep -E "STAGE_A LOCK|STAGE_A DEFER|STAGE_B LOCK" "$stdout_log" | head -2 | sed 's/^/    /' | tee -a "${LOG}"
  fi
done

log ""
log "==== DONE $(date) ===="
