#!/bin/bash
# run_R4_avia_outdoor.sh — Sprint 5 R4 evaluation: unified avia_outdoor.yaml + adaptive policy
# Test goal: Avia outdoor 9-seq mean ATE <= 0.314m (Faster-LIO baseline) at rate=1.0
#
# Uses 3 parallel containers (CPU pinning 0-3, 4-7, 8-11; ports 11311-11313)
# Each seq runs in dedicated container; 3 batches × 3 seqs = 9 total.
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-R4_avia_outdoor_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
RATE="1.0"
IMAGE="tofslam:ros1"
CONFIG="avia_outdoor.yaml"
TIMEOUT_S=1800  # 30min per seq (VI03=17min real-time at rate=1.0; safe margin)

CONTAINERS=(tofslam_R4_1 tofslam_R4_2 tofslam_R4_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

AVIA_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)

log() { echo "$*" | tee -a "${LOG}"; }
mkdir -p "${OUT_ROOT}"

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

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
  log "  [Build] in ${n} containers..."
  for i in $(seq 0 $((n-1))); do
    docker exec "${CONTAINERS[$i]}" bash -lc \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
  done
  wait
  log "  [Build] Done."

  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${batch_seqs[$i]}"
    local port="${PORTS[$i]}"
    local name="${CONTAINERS[$i]}"
    local out="/root/catkin_ws/dump/${seq}"
    log "  [${name}] ${seq} @ rate=${RATE}, ${CONFIG}"
    timeout ${TIMEOUT_S} docker exec "${name}" bash -lc \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       bash docker/run_avia_exp.sh ${CONFIG} ${seq} ${out} ${port} ${RATE}" \
      >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then
      log "    OK ${batch_seqs[$i]}"
    else
      log "    FAIL/TIMEOUT ${batch_seqs[$i]}"
    fi
  done
}

log "================================================================"
log "  R4 AVIA OUTDOOR EVAL — $(date)"
log "  Config: ${CONFIG} (unified, with adaptive policy)"
log "  Rate: ${RATE} (canonical, deterministic)"
log "  Sequences: ${#AVIA_SEQS[@]}"
log "  Target: 9-seq mean ATE <= 0.314m"
log "================================================================"

log ""; log "--- Batch 1/3: DK01, DK02, DY03 ---"
run_batch "${AVIA_SEQS[@]:0:3}"
log ""; log "--- Batch 2/3: DY04, OC03, OC04 ---"
run_batch "${AVIA_SEQS[@]:3:3}"
log ""; log "--- Batch 3/3: VI03, VI04, VI05 ---"
run_batch "${AVIA_SEQS[@]:6:3}"

cleanup

log ""
log "================================================================"
log "  RESULTS — 9-seq ATE Summary"
log "================================================================"
sum=0; cnt=0
printf "%-20s %10s\n" "Seq" "ATE (m)" | tee -a "${LOG}"
for seq in "${AVIA_SEQS[@]}"; do
  f="${OUT_ROOT}/${seq}/ate_result.txt"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" | awk '{print $2}')
    printf "%-20s %10.4f\n" "${seq}" "${rmse}" | tee -a "${LOG}"
    sum=$(echo "$sum + $rmse" | bc -l)
    cnt=$((cnt+1))
  else
    printf "%-20s %10s\n" "${seq}" "FAIL" | tee -a "${LOG}"
  fi
done
log "----------------------------------"
if [ $cnt -gt 0 ]; then
  mean=$(echo "scale=4; $sum / $cnt" | bc -l)
  log "9-seq Mean: ${mean} m  (${cnt}/9)"
  log ""
  log "Faster-LIO baseline (HARD gate): 0.314 m"
  log "E4 loose gate: 0.34 m"
  log "E4 tight gate: 0.32 m"
  log "Per-seq canonical: 0.296 m"
fi
