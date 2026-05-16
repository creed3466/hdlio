#!/bin/bash
# run_canonical_mid360_v3c.sh — Canonical Mid360 eval with unified v3c config
# Indoor (8) + Outdoor (9, including VI03) = 17 seq, rate=1.0
set -e
cd "$(dirname "$0")/.."

CONFIG="unified_mid360_v3c.yaml"
LABEL="canonical_mid360_v3c"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"

RATE="1.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_cv3c_1 tofslam_cv3c_2 tofslam_cv3c_3)
MEM="3g"

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
  for p in "${PORTS[@]}"; do fuser -k ${p}/tcp 2>/dev/null || true; done
}
trap cleanup EXIT

start_and_build() {
  cleanup
  sleep 3
  for i in 0 1 2; do
    docker run -d --rm --name "${CONTAINERS[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src:ro" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      -v "${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 3
  log "  [Build]..."
  for i in 0 1 2; do
    docker exec "${CONTAINERS[$i]}" bash -c \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1" &
  done
  wait
  for i in 0 1 2; do
    docker exec "${CONTAINERS[$i]}" pip3 install scipy numpy -q 2>/dev/null || true
  done
  log "  [Build] Done."
}

declare -A SEQ_LABELS
run_batch() {
  local runner=$1; shift
  local seqs=("$@")
  local n=${#seqs[@]}; [ $n -gt 3 ] && n=3
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${seqs[$i]}"
    local label="${SEQ_LABELS[$seq]:-$seq}"
    local port="${PORTS[$i]}"
    mkdir -p "${OUT_ROOT}/${label}"
    log "    [${CONTAINERS[$i]}] ${label} (${CONFIG})..."
    docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/${runner}" \
      "${CONFIG}" "${seq}" "/root/catkin_ws/dump/${label}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for idx in "${!PIDS[@]}"; do
    if wait "${PIDS[$idx]}" 2>/dev/null; then
      log "    ✓ ${seqs[$idx]} DONE"
    else
      log "    ✗ ${seqs[$idx]} FAILED"
    fi
  done
}

log "================================================================"
log "  CANONICAL MID360 v3c — $(date)"
log "  Config: ${CONFIG} (unified indoor/outdoor)"
log "  Rate: ${RATE} (canonical)"
log "================================================================"

start_and_build

# ---- OUTDOOR (9 seq, all v3c) ----
OUTDOOR_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)
OUTDOOR_SHORT=(DK01 DK02 DY03 DY04 OC03 OC04 VI03 VI04 VI05)
for i in $(seq 0 8); do SEQ_LABELS[${OUTDOOR_SEQS[$i]}]="${OUTDOOR_SHORT[$i]}"; done

log ""
log "  === Phase 1: Outdoor (9 seq) ==="
log "  Batch O-1/3: DK01, DK02, DY03"
run_batch "run_avia_exp.sh" "${OUTDOOR_SEQS[@]:0:3}"
log "  Batch O-2/3: DY04, OC03, OC04"
run_batch "run_avia_exp.sh" "${OUTDOOR_SEQS[@]:3:3}"
log "  Batch O-3/3: VI03, VI04, VI05"
run_batch "run_avia_exp.sh" "${OUTDOOR_SEQS[@]:6:3}"

# ---- INDOOR (8 seq, all v3c) ----
INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
INDOOR_SHORT=(iDark03 iDark04 iDyn01 iDyn02 iOcc01 iOcc02 iVI01 iVI02)
for i in $(seq 0 7); do SEQ_LABELS[${INDOOR_SEQS[$i]}]="${INDOOR_SHORT[$i]}"; done

log ""
log "  === Phase 2: Indoor (8 seq) ==="
log "  Batch I-1/3: iDark03, iDark04, iDyn01"
run_batch "run_avia_exp.sh" "${INDOOR_SEQS[@]:0:3}"
log "  Batch I-2/3: iDyn02, iOcc01, iOcc02"
run_batch "run_avia_exp.sh" "${INDOOR_SEQS[@]:3:3}"
log "  Batch I-3/3: iVI01, iVI02"
run_batch "run_avia_exp.sh" "${INDOOR_SEQS[@]:6:2}"

# ---- RESULTS ----
log ""
log "================================================================"
log "  RESULTS — ${CONFIG} (rate=${RATE})"
log "================================================================"

for group_name in "Outdoor" "Indoor"; do
  if [ "$group_name" = "Outdoor" ]; then
    LABELS=("${OUTDOOR_SHORT[@]}")
  else
    LABELS=("${INDOOR_SHORT[@]}")
  fi
  log ""
  log "  --- ${group_name} ---"
  total=0; count=0
  for label in "${LABELS[@]}"; do
    f="${OUT_ROOT}/${label}/ate_result.txt"
    if [ -f "$f" ]; then
      rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
      if [ -n "$rmse" ]; then
        printf "  %-12s %.4f m\n" "$label" "$rmse" | tee -a "${LOG}"
        total=$(python3 -c "print(${total}+${rmse})")
        count=$((count+1))
      fi
    else
      printf "  %-12s FAIL\n" "$label" | tee -a "${LOG}"
    fi
  done
  if [ $count -gt 0 ]; then
    mean=$(python3 -c "print(f'{${total}/${count}:.4f}')")
    log "  Mean (${count}/${#LABELS[@]}): ${mean} m"
  fi
done

cleanup
log ""
log "  COMPLETE — $(date)"
