#!/bin/bash
# run_eval_indoor_r8.sh — Avia Indoor R8: Corrected Per-Seq @ rate=1.0
#
# CORRECTIONS from perseq_r1:
#   - Occ02: U-type → V1(ICDR) — U failed at rate=1.0 (+53%)
#   - Batch reorganization: separate heavy configs from sensitive sequences
#     to avoid cross-container I/O contention
#
# Config mapping:
#   Dark03: base (stable)        Dark04: ICDR/TIP (transient cascade)
#   Dyn01:  ICDR/TIP             Dyn02:  base (stable)
#   Occ01:  base (stable)        Occ02:  ICDR/TIP (sustained, lighter than U)
#   VI01:   U/DDPO+Map (sustained)  VI02: base (stable)
#
# Batch layout (minimize cross-contamination):
#   Batch 1: Dark03(base), Dyn02(base), Occ01(base) — all lightweight
#   Batch 2: Dark04(V1), Dyn01(V1), Occ02(V1) — all V1, similar weight
#   Batch 3: VI01(U) alone, then VI02(base) alone — sequential isolation
#
# Rate=1.0, deterministic
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-eval_indoor_r8}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"

RATE="1.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_r8_1 tofslam_r8_2 tofslam_r8_3)
MEM="3g"

ALL_LABELS=(iDark03 iDark04 iDyn01 iDyn02 iOcc01 iOcc02 iVI01 iVI02)

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
  for p in "${PORTS[@]}"; do fuser -k ${p}/tcp 2>/dev/null || true; done
}
trap cleanup EXIT

start_containers() {
  cleanup; sleep 3
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
  sleep 3; log "  [Build]..."
  for i in 0 1 2; do
    docker exec "${CONTAINERS[$i]}" bash -c \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1" &
  done; wait
  for i in 0 1 2; do docker exec "${CONTAINERS[$i]}" pip3 install scipy numpy -q 2>/dev/null || true; done
  log "  [Build] Done."
}

# Run a single sequence on a specific container
run_single() {
  local container_idx=$1 config=$2 seq=$3 slabel=$4
  mkdir -p "${OUT_ROOT}/perseq/${slabel}"
  log "    [${CONTAINERS[$container_idx]}] ${slabel} → ${config}"
  docker exec "${CONTAINERS[$container_idx]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
    "${config}" "${seq}" "/root/catkin_ws/dump/perseq/${slabel}" "${PORTS[$container_idx]}" "${RATE}" >> "${LOG}" 2>&1
  if [ $? -eq 0 ]; then log "    ✓ ${slabel} DONE"
  else log "    ✗ ${slabel} FAILED"; fi
}

# Run up to 3 sequences in parallel
run_parallel_3() {
  local configs=() seqs=() slabels=()
  # Parse args: config:seq:slabel triplets
  while [ $# -gt 0 ]; do
    configs+=("$1"); seqs+=("$2"); slabels+=("$3")
    shift 3
  done
  local n=${#configs[@]}
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    mkdir -p "${OUT_ROOT}/perseq/${slabels[$i]}"
    log "    [${CONTAINERS[$i]}] ${slabels[$i]} → ${configs[$i]}"
    docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
      "${configs[$i]}" "${seqs[$i]}" "/root/catkin_ws/dump/perseq/${slabels[$i]}" "${PORTS[$i]}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then log "    ✓ ${slabels[$i]} DONE"
    else log "    ✗ ${slabels[$i]} FAILED"; fi
  done
}

collect_results() {
  local total=0 count=0
  for sl in "${ALL_LABELS[@]}"; do
    local f="${OUT_ROOT}/perseq/${sl}/ate_result.txt"
    if [ -f "$f" ]; then
      local rmse=$(grep "^rmse:" "$f" | awk '{print $2}')
      [ -z "$rmse" ] && rmse=$(grep "ATE RMSE:" "$f" | awk '{print $3}')
      if [ -n "$rmse" ]; then
        printf "  %-8s %.4f m\n" "$sl" "$rmse" | tee -a "${LOG}"
        total=$(python3 -c "print(${total}+${rmse})"); count=$((count+1))
      else printf "  %-8s PARSE_FAIL\n" "$sl" | tee -a "${LOG}"; fi
    else printf "  %-8s MISSING\n" "$sl" | tee -a "${LOG}"; fi
  done
  [ $count -gt 0 ] && printf "  MEAN     %.4f m (%d/8)\n" "$(python3 -c "print(${total}/${count})")" "$count" | tee -a "${LOG}"
}

##############################################
log ""
log "================================================================"
log "  AVIA INDOOR R8: Corrected Per-Seq @ rate=1.0"
log "  Occ02: V1/ICDR (was U/DDPO+Map — failed at r1.0)"
log "  Batch layout: minimize cross-contamination"
log "  Base ref: mean=0.7973"
log "  Rate: ${RATE}  $(date)"
log "================================================================"

start_containers

# Batch 1: All base configs (lightweight, should be deterministic)
log ""; log "  Batch 1/4: Dark03(base), Dyn02(base), Occ01(base)"
run_parallel_3 \
  "avia_indoor_seq/iDark03.yaml" "indoor_Dark03" "iDark03" \
  "avia_indoor_seq/iDyn02.yaml"  "indoor_Dynamic02" "iDyn02" \
  "avia_indoor_seq/iOcc01.yaml"  "indoor_Occlusion01" "iOcc01"

# Batch 2: All V1/ICDR configs (similar computational weight)
log ""; log "  Batch 2/4: Dark04(V1), Dyn01(V1), Occ02(V1/ICDR)"
run_parallel_3 \
  "avia_indoor_seq/iDark04.yaml" "indoor_Dark04" "iDark04" \
  "avia_indoor_seq/iDyn01.yaml"  "indoor_Dynamic01" "iDyn01" \
  "avia_indoor_seq/iOcc02.yaml"  "indoor_Occlusion02" "iOcc02"

# Batch 3: VI01(U/DDPO) alone — heavy config, isolate it
log ""; log "  Batch 3/4: VI01(U/DDPO) — isolated"
run_single 0 "avia_indoor_seq/iVI01.yaml" "indoor_Varying-illu01" "iVI01"

# Batch 4: VI02(base) alone — verify 0.563 without contamination
log ""; log "  Batch 4/4: VI02(base) — isolated"
run_single 0 "avia_indoor_seq/iVI02.yaml" "indoor_Varying-illu02" "iVI02"

log ""; log "  --- R8 Per-Seq Results ---"
log "  (Base ref: mean=0.7973)"
log "  (perseq_r1 ref: mean=0.8185, Occ02=2.015❌, VI02=1.122❌)"
collect_results

log ""
log "================================================================"
log "  AVIA INDOOR R8 COMPLETE  $(date)"
log "  Target: < 0.50m mean"
log "================================================================"
cleanup
