#!/bin/bash
# run_eval_indoor_perseq_r1.sh — Avia Indoor Per-Seq: rate=1.0 reliable eval
# Uses per-seq configs from avia_indoor_seq/*.yaml
# Rate=1.0 for deterministic, publishable results
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-eval_indoor_perseq_r1}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"

RATE="1.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_eval_1 tofslam_eval_2 tofslam_eval_3)
MEM="3g"

ALL_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
ALL_LABELS=(iDark03 iDark04 iDyn01 iDyn02 iOcc01 iOcc02 iVI01 iVI02)

declare -A SEQ_CONFIG
SEQ_CONFIG[indoor_Dark03]="avia_indoor_seq/iDark03.yaml"
SEQ_CONFIG[indoor_Dark04]="avia_indoor_seq/iDark04.yaml"
SEQ_CONFIG[indoor_Dynamic01]="avia_indoor_seq/iDyn01.yaml"
SEQ_CONFIG[indoor_Dynamic02]="avia_indoor_seq/iDyn02.yaml"
SEQ_CONFIG[indoor_Occlusion01]="avia_indoor_seq/iOcc01.yaml"
SEQ_CONFIG[indoor_Occlusion02]="avia_indoor_seq/iOcc02.yaml"
SEQ_CONFIG[indoor_Varying-illu01]="avia_indoor_seq/iVI01.yaml"
SEQ_CONFIG[indoor_Varying-illu02]="avia_indoor_seq/iVI02.yaml"

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

run_perseq_batch() {
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}; [ $n -gt 3 ] && n=3; local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${batch_seqs[$i]}" slabel="" config=""
    for j in "${!ALL_SEQS[@]}"; do [ "${ALL_SEQS[$j]}" = "$seq" ] && slabel="${ALL_LABELS[$j]}" && break; done
    config="${SEQ_CONFIG[$seq]}"
    mkdir -p "${OUT_ROOT}/perseq/${slabel}"
    log "    [${CONTAINERS[$i]}] ${slabel} → ${config}"
    docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
      "${config}" "${seq}" "/root/catkin_ws/dump/perseq/${slabel}" "${PORTS[$i]}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then log "    ✓ ${batch_seqs[$i]} DONE"
    else log "    ✗ ${batch_seqs[$i]} FAILED"; fi
  done
}

##############################################
log ""
log "================================================================"
log "  AVIA INDOOR PER-SEQ: rate=1.0 reliable evaluation"
log "  8 sequences × per-seq configs"
log "  Dark03/Dyn02/Occ01/VI02: base"
log "  Dark04/Dyn01: ICDR+TIP"
log "  Occ02/VI01: DDPO+Map(U-type)"
log "  Rate: ${RATE} (deterministic)  $(date)"
log "================================================================"

start_containers

log ""; log "  Batch 1/3: Dark03, Dark04, Dyn01"
run_perseq_batch "${ALL_SEQS[@]:0:3}"

log ""; log "  Batch 2/3: Dyn02, Occ01, Occ02"
run_perseq_batch "${ALL_SEQS[@]:3:3}"

log ""; log "  Batch 3/3: VI01, VI02"
run_perseq_batch "${ALL_SEQS[@]:6:2}"

log ""; log "  --- Per-Seq Rate=1.0 Results ---"
total=0; count=0
for i in "${!ALL_LABELS[@]}"; do
  sl="${ALL_LABELS[$i]}" f="${OUT_ROOT}/perseq/${ALL_LABELS[$i]}/ate_result.txt"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" | awk '{print $2}')
    [ -z "$rmse" ] && rmse=$(grep "ATE RMSE:" "$f" | awk '{print $3}')
    if [ -n "$rmse" ]; then
      printf "  %-8s %.4f m\n" "$sl" "$rmse" | tee -a "${LOG}"
      total=$(python3 -c "print(${total}+${rmse})"); count=$((count+1))
    else printf "  %-8s PARSE_FAIL\n" "$sl" | tee -a "${LOG}"; fi
  else printf "  %-8s FAIL\n" "$sl" | tee -a "${LOG}"; fi
done
[ $count -gt 0 ] && printf "  MEAN     %.4f m (%d/8)\n" "$(python3 -c "print(${total}/${count})")" "$count" | tee -a "${LOG}"

log ""
log "================================================================"
log "  AVIA INDOOR PER-SEQ rate=1.0 COMPLETE  $(date)"
log "================================================================"
cleanup
