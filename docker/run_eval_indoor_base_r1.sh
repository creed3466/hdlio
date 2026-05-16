#!/bin/bash
# run_eval_indoor_base_r1.sh — Avia Indoor BASE: rate=1.0 ground truth
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-eval_indoor_base_r1}"
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

run_batch() {
  local config=$1 tag=$2; shift 2; local batch_seqs=("$@")
  local n=${#batch_seqs[@]}; [ $n -gt 3 ] && n=3; local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${batch_seqs[$i]}" slabel=""
    for j in "${!ALL_SEQS[@]}"; do [ "${ALL_SEQS[$j]}" = "$seq" ] && slabel="${ALL_LABELS[$j]}" && break; done
    mkdir -p "${OUT_ROOT}/${tag}/${slabel}"
    log "    [${CONTAINERS[$i]}] ${tag}/${slabel}"
    docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
      "${config}" "${seq}" "/root/catkin_ws/dump/${tag}/${slabel}" "${PORTS[$i]}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then log "    ✓ ${batch_seqs[$i]} DONE"
    else log "    ✗ ${batch_seqs[$i]} FAILED"; fi
  done
}

##############################################
log "================================================================"
log "  AVIA INDOOR BASE: rate=1.0 ground truth  $(date)"
log "================================================================"

start_containers

run_batch "avia_indoor.yaml" "base" "${ALL_SEQS[@]:0:3}"
run_batch "avia_indoor.yaml" "base" "${ALL_SEQS[@]:3:3}"
run_batch "avia_indoor.yaml" "base" "${ALL_SEQS[@]:6:2}"

log ""; log "  --- Base rate=1.0 Results ---"
total=0; count=0
for i in "${!ALL_LABELS[@]}"; do
  sl="${ALL_LABELS[$i]}" f="${OUT_ROOT}/base/${ALL_LABELS[$i]}/ate_result.txt"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" | awk '{print $2}')
    [ -z "$rmse" ] && rmse=$(grep "ATE RMSE:" "$f" | awk '{print $3}')
    if [ -n "$rmse" ]; then
      printf "  %-8s %.4f m\n" "$sl" "$rmse" | tee -a "${LOG}"
      total=$(python3 -c "print(${total}+${rmse})"); count=$((count+1))
    fi
  fi
done
[ $count -gt 0 ] && printf "  MEAN     %.4f m (%d/8)\n" "$(python3 -c "print(${total}/${count})")" "$count" | tee -a "${LOG}"
log "================================================================"
cleanup
