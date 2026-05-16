#!/bin/bash
# run_indoor_only_screen.sh — Quick indoor-only screening
# Usage: bash docker/run_indoor_only_screen.sh [CONFIG] [LABEL]
set -e
cd "$(dirname "$0")/.."

CONFIG="${1:-unified_outdoor_mid360_v2a.yaml}"
LABEL="${2:-indoor_v2a_test}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="3.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_is_1 tofslam_is_2 tofslam_is_3)
MEM="3g"

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

log "================================================================"
log "  INDOOR-ONLY SCREENING — $(date)"
log "  Config: ${CONFIG}"
log "  Rate: ${RATE}"
log "================================================================"

cleanup 2>/dev/null || true
sleep 2
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

INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
INDOOR_SHORT=(iDark03 iDark04 iDyn01 iDyn02 iOcc01 iOcc02 iVI01 iVI02)
declare -A SEQ_LABELS
for i in $(seq 0 7); do SEQ_LABELS[${INDOOR_SEQS[$i]}]="${INDOOR_SHORT[$i]}"; done

declare -A BASELINE
BASELINE[iDark03]=0.1747; BASELINE[iDark04]=0.1822
BASELINE[iDyn01]=0.1374; BASELINE[iDyn02]=0.1384
BASELINE[iOcc01]=0.1512; BASELINE[iOcc02]=0.1449
BASELINE[iVI01]=0.1337; BASELINE[iVI02]=0.1343

run_batch() {
  local seqs=("$@")
  local n=${#seqs[@]}; [ $n -gt 3 ] && n=3
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${seqs[$i]}"
    local label="${SEQ_LABELS[$seq]}"
    local port="${PORTS[$i]}"
    mkdir -p "${OUT_ROOT}/${label}"
    log "    [${CONTAINERS[$i]}] ${label}..."
    docker exec "${CONTAINERS[$i]}" bash /root/catkin_ws/docker/run_avia_exp.sh \
      "${CONFIG}" "${seq}" "/root/catkin_ws/dump/${label}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}

log "  Batch 1/3: iDark03, iDark04, iDyn01"
run_batch "${INDOOR_SEQS[@]:0:3}"
log "  Batch 2/3: iDyn02, iOcc01, iOcc02"
run_batch "${INDOOR_SEQS[@]:3:3}"
log "  Batch 3/3: iVI01, iVI02"
run_batch "${INDOOR_SEQS[@]:6:2}"

# Results
log ""
log "  --- INDOOR RESULTS: ${CONFIG} ---"
total=0; count=0; wins=0
for label in "${INDOOR_SHORT[@]}"; do
  f="${OUT_ROOT}/${label}/ate_result.txt"
  base="${BASELINE[$label]}"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
    if [ -n "$rmse" ]; then
      delta=$(python3 -c "d=(${rmse}-${base})/${base}*100; print(f'{d:+.1f}%')")
      printf "  %-12s %8.4f m  (baseline: %.4f, %s)\n" "$label" "$rmse" "$base" "$delta" | tee -a "${LOG}"
      total=$(python3 -c "print(${total}+${rmse})")
      count=$((count+1))
      better=$(python3 -c "print('1' if ${rmse} <= ${base}*1.001 else '0')")
      [ "$better" = "1" ] && wins=$((wins+1))
    fi
  fi
done
if [ $count -gt 0 ]; then
  mean=$(python3 -c "print(f'{${total}/${count}:.4f}')")
  base_mean="0.1496"
  delta=$(python3 -c "d=(${total}/${count}-0.1496)/0.1496*100; print(f'{d:+.1f}%')")
  log "  Mean (${count}/8): ${mean}m (baseline: ${base_mean}m, ${delta})"
  log "  Wins: ${wins}/${count}"
fi
log ""
log "  INDOOR SCREENING COMPLETE — $(date)"
