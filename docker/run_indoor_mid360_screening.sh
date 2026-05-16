#!/bin/bash
# run_indoor_mid360_screening.sh — Screen unified indoor Mid360 config (8 sequences)
# Rate=3.0 (screening default per CLAUDE.md policy)
# Compares unified config vs per-seq canonical baseline (mean=0.150m)
set -e
cd "$(dirname "$0")/.."

CONFIG="${1:-unified_indoor_mid360_v1.yaml}"
LABEL="${2:-indoor_mid360_unified_v1}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"

RATE="3.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_indoor_1 tofslam_indoor_2 tofslam_indoor_3)
MEM="3g"

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do
    docker rm -f "$c" 2>/dev/null || true
  done
  for p in "${PORTS[@]}"; do
    fuser -k ${p}/tcp 2>/dev/null || true
  done
}
trap cleanup EXIT

# Indoor sequences + labels
INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
INDOOR_LABELS=(iDark03 iDark04 iDyn01 iDyn02 iOcc01 iOcc02 iVI01 iVI02)

# Canonical baselines for comparison
declare -A BASELINE
BASELINE[iDark03]=0.1747
BASELINE[iDark04]=0.1822
BASELINE[iDyn01]=0.1374
BASELINE[iDyn02]=0.1384
BASELINE[iOcc01]=0.1512
BASELINE[iOcc02]=0.1449
BASELINE[iVI01]=0.1337
BASELINE[iVI02]=0.1343

log "================================================================"
log "  INDOOR MID360 SCREENING — $(date)"
log "  Config: ${CONFIG}"
log "  Rate: ${RATE} (screening, non-deterministic)"
log "  Sequences: ${#INDOOR_SEQS[@]}"
log "  Baseline mean: 0.150m (per-seq canonical)"
log "================================================================"

cleanup
sleep 2

# Start 3 containers
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

# Build
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

run_batch() {
  local start=$1
  local end=$2
  local n=$((end - start))
  [ $n -gt 3 ] && n=3
  local PIDS=()
  for slot in $(seq 0 $((n-1))); do
    local sidx=$((start + slot))
    local seq="${INDOOR_SEQS[$sidx]}"
    local label="${INDOOR_LABELS[$sidx]}"
    local port="${PORTS[$slot]}"
    local container="${CONTAINERS[$slot]}"
    local out="/root/catkin_ws/dump/${label}"
    log "    [${container}] ${label} (${CONFIG})..."
    docker exec "${container}" bash /root/catkin_ws/docker/run_avia_exp.sh \
      "${CONFIG}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do
    wait "$pid" || true
  done
}

# Batch 1/3: iDark03, iDark04, iDyn01
log "  Batch 1/3: iDark03, iDark04, iDyn01"
run_batch 0 3

# Batch 2/3: iDyn02, iOcc01, iOcc02
log "  Batch 2/3: iDyn02, iOcc01, iOcc02"
run_batch 3 6

# Batch 3/3: iVI01, iVI02
log "  Batch 3/3: iVI01, iVI02"
run_batch 6 8

# Results
log ""
log "================================================================"
log "  RESULTS — ${CONFIG}"
log "================================================================"

total_ate=0
count=0
total_baseline=0
wins=0
losses=0

for i in $(seq 0 $((${#INDOOR_LABELS[@]}-1))); do
  label="${INDOOR_LABELS[$i]}"
  ate_file="${OUT_ROOT}/${label}/ate_result.txt"
  base="${BASELINE[$label]}"
  if [ -f "$ate_file" ]; then
    rmse=$(grep "^rmse:" "$ate_file" 2>/dev/null | awk '{print $2}')
    if [ -z "$rmse" ]; then
      rmse=$(grep "ATE RMSE:" "$ate_file" 2>/dev/null | awk '{print $3}')
    fi
    if [ -n "$rmse" ]; then
      delta=$(python3 -c "d=(${rmse}-${base})/${base}*100; print(f'{d:+.1f}%')")
      printf "  %-12s %8.4f m  (baseline: %.4f, %s)\n" "$label" "$rmse" "$base" "$delta" | tee -a "${LOG}"
      total_ate=$(python3 -c "print(${total_ate} + ${rmse})")
      total_baseline=$(python3 -c "print(${total_baseline} + ${base})")
      count=$((count + 1))
      better=$(python3 -c "print('1' if ${rmse} <= ${base} else '0')")
      [ "$better" = "1" ] && wins=$((wins + 1)) || losses=$((losses + 1))
    else
      printf "  %-12s PARSE_FAIL\n" "$label" | tee -a "${LOG}"
    fi
  else
    printf "  %-12s NO RESULT\n" "$label" | tee -a "${LOG}"
  fi
done

if [ $count -gt 0 ]; then
  mean_ate=$(python3 -c "print(f'{${total_ate}/${count}:.4f}')")
  mean_base=$(python3 -c "print(f'{${total_baseline}/${count}:.4f}')")
  mean_delta=$(python3 -c "d=(${total_ate}/${count} - ${total_baseline}/${count})/(${total_baseline}/${count})*100; print(f'{d:+.1f}%')")
  log ""
  log "  MEAN (${count}/8):  ${mean_ate} m  (baseline: ${mean_base} m, ${mean_delta})"
  log "  Wins: ${wins}/8,  Losses: ${losses}/8"
fi

log ""
log "  Completed: $(date)"
log "================================================================"
