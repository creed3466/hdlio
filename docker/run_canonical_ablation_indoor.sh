#!/bin/bash
# run_canonical_ablation_indoor.sh — Ablation study on Mid360 Indoor (8 sequences)
#
# Runs 4 ablation configs × 8 sequences = 32 experiments.
# Rate=3.0 for screening (not canonical paper numbers).
#
# Ablation configs:
#   1. no_surfel       — w/o Surfel Map
#   2. no_sigma2n      — w/o σ²_n
#   3. no_degen_fb     — w/o Degeneracy Feedback
#   4. no_l2           — w/o L2
#
# Output: dump/paper_canonical/ablation/indoor/{LABEL}/{SEQ}/
set -e
cd "$(dirname "$0")/.."

RATE="${1:-3.0}"
IMAGE="tofslam:ros1"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"

OUT_ROOT="dump/paper_canonical/ablation"
LOG="${OUT_ROOT}/ablation_indoor.log"

CONTAINER_NAMES=(tofslam_abl_i1 tofslam_abl_i2 tofslam_abl_i3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
MEM="3g"

# Ablation configs
ABLATION_OVERLAYS=(
  "ablation/ablation_no_surfel.yaml"
  "ablation/ablation_no_sigma2n.yaml"
  "ablation/ablation_no_degen_feedback.yaml"
  "ablation/ablation_no_l2.yaml"
)
ABLATION_LABELS=(
  "no_surfel"
  "no_sigma2n"
  "no_degen_fb"
  "no_l2"
)

# Indoor sequences — unified config (13 per-seq → 1, verified mean=0.150m)
INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
INDOOR_SHORT=(iDark03 iDark04 iDyn01 iDyn02 iOcc01 iOcc02 iVI01 iVI02)

declare -A SEQ_CFGS
SEQ_CFGS[indoor_Dark03]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Dark04]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Dynamic01]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Dynamic02]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Occlusion01]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Occlusion02]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Varying-illu01]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Varying-illu02]="unified_indoor_mid360_v1.yaml"

declare -A SEQ_LABELS
SEQ_LABELS[indoor_Dark03]="iDark03"
SEQ_LABELS[indoor_Dark04]="iDark04"
SEQ_LABELS[indoor_Dynamic01]="iDyn01"
SEQ_LABELS[indoor_Dynamic02]="iDyn02"
SEQ_LABELS[indoor_Occlusion01]="iOcc01"
SEQ_LABELS[indoor_Occlusion02]="iOcc02"
SEQ_LABELS[indoor_Varying-illu01]="iVI01"
SEQ_LABELS[indoor_Varying-illu02]="iVI02"

log() { echo "$*" | tee -a "${LOG}"; }
mkdir -p "${OUT_ROOT}"

cleanup() {
  for c in "${CONTAINER_NAMES[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
  for p in "${PORTS[@]}"; do fuser -k ${p}/tcp 2>/dev/null || true; done
}
trap cleanup EXIT

start_containers() {
  local n=$1
  cleanup
  sleep 3
  for i in $(seq 0 $((n-1))); do
    local label="${ABLATION_LABELS[$((BATCH_OFFSET + i))]}"
    for SEQ in "${INDOOR_SEQS[@]}"; do
      local short="${SEQ_LABELS[$SEQ]}"
      mkdir -p "${OUT_ROOT}/indoor/${label}/${short}"
    done
    docker run -d --rm --name "${CONTAINER_NAMES[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}/indoor/${label}:/root/catkin_ws/dump" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      -v "${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 3
  log "  [Build]..."
  for i in $(seq 0 $((n-1))); do
    docker exec "${CONTAINER_NAMES[$i]}" bash -c \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1" &
  done
  wait
  for i in $(seq 0 $((n-1))); do
    docker exec "${CONTAINER_NAMES[$i]}" pip3 install scipy numpy -q 2>/dev/null || true
  done
  log "  [Build] Done."
}

run_indoor_ablation() {
  local slot=$1
  local label_idx=$2
  local label="${ABLATION_LABELS[$label_idx]}"
  local overlay="${ABLATION_OVERLAYS[$label_idx]}"
  local port="${PORTS[$slot]}"
  local name="${CONTAINER_NAMES[$slot]}"

  for SEQ in "${INDOOR_SEQS[@]}"; do
    local cfg="${SEQ_CFGS[$SEQ]}"
    local short="${SEQ_LABELS[$SEQ]}"
    log "    [${label}] ${short} (${cfg} + ${overlay})"
    docker exec "$name" bash /root/catkin_ws/docker/run_avia_ablation_single.sh \
      "${cfg}" "${overlay}" "${SEQ}" "/root/catkin_ws/dump/${short}" "${port}" "${RATE}" \
      >> "${LOG}" 2>&1 || log "    ✗ ${short} FAILED"
  done
  log "  [${label}] All Indoor done."
}

collect() {
  log ""
  log "  --- Mid360 Indoor Ablation Results ---"
  printf "  %-12s" "Seq" | tee -a "${LOG}"
  for L in "${ABLATION_LABELS[@]}"; do printf " | %-10s" "$L" | tee -a "${LOG}"; done
  echo "" | tee -a "${LOG}"
  printf "  %s\n" "$(printf '%0.s-' {1..60})" | tee -a "${LOG}"

  declare -A SUMS
  declare -A COUNTS
  for L in "${ABLATION_LABELS[@]}"; do SUMS[$L]=0; COUNTS[$L]=0; done

  for short in "${INDOOR_SHORT[@]}"; do
    printf "  %-12s" "$short" | tee -a "${LOG}"
    for L in "${ABLATION_LABELS[@]}"; do
      local f="${OUT_ROOT}/indoor/${L}/${short}/ate_result.txt"
      if [ -f "$f" ]; then
        local rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
        if [ -n "$rmse" ]; then
          printf " | %-10s" "$rmse" | tee -a "${LOG}"
          SUMS[$L]=$(python3 -c "print(${SUMS[$L]} + ${rmse})")
          COUNTS[$L]=$((${COUNTS[$L]} + 1))
        else
          printf " | %-10s" "PARSE" | tee -a "${LOG}"
        fi
      else
        printf " | %-10s" "FAIL" | tee -a "${LOG}"
      fi
    done
    echo "" | tee -a "${LOG}"
  done

  printf "  %-12s" "Mean" | tee -a "${LOG}"
  for L in "${ABLATION_LABELS[@]}"; do
    if [ ${COUNTS[$L]} -gt 0 ]; then
      local mean=$(python3 -c "print(f'{${SUMS[$L]}/${COUNTS[$L]}:.4f}')")
      printf " | %-10s" "$mean" | tee -a "${LOG}"
    else
      printf " | %-10s" "N/A" | tee -a "${LOG}"
    fi
  done
  echo "" | tee -a "${LOG}"
}

log ""
log "================================================================"
log "  MID360 INDOOR ABLATION — $(date)"
log "  Rate: ${RATE}"
log "  Configs: ${#ABLATION_LABELS[@]}, Sequences: ${#INDOOR_SEQS[@]}"
log "  Total: $(( ${#ABLATION_LABELS[@]} * ${#INDOOR_SEQS[@]} )) experiments"
log "================================================================"

# Batch 1: no_surfel, no_sigma2n, no_degen_fb (3 in parallel)
BATCH_OFFSET=0
log "  Batch 1/2: no_surfel, no_sigma2n, no_degen_fb"
start_containers 3
PIDS=()
for slot in 0 1 2; do
  run_indoor_ablation "$slot" "$((BATCH_OFFSET + slot))" &
  PIDS+=($!)
done
for pid in "${PIDS[@]}"; do wait "$pid"; done

# Batch 2: no_l2 (1 container)
BATCH_OFFSET=3
log "  Batch 2/2: no_l2"
start_containers 1
run_indoor_ablation 0 3

collect

cleanup

log ""
log "================================================================"
log "  MID360 INDOOR ABLATION COMPLETE — $(date)"
log "  Results: ${OUT_ROOT}/indoor/"
log "================================================================"
