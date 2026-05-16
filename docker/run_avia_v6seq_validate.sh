#!/bin/bash
# run_avia_v6seq_validate.sh — Validate V6a-based per-seq Avia configs
# Each config = V6a template + minimal per-seq overlay
# Target: mean ATE < 0.30m
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-avia_v6seq_r1}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
RATE="3.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_av6s_1 tofslam_av6s_2 tofslam_av6s_3)
MEM="3g"
MAX_PARALLEL=3

# All 9 outdoor Avia sequences with their V6a-based per-seq configs
declare -A SEQ_CONFIG
SEQ_CONFIG[Dark01]="avia_v6_seq/dark01.yaml"
SEQ_CONFIG[Dark02]="avia_v6_seq/dark02.yaml"
SEQ_CONFIG[Dynamic03]="avia_v6_seq/dynamic03.yaml"
SEQ_CONFIG[Dynamic04]="avia_v6_seq/dynamic04.yaml"
SEQ_CONFIG[Occlusion03]="avia_v6_seq/occlusion03.yaml"
SEQ_CONFIG[Occlusion04]="avia_v6_seq/occlusion04.yaml"
SEQ_CONFIG[Varying-illu03]="avia_v6_seq/varying_illu03.yaml"
SEQ_CONFIG[Varying-illu04]="avia_v6_seq/varying_illu04.yaml"
SEQ_CONFIG[Varying-illu05]="avia_v6_seq/varying_illu05.yaml"

SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do
    docker rm -f "$c" 2>/dev/null || true
  done
}
trap cleanup EXIT

log "================================================================"
log "  AVIA V6a-BASED PER-SEQ VALIDATION"
log "  $(date)"
log "  Rate: ${RATE}"
log "  Sequences: ${SEQS[*]}"
log "  Base: unified_outdoor_avia_v6a.yaml + minimal per-seq overlays"
log "  Target: mean < 0.30m"
log "  Previous: canonical 0.339m, Eq4 bestmix 0.293m, unified V6a 0.432m"
log "  Overlay counts: DY03=0, VI04=1, VI05=1, DK01=3, OC03/04=3, DK02=4, DY04=3, VI03=10+"
log "================================================================"

cleanup
sleep 1

# Batch execution: 3 sequences at a time
total=${#SEQS[@]}
batch_idx=0

while [ $batch_idx -lt $total ]; do
  batch_end=$((batch_idx + MAX_PARALLEL))
  [ $batch_end -gt $total ] && batch_end=$total
  batch_size=$((batch_end - batch_idx))

  log "  Batch $((batch_idx / MAX_PARALLEL + 1)): seqs $((batch_idx+1))-${batch_end}"

  # Start containers
  for slot in $(seq 0 $((batch_size - 1))); do
    sidx=$((batch_idx + slot))
    seq="${SEQS[$sidx]}"
    mkdir -p "${OUT_ROOT}/${seq}"

    docker run -d --rm --name "${CONTAINERS[$slot]}" \
      --network host --cpuset-cpus "${CPUSETS[$slot]}" --memory "$MEM" --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src:ro" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}/${seq}:/root/catkin_ws/dump" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 2

  # Build
  for slot in $(seq 0 $((batch_size - 1))); do
    docker exec "${CONTAINERS[$slot]}" bash -c \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1" &
  done
  wait
  for slot in $(seq 0 $((batch_size - 1))); do
    docker exec "${CONTAINERS[$slot]}" pip3 install scipy numpy -q 2>/dev/null || true
  done

  # Run each seq with its per-seq config
  PIDS=()
  for slot in $(seq 0 $((batch_size - 1))); do
    sidx=$((batch_idx + slot))
    seq="${SEQS[$sidx]}"
    config="${SEQ_CONFIG[$seq]}"
    port="${PORTS[$slot]}"
    container="${CONTAINERS[$slot]}"

    log "  [${seq}] config=${config}..."
    docker exec "${container}" bash /root/catkin_ws/docker/run_avia_exp.sh \
      "${config}" "${seq}" "/root/catkin_ws/dump" "${port}" "${RATE}" \
      2>&1 | grep -E "(ATE|ERROR)" || true &
    PIDS+=($!)
  done

  for pid in "${PIDS[@]}"; do
    wait "$pid"
  done

  # Cleanup containers
  for slot in $(seq 0 $((batch_size - 1))); do
    docker rm -f "${CONTAINERS[$slot]}" 2>/dev/null || true
  done

  batch_idx=$batch_end
done

# Results
log ""
log "================================================================"
log "  RESULTS"
log "================================================================"

total_ate=0
count=0
for seq in "${SEQS[@]}"; do
  ate_file="${OUT_ROOT}/${seq}/ate_result.txt"
  if [ -f "$ate_file" ]; then
    ate=$(grep -oP 'rmse: \K[0-9.]+' "$ate_file" 2>/dev/null || echo "N/A")
    if [ "$ate" != "N/A" ]; then
      log "  ${seq}: ${ate}m"
      total_ate=$(echo "$total_ate + $ate" | bc -l)
      count=$((count + 1))
    else
      log "  ${seq}: PARSE ERROR"
    fi
  else
    log "  ${seq}: NO RESULT"
  fi
done

if [ $count -gt 0 ]; then
  mean_ate=$(echo "scale=4; $total_ate / $count" | bc -l)
  log ""
  log "  MEAN (${count}/9): ${mean_ate}m"
  log "  Target: < 0.30m"
  log "  Previous canonical: 0.339m"
  log "  Eq4 bestmix: 0.293m"
fi

log ""
log "  Completed: $(date)"
log "================================================================"
