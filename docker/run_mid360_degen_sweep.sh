#!/bin/bash
# run_mid360_degen_sweep.sh — Phase 1 degeneracy tuning sweep on Mid-360
#
# Goal: find unified-config tuning that strengthens the degeneracy ablation
#       narrative without hurting routine sequences.
#
# 5 experiments × 9 sequences (Mid-360 outdoor) × 1 run = 45 runs at rate=3.0
# Base: unified_mid360_v3c.yaml (applied to ALL 9 incl. VI03)
# Overlay: tuning/e{1..5}_*.yaml
#
# Output: dump/mid360_degen_sweep/{exp_label}/{SEQ}/
set -e
cd "$(dirname "$0")/.."

RATE="3.0"
IMAGE="tofslam:ros1"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"

OUT_ROOT="dump/mid360_degen_sweep"
LOG="${OUT_ROOT}/sweep.log"

CONTAINER_NAMES=(tofslam_swp_1 tofslam_swp_2 tofslam_swp_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
MEM="3g"

# 5 tuning experiments (overlay file under config/)
EXPERIMENTS=(
  "tuning/e1_ratio005.yaml:e1_ratio005"
  "tuning/e2_floor015.yaml:e2_floor015"
  "tuning/e3_ddpo05.yaml:e3_ddpo05"
  "tuning/e4_softfloor01.yaml:e4_softfloor01"
  "tuning/e5_geomcov.yaml:e5_geomcov"
)

BASE_CFG="unified_mid360_v3c.yaml"

# Mid-360 outdoor 9 sequences. Force unified base for ALL incl VI03 (sweep target).
MID_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)

log() { echo "$*" | tee -a "${LOG}"; }
mkdir -p "${OUT_ROOT}"

cleanup() {
  for c in "${CONTAINER_NAMES[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

start_containers_for_label() {
  local label=$1
  local n=$2
  cleanup
  sleep 1
  for i in $(seq 0 $((n-1))); do
    docker run -d --rm --name "${CONTAINER_NAMES[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}/${label}:/root/catkin_ws/dump" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 2
  log "    [Build]..."
  for i in $(seq 0 $((n-1))); do
    docker exec "${CONTAINER_NAMES[$i]}" bash -c \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1" &
  done
  wait
  for i in $(seq 0 $((n-1))); do
    docker exec "${CONTAINER_NAMES[$i]}" pip3 install scipy numpy -q 2>/dev/null || true
  done
  log "    [Build] Done."
}

run_batch() {
  local overlay=$1
  shift
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}
  [ $n -gt 3 ] && n=3
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${batch_seqs[$i]}"
    local port="${PORTS[$i]}"
    local name="${CONTAINER_NAMES[$i]}"
    local out="/root/catkin_ws/dump/${seq}"
    log "      [${name}] ${seq} (${BASE_CFG} + ${overlay})"
    docker exec "${name}" bash /root/catkin_ws/docker/run_avia_ablation_single.sh \
      "${BASE_CFG}" "${overlay}" "${seq}" "${out}" "${port}" "${RATE}" \
      >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then
      log "      OK ${batch_seqs[$i]}"
    else
      log "      FAIL ${batch_seqs[$i]}"
    fi
  done
}

collect_for_label() {
  local label=$1
  log ""
  log "  --- Results for ${label} ---"
  local sum=0
  local cnt=0
  for SEQ in "${MID_SEQS[@]}"; do
    local f="${OUT_ROOT}/${label}/${SEQ}/ate_result.txt"
    if [ -f "$f" ]; then
      local rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
      printf "    %-18s %.4f\n" "$SEQ" "${rmse}" | tee -a "${LOG}"
      sum=$(echo "$sum + $rmse" | bc -l)
      cnt=$((cnt+1))
    else
      printf "    %-18s FAIL\n" "$SEQ" | tee -a "${LOG}"
    fi
  done
  if [ $cnt -gt 0 ]; then
    local mean=$(echo "scale=4; $sum / $cnt" | bc -l)
    log "    -----"
    log "    9-seq Mean: ${mean} m  (${cnt}/9 succeeded)"
  fi
}

log ""
log "================================================================"
log "  MID-360 DEGENERACY SWEEP (Phase 1) — $(date)"
log "  Rate: ${RATE} (screening)"
log "  Base: ${BASE_CFG}"
log "  Experiments: ${#EXPERIMENTS[@]}"
log "  Sequences: ${#MID_SEQS[@]} (unified for ALL incl VI03)"
log "  Total: $(( ${#EXPERIMENTS[@]} * ${#MID_SEQS[@]} )) runs"
log "================================================================"

for exp_pair in "${EXPERIMENTS[@]}"; do
  IFS=':' read -r overlay label <<< "$exp_pair"
  log ""
  log "================================================"
  log "  [Sweep] $label  (overlay: $overlay)"
  log "================================================"

  for SEQ in "${MID_SEQS[@]}"; do
    mkdir -p "${OUT_ROOT}/${label}/${SEQ}"
  done

  log "    Batch 1/3: Dark01, Dark02, Dynamic03"
  start_containers_for_label "$label" 3
  run_batch "$overlay" "${MID_SEQS[@]:0:3}"

  log "    Batch 2/3: Dynamic04, Occlusion03, Occlusion04"
  start_containers_for_label "$label" 3
  run_batch "$overlay" "${MID_SEQS[@]:3:3}"

  log "    Batch 3/3: Varying-illu03, Varying-illu04, Varying-illu05"
  start_containers_for_label "$label" 3
  run_batch "$overlay" "${MID_SEQS[@]:6:3}"

  collect_for_label "$label"
done

cleanup

log ""
log "================================================================"
log "  SWEEP COMPLETE — $(date)"
log "  Results: ${OUT_ROOT}/"
log "================================================================"

# Final summary table
log ""
log "=== FINAL SUMMARY ==="
printf "  %-18s" "Seq" | tee -a "${LOG}"
for exp_pair in "${EXPERIMENTS[@]}"; do
  IFS=':' read -r _ label <<< "$exp_pair"
  printf " | %-14s" "$label" | tee -a "${LOG}"
done
echo "" | tee -a "${LOG}"
echo "  $(printf '%0.s-' {1..100})" | tee -a "${LOG}"
for SEQ in "${MID_SEQS[@]}"; do
  printf "  %-18s" "$SEQ" | tee -a "${LOG}"
  for exp_pair in "${EXPERIMENTS[@]}"; do
    IFS=':' read -r _ label <<< "$exp_pair"
    f="${OUT_ROOT}/${label}/${SEQ}/ate_result.txt"
    if [ -f "$f" ]; then
      rmse=$(grep "^rmse:" "$f" | awk '{print $2}')
      printf " | %-14.4f" "${rmse}" | tee -a "${LOG}"
    else
      printf " | %-14s" "FAIL" | tee -a "${LOG}"
    fi
  done
  echo "" | tee -a "${LOG}"
done
log ""
log "Baseline (unified_mid360_v3c.yaml, canonical r=1.0): 9-seq mean = 0.297m"
log "  (note: sweep is r=3.0 screening; r=3.0 baseline differs from r=1.0)"
