#!/bin/bash
# run_canonical_ablation_mid360.sh — Canonical ablation for Mid-360 outdoor
#
# Runs 4 ablation configs x 9 sequences = 36 experiments
# at rate=1.0 (canonical) to match paper_canonical/ main results.
#
# Uses 1-sequence-per-container batching to avoid roscore port conflicts.
# Mid360 configs: unified_mid360_v3c.yaml for 8/9, varying_illu03_v3c.yaml for VI03.
#
# Output: dump/paper_canonical/ablation/mid360/{LABEL}/{SEQ}/
set -e
cd "$(dirname "$0")/.."

RATE="1.0"
IMAGE="tofslam:ros1"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"

OUT_ROOT="dump/paper_canonical/ablation"
LOG="${OUT_ROOT}/ablation_mid360.log"

CONTAINER_NAMES=(tofslam_abl_1 tofslam_abl_2 tofslam_abl_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
MEM="3g"

# Ablation overlay configs
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

# Mid-360 outdoor sequences + per-seq configs
MID_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)
declare -A MID_CFGS
MID_CFGS[Dark01]="unified_mid360_v3c.yaml"
MID_CFGS[Dark02]="unified_mid360_v3c.yaml"
MID_CFGS[Dynamic03]="unified_mid360_v3c.yaml"
MID_CFGS[Dynamic04]="unified_mid360_v3c.yaml"
MID_CFGS[Occlusion03]="unified_mid360_v3c.yaml"
MID_CFGS[Occlusion04]="unified_mid360_v3c.yaml"
MID_CFGS[Varying-illu03]="mid360_seq/varying_illu03_v3c.yaml"
MID_CFGS[Varying-illu04]="unified_mid360_v3c.yaml"
MID_CFGS[Varying-illu05]="unified_mid360_v3c.yaml"

log() { echo "$*" | tee -a "${LOG}"; }
mkdir -p "${OUT_ROOT}"

cleanup() {
  for c in "${CONTAINER_NAMES[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

# start_containers_for_label — Start N containers for a specific ablation label.
# Each container mounts the label's output directory.
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
      -v "$(pwd)/${OUT_ROOT}/mid360/${label}:/root/catkin_ws/dump" \
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

# run_mid360_batch — Run a batch of Mid360 sequences (max 3) in parallel.
run_mid360_batch() {
  local overlay=$1
  shift
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}
  [ $n -gt 3 ] && n=3
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${batch_seqs[$i]}"
    local cfg="${MID_CFGS[$seq]}"
    local port="${PORTS[$i]}"
    local name="${CONTAINER_NAMES[$i]}"
    local out="/root/catkin_ws/dump/${seq}"
    log "      [${name}] ${seq} (${cfg} + ${overlay})"
    docker exec "${name}" bash /root/catkin_ws/docker/run_avia_ablation_single.sh \
      "${cfg}" "${overlay}" "${seq}" "${out}" "${port}" "${RATE}" \
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

collect() {
  log ""
  log "  --- Mid-360 Ablation Results ---"
  printf "  %-18s" "Seq" | tee -a "${LOG}"
  for L in "${ABLATION_LABELS[@]}"; do printf " | %-10s" "$L" | tee -a "${LOG}"; done
  echo "" | tee -a "${LOG}"
  printf "  %s\n" "$(printf '%0.s-' {1..70})" | tee -a "${LOG}"
  for SEQ in "${MID_SEQS[@]}"; do
    printf "  %-18s" "$SEQ" | tee -a "${LOG}"
    for L in "${ABLATION_LABELS[@]}"; do
      local f="${OUT_ROOT}/mid360/${L}/${SEQ}/ate_result.txt"
      if [ -f "$f" ]; then
        local rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
        printf " | %-10s" "${rmse:-PARSE}" | tee -a "${LOG}"
      else
        printf " | %-10s" "FAIL" | tee -a "${LOG}"
      fi
    done
    echo "" | tee -a "${LOG}"
  done
}

log ""
log "================================================================"
log "  MID-360 CANONICAL ABLATION — $(date)"
log "  Rate: ${RATE} (canonical)"
log "  Configs: ${#ABLATION_LABELS[@]}, Sequences: ${#MID_SEQS[@]}"
log "  Total: $(( ${#ABLATION_LABELS[@]} * ${#MID_SEQS[@]} )) experiments"
log "================================================================"

for abl_idx in "${!ABLATION_LABELS[@]}"; do
  label="${ABLATION_LABELS[$abl_idx]}"
  overlay="${ABLATION_OVERLAYS[$abl_idx]}"
  log ""
  log "  [Mid360] Config $((abl_idx+1))/${#ABLATION_LABELS[@]}: ${label} (${overlay})"

  for SEQ in "${MID_SEQS[@]}"; do
    mkdir -p "${OUT_ROOT}/mid360/${label}/${SEQ}"
  done

  # Batch 1/3: Dark01, Dark02, Dynamic03
  log "    Batch 1/3: Dark01, Dark02, Dynamic03"
  start_containers_for_label "$label" 3
  run_mid360_batch "$overlay" "${MID_SEQS[@]:0:3}"

  # Batch 2/3: Dynamic04, Occlusion03, Occlusion04
  log "    Batch 2/3: Dynamic04, Occlusion03, Occlusion04"
  start_containers_for_label "$label" 3
  run_mid360_batch "$overlay" "${MID_SEQS[@]:3:3}"

  # Batch 3/3: Varying-illu03, Varying-illu04, Varying-illu05
  log "    Batch 3/3: Varying-illu03, Varying-illu04, Varying-illu05"
  start_containers_for_label "$label" 3
  run_mid360_batch "$overlay" "${MID_SEQS[@]:6:3}"

  log "  [Mid360] ${label} — all 9 sequences done."
done

collect

cleanup

log ""
log "================================================================"
log "  MID-360 CANONICAL ABLATION COMPLETE — $(date)"
log "  Results: ${OUT_ROOT}/mid360/"
log "================================================================"
