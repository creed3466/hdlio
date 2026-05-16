#!/bin/bash
# run_canonical_ablation.sh — Canonical ablation for TABLE III (NTU) + TABLE IV (Avia)
#
# Runs 4 ablation configs x 9 sequences x 2 datasets = 72 experiments
# at rate=1.0 (canonical) to match paper_canonical/ main results.
#
# Ablation configs:
#   1. no_surfel       — w/o Surfel Map
#   2. no_sigma2n      — w/o sigma2_n
#   3. no_degen_fb     — w/o Degeneracy Feedback
#   4. no_l2           — w/o L2
#
# Execution model:
#   For each ablation config, run 3 sequences in parallel (1 per container),
#   then tear down containers and start fresh ones for the next batch of 3.
#   This avoids the roscore cleanup issue when running multiple sequences
#   sequentially in the same container.
#
# Output: dump/paper_canonical/ablation/{avia,ntu}/{LABEL}/{SEQ}/
set -e
cd "$(dirname "$0")/.."

RATE="1.0"
IMAGE="tofslam:ros1"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
NTU_DATA="/home/euntae/Project/dataset/ros1/ntu_viral"

OUT_ROOT="dump/paper_canonical/ablation"
LOG="${OUT_ROOT}/ablation.log"

CONTAINER_NAMES=(tofslam_abl_1 tofslam_abl_2 tofslam_abl_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
MEM="3g"

# Ablation configs (paper TABLE III/IV)
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

# Avia sequences + per-seq configs
AVIA_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)
declare -A AVIA_CFGS
AVIA_CFGS[Dark01]="avia_v6_seq/dark01.yaml"
AVIA_CFGS[Dark02]="avia_v6_seq/dark02.yaml"
AVIA_CFGS[Dynamic03]="avia_v6_seq/dynamic03.yaml"
AVIA_CFGS[Dynamic04]="avia_v6_seq/dynamic04.yaml"
AVIA_CFGS[Occlusion03]="avia_v6_seq/occlusion03.yaml"
AVIA_CFGS[Occlusion04]="avia_v6_seq/occlusion04.yaml"
AVIA_CFGS[Varying-illu03]="avia_v6_seq/varying_illu03.yaml"
AVIA_CFGS[Varying-illu04]="avia_v6_seq/varying_illu04.yaml"
AVIA_CFGS[Varying-illu05]="avia_v6_seq/varying_illu05.yaml"

# NTU sequences (all use ros1_ntu_viral.yaml)
NTU_SEQS=(eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03)

log() { echo "$*" | tee -a "${LOG}"; }
mkdir -p "${OUT_ROOT}"

cleanup() {
  for c in "${CONTAINER_NAMES[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

# start_containers_for_label — Start N containers, all mounting the same
# ablation label's output directory.
#
# Args: dataset label extra_vols n_containers
start_containers_for_label() {
  local dataset=$1
  local label=$2
  local extra_vols="$3"
  local n=$4
  cleanup
  sleep 1
  for i in $(seq 0 $((n-1))); do
    docker run -d --rm --name "${CONTAINER_NAMES[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}/${dataset}/${label}:/root/catkin_ws/dump" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      ${extra_vols} \
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

# run_avia_batch — Run a batch of Avia sequences (max 3) in parallel,
# each in its own container slot. 1 sequence per container.
#
# Args: overlay batch_seqs...
run_avia_batch() {
  local overlay=$1
  shift
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}
  [ $n -gt 3 ] && n=3
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${batch_seqs[$i]}"
    local cfg="${AVIA_CFGS[$seq]}"
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

# run_ntu_batch — Run a batch of NTU sequences (max 3) in parallel.
#
# Args: overlay batch_seqs...
run_ntu_batch() {
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
    log "      [${name}] ${seq}"
    docker exec "${name}" bash /root/catkin_ws/docker/run_ntu_ablation_single.sh \
      "${overlay}" "${seq}" "${out}" "${port}" "${RATE}" \
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
  local dataset=$1
  shift
  local seqs=("$@")
  log ""
  log "  --- ${dataset} Ablation Results ---"
  printf "  %-18s" "Seq" | tee -a "${LOG}"
  for L in "${ABLATION_LABELS[@]}"; do printf " | %-10s" "$L" | tee -a "${LOG}"; done
  echo "" | tee -a "${LOG}"
  printf "  %s\n" "$(printf '%0.s-' {1..70})" | tee -a "${LOG}"
  for SEQ in "${seqs[@]}"; do
    printf "  %-18s" "$SEQ" | tee -a "${LOG}"
    for L in "${ABLATION_LABELS[@]}"; do
      local f="${OUT_ROOT}/${dataset}/${L}/${SEQ}/ate_result.txt"
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
log "  CANONICAL ABLATION — $(date)"
log "  Rate: ${RATE} (canonical)"
log "  Configs: ${#ABLATION_LABELS[@]}, Avia: ${#AVIA_SEQS[@]}, NTU: ${#NTU_SEQS[@]}"
log "  Total: $(( ${#ABLATION_LABELS[@]} * (${#AVIA_SEQS[@]} + ${#NTU_SEQS[@]}) )) experiments"
log "================================================================"

############################################
# 1. AVIA ABLATION (4 configs x 9 seqs)
############################################
log ""
log "======== [1/2] AVIA ABLATION (rate=${RATE}) ========"

for abl_idx in "${!ABLATION_LABELS[@]}"; do
  label="${ABLATION_LABELS[$abl_idx]}"
  overlay="${ABLATION_OVERLAYS[$abl_idx]}"
  log ""
  log "  [Avia] Config $((abl_idx+1))/${#ABLATION_LABELS[@]}: ${label} (${overlay})"

  # Pre-create output dirs for all sequences
  for SEQ in "${AVIA_SEQS[@]}"; do
    mkdir -p "${OUT_ROOT}/avia/${label}/${SEQ}"
  done

  # Batch 1/4: Dark01, Dark02, Dynamic03
  log "    Batch 1/4: Dark01, Dark02, Dynamic03"
  start_containers_for_label "avia" "$label" "" 3
  run_avia_batch "$overlay" "${AVIA_SEQS[@]:0:3}"

  # Batch 2/4: Dynamic04, Occlusion03, Occlusion04
  log "    Batch 2/4: Dynamic04, Occlusion03, Occlusion04"
  start_containers_for_label "avia" "$label" "" 3
  run_avia_batch "$overlay" "${AVIA_SEQS[@]:3:3}"

  # Batch 3/4: Varying-illu03 (solo — long bag, contention-sensitive)
  log "    Batch 3/4: Varying-illu03 (solo)"
  start_containers_for_label "avia" "$label" "" 1
  run_avia_batch "$overlay" "Varying-illu03"

  # Batch 4/4: Varying-illu04, Varying-illu05
  log "    Batch 4/4: Varying-illu04, Varying-illu05"
  start_containers_for_label "avia" "$label" "" 2
  run_avia_batch "$overlay" "Varying-illu04" "Varying-illu05"

  log "  [Avia] ${label} — all 9 sequences done."
done

collect "avia" "${AVIA_SEQS[@]}"

############################################
# 2. NTU ABLATION (4 configs x 9 seqs)
############################################
log ""
log "======== [2/2] NTU ABLATION (rate=${RATE}) ========"

NTU_EXTRA_VOLS="-v ${NTU_DATA}:/root/catkin_ws/data/ntu_viral:ro"

for abl_idx in "${!ABLATION_LABELS[@]}"; do
  label="${ABLATION_LABELS[$abl_idx]}"
  overlay="${ABLATION_OVERLAYS[$abl_idx]}"
  log ""
  log "  [NTU] Config $((abl_idx+1))/${#ABLATION_LABELS[@]}: ${label} (${overlay})"

  # Pre-create output dirs for all sequences
  for SEQ in "${NTU_SEQS[@]}"; do
    mkdir -p "${OUT_ROOT}/ntu/${label}/${SEQ}"
  done

  # Batch 1/3: eee_01, eee_02, eee_03
  log "    Batch 1/3: eee_01, eee_02, eee_03"
  start_containers_for_label "ntu" "$label" "$NTU_EXTRA_VOLS" 3
  run_ntu_batch "$overlay" "${NTU_SEQS[@]:0:3}"

  # Batch 2/3: nya_01, nya_02, nya_03
  log "    Batch 2/3: nya_01, nya_02, nya_03"
  start_containers_for_label "ntu" "$label" "$NTU_EXTRA_VOLS" 3
  run_ntu_batch "$overlay" "${NTU_SEQS[@]:3:3}"

  # Batch 3/3: sbs_01, sbs_02, sbs_03
  log "    Batch 3/3: sbs_01, sbs_02, sbs_03"
  start_containers_for_label "ntu" "$label" "$NTU_EXTRA_VOLS" 3
  run_ntu_batch "$overlay" "${NTU_SEQS[@]:6:3}"

  log "  [NTU] ${label} — all 9 sequences done."
done

collect "ntu" "${NTU_SEQS[@]}"

cleanup

log ""
log "================================================================"
log "  CANONICAL ABLATION COMPLETE — $(date)"
log "  Results: ${OUT_ROOT}/"
log "================================================================"
