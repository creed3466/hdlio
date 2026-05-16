#!/bin/bash
# run_canonical_eval_indoor.sh — Canonical paper evaluation: Indoor (8 sequences)
# Appends to dump/paper_canonical/ alongside avia/mid360/ntu results.
# Rate=1.0 (mandatory for paper).
set -e
cd "$(dirname "$0")/.."

LABEL="paper_canonical"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"

RATE="1.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_canon_1 tofslam_canon_2 tofslam_canon_3)
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

start_and_build() {
  cleanup
  sleep 3
  for i in 0 1 2; do
    docker run -d --rm --name "${CONTAINERS[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src:ro" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}/indoor:/root/catkin_ws/dump" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      -v "${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 3
  log "  [Build] indoor..."
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
}

run_batch() {
  local runner=$1
  shift
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}
  [ $n -gt 3 ] && n=3
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${batch_seqs[$i]}"
    local cfg="${SEQ_CFGS[$seq]}"
    local label="${SEQ_LABELS[$seq]}"
    local port="${PORTS[$i]}"
    local out="/root/catkin_ws/dump/${label}"
    log "    [${CONTAINERS[$i]}] ${label} (${cfg})"
    docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/${runner}" \
      "${cfg}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then
      log "    ✓ ${batch_seqs[$i]} DONE"
    else
      log "    ✗ ${batch_seqs[$i]} FAILED"
    fi
  done
}

collect_results() {
  local labels=("$@")
  local total=0 count=0
  log ""
  log "  --- indoor Results ---"
  for label in "${labels[@]}"; do
    local f="${OUT_ROOT}/indoor/${label}/ate_result.txt"
    if [ -f "$f" ]; then
      local rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
      if [ -z "$rmse" ]; then
        rmse=$(grep "ATE RMSE:" "$f" 2>/dev/null | awk '{print $3}')
      fi
      if [ -n "$rmse" ]; then
        printf "  %-20s %.4f m\n" "$label" "$rmse" | tee -a "${LOG}"
        total=$(python3 -c "print(${total} + ${rmse})")
        count=$((count + 1))
      else
        printf "  %-20s PARSE_FAIL\n" "$label" | tee -a "${LOG}"
      fi
    else
      printf "  %-20s FAIL\n" "$label" | tee -a "${LOG}"
    fi
  done
  if [ $count -gt 0 ]; then
    local mean=$(python3 -c "print(f'{${total}/${count}:.4f}')")
    log "  Mean (${count}/${#labels[@]}): ${mean} m"
  fi
}

copy_configs() {
  local labels=("$@")
  local cfg_dir="${OUT_ROOT}/indoor/_configs"
  mkdir -p "${cfg_dir}"
  for seq in "${!SEQ_CFGS[@]}"; do
    local cfg="${SEQ_CFGS[$seq]}"
    local src="src/tof_slam/config/${cfg}"
    if [ -f "$src" ]; then
      cp "$src" "${cfg_dir}/$(basename "$cfg")"
    fi
  done
}

############################################
# INDOOR (8 sequences, per-seq configs)
############################################

declare -A SEQ_CFGS
declare -A SEQ_LABELS

INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
INDOOR_LABELS=(iDark03 iDark04 iDyn01 iDyn02 iOcc01 iOcc02 iVI01 iVI02)

# Unified v3c config for indoor+outdoor (verified mean=0.1504m, -0.05% vs v1)
SEQ_CFGS[indoor_Dark03]="unified_mid360_v3c.yaml"
SEQ_CFGS[indoor_Dark04]="unified_mid360_v3c.yaml"
SEQ_CFGS[indoor_Dynamic01]="unified_mid360_v3c.yaml"
SEQ_CFGS[indoor_Dynamic02]="unified_mid360_v3c.yaml"
SEQ_CFGS[indoor_Occlusion01]="unified_mid360_v3c.yaml"
SEQ_CFGS[indoor_Occlusion02]="unified_mid360_v3c.yaml"
SEQ_CFGS[indoor_Varying-illu01]="unified_mid360_v3c.yaml"
SEQ_CFGS[indoor_Varying-illu02]="unified_mid360_v3c.yaml"

SEQ_LABELS[indoor_Dark03]="iDark03"
SEQ_LABELS[indoor_Dark04]="iDark04"
SEQ_LABELS[indoor_Dynamic01]="iDyn01"
SEQ_LABELS[indoor_Dynamic02]="iDyn02"
SEQ_LABELS[indoor_Occlusion01]="iOcc01"
SEQ_LABELS[indoor_Occlusion02]="iOcc02"
SEQ_LABELS[indoor_Varying-illu01]="iVI01"
SEQ_LABELS[indoor_Varying-illu02]="iVI02"

log ""
log "================================================================"
log "  CANONICAL EVAL — INDOOR (8 seq, rate=${RATE})"
log "  $(date)"
log "================================================================"

start_and_build

log "  Batch 1/3: iDark03, iDark04, iDyn01"
run_batch "run_avia_exp.sh" "${INDOOR_SEQS[@]:0:3}"
log "  Batch 2/3: iDyn02, iOcc01, iOcc02"
run_batch "run_avia_exp.sh" "${INDOOR_SEQS[@]:3:3}"
log "  Batch 3/3: iVI01, iVI02"
run_batch "run_avia_exp.sh" "${INDOOR_SEQS[@]:6:2}"

collect_results "${INDOOR_LABELS[@]}"
copy_configs "${INDOOR_LABELS[@]}"

cleanup

log ""
log "================================================================"
log "  CANONICAL EVAL — INDOOR COMPLETE"
log "  $(date)"
log "================================================================"
