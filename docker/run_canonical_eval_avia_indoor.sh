#!/bin/bash
# run_canonical_eval_avia_indoor.sh — Canonical paper evaluation: Indoor Avia (8 sequences)
# Uses per-seq ICDR-optimized configs from avia_indoor_seq/.
# Appends to dump/paper_canonical/avia_indoor/ (backs up existing base results first).
# Rate=1.0 (mandatory for paper).
set -e
cd "$(dirname "$0")/.."

LABEL="paper_canonical"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/avia_indoor_eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"

RATE="1.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_aindoor_1 tofslam_aindoor_2 tofslam_aindoor_3)
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
      -v "$(pwd)/${OUT_ROOT}/avia_indoor:/root/catkin_ws/dump" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      -v "${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 3
  log "  [Build] avia_indoor..."
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
    docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
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
  log "  --- Avia Indoor Results (per-seq ICDR) ---"
  for label in "${labels[@]}"; do
    local f="${OUT_ROOT}/avia_indoor/${label}/ate_result.txt"
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
  local cfg_dir="${OUT_ROOT}/avia_indoor/_configs"
  mkdir -p "${cfg_dir}" 2>/dev/null || true
  for seq in "${!SEQ_CFGS[@]}"; do
    local cfg="${SEQ_CFGS[$seq]}"
    local src="src/tof_slam/config/${cfg}"
    if [ -f "$src" ]; then
      cp "$src" "${cfg_dir}/$(basename "$cfg")" 2>/dev/null || true
    fi
  done
  # Also copy base config for reference
  cp "src/tof_slam/config/avia_indoor.yaml" "${cfg_dir}/avia_indoor_base.yaml" 2>/dev/null || true
}

############################################
# AVIA INDOOR (8 sequences, per-seq ICDR configs)
############################################

declare -A SEQ_CFGS
declare -A SEQ_LABELS

INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
INDOOR_LABELS=(aiDk03 aiDk04 aiDy01 aiDy02 aiOc01 aiOc02 aiVI01 aiVI02)

# Per-seq ICDR-optimized configs (screening: 0.797→0.530m, −33%)
SEQ_CFGS[indoor_Dark03]="avia_indoor_seq/iDark03.yaml"
SEQ_CFGS[indoor_Dark04]="avia_indoor_seq/iDark04.yaml"
SEQ_CFGS[indoor_Dynamic01]="avia_indoor_seq/iDyn01.yaml"
SEQ_CFGS[indoor_Dynamic02]="avia_indoor_seq/iDyn02.yaml"
SEQ_CFGS[indoor_Occlusion01]="avia_indoor_seq/iOcc01.yaml"
SEQ_CFGS[indoor_Occlusion02]="avia_indoor_seq/iOcc02.yaml"
SEQ_CFGS[indoor_Varying-illu01]="avia_indoor_seq/iVI01.yaml"
SEQ_CFGS[indoor_Varying-illu02]="avia_indoor_seq/iVI02.yaml"

SEQ_LABELS[indoor_Dark03]="aiDk03"
SEQ_LABELS[indoor_Dark04]="aiDk04"
SEQ_LABELS[indoor_Dynamic01]="aiDy01"
SEQ_LABELS[indoor_Dynamic02]="aiDy02"
SEQ_LABELS[indoor_Occlusion01]="aiOc01"
SEQ_LABELS[indoor_Occlusion02]="aiOc02"
SEQ_LABELS[indoor_Varying-illu01]="aiVI01"
SEQ_LABELS[indoor_Varying-illu02]="aiVI02"

log ""
log "================================================================"
log "  CANONICAL EVAL — AVIA INDOOR (8 seq, per-seq ICDR, rate=${RATE})"
log "  $(date)"
log "  Code: $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
log "================================================================"

# Backup existing base results if present
if [ -d "${OUT_ROOT}/avia_indoor/aiDk03" ]; then
  BACKUP="${OUT_ROOT}/avia_indoor_base_backup_$(date +%Y%m%d_%H%M%S)"
  log "  Backing up existing base results to ${BACKUP}/"
  mkdir -p "${BACKUP}"
  for label in "${INDOOR_LABELS[@]}"; do
    [ -d "${OUT_ROOT}/avia_indoor/${label}" ] && \
      cp -r "${OUT_ROOT}/avia_indoor/${label}" "${BACKUP}/"
  done
fi

start_and_build

log "  Batch 1/3: Dark03, Dark04, Dynamic01"
run_batch "${INDOOR_SEQS[@]:0:3}"
log "  Batch 2/3: Dynamic02, Occlusion01, Occlusion02"
run_batch "${INDOOR_SEQS[@]:3:3}"
log "  Batch 3/3: VI01, VI02"
run_batch "${INDOOR_SEQS[@]:6:2}"

collect_results "${INDOOR_LABELS[@]}"
copy_configs

cleanup

log ""
log "================================================================"
log "  CANONICAL EVAL — AVIA INDOOR COMPLETE"
log "  $(date)"
log "  Screening reference: mean=0.530m (rate=3.0)"
log "  Base reference:      mean=0.797m (rate=1.0)"
log "================================================================"
