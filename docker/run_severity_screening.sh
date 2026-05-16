#!/bin/bash
# run_severity_screening.sh — Outdoor Avia 9-seq screening for Eq.(4) severity scaling
#
# Tests severity-scaled alpha modulation (ratio_ref parameter) against
# the baseline per-seq configs (avia_v6_seq/*.yaml).
#
# Usage:
#   bash docker/run_severity_screening.sh [RATIO_REF] [LABEL]
#
# Default: RATIO_REF=0.001, LABEL=eq4_severity_r{ratio}
# Rate: 3.0 (screening, non-canonical)
set -e
cd "$(dirname "$0")/.."

RATIO_REF="${1:-0.001}"
LABEL="${2:-eq4_severity_r${RATIO_REF}}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}/avia"

RATE="3.0"
IMAGE="tofslam:ros1"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_sev_1 tofslam_sev_2 tofslam_sev_3)
MEM="3g"

log() { echo "$*" | tee -a "${LOG}"; }

# Track overlay files for cleanup
OVERLAY_FILES=()

cleanup_containers() {
  for c in "${CONTAINERS[@]}"; do
    docker rm -f "$c" 2>/dev/null || true
  done
}

cleanup_all() {
  cleanup_containers
  # Remove overlay configs from host src (unique *_sev.yaml names)
  for f in "${OVERLAY_FILES[@]}"; do
    rm -f "$f" 2>/dev/null || true
  done
}
trap cleanup_all EXIT

log "================================================================"
log "  Eq.(4) Severity Screening — $(date)"
log "  ratio_ref: ${RATIO_REF}"
log "  Rate: ${RATE} (screening)"
log "  Output: ${OUT_ROOT}/avia/"
log "================================================================"

# ── Create overlay configs in host src dir ──
# Append frontend_degen_severity_ratio_ref to each per-seq config.
# Overlay files use *_sev.yaml suffix (unique, cleaned up at exit).
declare -A SEQ_CFGS
AVIA_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)
BASE_CFGS=(dark01 dark02 dynamic03 dynamic04 occlusion03 occlusion04 varying_illu03 varying_illu04 varying_illu05)
CONFIG_DIR="src/tof_slam/config"

for i in "${!AVIA_SEQS[@]}"; do
  src="${CONFIG_DIR}/avia_v6_seq/${BASE_CFGS[$i]}.yaml"
  dst="${CONFIG_DIR}/${BASE_CFGS[$i]}_sev.yaml"
  cp "$src" "$dst"
  echo "" >> "$dst"
  echo "# ---- Eq.(4) Severity Scaling Overlay ----" >> "$dst"
  echo "frontend_degen_severity_ratio_ref: ${RATIO_REF}" >> "$dst"
  OVERLAY_FILES+=("$dst")
  SEQ_CFGS[${AVIA_SEQS[$i]}]="${BASE_CFGS[$i]}_sev.yaml"
done

# Also save copies for reproducibility
OVERLAY_SAVE="${OUT_ROOT}/_overlay_configs"
mkdir -p "${OVERLAY_SAVE}"
for f in "${OVERLAY_FILES[@]}"; do
  cp "$f" "${OVERLAY_SAVE}/"
done
log "  Created ${#AVIA_SEQS[@]} overlay configs"

# ── Start containers ──
cleanup_containers 2>/dev/null || true
sleep 1
for i in 0 1 2; do
  docker run -d --rm --name "${CONTAINERS[$i]}" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_ROOT}/avia:/root/catkin_ws/dump" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    "${IMAGE}" bash -lc "sleep infinity" > /dev/null
done
sleep 2

# ── Build ──
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

# ── Run batches ──
run_batch() {
  local seqs=("$@")
  local n=${#seqs[@]}
  [ $n -gt 3 ] && n=3
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${seqs[$i]}"
    local cfg="${SEQ_CFGS[$seq]}"
    local port="${PORTS[$i]}"
    local out="/root/catkin_ws/dump/${seq}"
    log "    [${CONTAINERS[$i]}] ${seq} (${cfg})"
    docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
      "${cfg}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then
      log "    ✓ ${seqs[$i]} DONE"
    else
      log "    ✗ ${seqs[$i]} FAILED"
    fi
  done
}

log ""
log "  Batch 1/3: Dark01, Dark02, Dynamic03"
run_batch Dark01 Dark02 Dynamic03
log "  Batch 2/3: Dynamic04, Occlusion03, Occlusion04"
run_batch Dynamic04 Occlusion03 Occlusion04
log "  Batch 3/3: VI03, VI04, VI05"
run_batch Varying-illu03 Varying-illu04 Varying-illu05

# ── Collect results ──
log ""
log "  --- Avia Severity Screening Results (ratio_ref=${RATIO_REF}) ---"
total=0; count=0
for seq in "${AVIA_SEQS[@]}"; do
  f="${OUT_ROOT}/avia/${seq}/ate_result.txt"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
    [ -z "$rmse" ] && rmse=$(grep "ATE RMSE:" "$f" 2>/dev/null | awk '{print $3}')
    if [ -n "$rmse" ]; then
      printf "  %-20s %.4f m\n" "$seq" "$rmse" | tee -a "${LOG}"
      total=$(python3 -c "print(${total} + ${rmse})")
      count=$((count + 1))
    else
      printf "  %-20s PARSE_FAIL\n" "$seq" | tee -a "${LOG}"
    fi
  else
    printf "  %-20s FAIL\n" "$seq" | tee -a "${LOG}"
  fi
done
if [ $count -gt 0 ]; then
  mean=$(python3 -c "print(f'{${total}/${count}:.4f}')")
  log "  Mean (${count}/${#AVIA_SEQS[@]}): ${mean} m"
fi

# ── SOTA comparison ──
declare -A SOTA
SOTA[Dark01]=0.118; SOTA[Dark02]=0.645; SOTA[Dynamic03]=0.151
SOTA[Dynamic04]=0.367; SOTA[Occlusion03]=0.110; SOTA[Occlusion04]=0.147
SOTA[Varying-illu03]=0.363; SOTA[Varying-illu04]=0.110; SOTA[Varying-illu05]=0.142

declare -A CAN
CAN[Dark01]=0.142; CAN[Dark02]=0.601; CAN[Dynamic03]=0.168
CAN[Dynamic04]=0.287; CAN[Occlusion03]=0.179; CAN[Occlusion04]=0.267
CAN[Varying-illu03]=0.608; CAN[Varying-illu04]=0.240; CAN[Varying-illu05]=0.176

log ""
log "  --- Comparison (CAN = V6a-seq baseline 0.296m) ---"
for seq in "${AVIA_SEQS[@]}"; do
  f="${OUT_ROOT}/avia/${seq}/ate_result.txt"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
    [ -z "$rmse" ] && rmse=$(grep "ATE RMSE:" "$f" 2>/dev/null | awk '{print $3}')
    if [ -n "$rmse" ]; then
      can="${CAN[$seq]}"
      delta=$(python3 -c "print(f'{(${rmse}-${can})/${can}*100:+.1f}')")
      printf "  %-20s %.4f m  (vs CAN: %s%%)\n" "$seq" "$rmse" "$delta" | tee -a "${LOG}"
    fi
  fi
done

cleanup_all
log ""
log "  Completed: $(date)"
