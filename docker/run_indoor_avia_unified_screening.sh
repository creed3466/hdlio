#!/bin/bash
# run_indoor_avia_unified_screening.sh — Screen unified avia_indoor.yaml on all 8 indoor sequences
# Phase 1: alpha_degen_floor=0.3 + ratio_threshold=0.001 + soft_floor=1.0
# Baseline: backup mean=0.568m, current canonical mean=0.919m
# Target: mean <= 0.568m
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-indoor_avia_unified}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"

RATE="3.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_aiu_1 tofslam_aiu_2 tofslam_aiu_3)
MEM="3g"

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do
    docker rm -f "$c" 2>/dev/null || true
  done
}
trap cleanup EXIT

log "================================================================"
log "  INDOOR AVIA UNIFIED CONFIG SCREENING"
log "  $(date)"
log "  Config: avia_indoor.yaml (unified)"
log "  Rate: ${RATE}"
log "  Baseline (backup): mean=0.568m"
log "  Current (canonical): mean=0.919m"
log "================================================================"

# Start containers
cleanup
sleep 1
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
sleep 2

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

CONFIG="avia_indoor.yaml"

# ===== Batch 1: Dark03, Dark04, Dynamic01 =====
log ""
log "  === Batch 1/3: Dark03, Dark04, Dynamic01 ==="
BATCH1=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01)
LABELS1=(aiDk03 aiDk04 aiDy01)
PIDS=()
for si in 0 1 2; do
  seq="${BATCH1[$si]}"
  label="${LABELS1[$si]}"
  port="${PORTS[$si]}"
  out="/root/catkin_ws/dump/${label}"
  log "    [${CONTAINERS[$si]}] ${label} (${CONFIG})"
  docker exec "${CONTAINERS[$si]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
    "${CONFIG}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
  PIDS+=($!)
done
for i in "${!PIDS[@]}"; do
  if wait "${PIDS[$i]}" 2>/dev/null; then
    log "    done ${BATCH1[$i]}"
  else
    log "    FAILED ${BATCH1[$i]}"
  fi
done

# ===== Batch 2: Dynamic02, Occlusion01, Occlusion02 =====
log ""
log "  === Batch 2/3: Dynamic02, Occlusion01, Occlusion02 ==="
BATCH2=(indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02)
LABELS2=(aiDy02 aiOc01 aiOc02)
PIDS=()
for si in 0 1 2; do
  seq="${BATCH2[$si]}"
  label="${LABELS2[$si]}"
  port="${PORTS[$si]}"
  out="/root/catkin_ws/dump/${label}"
  log "    [${CONTAINERS[$si]}] ${label} (${CONFIG})"
  docker exec "${CONTAINERS[$si]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
    "${CONFIG}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
  PIDS+=($!)
done
for i in "${!PIDS[@]}"; do
  if wait "${PIDS[$i]}" 2>/dev/null; then
    log "    done ${BATCH2[$i]}"
  else
    log "    FAILED ${BATCH2[$i]}"
  fi
done

# ===== Batch 3: Varying-illu01, Varying-illu02 =====
log ""
log "  === Batch 3/3: Varying-illu01, Varying-illu02 ==="
BATCH3=(indoor_Varying-illu01 indoor_Varying-illu02)
LABELS3=(aiVI01 aiVI02)
PIDS=()
for si in 0 1; do
  seq="${BATCH3[$si]}"
  label="${LABELS3[$si]}"
  port="${PORTS[$si]}"
  out="/root/catkin_ws/dump/${label}"
  log "    [${CONTAINERS[$si]}] ${label} (${CONFIG})"
  docker exec "${CONTAINERS[$si]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
    "${CONFIG}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
  PIDS+=($!)
done
for i in "${!PIDS[@]}"; do
  if wait "${PIDS[$i]}" 2>/dev/null; then
    log "    done ${BATCH3[$i]}"
  else
    log "    FAILED ${BATCH3[$i]}"
  fi
done

# ===== Results =====
log ""
log "  ======== RESULTS ========"
log ""
log "  Seq                 Unified   Backup   Canon    Delta-vs-Backup"
log "  ---                 -------   ------   -----    ---------------"

ALL_LABELS=(aiDk03 aiDk04 aiDy01 aiDy02 aiOc01 aiOc02 aiVI01 aiVI02)
BACKUP_VALS=(0.418 0.481 0.728 0.418 0.883 0.604 0.520 0.495)
CANON_VALS=(1.843 0.410 1.071 0.518 0.796 1.167 0.849 0.701)

total=0
count=0
for i in "${!ALL_LABELS[@]}"; do
  label="${ALL_LABELS[$i]}"
  backup="${BACKUP_VALS[$i]}"
  canon="${CANON_VALS[$i]}"
  f="${OUT_ROOT}/${label}/ate_result.txt"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" | awk '{print $2}')
    if [ -n "$rmse" ]; then
      delta=$(python3 -c "r=${rmse}; b=${backup}; print(f'{(r-b)/b*100:+.1f}%')")
      printf "  %-20s %.3f    %.3f    %.3f    %s\n" "$label" "$rmse" "$backup" "$canon" "$delta" | tee -a "${LOG}"
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
  mean=$(python3 -c "print(f'{${total}/${count}:.3f}')")
  backup_mean="0.568"
  canon_mean="0.919"
  delta_backup=$(python3 -c "m=${mean}; print(f'{(m-0.568)/0.568*100:+.1f}%')")
  delta_canon=$(python3 -c "m=${mean}; print(f'{(m-0.919)/0.919*100:+.1f}%')")
  log ""
  log "  Mean (${count}/8):     ${mean}    ${backup_mean}    ${canon_mean}    ${delta_backup} vs backup, ${delta_canon} vs canon"
fi

log ""
log "======== Done: $(date) ========"
