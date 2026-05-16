#!/bin/bash
# run_indoor_avia_perseq_screening.sh — Screen per-seq indoor Avia configs (Eq4 ON)
# Per-seq configs: avia_indoor_seq/i{Dark03,Dark04,Dyn01,Dyn02,Occ01,Occ02,VI01,VI02}.yaml
# All configs have Eq(4) enabled with per-seq ICDR/std-degen and floor settings.
# Group A (std degen detect-only): Dk03, Dy02
# Group B (ICDR + Eq4): Dk04, Dy01, Oc01, Oc02, VI01, VI02
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-indoor_avia_perseq}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"

RATE="3.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_ps_1 tofslam_ps_2 tofslam_ps_3)
MEM="3g"

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do
    docker rm -f "$c" 2>/dev/null || true
  done
}
trap cleanup EXIT

log "================================================================"
log "  INDOOR AVIA PER-SEQ CONFIG SCREENING (Eq4 ON)"
log "  $(date)"
log "  Rate: ${RATE}"
log "  Target: mean ≤ 0.568m (backup level)"
log "  Backup per-seq mean: 0.568m"
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

# Per-seq config mapping (config inside avia_indoor_seq/)
declare -A SEQ_CONFIGS
SEQ_CONFIGS[indoor_Dark03]="avia_indoor_seq/iDark03.yaml"
SEQ_CONFIGS[indoor_Dark04]="avia_indoor_seq/iDark04.yaml"
SEQ_CONFIGS[indoor_Dynamic01]="avia_indoor_seq/iDyn01.yaml"
SEQ_CONFIGS[indoor_Dynamic02]="avia_indoor_seq/iDyn02.yaml"
SEQ_CONFIGS[indoor_Occlusion01]="avia_indoor_seq/iOcc01.yaml"
SEQ_CONFIGS[indoor_Occlusion02]="avia_indoor_seq/iOcc02.yaml"
SEQ_CONFIGS[indoor_Varying-illu01]="avia_indoor_seq/iVI01.yaml"
SEQ_CONFIGS[indoor_Varying-illu02]="avia_indoor_seq/iVI02.yaml"

run_batch() {
  local batch_name="$1"
  shift
  local seqs=()
  local labels=()
  while [ $# -gt 0 ]; do
    seqs+=("$1")
    labels+=("$2")
    shift 2
  done

  log ""
  log "  === ${batch_name} ==="
  PIDS=()
  for si in "${!seqs[@]}"; do
    local seq="${seqs[$si]}"
    local label="${labels[$si]}"
    local port="${PORTS[$si]}"
    local cfg="${SEQ_CONFIGS[$seq]}"
    local out="/root/catkin_ws/dump/${label}"
    log "    [${CONTAINERS[$si]}] ${label} (${cfg})"
    docker exec "${CONTAINERS[$si]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
      "${cfg}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then
      log "    done ${seqs[$i]}"
    else
      log "    FAILED ${seqs[$i]}"
    fi
  done
}

# ===== Batch 1: Dark03, Dark04, Dynamic01 =====
run_batch "Batch 1/3: Dk03(std+Eq4), Dk04(ICDR+Eq4), Dy01(ICDR+TIP+Eq4)" \
  indoor_Dark03 aiDk03 \
  indoor_Dark04 aiDk04 \
  indoor_Dynamic01 aiDy01

# ===== Batch 2: Dynamic02, Occlusion01, Occlusion02 =====
run_batch "Batch 2/3: Dy02(std+Eq4), Oc01(ICDR+Eq4 f=0.85), Oc02(ICDR+Eq4)" \
  indoor_Dynamic02 aiDy02 \
  indoor_Occlusion01 aiOc01 \
  indoor_Occlusion02 aiOc02

# ===== Batch 3: Varying-illu01, Varying-illu02 =====
run_batch "Batch 3/3: VI01(ICDR+TIP+Eq4), VI02(ICDR+TIP+Eq4)" \
  indoor_Varying-illu01 aiVI01 \
  indoor_Varying-illu02 aiVI02

# ===== Results =====
log ""
log "  ======== PER-SEQ SCREENING RESULTS ========"
log ""

# Per-seq design summary
log "  Design:"
log "    Dk03: std degen(detect-only) + Eq4(f=0.0)"
log "    Dk04: ICDR(rho=0.45,TIP=OFF) + Eq4(f=0.0)"
log "    Dy01: ICDR(rho=0.20,TIP=ON,β=0.7) + Eq4(f=0.0)"
log "    Dy02: std degen(detect-only) + Eq4(f=0.0)"
log "    Oc01: ICDR(rho=0.45,TIP=OFF) + Eq4(f=0.85)"
log "    Oc02: ICDR(rho=0.45,TIP=OFF) + Eq4(f=0.0)"
log "    VI01: ICDR(rho=0.30,TIP=ON,d=0.5) + Eq4(f=0.0)"
log "    VI02: ICDR(rho=0.35,TIP=ON,β=0.3) + Eq4(f=0.0)"
log ""

printf "  %-12s  %-10s  %-10s  %-10s  %-10s  %-14s\n" \
  "Seq" "PerSeq" "Unified" "Backup" "V4-ICDR" "Δ-vs-Backup" | tee -a "${LOG}"
printf "  %-12s  %-10s  %-10s  %-10s  %-10s  %-14s\n" \
  "---" "------" "-------" "------" "------" "-----------" | tee -a "${LOG}"

ALL_LABELS=(aiDk03 aiDk04 aiDy01 aiDy02 aiOc01 aiOc02 aiVI01 aiVI02)
BACKUP_VALS=(0.418 0.481 0.728 0.418 0.883 0.604 0.520 0.495)
V4_VALS=(0.369 0.491 1.033 0.529 0.802 1.341 0.854 0.598)
UNIFIED_VALS=(0.369 0.491 1.033 0.529 0.802 1.341 0.854 0.598)

total=0
count=0
for i in "${!ALL_LABELS[@]}"; do
  label="${ALL_LABELS[$i]}"
  backup="${BACKUP_VALS[$i]}"
  v4="${V4_VALS[$i]}"
  uni="${UNIFIED_VALS[$i]}"
  f="${OUT_ROOT}/${label}/ate_result.txt"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" | awk '{print $2}')
    if [ -n "$rmse" ]; then
      delta=$(python3 -c "r=${rmse}; b=${backup}; print(f'{(r-b)/b*100:+.1f}%')")
      printf "  %-12s  %-10.3f  %-10.3f  %-10.3f  %-10.3f  %-14s\n" \
        "$label" "$rmse" "$uni" "$backup" "$v4" "$delta" | tee -a "${LOG}"
      total=$(python3 -c "print(${total} + ${rmse})")
      count=$((count + 1))
    else
      printf "  %-12s  PARSE_FAIL\n" "$label" | tee -a "${LOG}"
    fi
  else
    printf "  %-12s  FAIL\n" "$label" | tee -a "${LOG}"
  fi
done

if [ $count -gt 0 ]; then
  mean=$(python3 -c "print(f'{${total}/${count}:.3f}')")
  backup_mean="0.568"
  delta_backup=$(python3 -c "m=${mean}; print(f'{(m-0.568)/0.568*100:+.1f}%')")
  log ""
  log "  Mean (${count}/8):  ${mean}     backup=${backup_mean}     ${delta_backup} vs backup"
  log ""
  if python3 -c "exit(0 if float('${mean}') <= 0.568 else 1)" 2>/dev/null; then
    log "  ✓ TARGET MET: mean ≤ 0.568m"
  else
    log "  ✗ TARGET NOT MET: mean > 0.568m"
  fi
fi

log ""
log "======== Done: $(date) ========"
