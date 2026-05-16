#!/bin/bash
# run_S13_V4_sanity.sh — Post-V4 winner sanity check on Dyn03 + VI03.
#
# After V4 grid identifies G5 winner on Dark01, re-run on Dyn03 + VI03
# with the WINNER CONFIG to verify generalization beyond Dark01.
#
# If Dyn03 ATE > 0.5m or VI03 ATE > 0.8m → architect §6 R3 ABORT
# (ρ_ref / σ²_base / ε didn't generalize → per-seq overlay re-emergence).
#
# Usage: bash docker/run_S13_V4_sanity.sh <WINNER_CONFIG_BASENAME>
#   e.g. bash docker/run_S13_V4_sanity.sh v4_5_s0.05_r15_e1.0e-2.yaml
#
set -e
cd "$(dirname "$0")/.."

WINNER_CFG="${1:?Usage: run_S13_V4_sanity.sh <winner config basename, e.g. v4_5_s0.05_r15_e1.0e-2.yaml>}"
# Assumes V4 generated config is still at dump/S13_V4_*/configs/<basename>
V4_DUMP="${2:-$(ls -1dt dump/S13_V4_* | head -1)}"
CFG_PATH="${V4_DUMP}/configs/${WINNER_CFG}"
[ -f "$CFG_PATH" ] || { echo "ERROR: winner config $CFG_PATH not found"; exit 1; }

LABEL="${3:-S13_V4_sanity_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/sanity.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
IMAGE="tofslam:ros1"
TIMEOUT_S=1200

SEQS=(Dynamic03 Varying-illu03)

CONTAINERS=(tofslam_S13V4S_1 tofslam_S13V4S_2)
CPUSETS=("0-3" "4-7")
PORTS=(11311 11312)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== V4 winner sanity smoke ===="
log "Winner config: ${WINNER_CFG}"
log "Path:          ${CFG_PATH}"
log "Seqs:          ${SEQS[*]}"
log "Rate:          ${RATE}"

# Copy winner config to a stable location inside src/tof_slam/config/v4_winner.yaml so loader finds it
cp "${CFG_PATH}" "src/tof_slam/config/v4_winner.yaml"

cleanup; sleep 1
for i in 0 1; do
  docker run -d --rm --init --name "${CONTAINERS[$i]}" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump:rw" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    -v "${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro" \
    "${IMAGE}" bash -lc "sleep infinity" > /dev/null
done
sleep 2
log "Building..."
for i in 0 1; do
  docker exec "${CONTAINERS[$i]}" bash -lc \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
done
wait
log "Build done."

PIDS=()
for i in 0 1; do
  seq="${SEQS[$i]}"
  log "    ${seq} on port ${PORTS[$i]}..."
  timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
    "bash /root/catkin_ws/docker/run_avia_exp.sh v4_winner.yaml ${seq} dump/${seq} ${PORTS[$i]} ${RATE} 2>&1" \
    > "${OUT_ROOT}/${seq}_stdout.log" 2>&1 &
  PIDS+=($!)
done
for pid in "${PIDS[@]}"; do wait "$pid" || true; done

# cleanup the temporary winner copy
rm -f "src/tof_slam/config/v4_winner.yaml"

log ""
log "==== Results ===="
for seq in "${SEQS[@]}"; do
  ate_file="${OUT_ROOT}/${seq}/ate_result.txt"
  if [ -f "$ate_file" ]; then
    rmse=$(grep -oE "^rmse: [0-9.]+" "$ate_file" | awk '{print $2}')
    log "  ${seq}: ${rmse}"
  else
    log "  ${seq}: MISSING"
  fi
done

# Gate checks (per docs/results/s13_v3_result_20260513.md "next steps")
DYN_ATE=$(grep -oE "^rmse: [0-9.]+" "${OUT_ROOT}/Dynamic03/ate_result.txt" 2>/dev/null | awk '{print $2}')
VI_ATE=$(grep -oE "^rmse: [0-9.]+" "${OUT_ROOT}/Varying-illu03/ate_result.txt" 2>/dev/null | awk '{print $2}')

log ""
log "Gate: Dyn03 ≤ 0.5m, VI03 ≤ 0.8m"
ABORT=0
if [ -n "$DYN_ATE" ] && python3 -c "exit(0 if ${DYN_ATE} > 0.5 else 1)" 2>/dev/null; then
  log "  Dyn03 ${DYN_ATE} > 0.5m → R3 trigger"
  ABORT=1
fi
if [ -n "$VI_ATE" ] && python3 -c "exit(0 if ${VI_ATE} > 0.8 else 1)" 2>/dev/null; then
  log "  VI03 ${VI_ATE} > 0.8m → R3 trigger"
  ABORT=1
fi

if [ $ABORT -eq 1 ]; then
  log "==== ⚠️  V4 winner did NOT generalize — Rule 16 R3 ABORT candidate ===="
  exit 2
else
  log "==== ✅ V4 winner generalizes — proceed to V5 ===="
  exit 0
fi
