#!/bin/bash
# run_S12_V3_determinism.sh — Sprint 12 V3 determinism SOP.
#
# Avia 3-seq (Dark01, Dynamic03, Varying-illu03) × 3 runs each at rate=1.0
# with default DG-A=OFF. Verifies:
#   - Inter-run CV=0% (bit-identical across 3 runs of same config)
#   - DG-A code path doesn't break determinism even when OFF
#   - Sprint 5 R9-C2++ baseline preserved (I-1)
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S12_V3_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/v3.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
IMAGE="tofslam:ros1"
TIMEOUT_S=2400

declare -A SEQ_CFGS
SEQ_CFGS[Dark01]="avia_v6_seq/dark01.yaml"
SEQ_CFGS[Dynamic03]="avia_v6_seq/dynamic03.yaml"
SEQ_CFGS[Varying-illu03]="avia_v6_seq/varying_illu03.yaml"

CONTAINERS=(tofslam_V3_1 tofslam_V3_2 tofslam_V3_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== S12 V3 determinism SOP (DG-A=OFF default) ===="
log "3 seqs × 3 runs @ rate=1.0, deterministic_queue_delay_ms=100"

run_batch() {
  local seq="$1"; local run_id="$2"
  local cfg="${SEQ_CFGS[$seq]}"
  cleanup; sleep 1
  # 3 parallel runs of the SAME seq (different output dirs)
  for i in 0 1 2; do
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
  for i in 0 1 2; do
    docker exec "${CONTAINERS[$i]}" bash -lc \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
  done
  wait
  local PIDS=()
  for i in 0 1 2; do
    out_dir="dump/${seq}_r${i}"
    log "    Run ${seq} r${i} on port ${PORTS[$i]}..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "bash /root/catkin_ws/docker/run_avia_exp.sh ${cfg} ${seq} ${out_dir} ${PORTS[$i]} ${RATE} 2>&1" \
      > "${OUT_ROOT}/${seq}_r${i}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done
  for c in "${CONTAINERS[@]}"; do
    docker exec "$c" bash -lc "killall -9 tofslam_node rosbag rosout rosmaster roscore 2>/dev/null || true" 2>/dev/null || true
  done
}

# 3 seqs in series, 3 parallel runs each
for seq in Dark01 Dynamic03 Varying-illu03; do
  log ""; log "==== ${seq} × 3 runs ===="
  run_batch "${seq}" "all"
done

log ""; log "==== CV=0% Analysis ===="
for seq in Dark01 Dynamic03 Varying-illu03; do
  vals=()
  for r in 0 1 2; do
    ate_file="${OUT_ROOT}/${seq}_r${r}/ate_result.txt"
    if [[ -f "$ate_file" ]]; then
      ate=$(grep -oE "^rmse: [0-9.]+" "$ate_file" | awk '{print $2}')
      vals+=("$ate")
      log "  ${seq} r${r}: ${ate}"
    else
      log "  ${seq} r${r}: MISSING"
    fi
  done
  # CV computation
  if [[ ${#vals[@]} -eq 3 ]]; then
    cv=$(python3 -c "
import statistics
v = [${vals[0]}, ${vals[1]}, ${vals[2]}]
m = statistics.mean(v)
s = statistics.stdev(v)
cv = 100.0 * s / m if m > 0 else 0
print(f'{cv:.6f}')")
    log "  ${seq} CV%: ${cv}"
    if python3 -c "exit(0 if ${cv} < 0.0001 else 1)" 2>/dev/null; then
      log "  ${seq}: PASS (CV ≈ 0)"
    else
      log "  ${seq}: FAIL (CV > 0 — non-deterministic)"
    fi
  fi
done

log ""; log "==== DONE $(date) ===="
