#!/bin/bash
# run_S13_V3_determinism.sh — Sprint 13 V3 (handoff) / B.V2 (architect G2) determinism SOP.
#
# Avia 3-seq (Dark01, Dynamic03, Varying-illu03) × 3 runs each at rate=1.0
# with **unified avia_outdoor.yaml** (P1 anisotropic IEKF flag=ON, post-V0).
# Verifies architect invariant I-3 (R-LAG/Determinism) on the new P1 path:
#   - Inter-run CV=0% (bit-identical across 3 runs of same seq)
#   - P1 code path (Ω_eff_L1 + Ω_eff_L2 + DG-A γ_cached) preserves determinism
#   - Unified config behavior reproducible
#
# Diff vs run_S12_V3_determinism.sh:
#   - SEQ_CFGS: per-seq avia_v6_seq/*.yaml  →  unified avia_outdoor.yaml (for all 3)
#   - Sprint 13 P1 keys active (frontend_anisotropic_iekf_enable=true at V0 commit)
#
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_V3_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/v3.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
IMAGE="tofslam:ros1"
TIMEOUT_S=2400

# Unified config for all 3 seqs — tests P1 path under unified scope (architect §1 objective).
declare -A SEQ_CFGS
SEQ_CFGS[Dark01]="avia_outdoor.yaml"
SEQ_CFGS[Dynamic03]="avia_outdoor.yaml"
SEQ_CFGS[Varying-illu03]="avia_outdoor.yaml"

CONTAINERS=(tofslam_S13V3_1 tofslam_S13V3_2 tofslam_S13V3_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== S13 V3 determinism SOP (P1 anisotropic IEKF flag=ON) ===="
log "3 seqs × 3 runs @ rate=1.0, unified avia_outdoor.yaml, V0 commit applied"
log "Architect G2 invariant I-3 check: γ_cached deterministic, Σ propagation deterministic"

run_batch() {
  local seq="$1"
  local cfg="${SEQ_CFGS[$seq]}"
  cleanup; sleep 1
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
    log "    Run ${seq} r${i} on port ${PORTS[$i]} cfg=${cfg}..."
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
  log ""; log "==== ${seq} × 3 runs (unified avia_outdoor.yaml, P1 ON) ===="
  run_batch "${seq}"
done

log ""; log "==== CV=0% Analysis ===="
ALL_PASS=1
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
      ALL_PASS=0
    fi
  done
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
      ALL_PASS=0
    fi
  else
    ALL_PASS=0
  fi
done

log ""
if [[ ${ALL_PASS} -eq 1 ]]; then
  log "==== S13 V3 DONE — ALL 3 seqs PASS (CV=0%) — $(date) ===="
  exit 0
else
  log "==== S13 V3 FAIL — at least one seq non-deterministic OR missing — $(date) ===="
  exit 1
fi
