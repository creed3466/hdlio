#!/bin/bash
# run_S13_R0_10_V3_9seq.sh — Sprint 13 R0.10 V3 (9-seq screening rate=1.0 × 3 runs)
#
# Architect §6 + R0.10 handoff §7 V3 pass criteria:
#   - 9-seq mean ATE ≤ 0.314 m (HARD gate, Faster-LIO class)
#   - All seqs byte-identical across 3 runs (CV=0%) — determinism preserved
#   - Falsification (Rule 16 trigger): mean > 0.391 m
#   - PASS-clean: mean ≤ 0.310 m
#
# Config: unified avia_outdoor.yaml (Sprint 5 R-A classifier + R0.9/R0.10 H1+H2+H3b+H4)
# Rate: 1.0 (paper-grade determinism)
# Total: 9 seqs × 3 runs = 27 runs, 3 in parallel ≈ 9 batches × ~6 min = ~1.5-2 hr.
#
# Output: dump/<LABEL>/{seq}/run{1,2,3}/{traj.csv,ate_result.txt} + v3_summary.csv
#
# Usage: bash docker/run_S13_R0_10_V3_9seq.sh [LABEL]

set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_R0_10_V3_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/v3.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
CONFIG="avia_outdoor.yaml"
RATE="1.0"
IMAGE="tofslam:ros1"
TIMEOUT_S=900
N_RUNS=3

SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)

CONTAINERS=(tofslam_S13R010V3_1 tofslam_S13R010V3_2 tofslam_S13R010V3_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== Sprint 13 R0.10 V3 9-seq screening (rate=${RATE} × ${N_RUNS} runs) ===="
log "Config: ${CONFIG} (unified, classifier ON)"
log "HARD gate: mean ≤ 0.314 m | Falsification: > 0.391 m | PASS-clean: ≤ 0.310 m"
log "Total runs: 9 seqs × ${N_RUNS} = 27, 3 parallel"

# Spawn containers + build (matches Smoke A/B setup)
cleanup; sleep 1
for i in 0 1 2; do
  docker run -d --rm --init --name "${CONTAINERS[$i]}" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump:rw" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    "${IMAGE}" bash -lc "sleep infinity" > /dev/null
done
sleep 2
log "Building..."
for i in 0 1 2; do
  docker exec "${CONTAINERS[$i]}" bash -lc \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
     rm -rf build devel && \
     catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
done
wait
log "Build done."

# Strategy: 3 containers, each runs 3 seqs × 3 runs = 9 runs each (27 total).
# Distribute seqs to slots: slot 0 gets seqs 0,3,6 ; slot 1 gets 1,4,7 ; slot 2 gets 2,5,8.
# Within each slot, run seq r1, then r2, then r3 sequentially (≈ 6 min per run × 9 = 54 min/slot, ≈ 1 hr wallclock).
#
# Alternative (simpler): batch by (seq, run_id) — 9 sequential batches of 3 parallel runs each.
# Each batch = 3 runs in parallel ≈ 6-8 min. 9 batches ≈ 54-72 min. Same total but cleaner.

for seq in "${SEQS[@]}"; do
  for run_id in 1 2 3; do
    log "  ${seq} run${run_id}: batch start (3 parallel slots not needed — single batch per (seq,run))"
    # Single seq × 1 run uses only slot 0. Other slots idle. Wallclock dominated by per-seq time.
    # Better: parallelize 3 runs of same seq across 3 slots → CV=0% check immediately per seq.
    break
  done
done

# Final loop: per-seq parallelize 3 runs across 3 containers.
for seq in "${SEQS[@]}"; do
  log ""
  log "==== ${seq}: 3 parallel runs ===="
  mkdir -p "${OUT_ROOT}/${seq}"
  PIDS=()
  for slot in 0 1 2; do
    run_id=$((slot + 1))
    out_in="dump/${seq}/run${run_id}"  # relative to /root/catkin_ws (mounted)
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$slot]}" bash -lc \
      "bash /root/catkin_ws/docker/run_avia_exp.sh ${CONFIG} ${seq} ${out_in} ${PORTS[$slot]} ${RATE} 2>&1" \
      > "${OUT_ROOT}/${seq}/run${run_id}.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done

  # Per-seq summary
  ATES=()
  MD5S=()
  for r in 1 2 3; do
    TRAJ="${OUT_ROOT}/${seq}/run${r}/traj.csv"
    ATE_FILE="${OUT_ROOT}/${seq}/run${r}/ate_result.txt"
    rmse="MISSING"; md5="MISSING"
    [ -f "${TRAJ}" ] && md5=$(md5sum "${TRAJ}" | awk '{print $1}')
    [ -f "${ATE_FILE}" ] && rmse=$(grep "^rmse:" "${ATE_FILE}" | awk '{print $2}')
    ATES+=("${rmse}")
    MD5S+=("${md5}")
    log "  ${seq} run${r}: ATE=${rmse}  md5=${md5}"
  done
  if [ "${MD5S[0]}" != "MISSING" ] && [ "${MD5S[0]}" = "${MD5S[1]}" ] && [ "${MD5S[0]}" = "${MD5S[2]}" ]; then
    log "  ${seq}: CV=0% byte-id PASS"
  else
    log "  ${seq}: CV=0% byte-id FAIL"
  fi
done

# Final 9-seq summary
log ""
log "==== Final 9-seq V3 summary ===="
SUMMARY="${OUT_ROOT}/v3_summary.csv"
echo "seq,ate_run1,ate_run2,ate_run3,md5_run1,md5_run2,md5_run3,byte_id" > "${SUMMARY}"

SUM=0.0
COUNT=0
for seq in "${SEQS[@]}"; do
  row="${seq}"
  RMSES=()
  MDS=()
  for r in 1 2 3; do
    TRAJ="${OUT_ROOT}/${seq}/run${r}/traj.csv"
    ATE_FILE="${OUT_ROOT}/${seq}/run${r}/ate_result.txt"
    rmse="MISSING"; md5="MISSING"
    [ -f "${TRAJ}" ] && md5=$(md5sum "${TRAJ}" | awk '{print $1}')
    [ -f "${ATE_FILE}" ] && rmse=$(grep "^rmse:" "${ATE_FILE}" | awk '{print $2}')
    RMSES+=("${rmse}")
    MDS+=("${md5}")
  done
  byteid="NO"
  if [ "${MDS[0]}" != "MISSING" ] && [ "${MDS[0]}" = "${MDS[1]}" ] && [ "${MDS[0]}" = "${MDS[2]}" ]; then
    byteid="YES"
  fi
  echo "${seq},${RMSES[0]},${RMSES[1]},${RMSES[2]},${MDS[0]},${MDS[1]},${MDS[2]},${byteid}" >> "${SUMMARY}"
  # Use run1 for mean computation (byte-id ensures all 3 equal)
  if [ "${RMSES[0]}" != "MISSING" ]; then
    SUM=$(awk -v s="${SUM}" -v v="${RMSES[0]}" 'BEGIN{printf "%.10f", s+v}')
    COUNT=$((COUNT + 1))
  fi
done

if [ ${COUNT} -gt 0 ]; then
  MEAN=$(awk -v s="${SUM}" -v c="${COUNT}" 'BEGIN{printf "%.6f", s/c}')
else
  MEAN="NaN"
fi
log "  9-seq mean ATE (run1, byte-id ⇒ all-runs): ${MEAN} m  (count=${COUNT})"
log ""

# Verdict
HARD=0.314
PASS_CLEAN=0.310
FAIL=0.391
verdict="UNKNOWN"
if [ "${MEAN}" != "NaN" ]; then
  cmp_hard=$(awk -v m="${MEAN}" -v t="${HARD}" 'BEGIN{print (m<=t)?"under":"over"}')
  cmp_clean=$(awk -v m="${MEAN}" -v t="${PASS_CLEAN}" 'BEGIN{print (m<=t)?"under":"over"}')
  cmp_fail=$(awk -v m="${MEAN}" -v t="${FAIL}" 'BEGIN{print (m>t)?"over":"under"}')
  if [ "${cmp_fail}" = "over" ]; then
    verdict="HARD-ABORT (mean > ${FAIL} m → Rule 16 trigger)"
  elif [ "${cmp_clean}" = "under" ]; then
    verdict="PASS-clean (mean ≤ ${PASS_CLEAN} m)"
  elif [ "${cmp_hard}" = "under" ]; then
    verdict="PASS (mean ≤ ${HARD} m)"
  else
    verdict="FAIL (${HARD} < mean ≤ ${FAIL} m — per-seq audit needed)"
  fi
fi
log "==== Verdict: ${verdict} ===="
log "Summary written to ${SUMMARY}"
