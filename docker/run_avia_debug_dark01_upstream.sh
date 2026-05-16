#!/bin/bash
# run_avia_debug_dark01_upstream.sh — Task #70 U1 upstream bisection.
#
# 10× Dark01 @ -r 1.0 with:
#   - config/avia_v6_seq/dark01.yaml  (ring_N = 0 — production)
#   - TOFSLAM_DEBUG_UPSTREAM=1  (per-point bp/wp/state CSV)
# Each run writes dump/<LABEL>/Dark01/run<N>/upstream_trace_run<N>.csv
# plus the normal debug_imu.csv/debug_state.csv/ate_result.txt.
#
# After the runs:
#   1. Classify each run as Class A or Class B by ATE RMSE.
#   2. Assert both classes present.
#   3. Hand off to docker/analyze_upstream_trace.py for lockstep bisection
#      into U1a (bp differs), U1b (bp identical / state differs), U1c
#      (bp+state identical / wp differs → IEEE-754 impossible).
#
# Distribution: 3-run gate — p1=1, p2=2, p3=3 (single run per container)
#
# Usage: bash docker/run_avia_debug_dark01_upstream.sh [LABEL]

set -e
cd "$(dirname "$0")/.."

LABEL="${1:-task70_u1_upstream_$(date +%Y%m%d_%H%M)}"
OUT_BASE="dump/${LABEL}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"

SEQ="Dark01"
CFG="avia_v6_seq/dark01.yaml"
RATE="1.0"
N_RUNS=3  # 3-run CV gate per docs/requirements.md §1-1c (was 10; reduced 2026-04-16)

# Class centroids from full_eval_20260414 Dark01 (ring_N=0) and Task #70
# Phase 2 ringbuf experiment. Phase 3 bisection uses these as anchors.
CLASS_A_REF="0.14429536234731874"
CLASS_B_REF="0.14629949601951586"

echo "======== Task #70 U1 Upstream Bisection ========"
echo "Label:   ${LABEL}"
echo "Runs:    ${N_RUNS}× ${SEQ} @ -r ${RATE} (ring_N=0, trace ON)"
echo "Started: $(date)"
echo ""

# Disk preflight — upstream trace is ~O(frames × points × 16 cols × 20 B) ≈
# 1500 × 10000 × 320 B ≈ 5 GB/run → 50 GB total for 10 runs. Require ≥60.
AVAIL_GB=$(df -BG --output=avail . | tail -1 | tr -d 'G ')
echo "[Preflight] Free disk: ${AVAIL_GB} GB (need ≥60)"
if [ "${AVAIL_GB}" -lt 60 ]; then
  echo "[ABORT] Insufficient disk for upstream traces."
  exit 1
fi

mkdir -p "${OUT_BASE}"

for c in tofslam_db_1 tofslam_db_2 tofslam_db_3; do
  docker rm -f "$c" 2>/dev/null || true
done

for i in 1 2 3; do
  cpus="$(( (i-1)*4 ))-$(( i*4-1 ))"
  docker run -d --rm --name "tofslam_db_$i" \
    --network host --cpuset-cpus "$cpus" --memory 3g --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_BASE}:/root/catkin_ws/dump" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    tofslam:ros1 bash -lc "sleep infinity" > /dev/null
done
sleep 3

echo "[Build] Parallel..."
build_one() {
  local c=$1
  docker exec "$c" bash -c \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
     rm -rf build devel && \
     catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3"
  docker exec "$c" pip3 install scipy numpy -q 2>/dev/null || true
}
build_one tofslam_db_1 &
BP1=$!
build_one tofslam_db_2 &
BP2=$!
build_one tofslam_db_3 &
BP3=$!
wait $BP1 $BP2 $BP3
echo "[Build] Done."

RUNS_P1=(1)
RUNS_P2=(2)
RUNS_P3=(3)

run_list() {
  local C=$1 PORT=$2
  shift 2
  for RUN in "$@"; do
    local OUT_DIR="/root/catkin_ws/dump/${SEQ}/run${RUN}"
    echo "[${C}] ${SEQ} run${RUN} @ -r ${RATE} (upstream trace ON) ..."
    docker exec \
      -e TOFSLAM_DEBUG_UPSTREAM=1 \
      -e TOFSLAM_DEBUG_DIR="${OUT_DIR}" \
      -e TOFSLAM_UPSTREAM_RUN_ID="run${RUN}" \
      "$C" \
      bash /root/catkin_ws/docker/run_avia_exp.sh \
      "${CFG}" "${SEQ}" "${OUT_DIR}" "$PORT" "${RATE}"
  done
}

run_list tofslam_db_1 11311 "${RUNS_P1[@]}" &
PID1=$!
run_list tofslam_db_2 11312 "${RUNS_P2[@]}" &
PID2=$!
run_list tofslam_db_3 11313 "${RUNS_P3[@]}" &
PID3=$!
wait $PID1 $PID2 $PID3
echo "[Multi-Run] Done."

for c in tofslam_db_1 tofslam_db_2 tofslam_db_3; do
  docker rm -f "$c" 2>/dev/null || true
done

echo ""
echo "======== RESULTS: ${SEQ} @ -r ${RATE} (ring_N=0 + upstream trace) ========"
CLASS_A_RUNS=()
CLASS_B_RUNS=()
UNCLASSIFIED=()

for RUN in $(seq 1 ${N_RUNS}); do
  F="${OUT_BASE}/${SEQ}/run${RUN}/ate_result.txt"
  if [ -f "$F" ]; then
    RMSE=$(grep "^rmse:" "$F" | awk '{print $2}')
    DA=$(python3 -c "print(abs(${RMSE}-${CLASS_A_REF}))")
    DB=$(python3 -c "print(abs(${RMSE}-${CLASS_B_REF}))")
    CLASS="?"
    if python3 -c "import sys; sys.exit(0 if ${DA}<1e-4 else 1)" 2>/dev/null; then
      CLASS="A"; CLASS_A_RUNS+=("${RUN}")
    elif python3 -c "import sys; sys.exit(0 if ${DB}<1e-4 else 1)" 2>/dev/null; then
      CLASS="B"; CLASS_B_RUNS+=("${RUN}")
    else
      UNCLASSIFIED+=("${RUN}:${RMSE}")
    fi
    printf "  run%02d: %s  [%s]\n" "$RUN" "$RMSE" "$CLASS"
  else
    printf "  run%02d: <missing ate_result.txt>\n" "$RUN"
    UNCLASSIFIED+=("${RUN}:MISSING")
  fi
done

echo ""
echo "----- Partition -----"
echo "Class A (ref ${CLASS_A_REF}): ${#CLASS_A_RUNS[@]} runs: ${CLASS_A_RUNS[*]:-none}"
echo "Class B (ref ${CLASS_B_REF}): ${#CLASS_B_RUNS[@]} runs: ${CLASS_B_RUNS[*]:-none}"
if [ "${#UNCLASSIFIED[@]}" -gt 0 ]; then
  echo "Unclassified: ${#UNCLASSIFIED[@]}: ${UNCLASSIFIED[*]}"
fi

if [ "${#CLASS_A_RUNS[@]}" -eq 0 ] || [ "${#CLASS_B_RUNS[@]}" -eq 0 ]; then
  echo ""
  echo "[ABORT] One class empty. Bisection requires both. Re-run with more N_RUNS."
  exit 2
fi

echo ""
echo "[PASS] Both classes populated. Running analyzer..."
python3 docker/analyze_upstream_trace.py "${OUT_BASE}/${SEQ}" \
  --class-a "${CLASS_A_RUNS[*]}" \
  --class-b "${CLASS_B_RUNS[*]}" | tee "${OUT_BASE}/bisection_report.txt"

echo ""
echo "======== Done: ${OUT_BASE}/ @ $(date) ========"
