#!/bin/bash
# run_avia_debug_dark01.sh — Dark01 determinism bisection experiment
#
# Runs Dark01 at -r 1.0 with TOFSLAM_DEBUG_DETERMINISM=1, 3 runs across
# 3 containers in parallel. Collects per-run debug_imu.csv + debug_state.csv
# to bisect H1 (startup race on gravity-init IMU window) vs H2 (runtime FP
# non-determinism amplifying identical inputs).
#
# Distribution: 3-run gate — p1=1, p2=2, p3=3 (single run per container)
#
# Usage: bash docker/run_avia_debug_dark01.sh [LABEL]

set -e
cd "$(dirname "$0")/.."

LABEL="${1:-avia_debug_dark01_$(date +%Y%m%d_%H%M)}"
OUT_BASE="dump/${LABEL}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"

SEQ="Dark01"
CFG="avia_v6_seq/dark01.yaml"
RATE="1.0"
N_RUNS=3  # 3-run CV gate per docs/requirements.md §1-1c (was 10; reduced 2026-04-16)

echo "======== Debug Multi-Run: ${N_RUNS}× ${SEQ} @ -r ${RATE}: $(date) ========"
echo "Output: ${OUT_BASE}/"
mkdir -p "${OUT_BASE}"

# Cleanup
for c in tofslam_db_1 tofslam_db_2 tofslam_db_3; do
  docker rm -f "$c" 2>/dev/null || true
done

# Launch 3 containers
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

# Build (parallel across containers)
echo "[Build] Starting..."
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

# Run-list per container (10 runs distributed across 3 containers)
RUNS_P1=(1)
RUNS_P2=(2)
RUNS_P3=(3)

run_list() {
  local C=$1 PORT=$2
  shift 2
  for RUN in "$@"; do
    local OUT_DIR="/root/catkin_ws/dump/${SEQ}/run${RUN}"
    echo "[${C}] ${SEQ} run${RUN} @ -r ${RATE} ..."
    # Task #36: propagate boundary-hash instrumentation env vars when set
    # on the host shell, so B6-B10 CSVs land in OUT_DIR automatically.
    local BOUNDARY_EXEC_ENV=()
    if [ -n "${TOFSLAM_DEBUG_BOUNDARY_HASH:-}" ]; then
      BOUNDARY_EXEC_ENV+=(
        -e "TOFSLAM_DEBUG_BOUNDARY_HASH=${TOFSLAM_DEBUG_BOUNDARY_HASH}"
        -e "TOFSLAM_BOUNDARY_RUN_ID=run${RUN}"
        -e "DUMP_DIR=${OUT_DIR}"
      )
    fi
    # Task #70 U1a Phase-1: propagate prescan_trace env vars when set on host.
    # Emits per-frame state_history_ trace CSV to OUT_DIR for divergence diff.
    local PRESCAN_EXEC_ENV=()
    if [ -n "${TOFSLAM_DEBUG_PRESCAN:-}" ]; then
      PRESCAN_EXEC_ENV+=(
        -e "TOFSLAM_DEBUG_PRESCAN=${TOFSLAM_DEBUG_PRESCAN}"
        -e "TOFSLAM_PRESCAN_RUN_ID=run${RUN}"
        -e "TOFSLAM_DEBUG_DIR=${OUT_DIR}"
      )
    fi
    docker exec -e TOFSLAM_DEBUG_DETERMINISM=1 \
      "${BOUNDARY_EXEC_ENV[@]}" "${PRESCAN_EXEC_ENV[@]}" "$C" \
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
echo "[Multi-Run] All runs complete."

# Cleanup
for c in tofslam_db_1 tofslam_db_2 tofslam_db_3; do
  docker rm -f "$c" 2>/dev/null || true
done

############################################
# RESULTS
############################################
echo ""
echo "======== RESULTS: ${SEQ} @ -r ${RATE} ========"
VALS=""
for RUN in $(seq 1 ${N_RUNS}); do
  F="${OUT_BASE}/${SEQ}/run${RUN}/ate_result.txt"
  if [ -f "$F" ]; then
    RMSE=$(grep "^rmse:" "$F" | awk '{print $2}')
    printf "  run%02d: %.6f\n" "$RUN" "$RMSE"
    VALS="${VALS} ${RMSE}"
  else
    printf "  run%02d: FAIL\n" "$RUN"
  fi
done
if [ -n "$VALS" ]; then
  echo "$VALS" | tr ' ' '\n' | grep -v '^$' | sort -n | awk '
  {a[NR]=$1; s+=$1; ss+=$1*$1}
  END {
    n=NR
    m=s/n
    sd = (n>1) ? sqrt((ss - n*m*m)/(n-1)) : 0
    cv = (m>0) ? sd/m*100 : 0
    printf "  ---- stats (n=%d) ----\n", n
    printf "  best:   %.6f\n", a[1]
    printf "  worst:  %.6f\n", a[n]
    printf "  mean:   %.6f\n", m
    printf "  stdev:  %.6f\n", sd
    printf "  CV:     %.2f%%\n", cv
    printf "  range:  %.6f\n", a[n]-a[1]
  }'
fi

echo ""
echo "======== Debug CSV diff (first few fields) ========"
echo "-- IMU prefix head (first 3 entries per run) --"
for RUN in $(seq 1 ${N_RUNS}); do
  F="${OUT_BASE}/${SEQ}/run${RUN}/debug_imu.csv"
  if [ -f "$F" ]; then
    printf "  run%02d: " "$RUN"
    head -2 "$F" | tail -1 | cut -d, -f1-3
  fi
done

echo ""
echo "-- Gravity-init state per run --"
for RUN in $(seq 1 ${N_RUNS}); do
  F="${OUT_BASE}/${SEQ}/run${RUN}/debug_state.csv"
  if [ -f "$F" ]; then
    printf "  run%02d: " "$RUN"
    awk -F, 'NR==2 {printf "t=%s pos=(%s,%s,%s) vel=(%s,%s,%s) grav=(%s,%s,%s)\n", $3,$4,$5,$6,$11,$12,$13,$14,$15,$16}' "$F"
  fi
done

echo ""
echo "======== Done: ${OUT_BASE}/ ========"
