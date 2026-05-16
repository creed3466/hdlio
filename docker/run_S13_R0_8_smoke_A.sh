#!/bin/bash
# run_S13_R0_8_smoke_A.sh — Sprint 13 R0.8 Smoke A
#
# Architect contract §7 Smoke A:
#   Dark01 single-seq, 2 runs rate=1.0 with avia_outdoor.yaml (unified config,
#   H1+H3a+master gate ON, H2 applied).
#   PASS: ATE ∈ [0.097, 0.117] m on both runs AND md5(traj.csv) byte-identical.
#
# Output: dump/<LABEL>/run{1,2}/{traj.csv, ate_result.txt}
#
# Wallclock: ~10 min build + ~3 min runs ≈ 13 min total.
#
# Usage:
#   bash docker/run_S13_R0_8_smoke_A.sh [LABEL]

set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_R0_8_smoke_A_$(date +%Y%m%d_%H%M)}"
OUT_BASE="dump/${LABEL}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
CONFIG="avia_outdoor.yaml"
SEQ="Dark01"
RATE="1.0"
N_RUNS=2

echo "================================================================"
echo "  Sprint 13 R0.8 — Smoke A (Dark01 baseline preservation)"
echo "  Label:   ${LABEL}"
echo "  Output:  ${OUT_BASE}/"
echo "  Config:  ${CONFIG}"
echo "  Seq:     ${SEQ} (rate=${RATE})"
echo "  N_runs:  ${N_RUNS}"
echo "  Started: $(date)"
echo "================================================================"

mkdir -p "${OUT_BASE}"

CONTAINER_NAMES=(tofslam_smoke_a_1 tofslam_smoke_a_2)
CPUSETS=("0-3" "4-7")
PORTS=(11311 11312)

# Cleanup
for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done

# Spawn
for i in 0 1; do
  name="${CONTAINER_NAMES[$i]}"
  docker run -d --rm --name "$name" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_BASE}:/root/catkin_ws/dump" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    tofslam:ros1 bash -lc "sleep infinity" > /dev/null
done
sleep 3

# Parallel build
echo "[build] parallel catkin_make Release..."
build_one() {
  local c=$1
  docker exec "$c" bash -c \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
     rm -rf build devel && \
     catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3"
  docker exec "$c" pip3 install scipy numpy -q 2>/dev/null || true
}
for c in "${CONTAINER_NAMES[@]}"; do
  build_one "$c" &
done
wait
echo "[build] done."

# Parallel runs: one Dark01 run per container
echo "[run] launching 2 parallel Dark01 runs rate=${RATE}..."
run_one() {
  local slot=$1
  local name="${CONTAINER_NAMES[$slot]}"
  local port="${PORTS[$slot]}"
  local run_id=$((slot + 1))
  local out_in="/root/catkin_ws/dump/run${run_id}"
  local log_host="${OUT_BASE}/run${run_id}.log"
  docker exec "$name" bash /root/catkin_ws/docker/run_avia_exp.sh \
    "${CONFIG}" "${SEQ}" "${out_in}" "${port}" "${RATE}" \
    > "${log_host}" 2>&1
}
for i in 0 1; do
  run_one $i &
done
wait
echo "[run] complete."

# Cleanup containers
for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done

# ----------------------------------------------------------------------
# Verdict
# ----------------------------------------------------------------------
echo ""
echo "================================================================"
echo "  Smoke A verdict — $(date)"
echo "================================================================"

LO=0.097
HI=0.117
ATE_1=""
ATE_2=""
MD5_1=""
MD5_2=""

for i in 1 2; do
  TRAJ="${OUT_BASE}/run${i}/traj.csv"
  ATE_FILE="${OUT_BASE}/run${i}/ate_result.txt"
  if [ -f "${TRAJ}" ]; then
    md5=$(md5sum "${TRAJ}" | awk '{print $1}')
    eval "MD5_${i}=\"${md5}\""
    if [ -f "${ATE_FILE}" ]; then
      rmse=$(grep "^rmse:" "${ATE_FILE}" | awk '{print $2}')
      eval "ATE_${i}=\"${rmse}\""
    fi
  fi
done

# Report
printf "  run1  ATE = %s   md5 = %s\n" "${ATE_1:-MISSING}" "${MD5_1:-MISSING}"
printf "  run2  ATE = %s   md5 = %s\n" "${ATE_2:-MISSING}" "${MD5_2:-MISSING}"

verdict_byte="FAIL"
verdict_ate1="FAIL"
verdict_ate2="FAIL"
if [ -n "${MD5_1}" ] && [ "${MD5_1}" = "${MD5_2}" ]; then
  verdict_byte="PASS"
fi
if [ -n "${ATE_1}" ]; then
  in_range=$(awk -v v="${ATE_1}" -v lo="${LO}" -v hi="${HI}" \
    'BEGIN{print (v>=lo && v<=hi) ? "PASS":"FAIL"}')
  verdict_ate1="${in_range}"
fi
if [ -n "${ATE_2}" ]; then
  in_range=$(awk -v v="${ATE_2}" -v lo="${LO}" -v hi="${HI}" \
    'BEGIN{print (v>=lo && v<=hi) ? "PASS":"FAIL"}')
  verdict_ate2="${in_range}"
fi

echo ""
echo "  byte-identical md5 .................. ${verdict_byte}"
echo "  ATE run1 ∈ [${LO}, ${HI}] m ......... ${verdict_ate1}"
echo "  ATE run2 ∈ [${LO}, ${HI}] m ......... ${verdict_ate2}"
echo ""

if [ "${verdict_byte}" = "PASS" ] && \
   [ "${verdict_ate1}" = "PASS" ] && \
   [ "${verdict_ate2}" = "PASS" ]; then
  echo "  ===== SMOKE A: PASS ====="
  exit 0
else
  echo "  ===== SMOKE A: FAIL ====="
  exit 1
fi
