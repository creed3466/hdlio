#!/bin/bash
# run_S13_R0_11_3_smoke_A.sh — Sprint 13 R0.11.3.1 Smoke A (VI03 H1' verification)
#
# Architect contract §7.1 Smoke A pass criteria:
#   PASS-clean: VI03 ATE ≤ 0.624 m on all 3 runs AND md5 byte-identical
#   PASS-hard : VI03 ATE ≤ 0.650 m on all 3 runs AND md5 byte-identical
#   FAIL → Rule 16: VI03 ATE > 0.650 m on ANY run
#
# Wallclock: ~6-8 min (containers rebuild + 3 parallel VI03 runs).
#
# Usage: bash docker/run_S13_R0_11_3_smoke_A.sh [LABEL]

set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_R0_11_3_smoke_A_$(date +%Y%m%d_%H%M)}"
OUT_BASE="dump/${LABEL}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
CONFIG="avia_outdoor.yaml"
RATE="1.0"
SEQ="Varying-illu03"
N_RUNS=3

CONTAINER_NAMES=(tofslam_r0113_1 tofslam_r0113_2 tofslam_r0113_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

echo "================================================================"
echo "  Sprint 13 R0.11.3.1 — Smoke A (VI03 H1' guard extension)"
echo "  Label:   ${LABEL}"
echo "  Output:  ${OUT_BASE}/"
echo "  Config:  ${CONFIG}"
echo "  Seq:     ${SEQ} (rate=${RATE}, N_runs=${N_RUNS})"
echo "  Started: $(date)"
echo "================================================================"

mkdir -p "${OUT_BASE}/${SEQ}"

# Always respawn for fresh build of new source
for name in tofslam_r011_vi03_1 tofslam_r011_vi03_2 tofslam_r011_vi03_3 \
            tofslam_r0113_1 tofslam_r0113_2 tofslam_r0113_3; do
  docker rm -f "$name" 2>/dev/null || true
done

for i in 0 1 2; do
  name="${CONTAINER_NAMES[$i]}"
  docker run -d --rm --init --name "$name" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_BASE}:/root/catkin_ws/dump:rw" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    tofslam:ros1 bash -lc "sleep infinity" > /dev/null
done
sleep 3

echo "[build] parallel catkin_make Release with H1' source..."
for c in "${CONTAINER_NAMES[@]}"; do
  docker exec "$c" bash -c \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
     rm -rf build devel && \
     catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3" &
done
wait
echo "[build] done."

# Run VI03 × 3 parallel
echo "[run] launching ${N_RUNS} parallel ${SEQ} runs..."
run_one() {
  local slot=$1
  local run_id=$2
  local name="${CONTAINER_NAMES[$slot]}"
  local port="${PORTS[$slot]}"
  local out_in="dump/${SEQ}/run${run_id}"
  local log_host="${OUT_BASE}/${SEQ}/run${run_id}.log"
  docker exec "$name" bash /root/catkin_ws/docker/run_avia_exp.sh \
    "${CONFIG}" "${SEQ}" "${out_in}" "${port}" "${RATE}" \
    > "${log_host}" 2>&1
}
for slot in 0 1 2; do
  run_one $slot $((slot + 1)) &
done
wait
echo "[run] ${SEQ} batch complete."

for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done

# Verdict
echo ""
echo "================================================================"
echo "  R0.11.3.1 Smoke A verdict — $(date)"
echo "================================================================"

PASS_CLEAN=0.624
PASS_HARD=0.650

ATES=()
MD5S=()
for i in 1 2 3; do
  TRAJ="${OUT_BASE}/${SEQ}/run${i}/traj.csv"
  ATE_FILE="${OUT_BASE}/${SEQ}/run${i}/ate_result.txt"
  md5="MISSING"; rmse="MISSING"
  [ -f "${TRAJ}" ] && md5=$(md5sum "${TRAJ}" | awk '{print $1}')
  [ -f "${ATE_FILE}" ] && rmse=$(grep "^rmse:" "${ATE_FILE}" | awk '{print $2}')
  ATES+=("${rmse}")
  MD5S+=("${md5}")
  printf "  run%d  ATE = %-22s  md5 = %s\n" "$i" "${rmse}" "${md5}"
done

if [ "${MD5S[0]}" != "MISSING" ] && \
   [ "${MD5S[0]}" = "${MD5S[1]}" ] && \
   [ "${MD5S[0]}" = "${MD5S[2]}" ]; then
  echo "  CV=0% byte-id .................. PASS"
  byteid=1
else
  echo "  CV=0% byte-id .................. FAIL"
  byteid=0
fi

verdict="UNKNOWN"
v="${ATES[0]}"
if [ "${v}" = "MISSING" ]; then
  verdict="FAIL (MISSING ATE)"
elif [ ${byteid} -eq 0 ]; then
  verdict="FAIL (non-determinism)"
else
  passclean=$(awk -v v="${v}" -v t="${PASS_CLEAN}" 'BEGIN{print (v<=t)?"yes":"no"}')
  passhard=$(awk -v v="${v}" -v t="${PASS_HARD}" 'BEGIN{print (v<=t)?"yes":"no"}')
  if [ "${passclean}" = "yes" ]; then
    verdict="PASS-clean — VI03=${v} ≤ ${PASS_CLEAN} m (architect prediction confirmed tightly)"
  elif [ "${passhard}" = "yes" ]; then
    verdict="PASS-hard — VI03=${v} ∈ (${PASS_CLEAN}, ${PASS_HARD}] m (Branch-Q1-B fallback candidate)"
  else
    verdict="FAIL → Rule 16 — VI03=${v} > ${PASS_HARD} m"
  fi
fi

echo ""
echo "  [H4] firings on VI03 (any line with class=CLASS_D):"
grep "\[H4\] pre-LOCK rebuild fired class=CLASS_D" "${OUT_BASE}/${SEQ}/run1.log" 2>/dev/null | head -2 | sed 's/^/    /'
echo ""
echo "  Stage A class (first run):"
grep -E "STAGE_A (LOCK|DEFER)" "${OUT_BASE}/${SEQ}/run1.log" 2>/dev/null | head -1 | sed 's/^/    /'
echo ""
echo "  P1-router log (CLASS_D dispatch):"
grep "P1-router.*class=6" "${OUT_BASE}/${SEQ}/run1.log" 2>/dev/null | head -1 | sed 's/^/    /'

echo ""
echo "================================================================"
echo "  Verdict: ${verdict}"
echo "================================================================"

if echo "${verdict}" | grep -qE "^PASS-(clean|hard)"; then
  exit 0
else
  exit 1
fi
