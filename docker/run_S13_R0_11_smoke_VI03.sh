#!/bin/bash
# run_S13_R0_11_smoke_VI03.sh — Sprint 13 R0.11.1 VI03 H1-revert verification
#
# Single-seq VI03 × 3 rate=1.0 to confirm H1 revert restores V3 Path B baseline.
#
# Pass criteria:
#   - VI03 ATE ≈ 1.230 m (V3 Path B P1-OFF measurement), tolerance ± 0.20 m
#       → range [1.030, 1.430] m considered PASS
#       → > 1.5 m → still pathological, escalate to user
#       → < 1.0 m → unexpected improvement, investigate
#   - byte-identical × 3 (CV=0%)
#   - Stage A LOCK class=CLASS_D rho_1=0.6293 (unchanged routing)
#   - [P1-router] log shows enable=false range_inv=OFF on CLASS_D dispatch
#   - 0 [H4] SPDLOG lines (still gated on OUTDOOR_DRIFT)
#
# Wallclock: ~6 min (containers rebuild + 3 parallel VI03 runs).
#
# Usage: bash docker/run_S13_R0_11_smoke_VI03.sh [LABEL]

set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_R0_11_smoke_VI03_$(date +%Y%m%d_%H%M)}"
OUT_BASE="dump/${LABEL}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
CONFIG="avia_outdoor.yaml"
RATE="1.0"
SEQ="Varying-illu03"
N_RUNS=3

CONTAINER_NAMES=(tofslam_r011_vi03_1 tofslam_r011_vi03_2 tofslam_r011_vi03_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

echo "================================================================"
echo "  Sprint 13 R0.11.1 — VI03 H1-revert smoke (kT_CLASS_D.p1 → {})"
echo "  Label:   ${LABEL}"
echo "  Output:  ${OUT_BASE}/"
echo "  Config:  ${CONFIG}"
echo "  Seq:     ${SEQ} (rate=${RATE}, N_runs=${N_RUNS})"
echo "  Started: $(date)"
echo "================================================================"

mkdir -p "${OUT_BASE}/${SEQ}"

# Always respawn for clean read of new source
for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done
# Also kill leftover Smoke A2/B containers if any
docker rm -f tofslam_r09_a_1 tofslam_r09_a_2 tofslam_r09_a_3 2>/dev/null || true

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

echo "[build] parallel catkin_make Release with H1-revert source..."
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
echo "  R0.11.1 VI03 H1-revert verdict — $(date)"
echo "================================================================"

LO=1.030
HI=1.430
PATHOLOGICAL=1.5

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

verdict="UNKNOWN"
if [ "${MD5S[0]}" != "MISSING" ] && \
   [ "${MD5S[0]}" = "${MD5S[1]}" ] && \
   [ "${MD5S[0]}" = "${MD5S[2]}" ]; then
  echo "  CV=0% byte-id .................. PASS"
  byteid=1
else
  echo "  CV=0% byte-id .................. FAIL"
  byteid=0
fi

v="${ATES[0]}"
if [ "${v}" = "MISSING" ]; then
  verdict="FAIL (MISSING ATE)"
elif [ ${byteid} -eq 0 ]; then
  verdict="FAIL (non-determinism)"
else
  pathological=$(awk -v v="${v}" -v t="${PATHOLOGICAL}" 'BEGIN{print (v>t)?"yes":"no"}')
  if [ "${pathological}" = "yes" ]; then
    verdict="FAIL — VI03=${v} > ${PATHOLOGICAL} m still pathological (Option B insufficient)"
  else
    in_baseline=$(awk -v v="${v}" -v lo="${LO}" -v hi="${HI}" \
      'BEGIN{print (v>=lo && v<=hi)?"yes":"no"}')
    if [ "${in_baseline}" = "yes" ]; then
      verdict="PASS — VI03=${v} ∈ [${LO}, ${HI}] m matches V3 Path B P1-OFF baseline"
    else
      verdict="PARTIAL — VI03=${v} outside [${LO}, ${HI}] m baseline; investigate"
    fi
  fi
fi
echo ""
echo "  P1-router log proof (first run):"
grep "P1-router" "${OUT_BASE}/${SEQ}/run1.log" 2>/dev/null | head -2 | sed 's/^/    /'
echo ""
echo "  Stage A class (first run):"
grep -E "STAGE_A (LOCK|DEFER)" "${OUT_BASE}/${SEQ}/run1.log" 2>/dev/null | head -1 | sed 's/^/    /'
echo ""
echo "  [H4] firings (any run):"
grep -c "\[H4\]" "${OUT_BASE}/${SEQ}/run1.log" 2>/dev/null | awk '{print "    run1: "$1" lines"}'
echo ""
echo "================================================================"
echo "  Verdict: ${verdict}"
echo "================================================================"

if [ "${verdict}" != "${verdict#PASS}" ]; then
  exit 0
else
  exit 1
fi
