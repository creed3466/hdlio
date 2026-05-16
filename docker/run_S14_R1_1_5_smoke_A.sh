#!/bin/bash
# run_S14_R1_1_5_smoke_A.sh — Sprint 14 R1.1.5 Smoke A (state replay HARD gate)
#
# Architect contract §7.1:
#   VI03 ATE ≤ 0.650m PASS-hard, ≤ 0.624m PASS-clean (predicted 0.608m)
#   DK01 byte-id md5 == 391b3ce1da44d0a566dd0424f6edbe28 (×3)
#   DK02 byte-id md5 == d6fe5dc0126e4197cf668e1d9fbd2a08 (×3)
#   All 3 seqs CV=0% byte-id
#
# Wallclock: ~12-15 min (containers + build + 3 seqs × 3 runs each).

set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S14_R1_1_5_smoke_A_$(date +%Y%m%d_%H%M)}"
OUT_BASE="dump/${LABEL}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
CONFIG="avia_outdoor.yaml"
RATE="1.0"
SEQS=(Varying-illu03 Dark01 Dark02)
N_RUNS=3

CONTAINER_NAMES=(tofslam_s14r115_1 tofslam_s14r115_2 tofslam_s14r115_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

echo "================================================================"
echo "  Sprint 14 R1.1.5 — Smoke A (state replay HARD gate)"
echo "  Label:   ${LABEL}"
echo "  HEAD:    $(git log --oneline -1)"
echo "  Seqs:    ${SEQS[*]} (rate=${RATE}, N_runs=${N_RUNS})"
echo "  Started: $(date)"
echo "================================================================"

mkdir -p "${OUT_BASE}"
for s in "${SEQS[@]}"; do mkdir -p "${OUT_BASE}/${s}"; done

# Clean prior containers
for n in tofslam_s14r115_1 tofslam_s14r115_2 tofslam_s14r115_3 \
         tofslam_r0113_1 tofslam_r0113_2 tofslam_r0113_3 \
         tofslam_r09_a_1 tofslam_r09_a_2 tofslam_r09_a_3 \
         tofslam_r011_vi03_1 tofslam_r011_vi03_2 tofslam_r011_vi03_3 \
         tofslam_preframe0 ; do
  docker rm -f "$n" 2>/dev/null || true
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

echo "[build] parallel catkin_make Release with R1.1 state replay source..."
build_one() {
  docker exec "$1" bash -c \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
     rm -rf build devel && \
     catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -5"
}
for c in "${CONTAINER_NAMES[@]}"; do build_one "$c" & done
wait
echo "[build] done."

# Verify build success across all 3
for c in "${CONTAINER_NAMES[@]}"; do
  if ! docker exec "$c" test -f /root/catkin_ws/devel/lib/tof_slam/tofslam_node; then
    echo "FATAL: build failed in ${c}"
    exit 2
  fi
done

# Run each seq × 3 in parallel (3 containers, 1 seq batch at a time)
run_one() {
  local slot=$1 seq=$2 run_id=$3
  local name="${CONTAINER_NAMES[$slot]}"
  local port="${PORTS[$slot]}"
  local out_in="dump/${seq}/run${run_id}"
  local log_host="${OUT_BASE}/${seq}/run${run_id}.log"
  docker exec "$name" bash /root/catkin_ws/docker/run_avia_exp.sh \
    "${CONFIG}" "${seq}" "${out_in}" "${port}" "${RATE}" \
    > "${log_host}" 2>&1
}

for seq in "${SEQS[@]}"; do
  echo "[run] ${seq}: 3 parallel runs..."
  for slot in 0 1 2; do
    run_one $slot "${seq}" $((slot + 1)) &
  done
  wait
done

for c in "${CONTAINER_NAMES[@]}"; do docker rm -f "$c" 2>/dev/null || true; done

echo ""
echo "================================================================"
echo "  Smoke A verdict — $(date)"
echo "================================================================"

DK01_HI=0.157
DK01_LO=0.057
DK01_MD5_EXPECTED=391b3ce1da44d0a566dd0424f6edbe28
DK02_HI=0.7185
DK02_LO=0.6185
DK02_MD5_EXPECTED=d6fe5dc0126e4197cf668e1d9fbd2a08
VI03_PASS_HARD=0.650
VI03_PASS_CLEAN=0.624

verdict_overall="PASS"

for seq in "${SEQS[@]}"; do
  echo ""
  echo "  --- ${seq} ---"
  ATES=(); MD5S=()
  for i in 1 2 3; do
    TRAJ="${OUT_BASE}/${seq}/run${i}/traj.csv"
    ATE_FILE="${OUT_BASE}/${seq}/run${i}/ate_result.txt"
    md5="MISSING"; rmse="MISSING"
    [ -f "${TRAJ}" ] && md5=$(md5sum "${TRAJ}" | awk '{print $1}')
    [ -f "${ATE_FILE}" ] && rmse=$(grep "^rmse:" "${ATE_FILE}" | awk '{print $2}')
    ATES+=("${rmse}"); MD5S+=("${md5}")
    printf "  run%d  ATE = %-22s  md5 = %s\n" "$i" "${rmse}" "${md5}"
  done
  # CV=0%
  if [ "${MD5S[0]}" != "MISSING" ] && \
     [ "${MD5S[0]}" = "${MD5S[1]}" ] && [ "${MD5S[0]}" = "${MD5S[2]}" ]; then
    echo "  CV=0% byte-id .................. PASS"
  else
    echo "  CV=0% byte-id .................. FAIL"
    verdict_overall="FAIL"
  fi
  # ATE
  v="${ATES[0]}"
  if [ "${v}" = "MISSING" ]; then
    verdict_overall="FAIL"
    continue
  fi
  if [ "${seq}" = "Varying-illu03" ]; then
    pc=$(awk -v v="${v}" -v t="${VI03_PASS_CLEAN}" 'BEGIN{print (v<=t)?"yes":"no"}')
    ph=$(awk -v v="${v}" -v t="${VI03_PASS_HARD}" 'BEGIN{print (v<=t)?"yes":"no"}')
    if [ "${pc}" = "yes" ]; then echo "  VI03 ATE ≤ ${VI03_PASS_CLEAN} ........... PASS-clean"
    elif [ "${ph}" = "yes" ]; then echo "  VI03 ATE ≤ ${VI03_PASS_HARD} ........... PASS-hard"
    else echo "  VI03 ATE > ${VI03_PASS_HARD} ............ FAIL (Rule 16)"; verdict_overall="FAIL"
    fi
  elif [ "${seq}" = "Dark01" ]; then
    in_band=$(awk -v v="${v}" -v lo="${DK01_LO}" -v hi="${DK01_HI}" 'BEGIN{print (v>=lo && v<=hi)?"yes":"no"}')
    if [ "${in_band}" = "yes" ]; then echo "  DK01 ATE ∈ [${DK01_LO}, ${DK01_HI}] ..... PASS"
    else echo "  DK01 ATE band FAIL: ${v} ......... FAIL"; verdict_overall="FAIL"; fi
    if [ "${MD5S[0]}" = "${DK01_MD5_EXPECTED}" ]; then echo "  DK01 md5 == expected ............. PASS (I-6 preserved)"
    else echo "  DK01 md5 mismatch: ${MD5S[0]} != ${DK01_MD5_EXPECTED} ... FAIL (I-6 broken)"; verdict_overall="FAIL"
    fi
  else # Dark02
    in_band=$(awk -v v="${v}" -v lo="${DK02_LO}" -v hi="${DK02_HI}" 'BEGIN{print (v>=lo && v<=hi)?"yes":"no"}')
    if [ "${in_band}" = "yes" ]; then echo "  DK02 ATE ∈ [${DK02_LO}, ${DK02_HI}] ..... PASS"
    else echo "  DK02 ATE band FAIL: ${v} ......... FAIL"; verdict_overall="FAIL"; fi
    if [ "${MD5S[0]}" = "${DK02_MD5_EXPECTED}" ]; then echo "  DK02 md5 == expected ............. PASS (H3b+H4 preserved)"
    else echo "  DK02 md5 mismatch ................ FAIL"; verdict_overall="FAIL"
    fi
  fi
done

echo ""
echo "  Replay log evidence (VI03 run1):"
grep -E "REPLAY fired|STAGE_A LOCK class=CLASS_D|\\[H4\\]" "${OUT_BASE}/Varying-illu03/run1.log" 2>/dev/null | head -3 | sed 's/^/    /'

echo ""
echo "================================================================"
echo "  Overall: ${verdict_overall}"
echo "================================================================"

if [ "${verdict_overall}" = "FAIL" ]; then exit 1; fi
exit 0
