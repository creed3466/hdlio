#!/bin/bash
# run_S13_R0_9_smoke_A.sh — Sprint 13 R0.9 Smoke A (H3b OUTDOOR_DRIFT)
#
# Architect contract §6 + §7 Smoke A pass criteria:
#   DK01 (H3b must NOT regress; H3b predicate must NOT fire on DK01):
#     - ATE ∈ [0.097, 0.117] m on all 3 runs
#     - md5(traj.csv) byte-identical across 3 runs (CV=0%)
#     - STAGE_B_LOCK class=CLEAN_DENSE (NOT STAGE_A OUTDOOR_DRIFT)
#   DK02 (H3b mechanism falsifiability):
#     - ATE ≤ 0.662 m on all 3 runs (≥60mm closure of 121mm gap vs V3 Path B 0.722m)
#     - md5(traj.csv) byte-identical across 3 runs (CV=0%)
#     - STAGE_A_LOCK class=OUTDOOR_DRIFT at frame=2
#
# Output: dump/<LABEL>/{Dark01,Dark02}/run{1,2,3}/{traj.csv,ate_result.txt,run.log}
#
# Wallclock: ~10 min build (3 parallel) + 2 batches × ~3 min runs ≈ 16 min total.
#
# Usage: bash docker/run_S13_R0_9_smoke_A.sh [LABEL]

set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_R0_9_smoke_A_$(date +%Y%m%d_%H%M)}"
OUT_BASE="dump/${LABEL}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
CONFIG="avia_outdoor.yaml"
RATE="1.0"
SEQS=(Dark01 Dark02)
N_RUNS=3

echo "================================================================"
echo "  Sprint 13 R0.9 — Smoke A (H3b OUTDOOR_DRIFT)"
echo "  Label:   ${LABEL}"
echo "  Output:  ${OUT_BASE}/"
echo "  Config:  ${CONFIG}"
echo "  Seqs:    ${SEQS[*]} (rate=${RATE})"
echo "  N_runs:  ${N_RUNS}"
echo "  Started: $(date)"
echo "================================================================"

mkdir -p "${OUT_BASE}"

CONTAINER_NAMES=(tofslam_r09_a_1 tofslam_r09_a_2 tofslam_r09_a_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

# Cleanup
for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done
docker rm -f tofslam_r09_build 2>/dev/null || true

# Spawn 3 containers
for i in 0 1 2; do
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

# Parallel build (all 3 in parallel)
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

# Batched runs: SEQ × N_RUNS = 6 total, 3-parallel.
# Batch 1: DK01 run1, DK01 run2, DK01 run3
# Batch 2: DK02 run1, DK02 run2, DK02 run3
run_one_in_slot() {
  local slot=$1
  local seq=$2
  local run_id=$3
  local name="${CONTAINER_NAMES[$slot]}"
  local port="${PORTS[$slot]}"
  local out_in="/root/catkin_ws/dump/${seq}/run${run_id}"
  local log_host="${OUT_BASE}/${seq}/run${run_id}.log"
  mkdir -p "${OUT_BASE}/${seq}"
  docker exec "$name" bash /root/catkin_ws/docker/run_avia_exp.sh \
    "${CONFIG}" "${seq}" "${out_in}" "${port}" "${RATE}" \
    > "${log_host}" 2>&1
}

for seq in "${SEQS[@]}"; do
  echo "[run] launching 3 parallel ${seq} runs rate=${RATE}..."
  for slot in 0 1 2; do
    run_one_in_slot $slot "${seq}" $((slot + 1)) &
  done
  wait
  echo "[run] ${seq} batch complete."
done

# Cleanup containers
for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done

# ----------------------------------------------------------------------
# Verdict
# ----------------------------------------------------------------------
echo ""
echo "================================================================"
echo "  Smoke A R0.9 verdict — $(date)"
echo "================================================================"

DK01_LO=0.097
DK01_HI=0.117
DK02_HI=0.662

verdict_overall="PASS"

for seq in "${SEQS[@]}"; do
  echo ""
  echo "  --- ${seq} ---"
  ATES=()
  MD5S=()
  for i in 1 2 3; do
    TRAJ="${OUT_BASE}/${seq}/run${i}/traj.csv"
    ATE_FILE="${OUT_BASE}/${seq}/run${i}/ate_result.txt"
    md5="MISSING"; rmse="MISSING"
    if [ -f "${TRAJ}" ]; then
      md5=$(md5sum "${TRAJ}" | awk '{print $1}')
    fi
    if [ -f "${ATE_FILE}" ]; then
      rmse=$(grep "^rmse:" "${ATE_FILE}" | awk '{print $2}')
    fi
    ATES+=("${rmse}")
    MD5S+=("${md5}")
    printf "  run%d  ATE = %-10s   md5 = %s\n" "$i" "${rmse}" "${md5}"
  done

  # Byte-identity check (CV=0%)
  if [ "${MD5S[0]}" != "MISSING" ] && \
     [ "${MD5S[0]}" = "${MD5S[1]}" ] && \
     [ "${MD5S[0]}" = "${MD5S[2]}" ]; then
    echo "  CV=0% byte-id .................. PASS"
  else
    echo "  CV=0% byte-id .................. FAIL"
    verdict_overall="FAIL"
  fi

  # ATE gate per seq
  if [ "${seq}" = "Dark01" ]; then
    for i in 0 1 2; do
      v="${ATES[$i]}"
      if [ "${v}" = "MISSING" ]; then
        echo "  ATE run$((i+1)) MISSING ........... FAIL"
        verdict_overall="FAIL"
      else
        in_range=$(awk -v v="${v}" -v lo="${DK01_LO}" -v hi="${DK01_HI}" \
          'BEGIN{print (v>=lo && v<=hi) ? "PASS":"FAIL"}')
        echo "  ATE run$((i+1)) ∈ [${DK01_LO}, ${DK01_HI}] ... ${in_range}"
        [ "${in_range}" = "FAIL" ] && verdict_overall="FAIL"
      fi
    done
  else  # Dark02
    for i in 0 1 2; do
      v="${ATES[$i]}"
      if [ "${v}" = "MISSING" ]; then
        echo "  ATE run$((i+1)) MISSING ........... FAIL"
        verdict_overall="FAIL"
      else
        below=$(awk -v v="${v}" -v hi="${DK02_HI}" \
          'BEGIN{print (v<=hi) ? "PASS":"FAIL"}')
        echo "  ATE run$((i+1)) ≤ ${DK02_HI} ........ ${below}"
        [ "${below}" = "FAIL" ] && verdict_overall="FAIL"
      fi
    done
  fi

  # STAGE_A/B class check
  for i in 1 2 3; do
    LOG="${OUT_BASE}/${seq}/run${i}.log"
    if [ -f "${LOG}" ]; then
      stage_a_line=$(grep -E "STAGE_A (LOCK|DEFER)" "${LOG}" | head -1)
      stage_b_line=$(grep -E "STAGE_B LOCK" "${LOG}" | head -1)
      if [ "${seq}" = "Dark01" ]; then
        # Expect STAGE_A DEFER + STAGE_B LOCK class=CLEAN_DENSE
        if echo "${stage_a_line}" | grep -q "STAGE_A DEFER"; then
          if echo "${stage_b_line}" | grep -q "class=CLEAN_DENSE"; then
            echo "  run$i Stage A=DEFER, Stage B=CLEAN_DENSE ... PASS"
          else
            echo "  run$i Stage B not CLEAN_DENSE: ${stage_b_line} ... FAIL"
            verdict_overall="FAIL"
          fi
        else
          echo "  run$i Stage A NOT DEFER: ${stage_a_line} ... FAIL"
          verdict_overall="FAIL"
        fi
      else  # Dark02
        # Expect STAGE_A LOCK class=OUTDOOR_DRIFT at frame=2
        if echo "${stage_a_line}" | grep -q "STAGE_A LOCK"; then
          if echo "${stage_a_line}" | grep -q "class=OUTDOOR_DRIFT"; then
            if echo "${stage_a_line}" | grep -q "frame=2 "; then
              echo "  run$i Stage A=OUTDOOR_DRIFT @ frame=2 ... PASS"
            else
              echo "  run$i wrong frame: ${stage_a_line} ... FAIL"
              verdict_overall="FAIL"
            fi
          else
            echo "  run$i Stage A class wrong: ${stage_a_line} ... FAIL"
            verdict_overall="FAIL"
          fi
        else
          echo "  run$i Stage A NOT LOCK: ${stage_a_line} ... FAIL"
          verdict_overall="FAIL"
        fi
      fi
    fi
  done
done

echo ""
echo "================================================================"
if [ "${verdict_overall}" = "PASS" ]; then
  echo "  ===== SMOKE A R0.9: PASS ====="
  exit 0
else
  echo "  ===== SMOKE A R0.9: FAIL ====="
  exit 1
fi
