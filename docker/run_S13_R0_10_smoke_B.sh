#!/bin/bash
# run_S13_R0_10_smoke_B.sh — Sprint 13 R0.10 Smoke B (VI03 cross-class regression)
#
# Purpose: verify H4 (OUTDOOR_DRIFT-gated) does NOT regress VI03 (CLASS_D).
# H1 kT_CLASS_D anisotropic-only IEKF unchanged; H4 epilogue gated by cls==OUTDOOR_DRIFT
# → no [H4] line expected; class lock = CLASS_D (Stage A).
#
# Pass criteria:
#   - ATE ≤ 0.65 m on all 3 runs (R0.10 architect §6 inherited Smoke B gate)
#   - md5(traj.csv) byte-identical across 3 runs (CV=0%)
#   - STAGE_A LOCK class=CLASS_D (NOT OUTDOOR_DRIFT)
#   - 0 [H4] SPDLOG lines (gating proof)
#
# Wallclock: ~6 min (containers + bin already built from Smoke A2; reuse).
#
# Usage: bash docker/run_S13_R0_10_smoke_B.sh [LABEL]

set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_R0_10_smoke_B_$(date +%Y%m%d_%H%M)}"
OUT_BASE="dump/${LABEL}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
CONFIG="avia_outdoor.yaml"
RATE="1.0"
SEQ="Varying-illu03"
N_RUNS=3

CONTAINER_NAMES=(tofslam_r09_a_1 tofslam_r09_a_2 tofslam_r09_a_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

echo "================================================================"
echo "  Sprint 13 R0.10 — Smoke B (VI03 cross-class regression)"
echo "  Label:   ${LABEL}"
echo "  Output:  ${OUT_BASE}/"
echo "  Config:  ${CONFIG}"
echo "  Seq:     ${SEQ} (rate=${RATE}, N_runs=${N_RUNS})"
echo "  Started: $(date)"
echo "================================================================"

mkdir -p "${OUT_BASE}/${SEQ}"

# Verify containers up + binary built; if missing, respawn.
need_rebuild=0
for name in "${CONTAINER_NAMES[@]}"; do
  if ! docker ps --format '{{.Names}}' | grep -q "^${name}$"; then
    echo "[setup] container ${name} missing → respawn required"
    need_rebuild=1
  fi
done

if [ ${need_rebuild} -eq 1 ]; then
  echo "[setup] respawning containers + rebuild..."
  for name in "${CONTAINER_NAMES[@]}"; do
    docker rm -f "$name" 2>/dev/null || true
  done
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
  for c in "${CONTAINER_NAMES[@]}"; do
    docker exec "$c" bash -c \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       rm -rf build devel && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3" &
  done
  wait
else
  echo "[setup] reusing existing containers (already built from Smoke A2)"
  # Remount dump into the running containers via path — they had earlier dump bind; need fresh path.
  # Strategy: since dump was mounted to dump/S13_R0_10_smoke_A2_..., we use docker cp out, OR
  # write to the container path then docker cp out. Simpler: write to a known in-container path
  # and cp out at the end. But run_avia_exp.sh takes absolute container-side out_dir, so we
  # just direct it to /root/catkin_ws/dump_smoke_b/${SEQ}/run${i} and cp out.
  echo "[setup] using in-container path /root/catkin_ws/dump_smoke_b for output"
fi

# Cleanup any prior stragglers inside containers
for c in "${CONTAINER_NAMES[@]}"; do
  docker exec "$c" bash -c "rm -rf /root/catkin_ws/dump_smoke_b && mkdir -p /root/catkin_ws/dump_smoke_b" 2>/dev/null || true
done

# Run VI03 × 3 in parallel (one per container)
echo "[run] launching 3 parallel ${SEQ} runs rate=${RATE}..."
run_one_in_slot() {
  local slot=$1
  local run_id=$2
  local name="${CONTAINER_NAMES[$slot]}"
  local port="${PORTS[$slot]}"
  local out_in="/root/catkin_ws/dump_smoke_b/${SEQ}/run${run_id}"
  local log_host="${OUT_BASE}/${SEQ}/run${run_id}.log"
  docker exec "$name" bash /root/catkin_ws/docker/run_avia_exp.sh \
    "${CONFIG}" "${SEQ}" "${out_in}" "${port}" "${RATE}" \
    > "${log_host}" 2>&1
  # Copy artifacts back to host
  local out_host="${OUT_BASE}/${SEQ}/run${run_id}"
  mkdir -p "${out_host}"
  docker cp "${name}:${out_in}/." "${out_host}/" 2>/dev/null || true
}
for slot in 0 1 2; do
  run_one_in_slot $slot $((slot + 1)) &
done
wait
echo "[run] ${SEQ} batch complete."

# Verdict
echo ""
echo "================================================================"
echo "  Smoke B R0.10 verdict — $(date)"
echo "================================================================"

VI03_HI=0.65
verdict_overall="PASS"

echo ""
echo "  --- ${SEQ} ---"
ATES=()
MD5S=()
for i in 1 2 3; do
  TRAJ="${OUT_BASE}/${SEQ}/run${i}/traj.csv"
  ATE_FILE="${OUT_BASE}/${SEQ}/run${i}/ate_result.txt"
  md5="MISSING"; rmse="MISSING"
  if [ -f "${TRAJ}" ]; then
    md5=$(md5sum "${TRAJ}" | awk '{print $1}')
  fi
  if [ -f "${ATE_FILE}" ]; then
    rmse=$(grep "^rmse:" "${ATE_FILE}" | awk '{print $2}')
  fi
  ATES+=("${rmse}")
  MD5S+=("${md5}")
  printf "  run%d  ATE = %-22s  md5 = %s\n" "$i" "${rmse}" "${md5}"
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

# ATE gate
for i in 0 1 2; do
  v="${ATES[$i]}"
  if [ "${v}" = "MISSING" ]; then
    echo "  ATE run$((i+1)) MISSING ........... FAIL"
    verdict_overall="FAIL"
  else
    below=$(awk -v v="${v}" -v hi="${VI03_HI}" \
      'BEGIN{print (v<=hi) ? "PASS":"FAIL"}')
    echo "  ATE run$((i+1)) ≤ ${VI03_HI} ........... ${below}"
    [ "${below}" = "FAIL" ] && verdict_overall="FAIL"
  fi
done

# Class check: STAGE_A LOCK class=CLASS_D, 0 [H4] lines
for i in 1 2 3; do
  LOG="${OUT_BASE}/${SEQ}/run${i}.log"
  if [ -f "${LOG}" ]; then
    stage_a=$(grep -E "STAGE_A (LOCK|DEFER)" "${LOG}" | head -1)
    h4_count=$(grep -c "\[H4\]" "${LOG}" || echo 0)
    if echo "${stage_a}" | grep -q "STAGE_A LOCK.*class=CLASS_D"; then
      echo "  run$i Stage A=CLASS_D LOCK ........ PASS"
    elif echo "${stage_a}" | grep -q "STAGE_A LOCK"; then
      echo "  run$i Stage A unexpected: ${stage_a} ... FAIL"
      verdict_overall="FAIL"
    else
      # STAGE_A DEFER → Stage B LOCK path; CLASS_D may come from Stage B
      stage_b=$(grep -E "STAGE_B LOCK" "${LOG}" | head -1)
      if echo "${stage_b}" | grep -q "class=CLASS_D"; then
        echo "  run$i Stage A=DEFER, Stage B=CLASS_D LOCK ... PASS"
      else
        echo "  run$i no CLASS_D lock: A=${stage_a} B=${stage_b} ... FAIL"
        verdict_overall="FAIL"
      fi
    fi
    if [ "${h4_count}" -eq 0 ]; then
      echo "  run$i [H4] absent (0 lines) ...... PASS — gating works"
    else
      echo "  run$i [H4] fired ${h4_count} lines ........ FAIL — H4 gating broken"
      verdict_overall="FAIL"
    fi
  fi
done

echo ""
echo "================================================================"
echo "  Overall: ${verdict_overall}"
echo "================================================================"

if [ "${verdict_overall}" = "FAIL" ]; then
  exit 1
fi
exit 0
