#!/bin/bash
# run_S13_R0_8_telemetry.sh — Sprint 13 R0.8 Step 0 telemetry runner
#
# Runs 9 Avia outdoor seqs × {rate=1.0, rate=3.0} = 18 runs against
# avia_outdoor.yaml (unified config) with TEMP telemetry hooks active.
# Captures roslaunch stdout per run; later aggregation extracts the
# [S13-TELEM] STAGE_A/STAGE_B_LOCK lines for H2 threshold tuning.
#
# Output: dump/<LABEL>/<seq>_r<rate>.log  (full roslaunch stdout)
#         dump/<LABEL>/<seq>_r<rate>/traj.csv etc. (standard run_avia_exp.sh outputs)
#
# Distribution: 3-container parallel; 6 runs per container (3 seqs × 2 rates)
#   p1: Dark01,   Dark02,         Dynamic03         × {1.0, 3.0}
#   p2: Dynamic04,Occlusion03,    Occlusion04       × {1.0, 3.0}
#   p3: Varying-illu03, Varying-illu04, Varying-illu05 × {1.0, 3.0}
#
# Wallclock estimate: ~10 min build + ~30-40 min runs ≈ 45 min total.
# (Architect contract budget: ~90 min — conservative upper bound.)
#
# Usage:
#   bash docker/run_S13_R0_8_telemetry.sh [LABEL]
#   bash docker/run_S13_R0_8_telemetry.sh aggregate <LABEL>   # parse logs into CSV

set -e
cd "$(dirname "$0")/.."

# ----------------------------------------------------------------------
# Sub-command: aggregate
# ----------------------------------------------------------------------
if [ "${1:-}" = "aggregate" ]; then
  LABEL="${2:?Usage: $0 aggregate <LABEL>}"
  OUT_BASE="dump/${LABEL}"
  CSV="${OUT_BASE}/telemetry_aggregate.csv"
  echo "[aggregate] scanning ${OUT_BASE}/*.log → ${CSV}"
  printf 'seq,rate,stage,frame,class,rho,max_degen,cos2_mean,pct_d3,pct_c02,pct_c03,vel_mean,ndeg_mean,corrs_mean,l1_count,corrs,frames_collected\n' > "${CSV}"
  shopt -s nullglob
  for log in "${OUT_BASE}"/*_r*.log; do
    base=$(basename "$log" .log)
    seq="${base%_r*}"
    rate="${base##*_r}"
    while IFS= read -r line; do
      # STAGE_A_LOCK
      if [[ "$line" =~ \[S13-TELEM\]\ STAGE_A_(LOCK|DEFER)\ frame=([0-9]+)\ class=([A-Za-z_]+)?\ ?rho_1=([0-9.\-]+)\ max_degen=([0-9]+)\ cos2_mean_inst=([0-9.\-]+)\ l1_count=([0-9]+)\ corrs=([0-9]+) ]]; then
        local_stage="STAGE_A_${BASH_REMATCH[1]}"
        printf '%s,%s,%s,%s,%s,%s,%s,%s,,,,,,,%s,%s,\n' \
          "$seq" "$rate" "$local_stage" "${BASH_REMATCH[2]}" "${BASH_REMATCH[3]:-NA}" \
          "${BASH_REMATCH[4]}" "${BASH_REMATCH[5]}" "${BASH_REMATCH[6]}" \
          "${BASH_REMATCH[7]}" "${BASH_REMATCH[8]}" >> "${CSV}"
      elif [[ "$line" =~ \[S13-TELEM\]\ STAGE_B_LOCK\ frame=([0-9]+)\ class=([A-Za-z_]+)\ rho_warmup=([0-9.\-]+)\ cos2_mean=([0-9.\-]+)\ pct_d3=([0-9.\-]+)\ pct_c02=([0-9.\-]+)\ pct_c03=([0-9.\-]+)\ vel_mean=([0-9.\-]+)\ ndeg_mean=([0-9.\-]+)\ corrs_mean=([0-9.\-]+)\ frames=([0-9]+) ]]; then
        printf '%s,%s,STAGE_B_LOCK,%s,%s,%s,,%s,%s,%s,%s,%s,%s,%s,,,%s\n' \
          "$seq" "$rate" \
          "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}" "${BASH_REMATCH[3]}" \
          "${BASH_REMATCH[4]}" "${BASH_REMATCH[5]}" "${BASH_REMATCH[6]}" \
          "${BASH_REMATCH[7]}" "${BASH_REMATCH[8]}" "${BASH_REMATCH[9]}" \
          "${BASH_REMATCH[10]}" "${BASH_REMATCH[11]}" >> "${CSV}"
      fi
    done < "$log"
  done
  shopt -u nullglob
  echo "[aggregate] done — rows: $(($(wc -l < "${CSV}") - 1))"
  echo "[aggregate] preview:"
  head -20 "${CSV}"
  exit 0
fi

# ----------------------------------------------------------------------
# Main: launch + run
# ----------------------------------------------------------------------
LABEL="${1:-S13_R0_8_telemetry_$(date +%Y%m%d_%H%M)}"
OUT_BASE="dump/${LABEL}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
CONFIG="avia_outdoor.yaml"

echo "================================================================"
echo "  Sprint 13 R0.8 — Step 0 telemetry"
echo "  Label:  ${LABEL}"
echo "  Output: ${OUT_BASE}/"
echo "  Config: ${CONFIG}"
echo "  Started: $(date)"
echo "================================================================"

mkdir -p "${OUT_BASE}"

# Container specs
CONTAINER_NAMES=(tofslam_s13t_p1 tofslam_s13t_p2 tofslam_s13t_p3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
MEM="3g"

# Sequence groups per container (3 seqs × 2 rates each)
SEQS_P1=(Dark01 Dark02 Dynamic03)
SEQS_P2=(Dynamic04 Occlusion03 Occlusion04)
SEQS_P3=(Varying-illu03 Varying-illu04 Varying-illu05)
RATES=(1.0 3.0)

# Cleanup any leftover containers
for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done

# Spawn containers
echo "[spawn] starting containers..."
for i in 0 1 2; do
  name="${CONTAINER_NAMES[$i]}"
  cpus="${CPUSETS[$i]}"
  docker run -d --rm --name "$name" \
    --network host --cpuset-cpus "$cpus" --memory "$MEM" --ipc private \
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

# ----------------------------------------------------------------------
# Per-container sequential runs
# ----------------------------------------------------------------------
run_seqs_in_container() {
  local slot=$1
  local -n seq_list=$2
  local name="${CONTAINER_NAMES[$slot]}"
  local port="${PORTS[$slot]}"
  local prefix="[${name}]"

  for SEQ in "${seq_list[@]}"; do
    for RATE in "${RATES[@]}"; do
      local label_key="${SEQ}_r${RATE}"
      local out_in="/root/catkin_ws/dump/${label_key}"
      local log_host="${OUT_BASE}/${label_key}.log"
      echo "${prefix} ${SEQ} rate=${RATE} → ${log_host}"
      # Capture full docker exec stdout/stderr (includes [S13-TELEM] from
      # roslaunch console output). Failure of a single run does NOT abort
      # the loop (set +e locally).
      ( set +e
        docker exec "$name" bash /root/catkin_ws/docker/run_avia_exp.sh \
          "${CONFIG}" "${SEQ}" "${out_in}" "${port}" "${RATE}" \
          > "${log_host}" 2>&1
        rc=$?
        if [ $rc -ne 0 ]; then
          echo "${prefix} WARN: ${label_key} exited rc=${rc}"
        fi
      )
    done
  done
}

echo "[run] launching per-container loops in parallel..."
run_seqs_in_container 0 SEQS_P1 &
P1=$!
run_seqs_in_container 1 SEQS_P2 &
P2=$!
run_seqs_in_container 2 SEQS_P3 &
P3=$!
wait $P1 $P2 $P3
echo "[run] all runs complete."

# Cleanup containers
for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done

# Quick summary
echo ""
echo "================================================================"
echo "  Step 0 telemetry done — $(date)"
echo "  Logs:"
ls -1 "${OUT_BASE}"/*.log 2>/dev/null | head -20
echo ""
echo "  Aggregate next:"
echo "    bash docker/run_S13_R0_8_telemetry.sh aggregate ${LABEL}"
echo "================================================================"
