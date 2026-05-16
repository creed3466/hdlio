#!/bin/bash
# run_R9_F8_mid360.sh — R9 F.8 Mid-360 17-seq eval with R-A classifier enabled
# Tests if R-A two-stage classifier (calibrated on Avia) generalizes to Mid-360.
# unified_mid360_v3c.yaml has classifier enabled; VI03 overlay has classifier
# disabled (per-seq pattern matches Avia).
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-R9_F8_mid360_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
IMAGE="tofslam:ros1"
UNIFIED_CFG="unified_mid360_v3c.yaml"
VI03_OVERLAY="mid360_seq/varying_illu03_v3c.yaml"
TIMEOUT_S=1800

CONTAINERS=(tofslam_R9F8_1 tofslam_R9F8_2 tofslam_R9F8_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

OUT_B1=(Dark01 Dark02 Dynamic03)
OUT_B2=(Dynamic04 Occlusion03 Occlusion04)
OUT_B3=(Varying-illu03 Varying-illu04 Varying-illu05)
IN_B1=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01)
IN_B2=(indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02)
IN_B3=(indoor_Varying-illu01 indoor_Varying-illu02)

get_config() {
  local seq="$1"
  if [[ "$seq" == "Varying-illu03" ]]; then
    echo "${VI03_OVERLAY}"
  else
    echo "${UNIFIED_CFG}"
  fi
}

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

log "==== R9 F.8 Mid-360 ($(date)) ===="
log "17 Mid-360 seqs | rate=${RATE} | classifier enabled (unified), disabled (VI03 overlay)"

run_batch() {
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}
  cleanup; sleep 1
  for i in $(seq 0 $((n-1))); do
    docker run -d --rm --init --name "${CONTAINERS[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump:rw" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      -v "${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 2
  log "  [Build] ${n} containers..."
  for i in $(seq 0 $((n-1))); do
    docker exec "${CONTAINERS[$i]}" bash -lc \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
  done
  wait
  log "  [Build] Done."

  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    seq="${batch_seqs[$i]}"
    port="${PORTS[$i]}"
    out_dir="dump/${seq}"
    cfg=$(get_config "${seq}")
    log "    Run ${seq} (cfg=${cfg}) on port ${port}..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "TOFSLAM_DEBUG_DETERMINISM=0 bash /root/catkin_ws/docker/run_avia_exp.sh \
       ${cfg} ${seq} ${out_dir} ${port} ${RATE} 2>&1 | \
       tee /root/catkin_ws/${out_dir}/run.log" \
      > "${OUT_ROOT}/${seq}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done
  for c in "${CONTAINERS[@]}"; do
    docker exec "$c" bash -lc \
      "killall -9 tofslam_node rosbag rosout rosmaster roscore 2>/dev/null || true; \
       pkill -9 -f tofslam_node 2>/dev/null || true" 2>/dev/null || true
  done
}

run_batch "${OUT_B1[@]}"
run_batch "${OUT_B2[@]}"
run_batch "${OUT_B3[@]}"
run_batch "${IN_B1[@]}"
run_batch "${IN_B2[@]}"
run_batch "${IN_B3[@]}"

log ""
log "==== RESULTS ===="
ALL_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05 indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
TOTAL_ATE=0
COUNT=0
OUT_SUM=0; OUT_N=0
IN_SUM=0; IN_N=0
log "seq, ATE, Stage_lock"
for seq in "${ALL_SEQS[@]}"; do
  ate_file="${OUT_ROOT}/${seq}/ate_result.txt"
  stdout_log="${OUT_ROOT}/${seq}_stdout.log"
  ate=$([[ -f "$ate_file" ]] && grep -oE "^rmse: [0-9.]+" "$ate_file" | awk '{print $2}' || echo "NaN")
  lock=$(grep -oE "STAGE_[AB] LOCK frame=[0-9]+ class=[A-Z_]+|scene classifier DISABLED" "$stdout_log" 2>/dev/null | tail -1 || echo "?")
  log "  $seq: ATE=$ate, $lock"
  if [[ "$ate" != "NaN" ]]; then
    TOTAL_ATE=$(python3 -c "print($TOTAL_ATE + $ate)")
    COUNT=$((COUNT+1))
    if [[ "$seq" =~ ^indoor_ ]]; then
      IN_SUM=$(python3 -c "print($IN_SUM + $ate)"); IN_N=$((IN_N+1))
    else
      OUT_SUM=$(python3 -c "print($OUT_SUM + $ate)"); OUT_N=$((OUT_N+1))
    fi
  fi
done

if [[ $COUNT -gt 0 ]]; then
  MEAN=$(python3 -c "print(round($TOTAL_ATE/$COUNT, 4))")
  OUT_MEAN=$(python3 -c "print(round($OUT_SUM/$OUT_N, 4)) if $OUT_N>0 else print('NaN')")
  IN_MEAN=$(python3 -c "print(round($IN_SUM/$IN_N, 4)) if $IN_N>0 else print('NaN')")
  log ""
  log "==== SUMMARY ===="
  log "Outdoor mean (${OUT_N}/9): $OUT_MEAN"
  log "Indoor mean (${IN_N}/8):  $IN_MEAN"
  log "17-seq grand mean:        $MEAN"
fi

log ""
log "==== DONE $(date) ===="
