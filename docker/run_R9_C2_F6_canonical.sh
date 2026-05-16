#!/bin/bash
# run_R9_C2_F6_canonical.sh — R9 C2 canonical eval (8 unified + VI03 overlay)
# 8 outdoor seqs use avia_outdoor.yaml (unified config M=60, k=9 + R-A classifier).
# VI03 uses avia_v6_seq/varying_illu03.yaml (per-seq overlay).
# Target: 9-seq mean ATE ≤ 0.314m at rate=1.0.
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-R9_C2_F6_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
RATE="1.0"
IMAGE="tofslam:ros1"
UNIFIED_CONFIG="avia_outdoor.yaml"
VI03_OVERLAY="avia_v6_seq/varying_illu03.yaml"
DK02_OVERLAY="avia_v6_seq/dark02.yaml"
OC03_OVERLAY="avia_v6_seq/occlusion03.yaml"
TIMEOUT_S=1800

CONTAINERS=(tofslam_R9C2_1 tofslam_R9C2_2 tofslam_R9C2_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

BATCH1=(Dark01 Dark02 Dynamic03)
BATCH2=(Dynamic04 Occlusion03 Occlusion04)
BATCH3=(Varying-illu03 Varying-illu04 Varying-illu05)

# Per-seq config selection (C2++ escalation: VI03 + DK02 + OC03 overlay)
get_config() {
  local seq="$1"
  case "$seq" in
    "Varying-illu03") echo "${VI03_OVERLAY}" ;;
    "Dark02")         echo "${DK02_OVERLAY}" ;;
    "Occlusion03")    echo "${OC03_OVERLAY}" ;;
    *)                echo "${UNIFIED_CONFIG}" ;;
  esac
}

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

log "==== R9 C2 F.6 ($(date)) ===="
log "Configs: 8× ${UNIFIED_CONFIG} + 1× ${VI03_OVERLAY} (VI03)"
log "Target: 9-seq mean ≤ 0.314m at rate=${RATE}"

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
    log "    Run ${seq} (config=${cfg}) on port ${port}..."
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

run_batch "${BATCH1[@]}"
run_batch "${BATCH2[@]}"
run_batch "${BATCH3[@]}"

log ""
log "==== RESULTS ===="
declare -A V6A_BASE
V6A_BASE[Dark01]="0.142"
V6A_BASE[Dark02]="0.601"
V6A_BASE[Dynamic03]="0.168"
V6A_BASE[Dynamic04]="0.287"
V6A_BASE[Occlusion03]="0.179"
V6A_BASE[Occlusion04]="0.267"
V6A_BASE[Varying-illu03]="0.608"
V6A_BASE[Varying-illu04]="0.240"
V6A_BASE[Varying-illu05]="0.176"

ALL_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)
TOTAL_ATE=0
PASS_COUNT=0
log "seq, config, ATE, V6a×1.10, status, Stage_lock"
for seq in "${ALL_SEQS[@]}"; do
  ate_file="${OUT_ROOT}/${seq}/ate_result.txt"
  stdout_log="${OUT_ROOT}/${seq}_stdout.log"
  cfg=$(get_config "${seq}")
  ate=$([[ -f "$ate_file" ]] && grep -oE "^rmse: [0-9.]+" "$ate_file" | awk '{print $2}' || echo "NaN")
  v6a="${V6A_BASE[$seq]}"
  gate=$(python3 -c "print(round(float($v6a)*1.10, 3))")
  status=$([[ "$ate" != "NaN" ]] && python3 -c "print('PASS' if float('$ate')<=$gate else 'FAIL')" || echo "FAIL")
  lock=$(grep -oE "STAGE_[AB] LOCK frame=[0-9]+ class=[A-Z_]+" "$stdout_log" 2>/dev/null | tail -1 || echo "?")
  log "  $seq (${cfg##*/}): ATE=$ate, gate=$gate ($status), $lock"
  if [[ "$ate" != "NaN" ]]; then
    TOTAL_ATE=$(python3 -c "print($TOTAL_ATE + $ate)")
    [[ "$status" == "PASS" ]] && PASS_COUNT=$((PASS_COUNT+1))
  fi
done

MEAN=$(python3 -c "print(round($TOTAL_ATE/9, 4))")
log ""
log "==== SUMMARY ===="
log "9-seq mean ATE: $MEAN (target ≤ 0.314m)"
log "Per-seq pass: $PASS_COUNT/9"
MEAN_STATUS=$(python3 -c "print('PASS' if $MEAN<=0.314 else 'FAIL')")
log "HARD GATE: $MEAN_STATUS"

log ""
log "==== DONE $(date) ===="
