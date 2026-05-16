#!/bin/bash
# run_R9_F7_indoor.sh — R9 F.7 indoor 8-seq canonical eval @ rate=1.0
# Unified avia_indoor.yaml for all 8 indoor seqs (classifier disabled).
# Target: indoor 9-seq mean ATE ≤ 0.80m (SOFT per user authorization).
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-R9_F7_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
IMAGE="tofslam:ros1"
TIMEOUT_S=1800
# Per-seq config selection (canonical indoor pattern)
declare -A SEQ_CFGS
SEQ_CFGS[indoor_Dark03]="avia_indoor_seq/iDark03.yaml"
SEQ_CFGS[indoor_Dark04]="avia_indoor_seq/iDark04.yaml"
SEQ_CFGS[indoor_Dynamic01]="avia_indoor_seq/iDyn01.yaml"
SEQ_CFGS[indoor_Dynamic02]="avia_indoor_seq/iDyn02.yaml"
SEQ_CFGS[indoor_Occlusion01]="avia_indoor_seq/iOcc01.yaml"
SEQ_CFGS[indoor_Occlusion02]="avia_indoor_seq/iOcc02.yaml"
SEQ_CFGS[indoor_Varying-illu01]="avia_indoor_seq/iVI01.yaml"
SEQ_CFGS[indoor_Varying-illu02]="avia_indoor_seq/iVI02.yaml"

CONTAINERS=(tofslam_R9F7_1 tofslam_R9F7_2 tofslam_R9F7_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

BATCH1=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01)
BATCH2=(indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02)
BATCH3=(indoor_Varying-illu01 indoor_Varying-illu02)

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

log "==== R9 F.7 INDOOR ($(date)) ===="
log "8 indoor seqs | unified avia_indoor.yaml | rate=${RATE}"
log "Target: 8-seq mean ≤ 0.80m"

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
    cfg="${SEQ_CFGS[$seq]}"
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

run_batch "${BATCH1[@]}"
run_batch "${BATCH2[@]}"
run_batch "${BATCH3[@]}"

log ""
log "==== RESULTS ===="
ALL_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
TOTAL_ATE=0
COUNT=0
log "seq, ATE"
for seq in "${ALL_SEQS[@]}"; do
  ate_file="${OUT_ROOT}/${seq}/ate_result.txt"
  if [[ -f "$ate_file" ]]; then
    ate=$(grep -oE "^rmse: [0-9.]+" "$ate_file" | awk '{print $2}')
    log "  $seq: ATE=$ate"
    TOTAL_ATE=$(python3 -c "print($TOTAL_ATE + $ate)")
    COUNT=$((COUNT+1))
  else
    log "  $seq: NO ATE"
  fi
done

if [[ $COUNT -gt 0 ]]; then
  MEAN=$(python3 -c "print(round($TOTAL_ATE/$COUNT, 4))")
  log ""
  log "==== SUMMARY ===="
  log "${COUNT}-seq mean ATE: $MEAN (target ≤ 0.80m)"
  MEAN_STATUS=$(python3 -c "print('PASS' if $MEAN<=0.80 else 'FAIL')")
  log "SOFT GATE: $MEAN_STATUS"
fi

log ""
log "==== DONE $(date) ===="
