#!/bin/bash
# run_S9_R1_1_avia_canonical.sh — Sprint 9 R1.1 Avia 17-seq eval @ rate=1.0
# Tests VoxelMap rank-2 tangent-plane sigma2_normal correction (1-line fix).
# Preserves Sprint 5 R9-C2++ per-seq overlay structure.
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S9_R1_1}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
IMAGE="tofslam:ros1"
TIMEOUT_S=2400

declare -A SEQ_CFGS
SEQ_CFGS[Dark01]="avia_v6_seq/dark01.yaml"
SEQ_CFGS[Dark02]="avia_v6_seq/dark02.yaml"
SEQ_CFGS[Dynamic03]="avia_v6_seq/dynamic03.yaml"
SEQ_CFGS[Dynamic04]="avia_v6_seq/dynamic04.yaml"
SEQ_CFGS[Occlusion03]="avia_v6_seq/occlusion03.yaml"
SEQ_CFGS[Occlusion04]="avia_v6_seq/occlusion04.yaml"
SEQ_CFGS[Varying-illu03]="avia_v6_seq/varying_illu03.yaml"
SEQ_CFGS[Varying-illu04]="avia_v6_seq/varying_illu04.yaml"
SEQ_CFGS[Varying-illu05]="avia_v6_seq/varying_illu05.yaml"
SEQ_CFGS[indoor_Dark03]="avia_indoor_seq/iDark03.yaml"
SEQ_CFGS[indoor_Dark04]="avia_indoor_seq/iDark04.yaml"
SEQ_CFGS[indoor_Dynamic01]="avia_indoor_seq/iDyn01.yaml"
SEQ_CFGS[indoor_Dynamic02]="avia_indoor_seq/iDyn02.yaml"
SEQ_CFGS[indoor_Occlusion01]="avia_indoor_seq/iOcc01.yaml"
SEQ_CFGS[indoor_Occlusion02]="avia_indoor_seq/iOcc02.yaml"
SEQ_CFGS[indoor_Varying-illu01]="avia_indoor_seq/iVI01.yaml"
SEQ_CFGS[indoor_Varying-illu02]="avia_indoor_seq/iVI02.yaml"

CONTAINERS=(tofslam_S9_1 tofslam_S9_2 tofslam_S9_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== S9-R1.1 Avia Canonical Eval ===="
log "Label: ${LABEL} | Rate: ${RATE}"
log "Mechanism: VoxelMap rank-2 tangent-plane sigma2_normal (iekf_updater.cpp:737)"

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
  for i in $(seq 0 $((n-1))); do
    docker exec "${CONTAINERS[$i]}" bash -lc \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
  done
  wait
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    seq="${batch_seqs[$i]}"
    cfg="${SEQ_CFGS[$seq]}"
    log "    Run ${seq} cfg=${cfg} port ${PORTS[$i]}..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "bash /root/catkin_ws/docker/run_avia_exp.sh ${cfg} ${seq} dump/${seq} ${PORTS[$i]} ${RATE} 2>&1 | tee /root/catkin_ws/dump/${seq}/run.log" \
      > "${OUT_ROOT}/${seq}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done
  for c in "${CONTAINERS[@]}"; do
    docker exec "$c" bash -lc "killall -9 tofslam_node rosbag rosout rosmaster roscore 2>/dev/null || true; pkill -9 -f tofslam_node 2>/dev/null || true" 2>/dev/null || true
  done
}

log ""; log "==== Avia outdoor 9-seq ===="
run_batch Dark01 Dark02 Dynamic03
run_batch Dynamic04 Occlusion03 Occlusion04
run_batch Varying-illu03 Varying-illu04 Varying-illu05

log ""; log "==== Avia indoor 8-seq ===="
run_batch indoor_Dark03 indoor_Dark04 indoor_Dynamic01
run_batch indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02
run_batch indoor_Varying-illu01 indoor_Varying-illu02

log ""; log "==== AGGREGATE ===="
OUT9=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)
IN8=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)

for bucket_name in "outdoor" "indoor"; do
  if [[ "$bucket_name" == "outdoor" ]]; then seqs=("${OUT9[@]}"); else seqs=("${IN8[@]}"); fi
  total=0; count=0
  for seq in "${seqs[@]}"; do
    f="${OUT_ROOT}/${seq}/ate_result.txt"
    if [[ -f "$f" ]]; then
      ate=$(grep -oE "^rmse: [0-9.]+" "$f" | awk '{print $2}')
      [[ -n "$ate" ]] && { log "  $bucket_name $seq: $ate"; total=$(python3 -c "print($total + $ate)"); count=$((count+1)); }
    fi
  done
  [[ $count -gt 0 ]] && { mean=$(python3 -c "print(round($total/$count, 4))"); log "  $bucket_name MEAN ($count/${#seqs[@]}): $mean"; }
done
log ""; log "==== DONE $(date) ===="
