#!/bin/bash
# run_S8_R1_avia_canonical.sh — Sprint 8 R1 Avia 17-seq canonical eval @ rate=1.0
#
# Tests: classifier strengthening + kT_IN_DEFAULT correction.
# Strict 1-YAML: avia_outdoor.yaml used for ALL 17 Avia seqs.
#
# HARD gates:
#   Avia outdoor 9-seq ≤ 0.283m
#   Avia indoor  8-seq ≤ 0.70m
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S8_R1_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
IMAGE="tofslam:ros1"
CFG="avia_outdoor.yaml"   # Single Avia YAML for all 17 seqs
TIMEOUT_S=2400

CONTAINERS=(tofslam_S8R_1 tofslam_S8R_2 tofslam_S8R_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== S8-R1 Avia Canonical Eval ===="
log "Label: ${LABEL}, YAML: ${CFG}, Rate: ${RATE}"

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
    log "    Run ${seq} (cfg=${CFG}) port ${PORTS[$i]}..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "bash /root/catkin_ws/docker/run_avia_exp.sh ${CFG} ${seq} dump/${seq} ${PORTS[$i]} ${RATE} 2>&1 | tee /root/catkin_ws/dump/${seq}/run.log" \
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

compute_mean() {
  local label="$1"; shift
  local total=0 count=0
  for seq in "$@"; do
    f="${OUT_ROOT}/${seq}/ate_result.txt"
    if [[ -f "$f" ]]; then
      ate=$(grep -oE "^rmse: [0-9.]+" "$f" | awk '{print $2}')
      [[ -n "$ate" ]] && { log "  $label $seq: $ate"; total=$(python3 -c "print($total + $ate)"); count=$((count+1)); }
    fi
  done
  [[ $count -gt 0 ]] && { local mean=$(python3 -c "print(round($total/$count, 4))"); log "  $label MEAN: $mean"; }
}

compute_mean "outdoor" "${OUT9[@]}"
compute_mean "indoor"  "${IN8[@]}"

log ""; log "==== DONE $(date) ===="
