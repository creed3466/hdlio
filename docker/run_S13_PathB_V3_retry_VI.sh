#!/bin/bash
# run_S13_PathB_V3_retry_VI.sh — VI03 + VI04 retry with extended timeout + clean state.
#
# Resolves runner cleanup gap from V3 extended run (VI03 timeout zombie roscore
# contaminated VI04 → CV 31%). This retry:
#   1. VI03: TIMEOUT_S = 1800 (30 min) instead of 900 (15 min)
#   2. Explicit roscore kill BEFORE each run start in each container
#   3. Fresh containers for VI04 (no contamination from VI03)
#   4. Both seqs × 3-run, 3 containers parallel
#
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_PathB_V3_retry_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/v3_retry.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
CFG="avia_outdoor.yaml"
IMAGE="tofslam:ros1"
TIMEOUT_S=1800     # 30 min — VI03 worst case ~17 min real

CONTAINERS=(tofslam_PBV3R_1 tofslam_PBV3R_2 tofslam_PBV3R_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup_containers() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup_containers EXIT

log "==== V3 retry — VI03 + VI04, timeout=${TIMEOUT_S}s, fresh containers per seq ===="

run_seq_fresh() {
  local seq="$1"
  log ""
  log "==== ${seq} × 3 runs (fresh containers) ===="

  # Fresh containers per seq — eliminates any zombie process contamination
  cleanup_containers
  sleep 2
  for i in 0 1 2; do
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
  log "  Building containers..."
  for i in 0 1 2; do
    docker exec "${CONTAINERS[$i]}" bash -lc \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
  done
  wait
  log "  Build done."

  # Pre-run roscore kill (defensive)
  for c in "${CONTAINERS[@]}"; do
    docker exec "$c" bash -lc "killall -9 rosmaster roscore tofslam_node rosbag rosout 2>/dev/null || true; sleep 1" 2>/dev/null || true
  done

  local PIDS=()
  for i in 0 1 2; do
    out_dir="dump/${seq}_r${i}"
    log "    ${seq} r${i} on port ${PORTS[$i]} (timeout ${TIMEOUT_S}s)..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "bash /root/catkin_ws/docker/run_avia_exp.sh ${CFG} ${seq} ${out_dir} ${PORTS[$i]} ${RATE} 2>&1" \
      > "${OUT_ROOT}/${seq}_r${i}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done

  # Post-run cleanup
  for c in "${CONTAINERS[@]}"; do
    docker exec "$c" bash -lc "killall -9 tofslam_node rosbag rosout rosmaster roscore 2>/dev/null || true" 2>/dev/null || true
  done
}

run_seq_fresh "Varying-illu03"
run_seq_fresh "Varying-illu04"

# Analysis
log ""
log "==== Retry Analysis ===="

for seq in Varying-illu03 Varying-illu04; do
  log ""
  log "  ${seq}:"
  for r in 0 1 2; do
    ate_file="${OUT_ROOT}/${seq}_r${r}/ate_result.txt"
    if [ -f "$ate_file" ]; then
      rmse=$(grep -oE "^rmse: [0-9.]+" "$ate_file" | awk '{print $2}')
      n_match=$(grep -oE "^n_matches: [0-9]+" "$ate_file" | awk '{print $2}')
      n_gt=$(grep -oE "^n_gt: [0-9]+" "$ate_file" | awk '{print $2}')
      coverage=$(python3 -c "print(f'{100.0 * ${n_match}/${n_gt}:.1f}%')")
      log "    r${r}: ${rmse}  (matches=${n_match}/${n_gt} cov=${coverage})"
    else
      log "    r${r}: MISSING (timeout?)"
    fi
  done
  log0="${OUT_ROOT}/${seq}_r0_stdout.log"
  cls=$(grep -oE 'STAGE_[AB] LOCK.*class=[A-Z_]+' "$log0" 2>/dev/null | head -1)
  log "    class lock: ${cls}"
done

log ""
log "==== S13 V3 retry DONE — $(date) ===="
