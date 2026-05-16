#!/bin/bash
# run_diag_single.sh — Run a single-sequence diagnostic screening in 3 parallel containers
# Uses the same container setup pattern as run_canonical_eval.sh (proven to work)
#
# Usage:
#   bash docker/run_diag_single.sh <ROUND_NAME> <CONFIG1:SEQ1:LABEL1> [CONFIG2:SEQ2:LABEL2] [CONFIG3:SEQ3:LABEL3]
#
# Example:
#   bash docker/run_diag_single.sh diag_vi03_oc03 \
#     avia_seq/vi03_no_geom.yaml:Varying-illu03:vi03_no_geom \
#     avia_seq/varying_illu03.yaml:Varying-illu03:vi03_with_geom \
#     avia_seq/oc03_no_corrq.yaml:Occlusion03:oc03_no_corrq

set -e
cd "$(dirname "$0")/.."

ROUND="${1:?Usage: run_diag_single.sh <ROUND_NAME> <CONFIG:SEQ:LABEL> ...}"
shift

# Parse config:seq:label triples
CFGS=()
SEQS=()
LBLS=()
for triple in "$@"; do
  IFS=':' read -r cfg seq lbl <<< "$triple"
  CFGS+=("$cfg")
  SEQS+=("$seq")
  LBLS+=("$lbl")
done

N=${#CFGS[@]}
if [ $N -eq 0 ] || [ $N -gt 3 ]; then
  echo "ERROR: Specify 1-3 config:seq:label triples"
  exit 1
fi

OUT_ROOT="dump/${ROUND}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

# Settings (same as canonical eval)
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_diag_1 tofslam_diag_2 tofslam_diag_3)
MEM="3g"
RATE="${DIAG_RATE:-3.0}"

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do
    docker rm -f "$c" 2>/dev/null || true
  done
}
trap cleanup EXIT

log "========================================="
log "  Diagnostic Screening: ${ROUND}"
log "  Rate: ${RATE}"
log "  Started: $(date)"
for i in $(seq 0 $((N-1))); do
  log "  [${i}] ${CFGS[$i]} → ${SEQS[$i]} → ${LBLS[$i]}"
done
log "========================================="

# ── Create containers (same pattern as canonical eval) ──
cleanup
sleep 1
for i in $(seq 0 $((N-1))); do
  docker run -d --rm --name "${CONTAINERS[$i]}" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    "${IMAGE}" bash -lc "sleep infinity" > /dev/null
done
sleep 2

# ── Build in all containers ──
log "[Build] Starting..."
PIDS=()
for i in $(seq 0 $((N-1))); do
  docker exec "${CONTAINERS[$i]}" bash -c \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
     catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1" &
  PIDS+=($!)
done
for pid in "${PIDS[@]}"; do wait "$pid"; done
for i in $(seq 0 $((N-1))); do
  docker exec "${CONTAINERS[$i]}" pip3 install scipy numpy -q 2>/dev/null || true
done
log "[Build] Done."

# ── Run experiments in parallel ──
log "[Run] Starting ${N} experiments..."
PIDS=()
for i in $(seq 0 $((N-1))); do
  local_out="/root/catkin_ws/dump/${LBLS[$i]}"
  log "  [${CONTAINERS[$i]}] ${SEQS[$i]} (${CFGS[$i]}) → ${LBLS[$i]}"
  docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/run_avia_exp.sh" \
    "${CFGS[$i]}" "${SEQS[$i]}" "${local_out}" "${PORTS[$i]}" "${RATE}" >> "${LOG}" 2>&1 &
  PIDS+=($!)
done
for i in "${!PIDS[@]}"; do
  if wait "${PIDS[$i]}" 2>/dev/null; then
    log "  ✓ ${LBLS[$i]} DONE"
  else
    log "  ✗ ${LBLS[$i]} FAILED"
  fi
done

# ── Collect results ──
log ""
log "--- Results ---"
for i in $(seq 0 $((N-1))); do
  f="${OUT_ROOT}/${LBLS[$i]}/ate_result.txt"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
    [ -z "$rmse" ] && rmse=$(grep "ATE RMSE:" "$f" 2>/dev/null | awk '{print $3}')
    printf "  %-25s %s m\n" "${LBLS[$i]}" "${rmse:-PARSE_FAIL}" | tee -a "${LOG}"
  else
    printf "  %-25s FAIL\n" "${LBLS[$i]}" | tee -a "${LOG}"
  fi
done

log ""
log "=== Diagnostic complete: $(date) ==="
