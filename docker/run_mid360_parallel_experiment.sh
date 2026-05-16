#!/bin/bash
# run_mid360_parallel_experiment.sh — Mid360 parallel screening runner
# Runs the SAME config on multiple sequences (max 3 parallel).
#
# Usage:
#   bash docker/run_mid360_parallel_experiment.sh <ROUND_NAME> <SEQ1> [SEQ2] [SEQ3] ...
#
# Example:
#   bash docker/run_mid360_parallel_experiment.sh phase14_combined Dark01 Dark02 Dynamic03
#
# - Uses ros1_m3dgr_mid360.yaml (common config)
# - 3 sequences max per batch, auto-batches if > 3
# - Single run per sequence (screening, not determinism validation)

set -e
cd "$(dirname "$0")/.."

ROUND="${1:?Usage: run_mid360_parallel_experiment.sh <ROUND_NAME> <SEQ1> [SEQ2] ...}"
shift

ALL_SEQS=("$@")
if [ ${#ALL_SEQS[@]} -eq 0 ]; then
  echo "ERROR: No sequences specified"
  exit 1
fi

CFG="ros1_m3dgr_mid360.yaml"
OUT_BASE="dump/${ROUND}"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
MAX_PARALLEL=3
RATE="3.0"

# Container specs
CONTAINER_NAMES=(tofslam_m360_p1 tofslam_m360_p2 tofslam_m360_p3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
MEM="3g"

echo "========================================="
echo "  Mid360 Parallel Screening: ${ROUND}"
echo "  Config: ${CFG}"
echo "  Sequences: ${ALL_SEQS[*]}"
echo "  Rate: ${RATE} (screening)"
echo "  Started: $(date)"
echo "========================================="

mkdir -p "${OUT_BASE}"

# Cleanup
for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done

start_container() {
  local slot=$1
  local name="${CONTAINER_NAMES[$slot]}"
  local cpuset="${CPUSETS[$slot]}"

  docker run -d --rm --name "$name" \
    --network host --cpuset-cpus "$cpuset" --memory "$MEM" \
    --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_BASE}:/root/catkin_ws/dump" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    tofslam:ros1 bash -lc "sleep infinity" > /dev/null

  sleep 2

  docker exec "$name" bash -c \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
     catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3"
  docker exec "$name" pip3 install scipy numpy -q 2>/dev/null || true
}

run_seq() {
  local slot=$1
  local seq=$2
  local name="${CONTAINER_NAMES[$slot]}"
  local port="${PORTS[$slot]}"

  echo "[${seq}] Starting on ${name} (port ${port})..."
  docker exec "$name" bash /root/catkin_ws/docker/run_avia_exp.sh \
    "${CFG}" "${seq}" "/root/catkin_ws/dump/${seq}" "$port" "${RATE}"
  echo "[${seq}] DONE"
}

# Batch execution
total=${#ALL_SEQS[@]}
batch_idx=0

while [ $batch_idx -lt $total ]; do
  batch_end=$((batch_idx + MAX_PARALLEL))
  [ $batch_end -gt $total ] && batch_end=$total
  batch_size=$((batch_end - batch_idx))

  echo ""
  echo "========================================="
  echo "  Batch $((batch_idx / MAX_PARALLEL + 1)): seqs $((batch_idx+1))-${batch_end} of ${total}"
  echo "========================================="

  # Start containers
  for slot in $(seq 0 $((batch_size - 1))); do
    echo "Starting container slot ${slot}..."
    start_container "$slot"
  done

  # Run sequences in parallel
  PIDS=()
  for slot in $(seq 0 $((batch_size - 1))); do
    sidx=$((batch_idx + slot))
    run_seq "$slot" "${ALL_SEQS[$sidx]}" &
    PIDS+=($!)
  done

  for pid in "${PIDS[@]}"; do
    wait "$pid"
  done

  # Cleanup containers
  for slot in $(seq 0 $((batch_size - 1))); do
    docker rm -f "${CONTAINER_NAMES[$slot]}" 2>/dev/null || true
  done

  batch_idx=$batch_end
done

# Results
echo ""
echo "========================================="
echo "  ${ROUND} RESULTS"
echo "  Completed: $(date)"
echo "========================================="

for SEQ in "${ALL_SEQS[@]}"; do
  F="${OUT_BASE}/${SEQ}/ate_result.txt"
  if [ -f "$F" ]; then
    RMSE=$(grep "^rmse:" "$F" | awk '{print $2}')
    printf "  %-20s: %.4f m\n" "${SEQ}" "$RMSE"
  else
    printf "  %-20s: FAIL\n" "${SEQ}"
  fi
done

echo ""
echo "========================================="
echo "  ${ROUND} COMPLETE"
echo "========================================="
