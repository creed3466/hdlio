#!/bin/bash
# run_iter5_validate.sh — 9-seq validation using same pattern as working screening scripts
# 3 batches × 3 sequences, proper container lifecycle per batch
set -e
cd "$(dirname "$0")/.."

RATE=3.0
OUT_BASE="dump/iter5_validate"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
MEM="3g"

# All 9 outdoor Avia sequences
declare -a BATCH1_SEQS=(Dark01 Dark02 Dynamic03)
declare -a BATCH1_CFGS=(avia_v6_seq/dark01.yaml avia_v6_seq/dark02.yaml avia_v6_seq/dynamic03.yaml)

declare -a BATCH2_SEQS=(Dynamic04 Occlusion03 Occlusion04)
declare -a BATCH2_CFGS=(avia_v6_seq/dynamic04.yaml avia_v6_seq/occlusion03.yaml avia_v6_seq/occlusion04.yaml)

declare -a BATCH3_SEQS=(Varying-illu03 Varying-illu04 Varying-illu05)
declare -a BATCH3_CFGS=(avia_v6_seq/varying_illu03.yaml avia_v6_seq/varying_illu04.yaml avia_v6_seq/varying_illu05.yaml)

echo "================================================================"
echo "  Iter5 Full Validation (9-seq, V6a-based per-seq)"
echo "  Rate: ${RATE}"
echo "  Started: $(date)"
echo "================================================================"

run_batch() {
  local batch_name=$1
  shift
  local -a seqs=()
  local -a cfgs=()
  local n=$((($# ) / 2))
  for ((i=0; i<n; i++)); do
    seqs+=("$1"); shift
    cfgs+=("$1"); shift
  done

  local containers=()
  for i in $(seq 0 $((n-1))); do
    containers+=("tofslam_val_${batch_name}_p$((i+1))")
  done

  echo ""
  echo "--- Batch ${batch_name}: ${seqs[*]} ---"

  # Cleanup
  for c in "${containers[@]}"; do docker rm -f "$c" 2>/dev/null || true; done

  # Create output dirs
  for i in $(seq 0 $((n-1))); do
    mkdir -p "${OUT_BASE}/${seqs[$i]}"
  done

  # Start containers
  for i in $(seq 0 $((n-1))); do
    docker run -d --rm --name "${containers[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" \
      --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src:ro" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_BASE}/${seqs[$i]}:/root/catkin_ws/dump" \
      -v "/home/euntae/Project/dataset/ros1/surfel_data:/root/catkin_ws/data/m3dgr_surfel:ro" \
      tofslam:ros1 bash -lc "sleep infinity" > /dev/null
  done
  sleep 2

  # Build (parallel)
  echo "  Building..."
  for i in $(seq 0 $((n-1))); do
    docker exec "${containers[$i]}" bash -c \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3" &
  done
  wait

  # Install deps
  for i in $(seq 0 $((n-1))); do
    docker exec "${containers[$i]}" pip3 install scipy numpy -q 2>/dev/null &
  done
  wait

  # Run experiments (parallel, each to its own run.log)
  echo "  Running..."
  for i in $(seq 0 $((n-1))); do
    docker exec "${containers[$i]}" bash /root/catkin_ws/docker/run_avia_exp.sh \
      "${cfgs[$i]}" "${seqs[$i]}" "/root/catkin_ws/dump" "${PORTS[$i]}" "$RATE" \
      > "${OUT_BASE}/${seqs[$i]}/run.log" 2>&1 &
  done
  wait

  # Collect results
  for i in $(seq 0 $((n-1))); do
    local logfile="${OUT_BASE}/${seqs[$i]}/run.log"
    local rmse=$(grep "ATE RMSE" "$logfile" 2>/dev/null | tail -1 | awk '{print $3}')
    echo "  ${seqs[$i]}: ${rmse:-NO_RESULT}m"
  done

  # Cleanup
  for c in "${containers[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}

# Run all 3 batches
run_batch "b1" "${BATCH1_SEQS[0]}" "${BATCH1_CFGS[0]}" "${BATCH1_SEQS[1]}" "${BATCH1_CFGS[1]}" "${BATCH1_SEQS[2]}" "${BATCH1_CFGS[2]}"
run_batch "b2" "${BATCH2_SEQS[0]}" "${BATCH2_CFGS[0]}" "${BATCH2_SEQS[1]}" "${BATCH2_CFGS[1]}" "${BATCH2_SEQS[2]}" "${BATCH2_CFGS[2]}"
run_batch "b3" "${BATCH3_SEQS[0]}" "${BATCH3_CFGS[0]}" "${BATCH3_SEQS[1]}" "${BATCH3_CFGS[1]}" "${BATCH3_SEQS[2]}" "${BATCH3_CFGS[2]}"

# Final summary
echo ""
echo "================================================================"
echo "  FINAL RESULTS"
echo "================================================================"
total=0
count=0
for seq in Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05; do
  logfile="${OUT_BASE}/${seq}/run.log"
  rmse=$(grep "ATE RMSE" "$logfile" 2>/dev/null | tail -1 | awk '{print $3}')
  if [ -n "$rmse" ]; then
    echo "  ${seq}: ${rmse}m"
    total=$(echo "$total + $rmse" | bc -l)
    count=$((count + 1))
  else
    echo "  ${seq}: NO_RESULT"
  fi
done

if [ $count -gt 0 ]; then
  mean=$(echo "scale=4; $total / $count" | bc -l)
  echo ""
  echo "  MEAN (${count}/9): ${mean}m"
  echo "  Target: < 0.300m"
fi
echo "  Completed: $(date)"
echo "================================================================"
