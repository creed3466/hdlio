#!/bin/bash
# run_ddpo_screening.sh — Quick DDPO+DARBF activation screening
# Runs Dk01, Dy03, VI03 in parallel (3 containers, rate=3.0)
# to check for catastrophic regression from enabling DDPO+DARBF.
#
# Baseline comparison (canonical rate=1.0):
#   Dk01: 0.118m, Dy03: 0.170m, VI03: 0.608m
#
# Usage: bash docker/run_ddpo_screening.sh

set -e
cd "$(dirname "$0")/.."

RATE="3.0"
IMAGE="tofslam:ros1"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
OUT_BASE="dump/ddpo_screening"

SEQS=(Dark01 Dynamic03 Varying-illu03)
CONFIGS=(avia_v6_seq/dark01.yaml avia_v6_seq/dynamic03.yaml avia_v6_seq/varying_illu03.yaml)
CANONICAL=(0.118 0.170 0.608)

CONTAINER_NAMES=(tofslam_scr_1 tofslam_scr_2 tofslam_scr_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
MEM="3g"

echo "========================================="
echo "  DDPO+DARBF Screening (rate=${RATE})"
echo "  Seqs: ${SEQS[*]}"
echo "  Started: $(date)"
echo "========================================="

# Cleanup
cleanup() {
  for c in "${CONTAINER_NAMES[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
  for p in "${PORTS[@]}"; do fuser -k ${p}/tcp 2>/dev/null || true; done
}
trap cleanup EXIT
cleanup
sleep 3

# Start 3 containers
for i in 0 1 2; do
  mkdir -p "${OUT_BASE}/${SEQS[$i]}"
  docker run -d --rm --name "${CONTAINER_NAMES[$i]}" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_BASE}/${SEQS[$i]}:/root/catkin_ws/dump" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  echo "  Container ${CONTAINER_NAMES[$i]} started (CPU=${CPUSETS[$i]}, port=${PORTS[$i]})"
done
sleep 3

# Build in all containers (parallel)
echo "  [Build] ..."
for i in 0 1 2; do
  docker exec "${CONTAINER_NAMES[$i]}" bash -c \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1" &
done
wait
for i in 0 1 2; do
  docker exec "${CONTAINER_NAMES[$i]}" pip3 install scipy numpy -q 2>/dev/null || true
done
echo "  [Build] Done."

# Run all 3 in parallel
echo ""
echo "  Running experiments..."
PIDS=()
for i in 0 1 2; do
  (
    echo "  [${SEQS[$i]}] Starting..."
    docker exec "${CONTAINER_NAMES[$i]}" bash /root/catkin_ws/docker/run_avia_exp.sh \
      "${CONFIGS[$i]}" "${SEQS[$i]}" "/root/catkin_ws/dump" "${PORTS[$i]}" "${RATE}"
    echo "  [${SEQS[$i]}] Done."
  ) &
  PIDS+=($!)
done

for pid in "${PIDS[@]}"; do wait "$pid"; done

# Collect results
echo ""
echo "========================================="
echo "  DDPO+DARBF Screening Results"
echo "========================================="
printf "  %-18s | %-10s | %-10s | %-8s\n" "Sequence" "DDPO+DARBF" "Canonical" "Delta%"
printf "  %s\n" "$(printf '%0.s-' {1..55})"

for i in 0 1 2; do
  f="${OUT_BASE}/${SEQS[$i]}/ate_result.txt"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
    if [ -n "$rmse" ]; then
      delta=$(python3 -c "can=${CANONICAL[$i]}; new=float('${rmse}'); print(f'{(new-can)/can*100:+.1f}')" 2>/dev/null || echo "?")
      printf "  %-18s | %-10s | %-10s | %-8s\n" "${SEQS[$i]}" "$rmse" "${CANONICAL[$i]}" "${delta}%"
    else
      printf "  %-18s | %-10s | %-10s | %-8s\n" "${SEQS[$i]}" "PARSE_ERR" "${CANONICAL[$i]}" "?"
    fi
  else
    printf "  %-18s | %-10s | %-10s | %-8s\n" "${SEQS[$i]}" "FAIL" "${CANONICAL[$i]}" "?"
  fi
done

echo ""
echo "  Completed: $(date)"
echo "========================================="
