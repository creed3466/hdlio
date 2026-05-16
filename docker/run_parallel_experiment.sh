#!/bin/bash
# run_parallel_experiment.sh — 병렬 실험 실행기
# 최대 3개 컨테이너(p1/p2/p3)를 동시에 돌려 config들을 병렬 처리한다.
# 각 컨테이너는 CPU pinning + ROS port 분리로 격리.
#
# Usage:
#   bash docker/run_parallel_experiment.sh <ROUND_NAME> <CONFIG1:LABEL1> [CONFIG2:LABEL2] [CONFIG3:LABEL3] ...
#
# Example:
#   bash docker/run_parallel_experiment.sh r4 \
#     exp_avia_pvfine_det.yaml:pvfine_det \
#     exp_avia_ossc_stable.yaml:ossc_stable \
#     exp_avia_dark02_specialist.yaml:dark02_spec
#
# - 3개 이하 config: 모두 동시 실행
# - 4개 이상 config: 3개씩 배치(batch)로 나눠 실행
# - VI 시퀀스(Varying-illu*)는 N_RUNS회 반복, 나머지는 1회

set -e
cd "$(dirname "$0")/.."

ROUND="${1:?Usage: run_parallel_experiment.sh <ROUND_NAME> <CONFIG:LABEL> ...}"
shift

# Parse config:label pairs
ALL_CONFIGS=()
ALL_LABELS=()
for pair in "$@"; do
  IFS=':' read -r cfg lbl <<< "$pair"
  ALL_CONFIGS+=("$cfg")
  ALL_LABELS+=("$lbl")
done

if [ ${#ALL_CONFIGS[@]} -eq 0 ]; then
  echo "ERROR: No configs specified"
  echo "Usage: run_parallel_experiment.sh <ROUND_NAME> <CONFIG:LABEL> ..."
  exit 1
fi

# Settings
OUT_BASE="dump/exp_avia"
SEQS=(Dark01 Dark02 Dynamic03 Varying-illu03 Varying-illu04)
MULTI_RUN_SEQS=(Varying-illu03 Varying-illu04)
N_RUNS=5
MAX_PARALLEL=3

# Container specs (CPU pinning + port isolation)
CONTAINER_NAMES=(tofslam_exp_p1 tofslam_exp_p2 tofslam_exp_p3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
MEM="3g"

echo "========================================="
echo "  Parallel Experiment: ${ROUND}"
echo "  Total configs: ${#ALL_CONFIGS[@]}"
echo "  Max parallel: ${MAX_PARALLEL}"
echo "  Seqs: ${SEQS[*]}"
echo "  Multi-run (${N_RUNS}x): ${MULTI_RUN_SEQS[*]}"
echo "  Started: $(date)"
echo "========================================="

# Cleanup any existing experiment containers
for name in "${CONTAINER_NAMES[@]}"; do
  docker rm -f "$name" 2>/dev/null || true
done

# ----- Functions -----

start_container() {
  local slot=$1  # 0, 1, 2
  local label=$2
  local name="${CONTAINER_NAMES[$slot]}"
  local cpuset="${CPUSETS[$slot]}"

  mkdir -p "${OUT_BASE}/${label}"
  for SEQ in "${SEQS[@]}"; do
    mkdir -p "${OUT_BASE}/${label}/${SEQ}"
  done

  docker run -d --rm --name "$name" \
    --network host --cpuset-cpus "$cpuset" --memory "$MEM" \
    --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_BASE}/${label}:/root/catkin_ws/dump" \
    -v "/home/euntae/Project/dataset/ros1/surfel_data:/root/catkin_ws/data/m3dgr_surfel:ro" \
    tofslam:ros1 bash -lc "sleep infinity" > /dev/null

  sleep 2

  # Build inside container
  docker exec "$name" bash -c \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3"
  docker exec "$name" pip3 install scipy numpy -q 2>/dev/null || true
}

run_all_seqs() {
  local slot=$1
  local config=$2
  local label=$3
  local name="${CONTAINER_NAMES[$slot]}"
  local port="${PORTS[$slot]}"

  for SEQ in "${SEQS[@]}"; do
    IS_MULTI=false
    for MS in "${MULTI_RUN_SEQS[@]}"; do
      [ "$SEQ" = "$MS" ] && IS_MULTI=true
    done

    if $IS_MULTI; then
      echo "[${label}] ${SEQ} (${N_RUNS} runs)..."
      for run in $(seq 1 $N_RUNS); do
        mkdir -p "${OUT_BASE}/${label}/${SEQ}/run${run}"
        docker exec "$name" bash /root/catkin_ws/docker/run_avia_exp.sh \
          "${config}" "${SEQ}" "/root/catkin_ws/dump/${SEQ}/run${run}" "$port"
      done
    else
      echo "[${label}] ${SEQ}..."
      docker exec "$name" bash /root/catkin_ws/docker/run_avia_exp.sh \
        "${config}" "${SEQ}" "/root/catkin_ws/dump/${SEQ}" "$port"
    fi
  done

  docker rm -f "$name" 2>/dev/null || true
  echo "[${label}] ✅ DONE"
}

# ----- Batch execution -----

total=${#ALL_CONFIGS[@]}
batch_idx=0

while [ $batch_idx -lt $total ]; do
  batch_end=$((batch_idx + MAX_PARALLEL))
  [ $batch_end -gt $total ] && batch_end=$total
  batch_size=$((batch_end - batch_idx))

  echo ""
  echo "========================================="
  echo "  Batch $((batch_idx / MAX_PARALLEL + 1)): configs $((batch_idx+1))-${batch_end} of ${total}"
  echo "========================================="

  # Start containers for this batch
  for slot in $(seq 0 $((batch_size - 1))); do
    cidx=$((batch_idx + slot))
    echo "Starting container slot ${slot} for ${ALL_LABELS[$cidx]}..."
    start_container "$slot" "${ALL_LABELS[$cidx]}"
  done

  # Run all sequences in parallel (background)
  PIDS=()
  for slot in $(seq 0 $((batch_size - 1))); do
    cidx=$((batch_idx + slot))
    run_all_seqs "$slot" "${ALL_CONFIGS[$cidx]}" "${ALL_LABELS[$cidx]}" &
    PIDS+=($!)
  done

  # Wait for all in this batch
  for pid in "${PIDS[@]}"; do
    wait "$pid"
  done

  batch_idx=$batch_end
done

# ----- Results -----

echo ""
echo "========================================="
echo "  ${ROUND} RESULTS"
echo "  Completed: $(date)"
echo "========================================="

declare -A SOTA
SOTA[Dark01]=0.118; SOTA[Dark02]=0.645; SOTA[Dynamic03]=0.151
# VI04 uses VI05 within-repo (0.199) as comparator; 0.102 was paper phantom (FU-4).
SOTA[Varying-illu03]=0.897; SOTA[Varying-illu04]=0.199

get_ate() {
  local f="$1"
  if [ -f "$f" ]; then grep "rmse" "$f" | awk -F: '{printf "%.4f", $2}'; else echo "FAIL"; fi
}

get_median() {
  local base="$1"; local n="$2"
  local vals=()
  for run in $(seq 1 $n); do
    local f="${base}/run${run}/ate_result.txt"
    [ -f "$f" ] && vals+=($(grep "rmse" "$f" | awk -F: '{printf "%.4f", $2}'))
  done
  [ ${#vals[@]} -eq 0 ] && { echo "FAIL"; return; }
  IFS=$'\n' sorted=($(sort -n <<<"${vals[*]}")); unset IFS
  echo "${sorted[$((${#sorted[@]} / 2))]}"
}

printf "\n%-16s" "Sequence"
for LABEL in "${ALL_LABELS[@]}"; do printf " | %-16s" "${LABEL}"; done
printf " | %-8s\n" "SOTA"
printf "%s\n" "$(printf '%0.s-' {1..120})"

for SEQ in "${SEQS[@]}"; do
  printf "%-16s" "${SEQ}"
  IS_MULTI=false
  for MS in "${MULTI_RUN_SEQS[@]}"; do
    [ "$SEQ" = "$MS" ] && IS_MULTI=true
  done
  for LABEL in "${ALL_LABELS[@]}"; do
    if $IS_MULTI; then
      ATE=$(get_median "${OUT_BASE}/${LABEL}/${SEQ}" $N_RUNS)
      printf " | %-16s" "${ATE}(med)"
    else
      ATE=$(get_ate "${OUT_BASE}/${LABEL}/${SEQ}/ate_result.txt")
      printf " | %-16s" "${ATE}"
    fi
  done
  printf " | %-8s\n" "${SOTA[$SEQ]}"
done

echo ""
echo "Multi-run detail (VI sequences):"
for LABEL in "${ALL_LABELS[@]}"; do
  echo "  ${LABEL}:"
  for SEQ in "${MULTI_RUN_SEQS[@]}"; do
    printf "    %-18s: " "${SEQ}"
    for run in $(seq 1 $N_RUNS); do
      f="${OUT_BASE}/${LABEL}/${SEQ}/run${run}/ate_result.txt"
      [ -f "$f" ] && printf "r%d=%.3f " "$run" "$(grep rmse $f | awk -F: '{print $2}')" || printf "r%d=FAIL " "$run"
    done
    echo ""
  done
done

echo ""
echo "========================================="
echo "  ${ROUND} COMPLETE"
echo "========================================="
