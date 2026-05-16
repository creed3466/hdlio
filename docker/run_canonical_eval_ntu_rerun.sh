#!/bin/bash
# run_canonical_eval_ntu_rerun.sh — Re-run NTU VIRAL canonical evaluation only
#
# Purpose: Re-run NTU with fixed run_ntu_exp.sh (proc/net/tcp cleanup)
# to replace truncated trajectories from the original canonical run.
#
# Uses 1-seq-per-container batching (identical to run_canonical_eval.sh §3).
# Algorithm code (src/tof_slam/) is unchanged from canonical commit f9c305b.
#
# Output: dump/paper_canonical/ntu/{SEQ}/ (overwrites existing truncated results)
set -e
cd "$(dirname "$0")/.."

OUT_ROOT="dump/paper_canonical"
LOG="${OUT_ROOT}/ntu_rerun.log"

NTU_DATA="/home/euntae/Project/dataset/ros1/ntu_viral"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"

RATE="1.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_ntu_1 tofslam_ntu_2 tofslam_ntu_3)
MEM="3g"

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do
    docker rm -f "$c" 2>/dev/null || true
  done
}
trap cleanup EXIT

# Verify algorithm code matches canonical
DIRTY=$(git diff --quiet src/tof_slam/ 2>/dev/null && echo "false" || echo "true")
if [ "${DIRTY}" = "true" ]; then
  echo "ERROR: src/tof_slam/ has uncommitted changes. Cannot re-run canonical."
  echo "The algorithm code must match canonical commit f9c305b."
  exit 1
fi

log ""
log "================================================================"
log "  NTU VIRAL CANONICAL RE-RUN — $(date)"
log "  Git: $(git log --oneline -1)"
log "  Rate: ${RATE} (canonical)"
log "  Fix: run_ntu_exp.sh proc/net/tcp cleanup (replaces fuser)"
log "  Output: ${OUT_ROOT}/ntu/"
log "================================================================"

declare -A SEQ_CFGS
NTU_SEQS=(eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03)
for s in "${NTU_SEQS[@]}"; do
  SEQ_CFGS[$s]="ros1_ntu_viral.yaml"
done

# Backup existing truncated results
BACKUP_DIR="${OUT_ROOT}/ntu_truncated_backup_$(date +%Y%m%d_%H%M%S)"
if [ -d "${OUT_ROOT}/ntu" ]; then
  cp -r "${OUT_ROOT}/ntu" "${BACKUP_DIR}"
  log "  Backed up truncated results to ${BACKUP_DIR}"
fi

# Pre-create output dirs
for s in "${NTU_SEQS[@]}"; do
  mkdir -p "${OUT_ROOT}/ntu/${s}"
done

start_and_build() {
  cleanup
  sleep 1
  for i in 0 1 2; do
    docker run -d --rm --name "${CONTAINERS[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src:ro" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}/ntu:/root/catkin_ws/dump" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      -v "${NTU_DATA}:/root/catkin_ws/data/ntu_viral:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 2
  log "  [Build]..."
  for i in 0 1 2; do
    docker exec "${CONTAINERS[$i]}" bash -c \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1" &
  done
  wait
  for i in 0 1 2; do
    docker exec "${CONTAINERS[$i]}" pip3 install scipy numpy -q 2>/dev/null || true
  done
  log "  [Build] Done."
}

run_batch() {
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}
  [ $n -gt 3 ] && n=3
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${batch_seqs[$i]}"
    local cfg="${SEQ_CFGS[$seq]}"
    local port="${PORTS[$i]}"
    local out="/root/catkin_ws/dump/${seq}"
    log "    [${CONTAINERS[$i]}] ${seq} (${cfg})"
    docker exec "${CONTAINERS[$i]}" bash /root/catkin_ws/docker/run_ntu_exp.sh \
      "${cfg}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then
      log "    OK ${batch_seqs[$i]}"
    else
      log "    FAIL ${batch_seqs[$i]}"
    fi
  done
}

# Run 3 batches of 3 sequences
log ""
log "  Batch 1/3: eee_01, eee_02, eee_03"
start_and_build
run_batch "${NTU_SEQS[@]:0:3}"

log "  Batch 2/3: nya_01, nya_02, nya_03"
start_and_build
run_batch "${NTU_SEQS[@]:3:3}"

log "  Batch 3/3: sbs_01, sbs_02, sbs_03"
start_and_build
run_batch "${NTU_SEQS[@]:6:3}"

# Collect results
log ""
log "  --- NTU VIRAL Re-run Results ---"
TOTAL=0
COUNT=0
for seq in "${NTU_SEQS[@]}"; do
  f="${OUT_ROOT}/ntu/${seq}/ate_result.txt"
  traj="${OUT_ROOT}/ntu/${seq}/traj_est.csv"
  if [ -f "$f" ]; then
    rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
    nlines=$(wc -l < "$traj" 2>/dev/null || echo "0")
    if [ -n "$rmse" ]; then
      printf "  %-12s  ATE=%.4f m  traj=%s lines\n" "$seq" "$rmse" "$nlines" | tee -a "${LOG}"
      TOTAL=$(python3 -c "print(${TOTAL} + ${rmse})")
      COUNT=$((COUNT + 1))
    else
      printf "  %-12s  PARSE_FAIL\n" "$seq" | tee -a "${LOG}"
    fi
  else
    printf "  %-12s  FAIL\n" "$seq" | tee -a "${LOG}"
  fi
done
if [ $COUNT -gt 0 ]; then
  MEAN=$(python3 -c "print(f'{${TOTAL}/${COUNT}:.4f}')")
  log "  Mean (${COUNT}/${#NTU_SEQS[@]}): ${MEAN} m"
fi

cleanup

log ""
log "================================================================"
log "  NTU VIRAL CANONICAL RE-RUN COMPLETE — $(date)"
log "  Results: ${OUT_ROOT}/ntu/"
log "================================================================"
