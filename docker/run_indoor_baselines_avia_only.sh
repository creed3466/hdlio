#!/bin/bash
# run_indoor_baselines_avia_only.sh — Run FAST-LIO2, iG-LIO, Point-LIO on Indoor Avia ONLY
# 3 algos × 8 seqs = 24 runs, 3 in parallel
set -euo pipefail
cd "$(dirname "$0")/.."

LABEL="${1:-indoor_baselines_avia_$(date +%Y%m%d_%H%M)}"
ALGOS=(fast_lio2 ig_lio point_lio)
INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
CPUSETS=("0-3" "4-7" "8-11")

LOG="dump/${LABEL}/eval.log"
mkdir -p "dump/${LABEL}"

log() { echo "$*" | tee -a "${LOG}"; }

COMMIT_MSG=$(git log --oneline -1 2>/dev/null || echo "unknown")

log "================================================================"
log "  INDOOR BASELINES (Avia only): FLIO2 + iG-LIO + P-LIO"
log "  $(date)"
log "  Git: ${COMMIT_MSG}"
log "  Label: ${LABEL}"
log "  Sequences: ${#INDOOR_SEQS[@]}"
log "================================================================"

GT_DIR="/home/euntae/Project/dataset/ros1/surfel_data/ground_truth"
EVAL_SCRIPT="docker/eval_ate_m3dgr.py"

run_parallel_3() {
  local algo=$1
  shift
  local seqs=("$@")
  local n=${#seqs[@]}
  local batch=0

  for ((start=0; start<n; start+=3)); do
    batch=$((batch+1))
    local end=$((start+3))
    [ $end -gt $n ] && end=$n
    local batch_seqs=("${seqs[@]:start:$((end-start))}")

    log "  Batch ${batch}: ${batch_seqs[*]}"
    local PIDS=()
    for i in "${!batch_seqs[@]}"; do
      local seq="${batch_seqs[$i]}"
      BASELINE_CPUSET="${CPUSETS[$i]}" \
        bash baselines/scripts/run_baseline.sh "${algo}" "avia" "${seq}" "${LABEL}" \
        >> "${LOG}" 2>&1 &
      PIDS+=($!)
    done

    for i in "${!PIDS[@]}"; do
      if wait "${PIDS[$i]}" 2>/dev/null; then
        log "    done ${batch_seqs[$i]}"
      else
        log "    FAILED ${batch_seqs[$i]}"
      fi
    done

    # ATE evaluation (best-effort, numpy may not be available on host)
    for seq in "${batch_seqs[@]}"; do
      local out_dir="dump/${LABEL}/${algo}/avia/${seq}"
      local traj="${out_dir}/traj.csv"
      local gt="${GT_DIR}/${seq}.txt"
      if [ -f "$traj" ] && [ -f "$gt" ]; then
        /tmp/figenv/bin/python "${EVAL_SCRIPT}" "$traj" "$gt" --output_dir "${out_dir}/" >> "${LOG}" 2>&1 \
          && log "    ATE: ${seq} OK" \
          || log "    ATE: ${seq} EVAL_SKIPPED"
      fi
    done
  done
}

collect_results() {
  local algo=$1
  shift
  local seqs=("$@")
  local total=0 count=0
  log ""
  log "  --- ${algo} / avia ---"
  for seq in "${seqs[@]}"; do
    local f="dump/${LABEL}/${algo}/avia/${seq}/ate_result.txt"
    if [ -f "$f" ]; then
      local rmse
      rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}' || true)
      if [ -n "$rmse" ]; then
        printf "  %-28s %s m\n" "$seq" "$rmse" | tee -a "${LOG}"
        total=$(python3 -c "print(${total} + ${rmse})")
        count=$((count + 1))
      else
        printf "  %-28s PARSE_FAIL\n" "$seq" | tee -a "${LOG}"
      fi
    else
      printf "  %-28s MISSING\n" "$seq" | tee -a "${LOG}"
    fi
  done
  if [ $count -gt 0 ]; then
    local mean=$(python3 -c "print(f'{${total}/${count}:.4f}')")
    log "  Mean (${count}/${#seqs[@]}): ${mean} m"
  fi
}

for algo in "${ALGOS[@]}"; do
  log ""
  log "======== ${algo} / avia (${#INDOOR_SEQS[@]} seq) ========"
  run_parallel_3 "${algo}" "${INDOOR_SEQS[@]}"
  collect_results "${algo}" "${INDOOR_SEQS[@]}"
done

log ""
log "======== ALL DONE ========"
log "  $(date)"
log "  Results: dump/${LABEL}/"
