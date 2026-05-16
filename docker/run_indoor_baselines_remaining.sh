#!/bin/bash
# Run remaining indoor avia baselines that didn't complete
set -uo pipefail
cd "$(dirname "$0")/.."

LABEL="indoor_baselines_avia_20260430_1832"
CPUSETS=("0-3" "4-7" "8-11")
INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
LOG="dump/${LABEL}/eval_remaining.log"

log() { echo "$*" | tee -a "${LOG}"; }

log "================================================================"
log "  REMAINING BASELINE RUNS: $(date)"
log "================================================================"

run_batch() {
  local algo=$1
  shift
  local seqs=("$@")
  local n=${#seqs[@]}

  for ((start=0; start<n; start+=3)); do
    local end=$((start+3))
    [ $end -gt $n ] && end=$n
    local batch_seqs=("${seqs[@]:start:$((end-start))}")
    log "  Batch: ${algo} ${batch_seqs[*]}"
    local PIDS=()
    for i in "${!batch_seqs[@]}"; do
      local seq="${batch_seqs[$i]}"
      local out_dir="dump/${LABEL}/${algo}/avia/${seq}"
      # Skip if already completed
      if [ -f "$out_dir/traj.csv" ]; then
        log "    SKIP ${seq} (already done)"
        continue
      fi
      BASELINE_CPUSET="${CPUSETS[$i]}" \
        bash baselines/scripts/run_baseline.sh "${algo}" "avia" "${seq}" "${LABEL}" \
        >> "${LOG}" 2>&1 &
      PIDS+=($!)
    done
    for pid in "${PIDS[@]}"; do
      wait "$pid" 2>/dev/null && true
    done
    log "  Batch done"
  done
}

# FAST-LIO2: only VI01 and VI02 missing
log ""
log "=== FAST-LIO2 remaining ==="
run_batch fast_lio2 indoor_Varying-illu01 indoor_Varying-illu02

# iG-LIO: all 8 missing
log ""
log "=== iG-LIO all ==="
run_batch ig_lio "${INDOOR_SEQS[@]}"

# Point-LIO: all 8 missing
log ""
log "=== Point-LIO all ==="
run_batch point_lio "${INDOOR_SEQS[@]}"

log ""
log "=== ALL REMAINING DONE ==="
log "$(date)"

# Summary
log ""
log "=== TRAJECTORY COUNT ==="
for algo in fast_lio2 ig_lio point_lio; do
  count=$(find "dump/${LABEL}/${algo}/avia/" -name "traj.csv" 2>/dev/null | wc -l)
  log "  ${algo}: ${count}/8"
done
