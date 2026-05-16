#!/bin/bash
# run_indoor_baselines.sh — Run FAST-LIO2, iG-LIO, Point-LIO on M3DGR Indoor
# Both Avia and Mid360 sensors, 8 sequences each = 48 runs total
# Runs 3 in parallel (matching CPU pinning: 0-3, 4-7, 8-11)
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-indoor_baselines}"
ALGOS=(fast_lio2 ig_lio point_lio)
DATASETS=(avia mid360)
INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
CPUSETS=("0-3" "4-7" "8-11")

LOG="dump/${LABEL}/eval.log"
mkdir -p "dump/${LABEL}"

log() { echo "$*" | tee -a "${LOG}"; }

COMMIT_MSG=$(git log --oneline -1 2>/dev/null || echo "unknown")

log "================================================================"
log "  INDOOR BASELINES: FLIO2 + iG-LIO + P-LIO × Avia + Mid360"
log "  $(date)"
log "  Git: ${COMMIT_MSG}"
log "  Sequences: ${#INDOOR_SEQS[@]} per sensor"
log "================================================================"

GT_DIR="/home/euntae/Project/dataset/ros1/surfel_data/ground_truth"
EVAL_SCRIPT="docker/eval_ate_m3dgr.py"

run_parallel_3() {
  local algo=$1 dataset=$2
  shift 2
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
        bash baselines/scripts/run_baseline.sh "${algo}" "${dataset}" "${seq}" "${LABEL}" \
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

    # Post-run ATE evaluation for each completed sequence
    for seq in "${batch_seqs[@]}"; do
      local out_dir="dump/${LABEL}/${algo}/${dataset}/${seq}"
      local traj="${out_dir}/traj.csv"
      local gt="${GT_DIR}/${seq}.txt"
      if [ -f "$traj" ] && [ -f "$gt" ]; then
        python3 "${EVAL_SCRIPT}" "$traj" "$gt" --output_dir "${out_dir}/" >> "${LOG}" 2>&1 \
          && log "    ATE: ${seq} OK" \
          || log "    ATE: ${seq} EVAL_FAILED"
      fi
    done
  done
}

collect_algo_results() {
  local algo=$1 dataset=$2
  shift 2
  local seqs=("$@")
  local total=0 count=0
  log ""
  log "  --- ${algo} / ${dataset} ---"
  for seq in "${seqs[@]}"; do
    # ATE result from baseline runner
    local f="dump/${LABEL}/${algo}/${dataset}/${seq}/ate_result.txt"
    if [ ! -f "$f" ]; then
      # Try alternate location
      f="dump/${LABEL}/${algo}/${dataset}/${seq}/ate.json"
    fi
    if [ -f "$f" ]; then
      local rmse=""
      if grep -q "^rmse:" "$f" 2>/dev/null; then
        rmse=$(grep "^rmse:" "$f" | awk '{print $2}')
      elif grep -q '"rmse"' "$f" 2>/dev/null; then
        rmse=$(python3 -c "import json; print(json.load(open('$f'))['rmse'])" 2>/dev/null)
      fi
      if [ -n "$rmse" ]; then
        printf "  %-25s %.6f m\n" "$seq" "$rmse" | tee -a "${LOG}"
        total=$(python3 -c "print(${total} + ${rmse})")
        count=$((count + 1))
      else
        printf "  %-25s PARSE_FAIL\n" "$seq" | tee -a "${LOG}"
      fi
    else
      printf "  %-25s MISSING\n" "$seq" | tee -a "${LOG}"
    fi
  done
  if [ $count -gt 0 ]; then
    local mean=$(python3 -c "print(f'{${total}/${count}:.6f}')")
    log "  Mean (${count}/${#seqs[@]}): ${mean} m"
  fi
}

# Run each algo × each sensor
for algo in "${ALGOS[@]}"; do
  for dataset in "${DATASETS[@]}"; do
    log ""
    log "======== ${algo} / ${dataset} (${#INDOOR_SEQS[@]} seq) ========"
    run_parallel_3 "${algo}" "${dataset}" "${INDOOR_SEQS[@]}"
    collect_algo_results "${algo}" "${dataset}" "${INDOOR_SEQS[@]}"
  done
done

log ""
log "======== ALL DONE ========"
log "  Results at: dump/${LABEL}/"
