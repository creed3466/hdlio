#!/bin/bash
# run_dlio_all.sh — Run DLIO baseline on all 35 sequences (3-way parallelism).
#
# Output: dump/<LABEL>/dlio/{avia,mid360,indoor,ntu}/{SEQ}/
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-baselines_dlio}"
REPO_ROOT="$(pwd)"
BASELINE_SH="${REPO_ROOT}/baselines/scripts/run_baseline.sh"

# All tasks: dataset:seq
TASKS=(
  avia:Dark01 avia:Dark02 avia:Dynamic03 avia:Dynamic04
  avia:Occlusion03 avia:Occlusion04
  avia:Varying-illu03 avia:Varying-illu04 avia:Varying-illu05
  mid360:Dark01 mid360:Dark02 mid360:Dynamic03 mid360:Dynamic04
  mid360:Occlusion03 mid360:Occlusion04
  mid360:Varying-illu03 mid360:Varying-illu04 mid360:Varying-illu05
  indoor:Dark03 indoor:Dark04 indoor:Dynamic01 indoor:Dynamic02
  indoor:Occlusion01 indoor:Occlusion02
  indoor:Varying-illu01 indoor:Varying-illu02
  ntu:eee_01 ntu:eee_02 ntu:eee_03
  ntu:nya_01 ntu:nya_02 ntu:nya_03
  ntu:sbs_01 ntu:sbs_02 ntu:sbs_03
)

CPUSETS=("0-3" "4-7" "8-11")
LOG="${REPO_ROOT}/dump/${LABEL}/dlio_all.log"
mkdir -p "${REPO_ROOT}/dump/${LABEL}"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }

log "DLIO evaluation: ${#TASKS[@]} sequences, label=${LABEL}"

run_one() {
    local slot=$1 dataset=$2 seq=$3
    local out="${REPO_ROOT}/dump/${LABEL}/dlio/${dataset}/${seq}"
    if [[ -f "${out}/traj.csv" ]] && [[ -s "${out}/traj.csv" ]]; then
        log "  SKIP ${dataset}/${seq}"
        return 0
    fi
    BASELINE_CPUSET="${CPUSETS[$slot]}" bash "${BASELINE_SH}" dlio "${dataset}" "${seq}" "${LABEL}" \
        >> "${LOG}" 2>&1 \
        && log "  OK ${dataset}/${seq}" \
        || log "  FAIL ${dataset}/${seq}"
}

# Process in batches of 3
idx=0
total=${#TASKS[@]}
while [[ ${idx} -lt ${total} ]]; do
    pids=()
    for slot in 0 1 2; do
        pos=$((idx + slot))
        if [[ ${pos} -ge ${total} ]]; then break; fi
        task="${TASKS[$pos]}"
        dataset="${task%%:*}"
        seq="${task##*:}"
        log "  [slot ${slot}] START ${dataset}/${seq}"
        run_one ${slot} "${dataset}" "${seq}" &
        pids+=($!)
    done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
    idx=$((idx + 3))
done

log "All DLIO runs complete."

# Collect ATE results
log ""
log "=== ATE Results ==="
for task in "${TASKS[@]}"; do
    dataset="${task%%:*}"
    seq="${task##*:}"
    traj="${REPO_ROOT}/dump/${LABEL}/dlio/${dataset}/${seq}/traj.csv"

    case "${dataset}" in
        avia|mid360) gt="/home/euntae/Project/dataset/ros1/surfel_data/ground_truth/${seq}.txt" ;;
        indoor)      gt="/home/euntae/Project/dataset/ros1/indoor/${seq}.txt" ;;
        ntu)         gt="/home/euntae/Project/dataset/ros1/ntu_viral/${seq}/ground_truth.csv" ;;
    esac

    if [[ -f "${traj}" ]] && [[ -s "${traj}" ]]; then
        ntu_flag=""
        [[ "${dataset}" == "ntu" ]] && ntu_flag="--ntu-viral"
        rmse=$(/tmp/figenv/bin/python "${REPO_ROOT}/baselines/scripts/compute_ate.py" \
            "${traj}" "${gt}" ${ntu_flag} 2>/dev/null \
            | python3 -c "import sys,json; print(f'{json.load(sys.stdin)[\"rmse_m\"]:.4f}')" 2>/dev/null) \
            || rmse="PARSE_FAIL"
        printf "  %-8s %-20s %s\n" "${dataset}" "${seq}" "${rmse}" | tee -a "${LOG}"
    else
        printf "  %-8s %-20s %s\n" "${dataset}" "${seq}" "NO_TRAJ" | tee -a "${LOG}"
    fi
done
