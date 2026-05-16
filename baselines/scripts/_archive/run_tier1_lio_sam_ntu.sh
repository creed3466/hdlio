#!/usr/bin/env bash
# Phase 2 Tier-1 rollout: LIO-SAM (brytsknguyen fork) on all 9 NTU VIRAL seqs.
#
# Runs 3 containers in parallel.  Each container uses an isolated loopback
# (bridge network, no --network host) so all 3 can bind roscore on 11311
# internally without conflict.  CPU pools (p1=0-3, p2=4-7, p3=8-11) per
# docs/0_docker_container.md.
#
# Output: dump/<LABEL>/lio_sam/ntu/<seq>/{traj.csv,odom.bag,stdout.log,ate.json}
# Aggregate summary appended to dump/<LABEL>/lio_sam_ntu_summary.tsv
#
# Usage:
#   bash baselines/scripts/run_tier1_lio_sam_ntu.sh [LABEL]
#
set -euo pipefail

LABEL="${1:-phase2_tier1_$(date +%Y%m%d)}"
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
RUN_SH="${REPO_ROOT}/baselines/scripts/run_baseline.sh"
COMPUTE="${REPO_ROOT}/baselines/scripts/compute_ate.py"
GT_DIR="${REPO_ROOT}/dump/ntu_gt_tum"
SUMMARY="${REPO_ROOT}/dump/${LABEL}/lio_sam_ntu_summary.tsv"

SEQS=(eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03)
POOLS=(0-3 4-7 8-11)  # CPU pools p1/p2/p3

mkdir -p "${REPO_ROOT}/dump/${LABEL}"
: > "${SUMMARY}"
echo -e "seq\tposes\tATE_RMSE_m\tmean_m\tmax_m\tstatus" >> "${SUMMARY}"

run_one() {
    local seq="$1"
    local pool_idx="$2"
    local cpuset="${POOLS[$pool_idx]}"
    local out_dir="${REPO_ROOT}/dump/${LABEL}/lio_sam/ntu/${seq}"
    local gt="${GT_DIR}/${seq}_gt.tum"

    mkdir -p "${out_dir}"
    echo "[T1] start ${seq} on cpuset=${cpuset}"
    # Dispatch via run_baseline.sh with CPU pool override via env
    BASELINE_CPUSET="${cpuset}" bash "${RUN_SH}" lio_sam ntu "${seq}" "${LABEL}" \
        > "${out_dir}/run.log" 2>&1 || {
            echo "[T1] FAIL ${seq} (run_baseline exit)"
            echo -e "${seq}\t-\t-\t-\t-\tRUN_FAIL" >> "${SUMMARY}"
            return 1; }

    if [[ ! -s "${out_dir}/traj.csv" ]]; then
        echo "[T1] FAIL ${seq} (no traj.csv)"
        echo -e "${seq}\t-\t-\t-\t-\tNO_TRAJ" >> "${SUMMARY}"
        return 1
    fi

    uv run --with numpy --with scipy python "${COMPUTE}" \
        "${out_dir}/traj.csv" "${gt}" --out "${out_dir}/ate.json" \
        > "${out_dir}/ate.log" 2>&1 || {
            echo "[T1] FAIL ${seq} (compute_ate)"
            echo -e "${seq}\t-\t-\t-\t-\tATE_FAIL" >> "${SUMMARY}"
            return 1; }

    # Extract metrics via python
    python3 -c "
import json
d=json.load(open('${out_dir}/ate.json'))
print(f'${seq}\t{d[\"n_poses\"]}\t{d[\"rmse_m\"]:.4f}\t{d[\"mean_m\"]:.4f}\t{d[\"max_m\"]:.4f}\tPASS')
" >> "${SUMMARY}"
    echo "[T1] done ${seq}"
}

export -f run_one
export REPO_ROOT RUN_SH COMPUTE GT_DIR SUMMARY LABEL
export POOLS

# Dispatch with up to 3 concurrent jobs, round-robin pool assignment.
i=0
for seq in "${SEQS[@]}"; do
    pool=$(( i % 3 ))
    run_one "${seq}" "${pool}" &
    pids[$i]=$!
    i=$(( i + 1 ))
    # Wait if we have 3 in flight
    if (( i % 3 == 0 )); then
        for p in "${pids[@]}"; do wait "${p}" || true; done
        pids=()
    fi
done
# Drain remainder
if (( ${#pids[@]} > 0 )); then
    for p in "${pids[@]}"; do wait "${p}" || true; done
fi

echo ""
echo "=== Tier-1 NTU LIO-SAM summary ==="
cat "${SUMMARY}"
