#!/usr/bin/env bash
# Phase 2 Tier-1 rollout: parameterized by <algo> <dataset>.
#
# Generalized form of run_tier1_lio_sam_ntu.sh — dispatches all sequences
# of a given dataset on a single algorithm with 3-way container concurrency
# and the canonical CPU pool layout (p1=0-3, p2=4-7, p3=8-11) per
# docs/0_docker_container.md.
#
# Determinism: rosbag play -r 1.0 (CLAUDE.md §6-3 mandatory for validation).
#
# Output:
#   dump/<LABEL>/<algo>/<dataset>/<seq>/{traj.csv,odom.bag,stdout.log,ate.json}
# Aggregate summary:
#   dump/<LABEL>/<algo>_<dataset>_summary.tsv
#
# Usage:
#   bash baselines/scripts/run_tier1_algo_dataset.sh <algo> <dataset> [LABEL]
#
# Examples:
#   bash baselines/scripts/run_tier1_algo_dataset.sh fast_lio2 avia phase2_tier1_20260413
#   bash baselines/scripts/run_tier1_algo_dataset.sh point_lio avia phase2_tier1_20260413
#   bash baselines/scripts/run_tier1_algo_dataset.sh ig_lio    avia phase2_tier1_20260413
#
set -euo pipefail

ALGO="${1:?algo required (fast_lio2|point_lio|ig_lio|lio_sam)}"
DATASET="${2:?dataset required (avia|mid360|ntu)}"
LABEL="${3:-phase2_tier1_$(date +%Y%m%d)}"

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
RUN_SH="${REPO_ROOT}/baselines/scripts/run_baseline.sh"
COMPUTE="${REPO_ROOT}/baselines/scripts/compute_ate.py"
SUMMARY="${REPO_ROOT}/dump/${LABEL}/${ALGO}_${DATASET}_summary.tsv"

# Sequence sets (canonical Tier-1 per phase2_plan.md, intersected with
# what actually exists on disk in the dataset roots).
case "${DATASET}" in
    avia|mid360)
        # Size-packed ordering so heavy seqs cluster in wave 1, finishing later
        # waves quickly. Bag sizes (GB): V03=14, V04=9.3, Dk02=7.6, V05=7.2,
        # Oc04=5.3, Dy04=4.4, Oc03=4.0, Dy03=3.2, Dk01=2.3. 3-way wave groups
        # are [0,1,2]/[3,4,5]/[6,7,8]. Wall-clock gate = sum(max per wave).
        SEQS=(Varying-illu03 Varying-illu04 Dark02 Varying-illu05 Occlusion04 Dynamic04 Occlusion03 Dynamic03 Dark01)
        ;;
    ntu)
        SEQS=(eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03)
        ;;
    *)
        echo "[ERROR] Unknown dataset: ${DATASET}"; exit 1 ;;
esac

# GT path resolver (per-seq) — must mirror paper_figures/alignment.py loaders.
gt_path_for() {
    local seq="$1"
    case "${DATASET}" in
        avia|mid360)
            echo "/home/euntae/Project/dataset/ros1/surfel_data/ground_truth/${seq}.txt" ;;
        ntu)
            echo "${REPO_ROOT}/dump/ntu_gt_tum/${seq}_gt.tum" ;;
    esac
}

POOLS=(0-3 4-7 8-11)  # CPU pools p1/p2/p3

mkdir -p "${REPO_ROOT}/dump/${LABEL}"
: > "${SUMMARY}"
echo -e "seq\tposes\tATE_RMSE_m\tmean_m\tmax_m\tstatus" >> "${SUMMARY}"

run_one() {
    local seq="$1"
    local pool_idx="$2"
    local cpuset="${POOLS[$pool_idx]}"
    local out_dir="${REPO_ROOT}/dump/${LABEL}/${ALGO}/${DATASET}/${seq}"
    local gt
    gt="$(gt_path_for "${seq}")"

    mkdir -p "${out_dir}"
    if [[ ! -f "${gt}" ]]; then
        echo "[T1] SKIP ${seq} (GT missing: ${gt})"
        echo -e "${seq}\t-\t-\t-\t-\tNO_GT" >> "${SUMMARY}"
        return 0
    fi

    echo "[T1] start ${ALGO}/${DATASET}/${seq} on cpuset=${cpuset}"
    BASELINE_CPUSET="${cpuset}" bash "${RUN_SH}" "${ALGO}" "${DATASET}" "${seq}" "${LABEL}" \
        > "${out_dir}/run.log" 2>&1 || {
            echo "[T1] FAIL ${seq} (run_baseline exit)"
            echo -e "${seq}\t-\t-\t-\t-\tRUN_FAIL" >> "${SUMMARY}"
            return 1; }

    if [[ ! -s "${out_dir}/traj.csv" ]]; then
        echo "[T1] FAIL ${seq} (no traj.csv)"
        echo -e "${seq}\t-\t-\t-\t-\tNO_TRAJ" >> "${SUMMARY}"
        return 1
    fi

    uv run --quiet --with numpy --with scipy python "${COMPUTE}" \
        "${out_dir}/traj.csv" "${gt}" --out "${out_dir}/ate.json" \
        > "${out_dir}/ate.log" 2>&1 || {
            echo "[T1] FAIL ${seq} (compute_ate)"
            echo -e "${seq}\t-\t-\t-\t-\tATE_FAIL" >> "${SUMMARY}"
            return 1; }

    python3 -c "
import json
d=json.load(open('${out_dir}/ate.json'))
print(f'${seq}\t{d[\"n_poses\"]}\t{d[\"rmse_m\"]:.4f}\t{d[\"mean_m\"]:.4f}\t{d[\"max_m\"]:.4f}\tPASS')
" >> "${SUMMARY}"
    echo "[T1] done ${seq}"
}

export -f run_one gt_path_for
export REPO_ROOT RUN_SH COMPUTE SUMMARY LABEL ALGO DATASET
export POOLS

# Dispatch with up to 3 concurrent jobs, round-robin pool assignment.
pids=()
i=0
for seq in "${SEQS[@]}"; do
    pool=$(( i % 3 ))
    run_one "${seq}" "${pool}" &
    pids[$i]=$!
    i=$(( i + 1 ))
    if (( i % 3 == 0 )); then
        for p in "${pids[@]}"; do wait "${p}" || true; done
        pids=()
    fi
done
if (( ${#pids[@]} > 0 )); then
    for p in "${pids[@]}"; do wait "${p}" || true; done
fi

echo ""
echo "=== Tier-1 ${ALGO}/${DATASET} summary ==="
cat "${SUMMARY}"
