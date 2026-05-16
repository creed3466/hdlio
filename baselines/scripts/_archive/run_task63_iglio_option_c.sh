#!/usr/bin/env bash
# Task #63 — iG-LIO Option C (motion-aware static init) eval runner.
#
# Dispatches 9 NTU VIRAL + 2 Avia Dynamic sequences for the iG-LIO image
# (already rebuilt with Option C). 3-way concurrent containers; CPU pools
# p1=0-3, p2=4-7, p3=8-11 per docs/0_docker_container.md. Determinism:
# rosbag play -r 1.0 (CLAUDE.md §6-3).
#
# Usage:
#   bash baselines/scripts/run_task63_iglio_option_c.sh [LABEL]
#
# Output:
#   dump/<LABEL>/ig_lio/{ntu,avia}/<seq>/{traj.csv,stdout.log,ate.json,run.log}
#   dump/<LABEL>/ig_lio_ntu_summary.tsv
#   dump/<LABEL>/ig_lio_avia_summary.tsv
#
# Wall-clock estimate: ~35 min NTU (9 seqs, 3×3) + ~12 min Avia (2 seqs, 1 wave).

set -euo pipefail

LABEL="${1:-task63_iglio_option_c_$(date +%Y%m%d_%H%M)}"

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
RUN_SH="${REPO_ROOT}/baselines/scripts/run_baseline.sh"
COMPUTE="${REPO_ROOT}/baselines/scripts/compute_ate.py"

NTU_SEQS=(eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03)
AVIA_SEQS=(Dynamic03 Dynamic04)

POOLS=(0-3 4-7 8-11)

mkdir -p "${REPO_ROOT}/dump/${LABEL}"

gt_path_for() {
    local dataset="$1"; local seq="$2"
    case "${dataset}" in
        avia|mid360)
            echo "/home/euntae/Project/dataset/ros1/surfel_data/ground_truth/${seq}.txt" ;;
        ntu)
            echo "${REPO_ROOT}/dump/ntu_gt_tum/${seq}_gt.tum" ;;
    esac
}

run_one() {
    local dataset="$1"; local seq="$2"; local pool_idx="$3"
    local cpuset="${POOLS[$pool_idx]}"
    local out_dir="${REPO_ROOT}/dump/${LABEL}/ig_lio/${dataset}/${seq}"
    local summary="${REPO_ROOT}/dump/${LABEL}/ig_lio_${dataset}_summary.tsv"
    local gt; gt="$(gt_path_for "${dataset}" "${seq}")"

    mkdir -p "${out_dir}"

    if [[ ! -f "${gt}" ]]; then
        echo "[T63] SKIP ${dataset}/${seq} (GT missing: ${gt})"
        echo -e "${seq}\t-\t-\t-\t-\tNO_GT" >> "${summary}"
        return 0
    fi

    echo "[T63] start ig_lio/${dataset}/${seq} on cpuset=${cpuset}"
    BASELINE_CPUSET="${cpuset}" bash "${RUN_SH}" ig_lio "${dataset}" "${seq}" "${LABEL}" \
        > "${out_dir}/run.log" 2>&1 || {
            echo "[T63] FAIL ${seq} (run_baseline exit)"
            echo -e "${seq}\t-\t-\t-\t-\tRUN_FAIL" >> "${summary}"
            return 1; }

    if [[ ! -s "${out_dir}/traj.csv" ]]; then
        echo "[T63] FAIL ${seq} (no traj.csv)"
        echo -e "${seq}\t-\t-\t-\t-\tNO_TRAJ" >> "${summary}"
        return 1
    fi

    uv run --quiet --with numpy --with scipy python "${COMPUTE}" \
        "${out_dir}/traj.csv" "${gt}" --out "${out_dir}/ate.json" \
        > "${out_dir}/ate.log" 2>&1 || {
            echo "[T63] FAIL ${seq} (compute_ate)"
            echo -e "${seq}\t-\t-\t-\t-\tATE_FAIL" >> "${summary}"
            return 1; }

    python3 -c "
import json
d=json.load(open('${out_dir}/ate.json'))
print(f'${seq}\t{d[\"n_poses\"]}\t{d[\"rmse_m\"]:.4f}\t{d[\"mean_m\"]:.4f}\t{d[\"max_m\"]:.4f}\tPASS')
" >> "${summary}"
    echo "[T63] done ${dataset}/${seq}"
}

export -f run_one gt_path_for
export REPO_ROOT RUN_SH COMPUTE LABEL POOLS

run_matrix() {
    local dataset="$1"; shift
    local -a seqs=("$@")
    local summary="${REPO_ROOT}/dump/${LABEL}/ig_lio_${dataset}_summary.tsv"
    : > "${summary}"
    echo -e "seq\tposes\tATE_RMSE_m\tmean_m\tmax_m\tstatus" >> "${summary}"
    export summary

    pids=()
    i=0
    for seq in "${seqs[@]}"; do
        pool=$(( i % 3 ))
        run_one "${dataset}" "${seq}" "${pool}" &
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
    echo "=== Tier-1 ig_lio/${dataset} summary (Option C) ==="
    cat "${summary}"
}

echo "[T63] label=${LABEL}"
echo "[T63] NTU matrix (9 seqs, 3-way parallel)..."
run_matrix ntu "${NTU_SEQS[@]}"

echo ""
echo "[T63] Avia matrix (2 seqs, 1 wave)..."
run_matrix avia "${AVIA_SEQS[@]}"

echo ""
echo "[T63] Done. Artifacts under dump/${LABEL}/"
