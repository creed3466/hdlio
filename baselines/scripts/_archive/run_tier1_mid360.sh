#!/usr/bin/env bash
# Phase 3 Tier-1 rollout: Mid-360 baselines across all 9 M3DGR surfel_data
# sequences. Mid-360 is unblocked via the livox_v2_to_v1_republish sidecar
# (see baselines/tools/livox_v2_to_v1_republish + phase3_mid360_plan.md).
#
# Default algorithm set: fast_lio2 + point_lio + ig_lio (3 algos that the
# namespace-only bridge fully resolves). LIO-SAM and SLICT both require an
# additional CustomMsg→PointCloud2 adapter and are therefore skipped by
# default; pass ALGOS="..." to override once that adapter lands.
#
# Parallelism: 3 concurrent containers per algo (CPU pools p1=0-3, p2=4-7,
# p3=8-11 per docs/0_docker_container.md). Each algo's 9-seq matrix runs
# sequentially; algos dispatch round-robin across pools.
#
# Determinism: rosbag play -r 1.0 (CLAUDE.md §6-3 mandatory for validation).
#
# Output:
#   dump/<LABEL>/<algo>/mid360/<seq>/{traj.csv,odom.bag,stdout.log,ate.json,republisher.log}
# Aggregate summary (per algo):
#   dump/<LABEL>/<algo>_mid360_summary.tsv
#
# Usage:
#   bash baselines/scripts/run_tier1_mid360.sh [LABEL] [ALGOS]
#
# Examples:
#   bash baselines/scripts/run_tier1_mid360.sh phase3_mid360_20260414
#   ALGOS="fast_lio2" bash baselines/scripts/run_tier1_mid360.sh smoke_20260414 fast_lio2
#
# Wall-clock estimate (9 seqs, 3-way parallel, -r 1.0):
#   Longest seq ≈ 17 min (Varying-illu03). 3 waves × ~17 min ≈ 51 min per algo.
#   3 algos sequential ≈ 2.5 h total.
#
set -euo pipefail

LABEL="${1:-phase3_mid360_$(date +%Y%m%d_%H%M)}"
ALGOS_ARG="${2:-${ALGOS:-fast_lio2 point_lio ig_lio}}"
read -r -a ALGOS_LIST <<< "${ALGOS_ARG}"

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
RUNNER="${REPO_ROOT}/baselines/scripts/run_tier1_algo_dataset.sh"

mkdir -p "${REPO_ROOT}/dump/${LABEL}"

# Pre-flight: republisher source must exist (run_baseline.sh validates too,
# but failing early avoids spawning 27 containers that all die identically).
REPUB_SRC="${REPO_ROOT}/baselines/tools/livox_v2_to_v1_republish"
if [[ ! -d "${REPUB_SRC}" ]]; then
    echo "[MID360] ERROR: republisher source missing at ${REPUB_SRC}"
    exit 1
fi

# Pre-flight: GT files
GT_DIR="/home/euntae/Project/dataset/ros1/surfel_data/ground_truth"
if [[ ! -d "${GT_DIR}" ]]; then
    echo "[MID360] WARN: GT dir ${GT_DIR} missing — ATE eval will SKIP"
fi

# Guard against algos known to require an additional adapter.
DEFERRED_ALGOS=(lio_sam slict)
for algo in "${ALGOS_LIST[@]}"; do
    for def in "${DEFERRED_ALGOS[@]}"; do
        if [[ "${algo}" == "${def}" ]]; then
            echo "[MID360] WARN: ${algo} is deferred — it consumes PointCloud2,"
            echo "                not CustomMsg. The namespace republisher alone"
            echo "                will not satisfy it. Run will likely fail fast."
            echo "                See baselines/docs/phase3_mid360_plan.md."
        fi
    done
done

echo "[MID360] label=${LABEL}"
echo "[MID360] algos=${ALGOS_LIST[*]}"
echo "[MID360] dataset=mid360 (9 seqs via run_tier1_algo_dataset.sh)"
echo ""

for algo in "${ALGOS_LIST[@]}"; do
    echo ""
    echo "================================================================"
    echo "== [$(date +%H:%M:%S)] Tier-1 Mid-360: ${algo}"
    echo "================================================================"
    bash "${RUNNER}" "${algo}" mid360 "${LABEL}" \
        2>&1 | tee "${REPO_ROOT}/dump/${LABEL}/${algo}_orch.log"
done

echo ""
echo "================================================================"
echo "== [$(date +%H:%M:%S)] All algos complete. Summaries:"
echo "================================================================"
for algo in "${ALGOS_LIST[@]}"; do
    s="${REPO_ROOT}/dump/${LABEL}/${algo}_mid360_summary.tsv"
    echo ""
    echo "--- ${algo} ---"
    [[ -f "$s" ]] && cat "$s" || echo "(missing ${s})"
done
