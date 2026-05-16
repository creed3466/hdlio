#!/usr/bin/env bash
# Phase 2 Tier-1 Avia rollout: sequentially run all 3 Avia algorithms
# (fast_lio2 → point_lio → ig_lio) across all 9 Avia seqs, 3-way
# container concurrency per algo.
#
# Total run matrix: 3 algos × 9 seqs = 27 runs.
# Wall-clock estimate: ~3 × 30 min ≈ 90 min at -r 1.0.
#
# Usage:
#   bash baselines/scripts/run_tier1_avia_all3.sh [LABEL]
set -euo pipefail

LABEL="${1:-phase2_tier1_avia_$(date +%Y%m%d)}"
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
RUNNER="${REPO_ROOT}/baselines/scripts/run_tier1_algo_dataset.sh"

mkdir -p "${REPO_ROOT}/dump/${LABEL}"

ALGOS=(fast_lio2 point_lio ig_lio)
for algo in "${ALGOS[@]}"; do
    echo ""
    echo "================================================================"
    echo "== [$(date +%H:%M:%S)] Tier-1 Avia: ${algo}"
    echo "================================================================"
    bash "${RUNNER}" "${algo}" avia "${LABEL}" \
        2>&1 | tee "${REPO_ROOT}/dump/${LABEL}/${algo}_orch.log"
done

echo ""
echo "================================================================"
echo "== [$(date +%H:%M:%S)] All 3 algos complete. Summaries:"
echo "================================================================"
for algo in "${ALGOS[@]}"; do
    s="${REPO_ROOT}/dump/${LABEL}/${algo}_avia_summary.tsv"
    echo ""
    echo "--- ${algo} ---"
    [[ -f "$s" ]] && cat "$s" || echo "(missing ${s})"
done
