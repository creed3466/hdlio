#!/usr/bin/env bash
# Run Point-LIO on remaining Indoor Mid360 sequences (batch 2+3).
# Uses run_baseline.sh with indoor_ prefix bags from surfel_data.
set -euo pipefail

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
LABEL="${1:-plio_indoor_canonical}"
COMPUTE="${REPO_ROOT}/baselines/scripts/compute_ate.py"
GT_DIR="/home/euntae/Project/dataset/ros1/indoor"

run_batch() {
    local seqs=("$@")
    local cpus=("0-3" "4-7" "8-11")
    local n=${#seqs[@]}
    [ $n -gt 3 ] && n=3
    local pids=()

    for i in $(seq 0 $((n-1))); do
        local seq="${seqs[$i]}"
        local bag_seq="indoor_${seq}"
        (
            BASELINE_CPUSET="${cpus[$i]}" bash "${REPO_ROOT}/baselines/scripts/run_baseline.sh" \
                point_lio mid360 "${bag_seq}" "${LABEL}" 2>&1

            # Compute ATE
            local out="${REPO_ROOT}/dump/${LABEL}/point_lio/mid360/${bag_seq}"
            local gt="${GT_DIR}/${seq}.txt"
            if [[ -s "${out}/traj.csv" ]] && [[ -f "${gt}" ]]; then
                uv run --quiet --with numpy --with scipy python "${COMPUTE}" \
                    "${out}/traj.csv" "${gt}" --out "${out}/ate.json" \
                    > "${out}/ate.log" 2>&1 && {
                    python3 -c "
import json
d=json.load(open('${out}/ate.json'))
print(f'[ATE] ${seq}: RMSE={d[\"rmse_m\"]:.4f}m')
"
                } || echo "[FAIL] ${seq}: compute_ate failed"
            else
                echo "[FAIL] ${seq}: no traj.csv (${out}/traj.csv) or GT (${gt})"
            fi
        ) &
        pids+=($!)
    done

    for pid in "${pids[@]}"; do
        wait "${pid}" || echo "[WARN] PID ${pid} exited non-zero"
    done
}

echo "==========================================="
echo "  Point-LIO Indoor Batch 2+3"
echo "  $(date)"
echo "==========================================="

# Batch 2: Dynamic02, Occlusion01, Occlusion02
echo ""; echo "[Batch 2/3] Dynamic02, Occlusion01, Occlusion02"
run_batch Dynamic02 Occlusion01 Occlusion02

# Batch 3: Varying-illu01, Varying-illu02
echo ""; echo "[Batch 3/3] Varying-illu01, Varying-illu02"
run_batch Varying-illu01 Varying-illu02

# Summary
echo ""
echo "==========================================="
echo "  All Point-LIO Indoor Results"
echo "  $(date)"
echo "==========================================="

for seq in Dark03 Dark04 Dynamic01 Dynamic02 Occlusion01 Occlusion02 Varying-illu01 Varying-illu02; do
    # Check both possible output paths
    for outdir in \
        "${REPO_ROOT}/dump/${LABEL}/point_lio/indoor/${seq}" \
        "${REPO_ROOT}/dump/${LABEL}/point_lio/mid360/indoor_${seq}"; do
        if [[ -f "${outdir}/ate.json" ]]; then
            rmse=$(python3 -c "import json; d=json.load(open('${outdir}/ate.json')); print(f'{d[\"rmse_m\"]:.4f}')")
            printf "  %-20s %s m\n" "${seq}" "${rmse}"
            break
        fi
    done
done
echo "==========================================="
