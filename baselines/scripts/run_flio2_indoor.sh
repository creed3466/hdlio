#!/usr/bin/env bash
# Run FAST-LIO2 on 3 indoor Mid360 sequences to verify M3DGR paper SOTA values.
# Indoor bags are at ~/Project/dataset/ros1/indoor/ (not surfel_data/).
set -euo pipefail

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
LABEL="${1:-flio2_indoor_$(date +%Y%m%d_%H%M)}"
ALGO="fast_lio2"
IMAGE="baselines-${ALGO}:ros1"
CONFIG="${REPO_ROOT}/baselines/configs/${ALGO}/mid360.yaml"
REPUB_SRC="${REPO_ROOT}/baselines/tools/livox_v2_to_v1_republish"
SCRIPTS_DIR="${REPO_ROOT}/baselines/scripts"
ALGO_SRC="${REPO_ROOT}/baselines/algorithms/${ALGO}"
COMPUTE="${REPO_ROOT}/baselines/scripts/compute_ate.py"

SEQS=(Dynamic01 Occlusion01 Varying-illu01)
CPUS=("0-3" "4-7" "8-11")
BAG_DIR="/home/euntae/Project/dataset/ros1/indoor"
GT_DIR="/home/euntae/Project/dataset/ros1/indoor"

echo "==========================================="
echo "  FAST-LIO2 Indoor Baseline Verification"
echo "  Label: ${LABEL}"
echo "  Seqs: ${SEQS[*]}"
echo "==========================================="

pids=()
for i in "${!SEQS[@]}"; do
    seq="${SEQS[$i]}"
    cpus="${CPUS[$i]}"
    bag="${BAG_DIR}/${seq}.bag"
    gt="${GT_DIR}/${seq}.txt"
    out="${REPO_ROOT}/dump/${LABEL}/${ALGO}/indoor/${seq}"
    container="baseline_flio2_indoor_${seq}_$$"

    if [[ ! -f "${bag}" ]]; then
        echo "[SKIP] ${seq}: bag not found at ${bag}"
        continue
    fi

    mkdir -p "${out}"
    echo "[START] ${seq} cpus=${cpus}"

    (
        docker run --rm \
            --name "${container}" \
            --cpuset-cpus="${cpus}" \
            --memory=6g --cpus=3 \
            --ipc=private \
            -v "${ALGO_SRC}:/algo_src:ro" \
            -v "${SCRIPTS_DIR}:/baselines_scripts:ro" \
            -v "${CONFIG}:/config/params.yaml:ro" \
            -v "${bag}:/bag/input.bag:ro" \
            -v "${out}:/out:rw" \
            -v "${REPUB_SRC}:/republisher_src:ro" \
            -e ALGO="${ALGO}" \
            -e DATASET="mid360" \
            -e SEQ="${seq}" \
            "${IMAGE}" \
            /bin/bash -c "bash /baselines_scripts/run_inside_${ALGO}.sh 2>&1 | tee /out/stdout.log"

        echo "[DONE_CONTAINER] ${seq}"

        # Compute ATE
        if [[ -s "${out}/traj.csv" ]] && [[ -f "${gt}" ]]; then
            uv run --quiet --with numpy --with scipy python "${COMPUTE}" \
                "${out}/traj.csv" "${gt}" --out "${out}/ate.json" \
                > "${out}/ate.log" 2>&1 && {
                python3 -c "
import json
d=json.load(open('${out}/ate.json'))
print(f'[ATE] ${seq}: RMSE={d[\"rmse_m\"]:.4f}m  mean={d[\"mean_m\"]:.4f}m  max={d[\"max_m\"]:.4f}m  poses={d[\"n_poses\"]}')
"
            } || echo "[FAIL] ${seq}: compute_ate failed"
        else
            echo "[FAIL] ${seq}: no traj.csv or GT"
        fi
    ) &
    pids+=($!)
done

echo "Waiting for ${#pids[@]} containers..."
for pid in "${pids[@]}"; do
    wait "${pid}" || true
done

echo ""
echo "==========================================="
echo "  FAST-LIO2 Indoor Results"
echo "  Completed: $(date)"
echo "==========================================="

for seq in "${SEQS[@]}"; do
    out="${REPO_ROOT}/dump/${LABEL}/${ALGO}/indoor/${seq}"
    if [[ -f "${out}/ate.json" ]]; then
        python3 -c "
import json
d=json.load(open('${out}/ate.json'))
print(f'  ${seq}: RMSE={d[\"rmse_m\"]:.4f}m')
"
    else
        echo "  ${seq}: FAILED (check ${out}/run.log)"
    fi
done

echo ""
echo "  Paper SOTA reference:"
echo "    Dynamic01:      0.12m (Faster-LIO/DLIO/HM-LIO)"
echo "    Occlusion01:    0.11m (Fast-LIO2/Faster-LIO)"
echo "    Varying-illu01: 0.10m (DLIO)"
echo "==========================================="
