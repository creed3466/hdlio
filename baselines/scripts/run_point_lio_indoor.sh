#!/usr/bin/env bash
# Run Point-LIO on all 8 M3DGR Indoor Mid360 sequences (3 batches of 3/3/2).
# Indoor bags: ~/Project/dataset/ros1/indoor/{SEQ}.bag
# GT: ~/Project/dataset/ros1/indoor/{SEQ}.txt
set -euo pipefail

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
LABEL="${1:-plio_indoor_$(date +%Y%m%d_%H%M)}"
ALGO="point_lio"
IMAGE="baselines-${ALGO}:ros1"
CONFIG="${REPO_ROOT}/baselines/configs/${ALGO}/mid360.yaml"
REPUB_SRC="${REPO_ROOT}/baselines/tools/livox_v2_to_v1_republish"
SCRIPTS_DIR="${REPO_ROOT}/baselines/scripts"
ALGO_SRC="${REPO_ROOT}/baselines/algorithms/${ALGO}"
COMPUTE="${REPO_ROOT}/baselines/scripts/compute_ate.py"

ALL_SEQS=(Dark03 Dark04 Dynamic01 Dynamic02 Occlusion01 Occlusion02 Varying-illu01 Varying-illu02)
CPUS=("0-3" "4-7" "8-11")
BAG_DIR="/home/euntae/Project/dataset/ros1/indoor"
GT_DIR="/home/euntae/Project/dataset/ros1/indoor"
OUT_BASE="${REPO_ROOT}/dump/${LABEL}/${ALGO}/indoor"

echo "==========================================="
echo "  Point-LIO Indoor Evaluation (8 seq)"
echo "  Label: ${LABEL}"
echo "  $(date)"
echo "==========================================="

run_batch() {
    local batch_seqs=("$@")
    local n=${#batch_seqs[@]}
    [ $n -gt 3 ] && n=3
    local pids=()

    for i in $(seq 0 $((n-1))); do
        local seq="${batch_seqs[$i]}"
        local cpus="${CPUS[$i]}"
        local bag="${BAG_DIR}/${seq}.bag"
        local gt="${GT_DIR}/${seq}.txt"
        local out="${OUT_BASE}/${seq}"
        local container="baseline_plio_indoor_${seq}_$$"

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
                echo "[FAIL] ${seq}: no traj.csv or GT"
            fi
        ) &
        pids+=($!)
    done

    for pid in "${pids[@]}"; do
        wait "${pid}" || true
    done
}

# Batch 1: Dark03, Dark04, Dynamic01
echo ""; echo "[Batch 1/3] Dark03, Dark04, Dynamic01"
run_batch "${ALL_SEQS[@]:0:3}"

# Batch 2: Dynamic02, Occlusion01, Occlusion02
echo ""; echo "[Batch 2/3] Dynamic02, Occlusion01, Occlusion02"
run_batch "${ALL_SEQS[@]:3:3}"

# Batch 3: Varying-illu01, Varying-illu02
echo ""; echo "[Batch 3/3] Varying-illu01, Varying-illu02"
run_batch "${ALL_SEQS[@]:6:2}"

# Summary
echo ""
echo "==========================================="
echo "  Point-LIO Indoor Results"
echo "  Completed: $(date)"
echo "==========================================="

total=0
count=0
for seq in "${ALL_SEQS[@]}"; do
    out="${OUT_BASE}/${seq}"
    if [[ -f "${out}/ate.json" ]]; then
        rmse=$(python3 -c "import json; d=json.load(open('${out}/ate.json')); print(f'{d[\"rmse_m\"]:.4f}')")
        printf "  %-20s %s m\n" "${seq}" "${rmse}"
        total=$(python3 -c "print(${total} + ${rmse})")
        count=$((count + 1))
    else
        printf "  %-20s FAILED\n" "${seq}"
    fi
done

if [[ $count -gt 0 ]]; then
    mean=$(python3 -c "print(f'{${total}/${count}:.4f}')")
    echo "  Mean (${count}/${#ALL_SEQS[@]}): ${mean} m"
fi

echo "==========================================="
