#!/usr/bin/env bash
# Run Surfel-LIO on all 8 M3DGR Indoor Mid360 sequences.
# Step 1: Convert rosbags to PLY+CSV using a ROS container
# Step 2: Run Surfel-LIO headless on each converted dataset
set -euo pipefail

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
LABEL="${1:-slio_indoor_$(date +%Y%m%d_%H%M)}"
SLIO_IMAGE="baselines-surfel_lio:latest"
ROS_IMAGE="baselines-fast_lio2:ros1"  # Any ROS1 image for bag conversion
ALGO_SRC="${REPO_ROOT}/baselines/algorithms/surfel_lio"
CONFIG="${ALGO_SRC}/config/mid360.yaml"
SCRIPTS_DIR="${REPO_ROOT}/baselines/scripts"
COMPUTE="${REPO_ROOT}/baselines/scripts/compute_ate.py"

ALL_SEQS=(Dark03 Dark04 Dynamic01 Dynamic02 Occlusion01 Occlusion02 Varying-illu01 Varying-illu02)
BAG_DIR="/home/euntae/Project/dataset/ros1/indoor"
GT_DIR="/home/euntae/Project/dataset/ros1/indoor"
DATA_DIR="${REPO_ROOT}/dump/${LABEL}/surfel_lio_data"
OUT_DIR="${REPO_ROOT}/dump/${LABEL}/surfel_lio/indoor"

echo "==========================================="
echo "  Surfel-LIO Indoor Evaluation (8 seq)"
echo "  Label: ${LABEL}"
echo "  $(date)"
echo "==========================================="

# --- Step 1: Convert bags to PLY+CSV ---
echo ""
echo "=== Step 1: Converting rosbags to PLY+CSV ==="

for seq in "${ALL_SEQS[@]}"; do
    bag="${BAG_DIR}/${seq}.bag"
    seq_data="${DATA_DIR}/${seq}"

    if [ -d "${seq_data}/lidar" ] && [ -f "${seq_data}/imu_data.csv" ]; then
        echo "  [SKIP] ${seq}: already converted"
        continue
    fi

    mkdir -p "${seq_data}"
    echo "  [CONVERT] ${seq}..."

    docker run --rm \
        --cpuset-cpus="0-7" \
        --memory=8g \
        -v "${bag}:/bag/input.bag:ro" \
        -v "${seq_data}:/out:rw" \
        -v "${SCRIPTS_DIR}:/scripts:ro" \
        "${ROS_IMAGE}" \
        /bin/bash -c "
            source /opt/ros/noetic/setup.bash
            pip3 install -q rosbag rospkg 2>/dev/null || true
            python3 /scripts/convert_bag_to_surfel_lio.py /bag/input.bag /out \
                --imu-topic /livox/mid360/imu \
                --lidar-topic /livox/mid360/lidar
        " 2>&1 | tail -5

    n_scans=$(ls "${seq_data}/lidar/"*.ply 2>/dev/null | wc -l)
    echo "  [DONE] ${seq}: ${n_scans} scans"
done

# --- Step 2: Run Surfel-LIO ---
echo ""
echo "=== Step 2: Running Surfel-LIO ==="

CPUS=("0-3" "4-7" "8-11")

run_batch() {
    local batch_seqs=("$@")
    local n=${#batch_seqs[@]}
    [ $n -gt 3 ] && n=3
    local pids=()

    for i in $(seq 0 $((n-1))); do
        local seq="${batch_seqs[$i]}"
        local seq_data="${DATA_DIR}/${seq}"
        local out="${OUT_DIR}/${seq}"
        mkdir -p "${out}"

        echo "  [START] ${seq} cpus=${CPUS[$i]}"
        (
            docker run --rm \
                --cpuset-cpus="${CPUS[$i]}" \
                --memory=4g \
                --ipc=private \
                -v "${seq_data}:/data:ro" \
                -v "${CONFIG}:/config/mid360.yaml:ro" \
                -v "${out}:/out:rw" \
                "${SLIO_IMAGE}" \
                /bin/bash -c "
                    cd /opt/surfel_lio/build && \
                    ./lio_player /config/mid360.yaml /data --headless 2>&1 | tee /out/stdout.log && \
                    cp /data/trajectory_tum.txt /out/traj.csv 2>/dev/null || \
                    cp trajectory_tum.txt /out/traj.csv 2>/dev/null || \
                    find /opt/surfel_lio /data /tmp -name 'trajectory*' -exec cp {} /out/traj.csv \; 2>/dev/null || true
                "

            echo "  [DONE_CONTAINER] ${seq}"

            # Compute ATE
            local gt="${GT_DIR}/${seq}.txt"
            if [ -s "${out}/traj.csv" ] && [ -f "${gt}" ]; then
                uv run --quiet --with numpy --with scipy python "${COMPUTE}" \
                    "${out}/traj.csv" "${gt}" --out "${out}/ate.json" \
                    > "${out}/ate.log" 2>&1 && {
                    python3 -c "
import json
d=json.load(open('${out}/ate.json'))
print(f'  [ATE] ${seq}: RMSE={d[\"rmse_m\"]:.4f}m')
"
                } || echo "  [FAIL] ${seq}: compute_ate failed"
            else
                echo "  [FAIL] ${seq}: no traj.csv or GT"
            fi
        ) &
        pids+=($!)
    done

    for pid in "${pids[@]}"; do
        wait "${pid}" || echo "  [WARN] PID ${pid} exited non-zero"
    done
}

# Batch 1: Dark03, Dark04, Dynamic01
echo "[Batch 1/3] Dark03, Dark04, Dynamic01"
run_batch "${ALL_SEQS[@]:0:3}"

# Batch 2: Dynamic02, Occlusion01, Occlusion02
echo "[Batch 2/3] Dynamic02, Occlusion01, Occlusion02"
run_batch "${ALL_SEQS[@]:3:3}"

# Batch 3: Varying-illu01, Varying-illu02
echo "[Batch 3/3] Varying-illu01, Varying-illu02"
run_batch "${ALL_SEQS[@]:6:2}"

# Summary
echo ""
echo "==========================================="
echo "  Surfel-LIO Indoor Results"
echo "  $(date)"
echo "==========================================="

for seq in "${ALL_SEQS[@]}"; do
    if [ -f "${OUT_DIR}/${seq}/ate.json" ]; then
        rmse=$(python3 -c "import json; d=json.load(open('${OUT_DIR}/${seq}/ate.json')); print(f'{d[\"rmse_m\"]:.4f}')")
        printf "  %-20s %s m\n" "${seq}" "${rmse}"
    else
        printf "  %-20s FAILED\n" "${seq}"
    fi
done
echo "==========================================="
