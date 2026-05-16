#!/usr/bin/env bash
# Run a single baseline algorithm on a single sequence.
#
# Usage:
#   run_baseline.sh <algo> <dataset> <seq> [label]
#
# Examples:
#   run_baseline.sh fast_lio2 avia Dark01
#   run_baseline.sh point_lio mid360 Dynamic03
#   run_baseline.sh lio_sam ntu eee_01
#   run_baseline.sh slict ntu eee_01
#
# Output: dump/baselines_<DATE>/<algo>/<dataset>/<seq>/{traj.csv,stdout.log,ate.json}

set -euo pipefail

ALGO="${1:?algo required (fast_lio2|faster_lio|point_lio|lio_sam|ig_lio|slict|dlio)}"
DATASET="${2:?dataset required (avia|mid360|ntu|indoor)}"
SEQ="${3:?seq required (e.g. Dark01, eee_01)}"
LABEL="${4:-baselines_$(date +%Y%m%d)}"

# Path resolution
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
BASELINES_ROOT="${REPO_ROOT}/baselines"
ALGO_SRC="${BASELINES_ROOT}/algorithms/${ALGO}"
SCRIPTS_DIR="${BASELINES_ROOT}/scripts"
CONFIG="${BASELINES_ROOT}/configs/${ALGO}/${DATASET}.yaml"
OUT_DIR="${REPO_ROOT}/dump/${LABEL}/${ALGO}/${DATASET}/${SEQ}"

mkdir -p "${OUT_DIR}"

# Bag path
case "${DATASET}" in
    avia|mid360)
        BAG_PATH="/home/euntae/Project/dataset/ros1/surfel_data/${SEQ}.bag"
        ;;
    indoor)
        BAG_PATH="/home/euntae/Project/dataset/ros1/surfel_data/indoor_${SEQ}.bag"
        ;;
    ntu)
        BAG_PATH="/home/euntae/Project/dataset/ros1/ntu_viral/${SEQ}/${SEQ}.bag"
        ;;
    *)
        echo "[ERROR] Unknown dataset: ${DATASET}"; exit 1 ;;
esac

if [[ ! -f "${BAG_PATH}" ]]; then
    echo "[ERROR] Bag not found: ${BAG_PATH}"
    exit 1
fi

IMAGE="baselines-${ALGO}:ros1"
CONTAINER="baseline_${ALGO}_${SEQ}_$$"

echo "[RUN] ${ALGO} | ${DATASET}/${SEQ}"
echo "       bag: ${BAG_PATH}"
echo "       out: ${OUT_DIR}"

# Mount layout
# /algo_src         - upstream algorithm source (read-only)
# /config           - per-seq config (read-only)
# /bag              - rosbag (read-only)
# /out              - output directory (rw)
# /republisher_src  - livox v2→v1 namespace bridge (Mid-360 only, read-only)

# Mid-360/Indoor bags record /livox/mid360/lidar as livox_ros_driver2/CustomMsg.
# Non-DLIO algos consume livox_ros_driver/CustomMsg and need the v2→v1
# namespace republisher sidecar. DLIO natively subscribes to CustomMsg via
# its Livox callback (v1/v2 are MD5-identical) — no republisher needed.
REPUB_MOUNT=""
if [[ ( "${DATASET}" == "mid360" || "${DATASET}" == "indoor" ) && "${ALGO}" != "dlio" ]]; then
    REPUB_SRC="${BASELINES_ROOT}/tools/livox_v2_to_v1_republish"
    if [[ ! -d "${REPUB_SRC}" ]]; then
        echo "[ERROR] ${DATASET} requires ${REPUB_SRC} (livox namespace republisher)"
        exit 1
    fi
    REPUB_MOUNT="-v ${REPUB_SRC}:/republisher_src:ro"
fi

# Parallel-run support: caller may pin CPU pool via BASELINE_CPUSET env
# (docs/0_docker_container.md: p1=0-3, p2=4-7, p3=8-11; host reserves 12-15).
CPUSET_ARG=""
if [[ -n "${BASELINE_CPUSET:-}" ]]; then
    CPUSET_ARG="--cpuset-cpus=${BASELINE_CPUSET}"
fi

docker run --rm --name "${CONTAINER}" \
    --ipc private \
    --memory=12g --cpus=3 \
    ${CPUSET_ARG} \
    -v "${ALGO_SRC}:/algo_src:ro" \
    -v "${SCRIPTS_DIR}:/baselines_scripts:ro" \
    -v "${CONFIG}:/config/params.yaml:ro" \
    -v "${BAG_PATH}:/bag/input.bag:ro" \
    -v "${OUT_DIR}:/out:rw" \
    ${REPUB_MOUNT} \
    -e ALGO="${ALGO}" \
    -e DATASET="${DATASET}" \
    -e SEQ="${SEQ}" \
    "${IMAGE}" \
    /bin/bash -c "bash /baselines_scripts/run_inside_${ALGO}.sh 2>&1 | tee /out/stdout.log"

echo "[DONE] ${ALGO}/${DATASET}/${SEQ} → ${OUT_DIR}"
