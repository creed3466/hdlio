#!/usr/bin/env bash
# Build per-algorithm image: usage: build_algo.sh <fast_lio2|point_lio|lio_sam|ig_lio|slict|dlio>
set -euo pipefail

ALGO="${1:-}"
if [[ -z "${ALGO}" ]]; then
    echo "Usage: $0 <fast_lio2|faster_lio|point_lio|lio_sam|ig_lio|slict|dlio>"
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BASELINES_ROOT="$( dirname "${SCRIPT_DIR}" )"
DOCKERFILE="${BASELINES_ROOT}/docker/${ALGO}.Dockerfile"

if [[ ! -f "${DOCKERFILE}" ]]; then
    echo "[ERROR] Dockerfile not found: ${DOCKERFILE}"
    exit 1
fi

# Ensure base image exists
if ! docker image inspect baselines-base:ros1 >/dev/null 2>&1; then
    echo "[INFO] Building baselines-base:ros1 first..."
    "${SCRIPT_DIR}/build_base.sh"
fi

docker build \
    -f "${DOCKERFILE}" \
    -t "baselines-${ALGO}:ros1" \
    "${BASELINES_ROOT}"
echo "[OK] baselines-${ALGO}:ros1 built"
