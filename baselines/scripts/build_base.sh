#!/usr/bin/env bash
# Build baselines-base:ros1 image (shared ROS1 noetic layer)
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BASELINES_ROOT="$( dirname "${SCRIPT_DIR}" )"

docker build \
    -f "${BASELINES_ROOT}/docker/base.Dockerfile" \
    -t baselines-base:ros1 \
    "${BASELINES_ROOT}"
echo "[OK] baselines-base:ros1 built"
