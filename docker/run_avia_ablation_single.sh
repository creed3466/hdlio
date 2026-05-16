#!/bin/bash
# run_avia_ablation_single.sh — Container-internal: run single Avia sequence
# with per-sequence base config + ablation overlay.
#
# Usage: run_avia_ablation_single.sh <SEQ_CONFIG> <ABLATION_OVERLAY> <SEQ> <OUT_DIR> [PORT] [RATE]
#
# Example:
#   run_avia_ablation_single.sh avia_seq/dark01.yaml ablation/ablation_no_surfel.yaml \
#     Dark01 /root/catkin_ws/dump/Dark01 11311 3.0
#
# The script merges SEQ_CONFIG (base) with ABLATION_OVERLAY (overrides) into
# a temporary YAML via Python dict merge, then delegates to run_avia_exp.sh.

set -e

SEQ_CONFIG="${1:?Usage: run_avia_ablation_single.sh <SEQ_CONFIG> <ABLATION_OVERLAY> <SEQ> <OUT_DIR> [PORT] [RATE]}"
ABLATION_OVERLAY="${2:?Missing ABLATION_OVERLAY}"
SEQ="${3:?Missing SEQ}"
OUT_DIR="${4:?Missing OUT_DIR}"
PORT="${5:-11311}"
RATE="${6:-3.0}"

CONFIG_DIR="/root/catkin_ws/src/tof_slam/config"
BASE_FILE="${CONFIG_DIR}/${SEQ_CONFIG}"
OVERLAY_FILE="${CONFIG_DIR}/${ABLATION_OVERLAY}"

if [ ! -f "${BASE_FILE}" ]; then
  echo "ERROR: Base config not found: ${BASE_FILE}"
  exit 1
fi
if [ ! -f "${OVERLAY_FILE}" ]; then
  echo "ERROR: Ablation overlay not found: ${OVERLAY_FILE}"
  exit 1
fi

# Generate merged config via Python dict merge (safe, no duplicate-key issues).
MERGED_NAME="_ablation_merged_${SEQ}_${PORT}_$$.yaml"
MERGED_FILE="${CONFIG_DIR}/${MERGED_NAME}"

python3 -c "
import yaml
with open('${BASE_FILE}') as f:
    base = yaml.safe_load(f)
with open('${OVERLAY_FILE}') as f:
    overlay = yaml.safe_load(f)
if base is None: base = {}
if overlay is None: overlay = {}
base.update(overlay)
with open('${MERGED_FILE}', 'w') as f:
    yaml.dump(base, f, default_flow_style=False, sort_keys=False)
"

echo "[ablation] Merged ${SEQ_CONFIG} + ${ABLATION_OVERLAY} -> ${MERGED_NAME}"

# Delegate to standard runner
bash /root/catkin_ws/docker/run_avia_exp.sh "${MERGED_NAME}" "${SEQ}" "${OUT_DIR}" "${PORT}" "${RATE}"

# Cleanup merged file
rm -f "${MERGED_FILE}"
