#!/bin/bash
# run_ntu_ablation_single.sh — Container-internal: run single NTU VIRAL sequence
# with base config + ablation overlay.
#
# Usage: run_ntu_ablation_single.sh <ABLATION_OVERLAY> <SEQ> <OUT_DIR> [PORT] [RATE]

set -e

ABLATION_OVERLAY="${1:?Usage: run_ntu_ablation_single.sh <ABLATION_OVERLAY> <SEQ> <OUT_DIR> [PORT] [RATE]}"
SEQ="${2:?Missing SEQ}"
OUT_DIR="${3:?Missing OUT_DIR}"
PORT="${4:-11311}"
RATE="${5:-1.0}"

CONFIG_DIR="/root/catkin_ws/src/tof_slam/config"
BASE_FILE="${CONFIG_DIR}/ros1_ntu_viral.yaml"
OVERLAY_FILE="${CONFIG_DIR}/${ABLATION_OVERLAY}"

if [ ! -f "${BASE_FILE}" ]; then
  echo "ERROR: Base config not found: ${BASE_FILE}"
  exit 1
fi
if [ ! -f "${OVERLAY_FILE}" ]; then
  echo "ERROR: Ablation overlay not found: ${OVERLAY_FILE}"
  exit 1
fi

# Generate merged config
MERGED_NAME="_ablation_ntu_merged_${SEQ}_${PORT}_$$.yaml"
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

echo "[ntu-ablation] Merged ros1_ntu_viral.yaml + ${ABLATION_OVERLAY} -> ${MERGED_NAME}"

# Delegate to NTU runner
bash /root/catkin_ws/docker/run_ntu_exp.sh "${MERGED_NAME}" "${SEQ}" "${OUT_DIR}" "${PORT}" "${RATE}"

# Cleanup
rm -f "${MERGED_FILE}"
