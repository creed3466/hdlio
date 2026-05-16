#!/bin/bash
# run_indoor_abl_batch.sh — Run one ablation config on all 8 indoor sequences
# Usage: run_indoor_abl_batch.sh <ABLATION_IDX> [RATE]
#   ABLATION_IDX: 0=no_surfel, 1=no_sigma2n, 2=no_degen_fb, 3=no_l2
set -e
cd "$(dirname "$0")/.."

IDX="${1:?Usage: run_indoor_abl_batch.sh <0-3> [RATE]}"
RATE="${2:-3.0}"
IMAGE="tofslam:ros1"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
OUT_ROOT="dump/paper_canonical/ablation"
CNAME="tofslam_abl_single"
PORT=11311
MEM="3g"

OVERLAYS=("ablation/ablation_no_surfel.yaml" "ablation/ablation_no_sigma2n.yaml" "ablation/ablation_no_degen_feedback.yaml" "ablation/ablation_no_l2.yaml")
LABELS=("no_surfel" "no_sigma2n" "no_degen_fb" "no_l2")

LABEL="${LABELS[$IDX]}"
OVERLAY="${OVERLAYS[$IDX]}"

declare -A CFGS
CFGS[indoor_Dark03]="unified_indoor_mid360_v1.yaml"
CFGS[indoor_Dark04]="unified_indoor_mid360_v1.yaml"
CFGS[indoor_Dynamic01]="unified_indoor_mid360_v1.yaml"
CFGS[indoor_Dynamic02]="unified_indoor_mid360_v1.yaml"
CFGS[indoor_Occlusion01]="unified_indoor_mid360_v1.yaml"
CFGS[indoor_Occlusion02]="unified_indoor_mid360_v1.yaml"
CFGS[indoor_Varying-illu01]="unified_indoor_mid360_v1.yaml"
CFGS[indoor_Varying-illu02]="unified_indoor_mid360_v1.yaml"
declare -A SHORT
SHORT[indoor_Dark03]="iDark03"; SHORT[indoor_Dark04]="iDark04"
SHORT[indoor_Dynamic01]="iDyn01"; SHORT[indoor_Dynamic02]="iDyn02"
SHORT[indoor_Occlusion01]="iOcc01"; SHORT[indoor_Occlusion02]="iOcc02"
SHORT[indoor_Varying-illu01]="iVI01"; SHORT[indoor_Varying-illu02]="iVI02"

SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)

echo "=== ${LABEL} (${OVERLAY}) ==="
for SEQ in "${SEQS[@]}"; do
  S="${SHORT[$SEQ]}"
  mkdir -p "${OUT_ROOT}/indoor/${LABEL}/${S}"
done

docker rm -f "$CNAME" 2>/dev/null || true
fuser -k ${PORT}/tcp 2>/dev/null || true
sleep 2

docker run -d --rm --name "$CNAME" \
  --network host --cpuset-cpus "0-11" --memory "$MEM" --ipc private \
  -v "$(pwd)/src:/root/catkin_ws/src" \
  -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
  -v "$(pwd)/${OUT_ROOT}/indoor/${LABEL}:/root/catkin_ws/dump" \
  -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
  -v "${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro" \
  "${IMAGE}" bash -lc "sleep infinity" > /dev/null

sleep 2
echo "[Build]..."
docker exec "$CNAME" bash -c \
  "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1"
docker exec "$CNAME" pip3 install scipy numpy -q 2>/dev/null || true
echo "[Build] Done."

for SEQ in "${SEQS[@]}"; do
  S="${SHORT[$SEQ]}"
  CFG="${CFGS[$SEQ]}"
  # Skip if already done
  if [ -f "${OUT_ROOT}/indoor/${LABEL}/${S}/ate_result.txt" ]; then
    echo "  [${LABEL}] ${S} — already done, skip"
    continue
  fi
  echo "  [${LABEL}] ${S} (${CFG} + ${OVERLAY})"
  docker exec "$CNAME" bash /root/catkin_ws/docker/run_avia_ablation_single.sh \
    "${CFG}" "${OVERLAY}" "${SEQ}" "/root/catkin_ws/dump/${S}" "${PORT}" "${RATE}" 2>&1 | tail -5
done

docker rm -f "$CNAME" 2>/dev/null || true
echo "=== ${LABEL} DONE ==="
