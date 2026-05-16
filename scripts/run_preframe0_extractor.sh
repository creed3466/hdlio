#!/bin/bash
# run_preframe0_extractor.sh — Sprint 14 B.R0.4.1 Phase 1 separation gate.
#
# Iterates 9 Avia outdoor seqs through preframe0_extractor.py inside the
# tofslam:ros1 Docker container. Output:
#   docs/experiments/preframe0_classifier_features.csv (9 rows + header)
#   docs/experiments/preframe0_extractor.stderr.log (per-seq stderr)
#
# Wallclock: ~1-2 min total (small bag prefix read, no algorithm runs).

set -e
cd "$(dirname "$0")/.."

OUT_CSV="docs/experiments/preframe0_classifier_features.csv"
OUT_LOG="docs/experiments/preframe0_extractor.stderr.log"
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
IMG="tofslam:ros1"
CONT="tofslam_preframe0"

mkdir -p docs/experiments
# Fresh output (Phase 1 is one-shot; do NOT append across runs)
rm -f "${OUT_CSV}" "${OUT_LOG}"

SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)

docker rm -f "${CONT}" 2>/dev/null || true
docker run -d --rm --name "${CONT}" \
  --network host \
  -v "$(pwd)/src:/root/catkin_ws/src:ro" \
  -v "$(pwd)/scripts:/root/catkin_ws/scripts:ro" \
  -v "$(pwd)/docs:/root/catkin_ws/docs:rw" \
  -v "${HOST_SURFEL}:/data/surfel_data:ro" \
  "${IMG}" bash -lc "sleep infinity" > /dev/null
sleep 2

# Build livox message Python bindings inside container
echo "[setup] building Livox message bindings..."
docker exec "${CONT}" bash -lc "cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 livox_ros_driver_generate_messages_py 2>&1 | tail -2"

# Ensure numpy (usually pre-installed)
docker exec "${CONT}" pip3 install --quiet numpy 2>/dev/null || true

echo "[run] iterating ${#SEQS[@]} sequences..."
for s in "${SEQS[@]}"; do
  bag="/data/surfel_data/${s}.bag"
  echo "  -> ${s}"
  docker exec "${CONT}" bash -lc \
    "cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
     spdlog_level=err python3 scripts/preframe0_extractor.py \
       --bag ${bag} \
       --imu-topic /livox/avia/imu \
       --lidar-topic /livox/avia/lidar \
       --seq-name ${s} \
       --out docs/experiments/preframe0_classifier_features.csv \
       2>>docs/experiments/preframe0_extractor.stderr.log >/dev/null"
done

docker rm -f "${CONT}" 2>/dev/null || true

echo ""
echo "================================================================"
echo "  Phase 1 extractor results (docs/experiments/preframe0_classifier_features.csv)"
echo "================================================================"
column -t -s, "${OUT_CSV}" | head -1
column -t -s, "${OUT_CSV}" | awk 'NR>1 { print }' | column -t

echo ""
echo "================================================================"
echo "  Phase 1 gate evaluation"
echo "================================================================"
python3 -c "
import csv
rows = list(csv.DictReader(open('${OUT_CSV}')))
verdict = 'PASS'
for r in rows:
    seq = r['seq']
    pa = r['predicate_all'] == '1'
    if seq == 'Varying-illu03':
        expected = True
    else:
        expected = False
    label = 'EXPECTED' if pa == expected else 'UNEXPECTED'
    status = 'PASS' if pa == expected else 'FAIL'
    if pa != expected:
        verdict = 'FAIL'
    print(f'  {seq:<22} predicate_all={pa!s:<5}  expected={expected!s:<5}  {status}')
print()
print(f'  Overall Phase 1 verdict: {verdict}')
"
