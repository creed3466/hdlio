#!/bin/bash
# run_S13_V5_9seq_screening.sh — Sprint 13 V5 (handoff) / B.V6 (architect G7) screening.
#
# Avia outdoor 9-seq unified rate=3.0 single-run per seq.
# HARD gate: 9-seq mean ≤ 0.314 m (Faster-LIO class).
# Falsification: > 0.391 m → Rule 16 ABORT.
# Aspirational: ≤ 0.310 m.
#
# Ref: docs/research/sprint13_architecture_20260513.md §7 B.V6, §2 HARD gates.
#
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_V5_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/v5.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="3.0"
CFG="avia_outdoor.yaml"   # unified — P1 flag=ON after V0 commit
IMAGE="tofslam:ros1"
TIMEOUT_S=900

SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)

CONTAINERS=(tofslam_S13V5_1 tofslam_S13V5_2 tofslam_S13V5_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== S13 V5 9-seq screening (G7) ===="
log "9 seqs × 1 run @ rate=${RATE}, unified ${CFG} (P1 flag=ON)"
log "HARD gate: mean ≤ 0.314 m | Falsification: > 0.391 m | Aspirational: ≤ 0.310 m"

# Build containers once
cleanup; sleep 1
for i in 0 1 2; do
  docker run -d --rm --init --name "${CONTAINERS[$i]}" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump:rw" \
    -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
    -v "${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro" \
    "${IMAGE}" bash -lc "sleep infinity" > /dev/null
done
sleep 2
log "Building..."
for i in 0 1 2; do
  docker exec "${CONTAINERS[$i]}" bash -lc \
    "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
done
wait
log "Build done."

# 9 seqs in 3 batches of 3
total=${#SEQS[@]}
batch_size=3
for ((batch=0; batch<total; batch+=batch_size)); do
  log ""
  log "==== Batch $((batch/batch_size + 1))/$((total/batch_size)) (seqs $((batch+1))-$((batch+batch_size))) ===="
  PIDS=()
  for i in 0 1 2; do
    idx=$((batch + i))
    [ $idx -ge $total ] && continue
    seq="${SEQS[$idx]}"
    log "    ${seq} on port ${PORTS[$i]} (slot ${i})..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "bash /root/catkin_ws/docker/run_avia_exp.sh ${CFG} ${seq} dump/${seq} ${PORTS[$i]} ${RATE} 2>&1" \
      > "${OUT_ROOT}/${seq}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done
done

log ""
log "==== 9-seq results ===="
SUMMARY="${OUT_ROOT}/v5_summary.csv"
echo "seq,ate_rmse,ate_mean" > "${SUMMARY}"
TOTAL=0
COUNT=0
declare -a SEQ_VALS
for seq in "${SEQS[@]}"; do
  ate_file="${OUT_ROOT}/${seq}/ate_result.txt"
  if [ -f "$ate_file" ]; then
    rmse=$(grep -oE "^rmse: [0-9.]+" "$ate_file" | awk '{print $2}')
    mean=$(grep -oE "^mean: [0-9.]+" "$ate_file" | awk '{print $2}')
    echo "${seq},${rmse},${mean}" >> "${SUMMARY}"
    log "  ${seq}: rmse=${rmse}"
    SEQ_VALS+=("$rmse")
    COUNT=$((COUNT+1))
  else
    log "  ${seq}: MISSING"
    echo "${seq},," >> "${SUMMARY}"
  fi
done

if [ $COUNT -eq 9 ]; then
  MEAN=$(python3 -c "print(sum([${SEQ_VALS[*]// /,}]) / 9.0)")
  log ""
  log "==== 9-seq mean RMSE: ${MEAN} ===="
  log "    HARD gate: ≤ 0.314 m"
  log "    Falsification: > 0.391 m → Rule 16 ABORT"
  log "    Aspirational: ≤ 0.310 m"

  PASS=$(python3 -c "exit(0 if ${MEAN} <= 0.314 else 1)" && echo PASS || echo FAIL)
  ASP=$(python3 -c "exit(0 if ${MEAN} <= 0.310 else 1)" && echo ASP_PASS || echo ASP_FAIL)
  ABORT=$(python3 -c "exit(0 if ${MEAN} > 0.391 else 1)" && echo ABORT_TRIGGER || echo ABORT_SAFE)

  log "    Verdict (HARD): ${PASS}"
  log "    Verdict (aspirational): ${ASP}"
  log "    Verdict (Rule 16): ${ABORT}"

  if [ "${ABORT}" = "ABORT_TRIGGER" ]; then
    log ""; log "==== ⚠️  RULE 16 ABORT TRIGGERED ===="
    log "Action: rollback to pre-step-S13-V0 + invalidate state + Step A failure record."
    exit 2
  elif [ "${PASS}" = "PASS" ]; then
    log ""; log "==== ✅ G7 PASS — proceed to V6 (G8 canonical) ===="
    exit 0
  else
    log ""; log "==== ⚠️  G7 HARD GATE FAIL but no ABORT ===="
    log "Action: research re-entry or grid extension; not Rule 16 unless > 0.391 m."
    exit 1
  fi
else
  log ""; log "==== Only ${COUNT}/9 seqs completed — partial result ===="
  exit 1
fi
