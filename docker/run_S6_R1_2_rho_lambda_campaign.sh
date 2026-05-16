#!/bin/bash
# run_S6_R1_2_rho_lambda_campaign.sh — Sprint 6 R1.2 ρ_λ measurement campaign
# Replays all 34 canonical sequences (Avia 17 + Mid-360 17) at rate=1.0 using
# canonical Sprint-5 configs. Captures eigenvalue_ratio per frame in
# diagnostics.csv (instrumentation patch S6-R1.2, not algorithm change).
#
# Output: dump/S6_rho_campaign/{seq}/diagnostics.csv
# Post-analysis: scripts/s6_rho_lambda_analysis.py (separate)
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S6_rho_campaign}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/campaign.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
HOST_NTU="/home/euntae/Project/dataset/ros1/ntu_viral"
RATE="1.0"
IMAGE="tofslam:ros1"
TIMEOUT_S=2400

# ========================================================================
# Sequence → config mapping (Sprint 5 canonical, per CLAUDE.md table)
# ========================================================================

declare -A SEQ_CFGS
# Avia outdoor 9-seq (canonical per-seq YAMLs)
SEQ_CFGS[Dark01]="avia_v6_seq/dark01.yaml"
SEQ_CFGS[Dark02]="avia_v6_seq/dark02.yaml"
SEQ_CFGS[Dynamic03]="avia_v6_seq/dynamic03.yaml"
SEQ_CFGS[Dynamic04]="avia_v6_seq/dynamic04.yaml"
SEQ_CFGS[Occlusion03]="avia_v6_seq/occlusion03.yaml"
SEQ_CFGS[Occlusion04]="avia_v6_seq/occlusion04.yaml"
SEQ_CFGS[Varying-illu03]="avia_v6_seq/varying_illu03.yaml"
SEQ_CFGS[Varying-illu04]="avia_v6_seq/varying_illu04.yaml"
SEQ_CFGS[Varying-illu05]="avia_v6_seq/varying_illu05.yaml"

# Avia indoor 8-seq
SEQ_CFGS[indoor_Dark03]="avia_indoor_seq/iDark03.yaml"
SEQ_CFGS[indoor_Dark04]="avia_indoor_seq/iDark04.yaml"
SEQ_CFGS[indoor_Dynamic01]="avia_indoor_seq/iDyn01.yaml"
SEQ_CFGS[indoor_Dynamic02]="avia_indoor_seq/iDyn02.yaml"
SEQ_CFGS[indoor_Occlusion01]="avia_indoor_seq/iOcc01.yaml"
SEQ_CFGS[indoor_Occlusion02]="avia_indoor_seq/iOcc02.yaml"
SEQ_CFGS[indoor_Varying-illu01]="avia_indoor_seq/iVI01.yaml"
SEQ_CFGS[indoor_Varying-illu02]="avia_indoor_seq/iVI02.yaml"

# Mid-360 outdoor 9-seq
SEQ_CFGS[m_Dark01]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_Dark02]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_Dynamic03]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_Dynamic04]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_Occlusion03]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_Occlusion04]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_Varying-illu03]="mid360_seq/varying_illu03_v3c.yaml"
SEQ_CFGS[m_Varying-illu04]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_Varying-illu05]="unified_mid360_v3c.yaml"

# Mid-360 indoor 8-seq
SEQ_CFGS[m_indoor_Dark03]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_indoor_Dark04]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_indoor_Dynamic01]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_indoor_Dynamic02]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_indoor_Occlusion01]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_indoor_Occlusion02]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_indoor_Varying-illu01]="unified_mid360_v3c.yaml"
SEQ_CFGS[m_indoor_Varying-illu02]="unified_mid360_v3c.yaml"

# ========================================================================
# Runner selection per seq (avia vs mid360)
# ========================================================================

is_avia()    { [[ "$1" != m_* ]]; }
is_indoor()  { [[ "$1" == indoor_* || "$1" == m_indoor_* ]]; }

# Strip m_ prefix for actual seq name passed to runner
seq_real() {
  local s="$1"
  if [[ "$s" == m_* ]]; then echo "${s#m_}"; else echo "$s"; fi
}

# ========================================================================
# Parallel orchestration: 3 containers, CPU 0-3/4-7/8-11, ports 11311-13
# ========================================================================

CONTAINERS=(tofslam_S6_1 tofslam_S6_2 tofslam_S6_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
}
trap cleanup EXIT

log "==== S6-R1.2 ρ_λ MEASUREMENT CAMPAIGN ===="
log "Date:     $(date)"
log "Label:    ${LABEL}"
log "Rate:     ${RATE}"
log "Seqs:     34 (Avia 17 + Mid-360 17)"
log "Goal:     Capture eigenvalue_ratio per frame in diagnostics.csv"
log "Patch:    S6-R1.2 instrumentation (no algorithm change)"

run_batch() {
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}
  cleanup; sleep 1

  for i in $(seq 0 $((n-1))); do
    docker run -d --rm --init --name "${CONTAINERS[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump:rw" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      -v "${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro" \
      -v "${HOST_NTU}:/home/euntae/Project/dataset/ros1/ntu_viral:ro" \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 2

  log "  [Build] ${n} containers..."
  for i in $(seq 0 $((n-1))); do
    docker exec "${CONTAINERS[$i]}" bash -lc \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 >/dev/null 2>&1" &
  done
  wait
  log "  [Build] Done."

  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    seq="${batch_seqs[$i]}"
    port="${PORTS[$i]}"
    out_dir="dump/${seq}"
    cfg="${SEQ_CFGS[$seq]}"
    real_seq=$(seq_real "$seq")

    # Runner choice: Avia uses run_avia_exp.sh; Mid-360 uses same runner with
    # different cfg (unified_mid360_v3c handles Mid-360 topic remapping).
    log "    Run ${seq} (real=${real_seq}, cfg=${cfg}) on port ${port}..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "TOFSLAM_DEBUG_DETERMINISM=0 bash /root/catkin_ws/docker/run_avia_exp.sh \
       ${cfg} ${real_seq} ${out_dir} ${port} ${RATE} 2>&1 | \
       tee /root/catkin_ws/${out_dir}/run.log" \
      > "${OUT_ROOT}/${seq}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done

  for c in "${CONTAINERS[@]}"; do
    docker exec "$c" bash -lc \
      "killall -9 tofslam_node rosbag rosout rosmaster roscore 2>/dev/null || true; \
       pkill -9 -f tofslam_node 2>/dev/null || true" 2>/dev/null || true
  done
}

# ========================================================================
# Batched execution: 3-by-3
# ========================================================================

# Avia outdoor batches
BATCH_A1=(Dark01 Dark02 Dynamic03)
BATCH_A2=(Dynamic04 Occlusion03 Occlusion04)
BATCH_A3=(Varying-illu03 Varying-illu04 Varying-illu05)
# Avia indoor batches
BATCH_AI1=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01)
BATCH_AI2=(indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02)
BATCH_AI3=(indoor_Varying-illu01 indoor_Varying-illu02)
# Mid-360 outdoor batches
BATCH_M1=(m_Dark01 m_Dark02 m_Dynamic03)
BATCH_M2=(m_Dynamic04 m_Occlusion03 m_Occlusion04)
BATCH_M3=(m_Varying-illu03 m_Varying-illu04 m_Varying-illu05)
# Mid-360 indoor batches
BATCH_MI1=(m_indoor_Dark03 m_indoor_Dark04 m_indoor_Dynamic01)
BATCH_MI2=(m_indoor_Dynamic02 m_indoor_Occlusion01 m_indoor_Occlusion02)
BATCH_MI3=(m_indoor_Varying-illu01 m_indoor_Varying-illu02)

log ""
log "==== Stage 1: Avia outdoor (9 seqs) ===="
run_batch "${BATCH_A1[@]}"
run_batch "${BATCH_A2[@]}"
run_batch "${BATCH_A3[@]}"

log ""
log "==== Stage 2: Avia indoor (8 seqs) ===="
run_batch "${BATCH_AI1[@]}"
run_batch "${BATCH_AI2[@]}"
run_batch "${BATCH_AI3[@]}"

log ""
log "==== Stage 3: Mid-360 outdoor (9 seqs) ===="
run_batch "${BATCH_M1[@]}"
run_batch "${BATCH_M2[@]}"
run_batch "${BATCH_M3[@]}"

log ""
log "==== Stage 4: Mid-360 indoor (8 seqs) ===="
run_batch "${BATCH_MI1[@]}"
run_batch "${BATCH_MI2[@]}"
run_batch "${BATCH_MI3[@]}"

log ""
log "==== DONE $(date) ===="
log "Output: ${OUT_ROOT}/{seq}/diagnostics.csv"
log "Run post-analysis: python3 docker/s6_rho_lambda_analysis.py ${OUT_ROOT}"
