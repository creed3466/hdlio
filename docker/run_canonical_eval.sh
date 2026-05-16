#!/bin/bash
# run_canonical_eval.sh — Canonical paper evaluation (AUTHORITATIVE)
#
# Produces the SINGLE definitive set of results for the HD-LIO paper.
# ALL sequences run with the SAME code, SAME rate=1.0, SAME evaluation pipeline.
#
# Datasets: Avia(9) → Mid360(9) → NTU(9) = 27 sequences
# Rate: 1.0 (mandatory for determinism + paper conclusions)
# Output: dump/paper_canonical/ with _meta.json
#
# Usage:
#   bash docker/run_canonical_eval.sh              # default label
#   bash docker/run_canonical_eval.sh my_label      # custom label
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-paper_canonical}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

# ── Dataset paths ──
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
GT_AVIA="/home/euntae/Project/dataset/ros1/m3dgr/ground_truth"
NTU_DATA="/home/euntae/Project/dataset/ros1/ntu_viral"

# ── RATE=1.0 (MANDATORY for paper) ──
RATE="1.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_canon_1 tofslam_canon_2 tofslam_canon_3)
MEM="3g"

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do
    docker rm -f "$c" 2>/dev/null || true
  done
}
trap cleanup EXIT

# ── Generate _meta.json ──
COMMIT_HASH=$(git rev-parse HEAD 2>/dev/null || echo "unknown")
COMMIT_MSG=$(git log --oneline -1 2>/dev/null || echo "unknown")
DIRTY=$(git diff --quiet src/tof_slam/ 2>/dev/null && echo "false" || echo "true")
cat > "${OUT_ROOT}/_meta.json" <<METAEOF
{
  "type": "canonical_paper_evaluation",
  "date": "$(date -Iseconds)",
  "git_commit": "${COMMIT_HASH}",
  "git_message": "${COMMIT_MSG}",
  "uncommitted_changes": ${DIRTY},
  "rate": ${RATE},
  "datasets": ["avia", "mid360", "ntu_viral"],
  "total_sequences": 27,
  "evaluation_script": "docker/run_canonical_eval.sh",
  "configs": {
    "avia": "per-sequence (avia_v6_seq/*.yaml)",
    "mid360": "unified (unified_mid360_v3c.yaml) + VI03 per-seq (mid360_seq/varying_illu03_v3c.yaml)",
    "ntu_viral": "single (ros1_ntu_viral.yaml)"
  }
}
METAEOF

log "================================================================"
log "  CANONICAL PAPER EVALUATION — $(date)"
log "  Git: ${COMMIT_MSG}"
log "  Rate: ${RATE} (canonical, deterministic)"
log "  Output: ${OUT_ROOT}/"
log "================================================================"

if [ "${DIRTY}" = "true" ]; then
  log ""
  log "  ⚠ WARNING: Uncommitted changes detected in src/tof_slam/"
  log "  Commit before running for full reproducibility."
  log ""
fi

############################################
# HELPER FUNCTIONS
############################################
start_and_build() {
  local dataset=$1
  local extra_vols="$2"
  cleanup
  sleep 1
  for i in 0 1 2; do
    docker run -d --rm --name "${CONTAINERS[$i]}" \
      --network host --cpuset-cpus "${CPUSETS[$i]}" --memory "$MEM" --ipc private \
      -v "$(pwd)/src:/root/catkin_ws/src:ro" \
      -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
      -v "$(pwd)/${OUT_ROOT}/${dataset}:/root/catkin_ws/dump" \
      -v "${HOST_SURFEL}:/root/catkin_ws/data/m3dgr_surfel:ro" \
      ${extra_vols} \
      "${IMAGE}" bash -lc "sleep infinity" > /dev/null
  done
  sleep 2
  log "  [Build] ${dataset}..."
  for i in 0 1 2; do
    docker exec "${CONTAINERS[$i]}" bash -c \
      "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
       catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -1" &
  done
  wait
  for i in 0 1 2; do
    docker exec "${CONTAINERS[$i]}" pip3 install scipy numpy -q 2>/dev/null || true
  done
  log "  [Build] Done."
}

run_batch() {
  local dataset=$1 runner=$2
  shift 2
  local batch_seqs=("$@")
  local n=${#batch_seqs[@]}
  [ $n -gt 3 ] && n=3
  local PIDS=()
  for i in $(seq 0 $((n-1))); do
    local seq="${batch_seqs[$i]}"
    local cfg="${SEQ_CFGS[$seq]}"
    local port="${PORTS[$i]}"
    local out="/root/catkin_ws/dump/${seq}"
    log "    [${CONTAINERS[$i]}] ${seq} (${cfg})"
    docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/${runner}" \
      "${cfg}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}" 2>/dev/null; then
      log "    ✓ ${batch_seqs[$i]} DONE"
    else
      log "    ✗ ${batch_seqs[$i]} FAILED"
    fi
  done
}

collect_results() {
  local dataset=$1
  shift
  local seqs=("$@")
  local total=0 count=0
  log ""
  log "  --- ${dataset} Results ---"
  for seq in "${seqs[@]}"; do
    local f="${OUT_ROOT}/${dataset}/${seq}/ate_result.txt"
    if [ -f "$f" ]; then
      local rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
      if [ -z "$rmse" ]; then
        rmse=$(grep "ATE RMSE:" "$f" 2>/dev/null | awk '{print $3}')
      fi
      if [ -n "$rmse" ]; then
        printf "  %-20s %.4f m\n" "$seq" "$rmse" | tee -a "${LOG}"
        total=$(python3 -c "print(${total} + ${rmse})")
        count=$((count + 1))
      else
        printf "  %-20s PARSE_FAIL\n" "$seq" | tee -a "${LOG}"
      fi
    else
      printf "  %-20s FAIL\n" "$seq" | tee -a "${LOG}"
    fi
  done
  if [ $count -gt 0 ]; then
    local mean=$(python3 -c "print(f'{${total}/${count}:.4f}')")
    log "  Mean (${count}/${#seqs[@]}): ${mean} m"
  fi
}

# Copy used config files into output for reproducibility
copy_configs() {
  local dataset=$1
  shift
  local seqs=("$@")
  local cfg_dir="${OUT_ROOT}/${dataset}/_configs"
  # Non-fatal: configs are for reproducibility, not critical for eval.
  # Docker creates output dirs as root — copy may fail on host.
  mkdir -p "${cfg_dir}" 2>/dev/null || true
  for seq in "${seqs[@]}"; do
    local cfg="${SEQ_CFGS[$seq]}"
    local src="src/tof_slam/config/${cfg}"
    [ -f "$src" ] && cp "$src" "${cfg_dir}/$(basename "$cfg")" 2>/dev/null || true
  done
}

############################################
# 1. AVIA (9 sequences, per-seq configs)
############################################
log ""
log "======== [1/3] M3DGR AVIA (9 seq, rate=${RATE}) ========"

declare -A SEQ_CFGS
AVIA_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)
SEQ_CFGS[Dark01]="avia_v6_seq/dark01.yaml"
SEQ_CFGS[Dark02]="avia_v6_seq/dark02.yaml"
SEQ_CFGS[Dynamic03]="avia_v6_seq/dynamic03.yaml"
SEQ_CFGS[Dynamic04]="avia_v6_seq/dynamic04.yaml"
SEQ_CFGS[Occlusion03]="avia_v6_seq/occlusion03.yaml"
SEQ_CFGS[Occlusion04]="avia_v6_seq/occlusion04.yaml"
SEQ_CFGS[Varying-illu03]="avia_v6_seq/varying_illu03.yaml"
SEQ_CFGS[Varying-illu04]="avia_v6_seq/varying_illu04.yaml"
SEQ_CFGS[Varying-illu05]="avia_v6_seq/varying_illu05.yaml"

start_and_build "avia" ""

# Batch grouping: short bags together, long bags isolated to avoid I/O contention.
# VI03 (17min) must run solo — batch contention at rate=1.0 causes 0.608→0.988 artifact.
# VI05 (8min) also affected when co-batched with long bags (0.150→0.175 artifact).
log "  Batch 1/4: Dark01, Dark02, Dynamic03"
run_batch "avia" "run_avia_exp.sh" "${AVIA_SEQS[@]:0:3}"
log "  Batch 2/4: Dynamic04, Occlusion03, Occlusion04"
run_batch "avia" "run_avia_exp.sh" "${AVIA_SEQS[@]:3:3}"
log "  Batch 3/4: Varying-illu03 (solo — long bag, contention-sensitive)"
run_batch "avia" "run_avia_exp.sh" "Varying-illu03"
log "  Batch 4/4: Varying-illu04, Varying-illu05"
run_batch "avia" "run_avia_exp.sh" "Varying-illu04" "Varying-illu05"

collect_results "avia" "${AVIA_SEQS[@]}"
copy_configs "avia" "${AVIA_SEQS[@]}"

############################################
# 2. MID360 (9 sequences, unified v3c + VI03 per-seq)
############################################
log ""
log "======== [2/3] M3DGR MID360 (9 seq, rate=${RATE}) ========"

MID_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)
SEQ_CFGS[Dark01]="unified_mid360_v3c.yaml"
SEQ_CFGS[Dark02]="unified_mid360_v3c.yaml"
SEQ_CFGS[Dynamic03]="unified_mid360_v3c.yaml"
SEQ_CFGS[Dynamic04]="unified_mid360_v3c.yaml"
SEQ_CFGS[Occlusion03]="unified_mid360_v3c.yaml"
SEQ_CFGS[Occlusion04]="unified_mid360_v3c.yaml"
SEQ_CFGS[Varying-illu03]="mid360_seq/varying_illu03_v3c.yaml"
SEQ_CFGS[Varying-illu04]="unified_mid360_v3c.yaml"
SEQ_CFGS[Varying-illu05]="unified_mid360_v3c.yaml"

start_and_build "mid360" ""

log "  Batch 1/3: Dark01, Dark02, Dynamic03"
run_batch "mid360" "run_avia_exp.sh" "${MID_SEQS[@]:0:3}"
log "  Batch 2/3: Dynamic04, Occlusion03, Occlusion04"
run_batch "mid360" "run_avia_exp.sh" "${MID_SEQS[@]:3:3}"
log "  Batch 3/3: VI03, VI04, VI05"
run_batch "mid360" "run_avia_exp.sh" "${MID_SEQS[@]:6:3}"

collect_results "mid360" "${MID_SEQS[@]}"
copy_configs "mid360" "${MID_SEQS[@]}"

############################################
# 3. NTU VIRAL (9 sequences, single config)
############################################
log ""
log "======== [3/3] NTU VIRAL (9 seq, rate=${RATE}) ========"

NTU_SEQS=(eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03)
for s in "${NTU_SEQS[@]}"; do
  SEQ_CFGS[$s]="ros1_ntu_viral.yaml"
done

start_and_build "ntu" "-v ${NTU_DATA}:/root/catkin_ws/data/ntu_viral:ro"

log "  Batch 1/3: eee_01, eee_02, eee_03"
run_batch "ntu" "run_ntu_exp.sh" "${NTU_SEQS[@]:0:3}"
log "  Batch 2/3: nya_01, nya_02, nya_03"
run_batch "ntu" "run_ntu_exp.sh" "${NTU_SEQS[@]:3:3}"
log "  Batch 3/3: sbs_01, sbs_02, sbs_03"
run_batch "ntu" "run_ntu_exp.sh" "${NTU_SEQS[@]:6:3}"

collect_results "ntu" "${NTU_SEQS[@]}"
copy_configs "ntu" "${NTU_SEQS[@]}"

cleanup

############################################
# FINAL SUMMARY
############################################
log ""
log "================================================================"
log "  CANONICAL PAPER EVALUATION — COMPLETE"
log "  $(date)"
log "  Git: ${COMMIT_MSG}"
log "  Rate: ${RATE}"
log "================================================================"
log ""
log "  Results in: ${OUT_ROOT}/"
log "  Metadata:   ${OUT_ROOT}/_meta.json"
log "  Configs:    ${OUT_ROOT}/{avia,mid360,ntu}/_configs/"
