#!/bin/bash
# run_full_verification.sh — Full dataset verification (all 43 sequences)
#
# Runs ALL 5 datasets at rate=3.0 (screening) and compares to canonical baselines.
# Datasets: Avia(9) + Mid360(9) + Indoor Mid360(8) + Avia Indoor(8) + NTU(9)
#
# Usage:
#   bash docker/run_full_verification.sh [LABEL]
#
# Output: dump/<LABEL>/  (default: full_verify_v1)
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-full_verify_v1}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"
mkdir -p "${OUT_ROOT}"

# ── Dataset paths ──
HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
GT_AVIA="/home/euntae/Project/dataset/ros1/m3dgr/ground_truth"
NTU_DATA="/home/euntae/Project/dataset/ros1/ntu_viral"

# ── RATE=3.0 (screening) ──
RATE="3.0"
IMAGE="tofslam:ros1"
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)
CONTAINERS=(tofslam_fv_1 tofslam_fv_2 tofslam_fv_3)
MEM="3g"

log() { echo "$*" | tee -a "${LOG}"; }

cleanup() {
  for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done
  for p in "${PORTS[@]}"; do fuser -k ${p}/tcp 2>/dev/null || true; done
}
trap cleanup EXIT

# ── Canonical baselines (from dump/paper_canonical/) ──
declare -A BASELINE
# Avia Outdoor
BASELINE[avia_Dark01]=0.0960;    BASELINE[avia_Dark02]=0.5909
BASELINE[avia_Dynamic03]=0.1637; BASELINE[avia_Dynamic04]=0.2779
BASELINE[avia_Occlusion03]=0.2052; BASELINE[avia_Occlusion04]=0.2572
BASELINE[avia_Varying-illu03]=0.9877; BASELINE[avia_Varying-illu04]=0.3092
BASELINE[avia_Varying-illu05]=0.1620
# Mid360 Outdoor
BASELINE[mid360_Dark01]=0.1599;  BASELINE[mid360_Dark02]=0.2301
BASELINE[mid360_Dynamic03]=0.1685; BASELINE[mid360_Dynamic04]=0.1843
BASELINE[mid360_Occlusion03]=0.3006; BASELINE[mid360_Occlusion04]=0.2137
BASELINE[mid360_Varying-illu03]=0.8689; BASELINE[mid360_Varying-illu04]=0.3133
BASELINE[mid360_Varying-illu05]=0.1443
# Indoor Mid360
BASELINE[indoor_iDark03]=0.1747;  BASELINE[indoor_iDark04]=0.1822
BASELINE[indoor_iDyn01]=0.1374;   BASELINE[indoor_iDyn02]=0.1384
BASELINE[indoor_iOcc01]=0.1512;   BASELINE[indoor_iOcc02]=0.1449
BASELINE[indoor_iVI01]=0.1337;    BASELINE[indoor_iVI02]=0.1343
# Avia Indoor
BASELINE[avia_indoor_aiDk03]=1.8425; BASELINE[avia_indoor_aiDk04]=0.4104
BASELINE[avia_indoor_aiDy01]=1.0710; BASELINE[avia_indoor_aiDy02]=0.5185
BASELINE[avia_indoor_aiOc01]=0.7958; BASELINE[avia_indoor_aiOc02]=1.1672
BASELINE[avia_indoor_aiVI01]=0.8492; BASELINE[avia_indoor_aiVI02]=0.7009
# NTU Viral
BASELINE[ntu_eee_01]=0.0571; BASELINE[ntu_eee_02]=0.0522; BASELINE[ntu_eee_03]=0.0886
BASELINE[ntu_nya_01]=0.0494; BASELINE[ntu_nya_02]=0.0824; BASELINE[ntu_nya_03]=0.0758
BASELINE[ntu_sbs_01]=0.0634; BASELINE[ntu_sbs_02]=0.0772; BASELINE[ntu_sbs_03]=0.0688

# ── Generate meta ──
COMMIT_HASH=$(git rev-parse HEAD 2>/dev/null || echo "unknown")
COMMIT_MSG=$(git log --oneline -1 2>/dev/null || echo "unknown")
cat > "${OUT_ROOT}/_meta.json" <<METAEOF
{
  "type": "full_verification",
  "date": "$(date -Iseconds)",
  "git_commit": "${COMMIT_HASH}",
  "git_message": "${COMMIT_MSG}",
  "rate": ${RATE},
  "canonical_commit": "3dc1c28",
  "datasets": ["avia", "mid360", "indoor", "avia_indoor", "ntu"],
  "total_sequences": 43
}
METAEOF

log "================================================================"
log "  FULL DATASET VERIFICATION — $(date)"
log "  Git: ${COMMIT_MSG}"
log "  Rate: ${RATE} (screening)"
log "  Canonical baseline: 3dc1c28"
log "  Output: ${OUT_ROOT}/"
log "================================================================"

############################################
# HELPERS
############################################
start_containers() {
  local dataset=$1
  local extra_vols="$2"
  cleanup
  sleep 2
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
    local label="${SEQ_OUT_LABELS[$seq]:-$seq}"
    local port="${PORTS[$i]}"
    local out="/root/catkin_ws/dump/${label}"
    log "    [${CONTAINERS[$i]}] ${label} (${cfg})"
    docker exec "${CONTAINERS[$i]}" bash "/root/catkin_ws/docker/${runner}" \
      "${cfg}" "${seq}" "${out}" "${port}" "${RATE}" >> "${LOG}" 2>&1 &
    PIDS+=($!)
  done
  for idx in "${!PIDS[@]}"; do
    if wait "${PIDS[$idx]}" 2>/dev/null; then
      log "    OK ${batch_seqs[$idx]}"
    else
      log "    FAIL ${batch_seqs[$idx]}"
    fi
  done
}

collect_and_compare() {
  local dataset=$1
  shift
  local labels=("$@")
  local total=0 count=0 total_base=0 wins=0 losses=0

  log ""
  log "  --- ${dataset} Results ---"
  for label in "${labels[@]}"; do
    local f="${OUT_ROOT}/${dataset}/${label}/ate_result.txt"
    local base_key="${dataset}_${label}"
    local base="${BASELINE[$base_key]}"
    if [ -f "$f" ]; then
      local rmse=$(grep "^rmse:" "$f" 2>/dev/null | awk '{print $2}')
      [ -z "$rmse" ] && rmse=$(grep "ATE RMSE:" "$f" 2>/dev/null | awk '{print $3}')
      if [ -n "$rmse" ] && [ -n "$base" ]; then
        local delta=$(python3 -c "d=(${rmse}-${base})/${base}*100; print(f'{d:+.1f}%')")
        printf "  %-20s %8.4f m  (can: %.4f, %s)\n" "$label" "$rmse" "$base" "$delta" | tee -a "${LOG}"
        total=$(python3 -c "print(${total} + ${rmse})")
        total_base=$(python3 -c "print(${total_base} + ${base})")
        count=$((count + 1))
        better=$(python3 -c "print('1' if ${rmse} <= ${base} * 1.05 else '0')")
        [ "$better" = "1" ] && wins=$((wins + 1)) || losses=$((losses + 1))
      elif [ -n "$rmse" ]; then
        printf "  %-20s %8.4f m  (no baseline)\n" "$label" "$rmse" | tee -a "${LOG}"
        total=$(python3 -c "print(${total} + ${rmse})")
        count=$((count + 1))
      else
        printf "  %-20s PARSE_FAIL\n" "$label" | tee -a "${LOG}"
      fi
    else
      printf "  %-20s NO RESULT\n" "$label" | tee -a "${LOG}"
    fi
  done

  if [ $count -gt 0 ]; then
    local mean=$(python3 -c "print(f'{${total}/${count}:.4f}')")
    if [ "$total_base" != "0" ]; then
      local mean_base=$(python3 -c "print(f'{${total_base}/${count}:.4f}')")
      local mean_delta=$(python3 -c "d=(${total}/${count}-${total_base}/${count})/(${total_base}/${count})*100; print(f'{d:+.1f}%')")
      log "  ${dataset} Mean (${count}/${#labels[@]}): ${mean} m (can: ${mean_base}, ${mean_delta})"
      log "  Pass(<5%): ${wins}/${count}, Regress(>5%): ${losses}/${count}"
    else
      log "  ${dataset} Mean (${count}/${#labels[@]}): ${mean} m"
    fi
  fi
}

############################################
# 1. AVIA OUTDOOR (9 sequences)
############################################
log ""
log "======== [1/5] M3DGR AVIA OUTDOOR (9 seq) ========"

declare -A SEQ_CFGS
declare -A SEQ_OUT_LABELS

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

start_containers "avia" ""
mkdir -p "${OUT_ROOT}/avia"

log "  Batch 1/3: Dark01, Dark02, Dynamic03"
run_batch "avia" "run_avia_exp.sh" "${AVIA_SEQS[@]:0:3}"
log "  Batch 2/3: Dynamic04, Occlusion03, Occlusion04"
run_batch "avia" "run_avia_exp.sh" "${AVIA_SEQS[@]:3:3}"
log "  Batch 3/3: VI03, VI04, VI05"
run_batch "avia" "run_avia_exp.sh" "${AVIA_SEQS[@]:6:3}"

collect_and_compare "avia" "${AVIA_SEQS[@]}"

############################################
# 2. MID360 OUTDOOR (9 sequences)
############################################
log ""
log "======== [2/5] M3DGR MID360 OUTDOOR (9 seq) ========"

MID_SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)
SEQ_CFGS[Dark01]="unified_outdoor_mid360_v2a.yaml"
SEQ_CFGS[Dark02]="unified_outdoor_mid360_v2a.yaml"
SEQ_CFGS[Dynamic03]="unified_outdoor_mid360_v2a.yaml"
SEQ_CFGS[Dynamic04]="unified_outdoor_mid360_v2a.yaml"
SEQ_CFGS[Occlusion03]="unified_outdoor_mid360_v2a.yaml"
SEQ_CFGS[Occlusion04]="unified_outdoor_mid360_v2a.yaml"
SEQ_CFGS[Varying-illu03]="mid360_seq/varying_illu03_f50_skf050.yaml"
SEQ_CFGS[Varying-illu04]="unified_outdoor_mid360_v2a.yaml"
SEQ_CFGS[Varying-illu05]="unified_outdoor_mid360_v2a.yaml"

start_containers "mid360" ""
mkdir -p "${OUT_ROOT}/mid360"

log "  Batch 1/3: Dark01, Dark02, Dynamic03"
run_batch "mid360" "run_avia_exp.sh" "${MID_SEQS[@]:0:3}"
log "  Batch 2/3: Dynamic04, Occlusion03, Occlusion04"
run_batch "mid360" "run_avia_exp.sh" "${MID_SEQS[@]:3:3}"
log "  Batch 3/3: VI03, VI04, VI05"
run_batch "mid360" "run_avia_exp.sh" "${MID_SEQS[@]:6:3}"

collect_and_compare "mid360" "${MID_SEQS[@]}"

############################################
# 3. INDOOR MID360 (8 sequences)
############################################
log ""
log "======== [3/5] M3DGR INDOOR MID360 (8 seq) ========"

INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
INDOOR_LABELS=(iDark03 iDark04 iDyn01 iDyn02 iOcc01 iOcc02 iVI01 iVI02)

SEQ_CFGS[indoor_Dark03]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Dark04]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Dynamic01]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Dynamic02]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Occlusion01]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Occlusion02]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Varying-illu01]="unified_indoor_mid360_v1.yaml"
SEQ_CFGS[indoor_Varying-illu02]="unified_indoor_mid360_v1.yaml"

for i in $(seq 0 $((${#INDOOR_SEQS[@]}-1))); do
  SEQ_OUT_LABELS[${INDOOR_SEQS[$i]}]="${INDOOR_LABELS[$i]}"
done

start_containers "indoor" "-v ${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro"
mkdir -p "${OUT_ROOT}/indoor"

log "  Batch 1/3: iDark03, iDark04, iDyn01"
run_batch "indoor" "run_avia_exp.sh" "${INDOOR_SEQS[@]:0:3}"
log "  Batch 2/3: iDyn02, iOcc01, iOcc02"
run_batch "indoor" "run_avia_exp.sh" "${INDOOR_SEQS[@]:3:3}"
log "  Batch 3/3: iVI01, iVI02"
run_batch "indoor" "run_avia_exp.sh" "${INDOOR_SEQS[@]:6:2}"

collect_and_compare "indoor" "${INDOOR_LABELS[@]}"

############################################
# 4. AVIA INDOOR (8 sequences)
############################################
log ""
log "======== [4/5] M3DGR AVIA INDOOR (8 seq) ========"

AVIA_INDOOR_SEQS=(indoor_Dark03 indoor_Dark04 indoor_Dynamic01 indoor_Dynamic02 indoor_Occlusion01 indoor_Occlusion02 indoor_Varying-illu01 indoor_Varying-illu02)
AVIA_INDOOR_LABELS=(aiDk03 aiDk04 aiDy01 aiDy02 aiOc01 aiOc02 aiVI01 aiVI02)

SEQ_CFGS[indoor_Dark03]="avia_indoor_seq/iDark03.yaml"
SEQ_CFGS[indoor_Dark04]="avia_indoor_seq/iDark04.yaml"
SEQ_CFGS[indoor_Dynamic01]="avia_indoor_seq/iDyn01.yaml"
SEQ_CFGS[indoor_Dynamic02]="avia_indoor_seq/iDyn02.yaml"
SEQ_CFGS[indoor_Occlusion01]="avia_indoor_seq/iOcc01.yaml"
SEQ_CFGS[indoor_Occlusion02]="avia_indoor_seq/iOcc02.yaml"
SEQ_CFGS[indoor_Varying-illu01]="avia_indoor_seq/iVI01.yaml"
SEQ_CFGS[indoor_Varying-illu02]="avia_indoor_seq/iVI02.yaml"

for i in $(seq 0 $((${#AVIA_INDOOR_SEQS[@]}-1))); do
  SEQ_OUT_LABELS[${AVIA_INDOOR_SEQS[$i]}]="${AVIA_INDOOR_LABELS[$i]}"
done

start_containers "avia_indoor" "-v ${HOST_INDOOR}:/home/euntae/Project/dataset/ros1/indoor:ro"
mkdir -p "${OUT_ROOT}/avia_indoor"

log "  Batch 1/3: aiDk03, aiDk04, aiDy01"
run_batch "avia_indoor" "run_avia_exp.sh" "${AVIA_INDOOR_SEQS[@]:0:3}"
log "  Batch 2/3: aiDy02, aiOc01, aiOc02"
run_batch "avia_indoor" "run_avia_exp.sh" "${AVIA_INDOOR_SEQS[@]:3:3}"
log "  Batch 3/3: aiVI01, aiVI02"
run_batch "avia_indoor" "run_avia_exp.sh" "${AVIA_INDOOR_SEQS[@]:6:2}"

collect_and_compare "avia_indoor" "${AVIA_INDOOR_LABELS[@]}"

############################################
# 5. NTU VIRAL (9 sequences)
############################################
log ""
log "======== [5/5] NTU VIRAL (9 seq) ========"

NTU_SEQS=(eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03)
for s in "${NTU_SEQS[@]}"; do
  SEQ_CFGS[$s]="ros1_ntu_viral.yaml"
  SEQ_OUT_LABELS[$s]="$s"
done

start_containers "ntu" "-v ${NTU_DATA}:/root/catkin_ws/data/ntu_viral:ro"
mkdir -p "${OUT_ROOT}/ntu"

log "  Batch 1/3: eee_01, eee_02, eee_03"
run_batch "ntu" "run_ntu_exp.sh" "${NTU_SEQS[@]:0:3}"
log "  Batch 2/3: nya_01, nya_02, nya_03"
run_batch "ntu" "run_ntu_exp.sh" "${NTU_SEQS[@]:3:3}"
log "  Batch 3/3: sbs_01, sbs_02, sbs_03"
run_batch "ntu" "run_ntu_exp.sh" "${NTU_SEQS[@]:6:3}"

collect_and_compare "ntu" "${NTU_SEQS[@]}"

cleanup

############################################
# FINAL SUMMARY
############################################
log ""
log "================================================================"
log "  FULL VERIFICATION COMPLETE — $(date)"
log "  Git: ${COMMIT_MSG}"
log "  Rate: ${RATE} (screening)"
log "  Total: 43 sequences across 5 datasets"
log "  Results: ${OUT_ROOT}/"
log "================================================================"
