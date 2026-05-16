#!/bin/bash
# run_canonical_eval_resume.sh — Resume canonical eval from Mid360 onward
# Avia (9/9) already complete in dump/paper_canonical/avia/
set -e
cd "$(dirname "$0")/.."

LABEL="paper_canonical"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/eval.log"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
GT_AVIA="/home/euntae/Project/dataset/ros1/m3dgr/ground_truth"
NTU_DATA="/home/euntae/Project/dataset/ros1/ntu_viral"

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
  # Extra: kill any lingering roscore on our ports
  for p in "${PORTS[@]}"; do
    fuser -k ${p}/tcp 2>/dev/null || true
  done
}
trap cleanup EXIT

start_and_build() {
  local dataset=$1
  local extra_vols="$2"
  cleanup
  sleep 3  # Extra wait to ensure ports are fully released
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
  sleep 3
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

copy_configs() {
  local dataset=$1
  shift
  local seqs=("$@")
  local cfg_dir="${OUT_ROOT}/${dataset}/_configs"
  mkdir -p "${cfg_dir}"
  for seq in "${seqs[@]}"; do
    local cfg="${SEQ_CFGS[$seq]}"
    local src="src/tof_slam/config/${cfg}"
    if [ -f "$src" ]; then
      cp "$src" "${cfg_dir}/$(basename "$cfg")"
    fi
  done
}

declare -A SEQ_CFGS

log ""
log "================================================================"
log "  CANONICAL EVAL RESUME — $(date)"
log "  Avia already complete (9/9, mean=0.4007m)"
log "  Resuming: Mid360 → NTU"
log "================================================================"

############################################
# 2. MID360 (9 sequences)
############################################
log ""
log "======== [2/3] M3DGR MID360 (9 seq, rate=${RATE}) ========"

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
# 3. NTU VIRAL (9 sequences)
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

log ""
log "================================================================"
log "  CANONICAL EVAL RESUME — COMPLETE"
log "  $(date)"
log "================================================================"
log ""
log "  Results in: ${OUT_ROOT}/"
