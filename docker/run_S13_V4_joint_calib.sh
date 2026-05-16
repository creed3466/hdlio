#!/bin/bash
# run_S13_V4_joint_calib.sh — Sprint 13 V4 (handoff) / B.V3 (architect G3+G5) joint calibration grid.
#
# 48-config grid on Dark01 rate=1.0:
#   σ²_base = lidar_noise_std² where lidar_noise_std ∈ {0.01, 0.02, 0.05, 0.10}  (4 values)
#   range_inverse_ref ∈ {5, 10, 15, 25}                                          (4 values)
#   ε = anisotropic_iekf_epsilon ∈ {1e-2, 1e-3, 1e-4}                            (3 values)
#
# Pass criteria (G5):
#   (a) Dark01 ATE ≤ 0.115 m on ≥ 1 tuple
#   (b) Winner is interior-point (not at grid boundary)
#   (c) ±1-step sensitivity ≤ 0.05 m
#
# Ref: docs/research/sprint13_architecture_20260513.md §7 B.V3, §9 V4, §6 R3.
#
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_V4_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/v4.log"
CFG_DIR="${OUT_ROOT}/configs"
mkdir -p "${OUT_ROOT}" "${CFG_DIR}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
SEQ="Dark01"
IMAGE="tofslam:ros1"
TIMEOUT_S=900

# Grid axes
STDS=(0.01 0.02 0.05 0.10)
REFS=(5 10 15 25)
EPSS=(1.0e-2 1.0e-3 1.0e-4)

CONTAINERS=(tofslam_S13V4_1 tofslam_S13V4_2 tofslam_S13V4_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== S13 V4 joint calibration grid ===="
log "Grid: 4(σ²_base) × 4(range_inv_ref) × 3(ε) = 48 configs"
log "Dark01 rate=1.0 single-run per config (3 containers parallel)"

# ----- Generate 48 derived configs -----
config_idx=0
ALL_CFGS=()
ALL_TAGS=()
for std in "${STDS[@]}"; do
  for ref in "${REFS[@]}"; do
    for eps in "${EPSS[@]}"; do
      tag="s${std}_r${ref}_e${eps}"
      cfg_path="${CFG_DIR}/v4_${config_idx}_${tag}.yaml"
      # Generate config = avia_outdoor.yaml + 3-axis override + range_inverse enable
      cp src/tof_slam/config/avia_outdoor.yaml "${cfg_path}"
      cat >> "${cfg_path}" <<EOF

# ---- V4 joint calib override (config_${config_idx}: σ²_base=${std}², range_inv_ref=${ref}, ε=${eps}) ----
frontend_lidar_noise_std: ${std}
frontend_enable_range_inverse_weight: true
frontend_range_inverse_ref: ${ref}
frontend_range_inverse_power: 1.0
frontend_range_inverse_min_ratio: 0.1
frontend_anisotropic_iekf_epsilon: ${eps}
EOF
      ALL_CFGS+=("v4_${config_idx}_${tag}.yaml")
      ALL_TAGS+=("${tag}")
      config_idx=$((config_idx + 1))
    done
  done
done
log "Generated ${#ALL_CFGS[@]} configs at ${CFG_DIR}/"

# ----- Build containers (3) once and reuse -----
cleanup; sleep 1
for i in 0 1 2; do
  docker run -d --rm --init --name "${CONTAINERS[$i]}" \
    --network host --cpuset-cpus "${CPUSETS[$i]}" --memory 3g --ipc private \
    -v "$(pwd)/src:/root/catkin_ws/src" \
    -v "$(pwd)/docker:/root/catkin_ws/docker:ro" \
    -v "$(pwd)/${OUT_ROOT}:/root/catkin_ws/dump:rw" \
    -v "$(pwd)/${CFG_DIR}:/root/catkin_ws/src/tof_slam/config/v4_configs:ro" \
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

# ----- Run all 48 configs in 16 batches of 3 -----
SUMMARY="${OUT_ROOT}/v4_summary.csv"
echo "config_idx,tag,sigma_std,range_ref,epsilon,ate_rmse,ate_mean" > "${SUMMARY}"

total=${#ALL_CFGS[@]}
batch_size=3
for ((batch=0; batch<total; batch+=batch_size)); do
  log ""
  log "==== Batch $((batch/batch_size + 1))/$((total/batch_size)) (configs ${batch}-$((batch+2))) ===="
  PIDS=()
  for i in 0 1 2; do
    idx=$((batch + i))
    [ $idx -ge $total ] && continue
    cfg="${ALL_CFGS[$idx]}"
    tag="${ALL_TAGS[$idx]}"
    out_dir="dump/cfg_${idx}_${tag}"
    log "    cfg_${idx} ${tag} on port ${PORTS[$i]}..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "bash /root/catkin_ws/docker/run_avia_exp.sh v4_configs/${cfg} ${SEQ} ${out_dir} ${PORTS[$i]} ${RATE} 2>&1" \
      > "${OUT_ROOT}/cfg_${idx}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done
  # collect ATE
  for i in 0 1 2; do
    idx=$((batch + i))
    [ $idx -ge $total ] && continue
    tag="${ALL_TAGS[$idx]}"
    ate_file="${OUT_ROOT}/cfg_${idx}_${tag}/ate_result.txt"
    if [ -f "$ate_file" ]; then
      rmse=$(grep -oE "^rmse: [0-9.]+" "$ate_file" | awk '{print $2}')
      mean=$(grep -oE "^mean: [0-9.]+" "$ate_file" | awk '{print $2}')
      # extract grid axes from tag
      std=$(echo $tag | sed -E 's/s([0-9.]+)_.*/\1/')
      ref=$(echo $tag | sed -E 's/.*_r([0-9]+)_.*/\1/')
      eps=$(echo $tag | sed -E 's/.*_e([0-9.e-]+)/\1/')
      echo "${idx},${tag},${std},${ref},${eps},${rmse},${mean}" >> "${SUMMARY}"
      log "    cfg_${idx} ATE=${rmse}"
    else
      log "    cfg_${idx} MISSING ATE"
      echo "${idx},${tag},,,,," >> "${SUMMARY}"
    fi
  done
done

log ""
log "==== Grid done. Summary at ${SUMMARY} ===="
log "Top 5 configs by ATE:"
sort -t',' -k6 -g "${SUMMARY}" | grep -v "^config_idx" | head -5 | tee -a "${LOG}"

# Identify winner + G5 check (manual interpretation expected)
WINNER=$(sort -t',' -k6 -g "${SUMMARY}" | grep -v "^config_idx" | head -1)
WINNER_RMSE=$(echo "$WINNER" | cut -d',' -f6)
log ""
log "Winner: ${WINNER}"
log "Winner ATE: ${WINNER_RMSE}"
if python3 -c "exit(0 if ${WINNER_RMSE} <= 0.115 else 1)" 2>/dev/null; then
  log "G5(a) PASS — winner ≤ 0.115m"
else
  log "G5(a) FAIL — winner > 0.115m → Rule 16 candidate"
fi
log ""
log "==== S13 V4 DONE — $(date) ===="
log "Next: manual G5(b) interior-point check + G5(c) ±1-step sensitivity"
