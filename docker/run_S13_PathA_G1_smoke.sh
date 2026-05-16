#!/bin/bash
# run_S13_PathA_G1_smoke.sh — S13 Path A G1 Smoke A telemetry runner.
#
# Architect §7 B.V1: DG-A telemetry on Dark01/Dark02/VI03/Dyn04 unified rate=3.0 parallel ×3.
# Purpose: measure ρ_L2 distribution → bake ρ_ref_avia.
# Gate criteria (architect §7 G1):
#   (a) ρ_L1 disparity ≥ 50× (max(ρ_L1)/min(ρ_L1) over frames)
#   (b) cos²θ_12 std ≥ 0.05
#   (c) γ discriminative — verified via DG-A formula post-bake
#   (d) record ρ_ref_avia_provisional
#
# Configs: unified avia_outdoor.yaml with V0p activation (P1 + DG-A telemetry, NO range_inverse).
#
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_PathA_G1_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/g1.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="3.0"     # G1 screening rate (not canonical 1.0)
CFG="avia_outdoor.yaml"
IMAGE="tofslam:ros1"
TIMEOUT_S=600

SEQS=(Dark01 Dark02 Varying-illu03 Dynamic04)

CONTAINERS=(tofslam_PA_G1_1 tofslam_PA_G1_2 tofslam_PA_G1_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== S13 Path A — G1 Smoke A telemetry ===="
log "4 seqs × 3 parallel @ rate=${RATE}, unified ${CFG} (V0p commit)"
log "Measures ρ_L1/L2/full + cos² via diagnostics CSV"

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

# 4 seqs in batches of 3 (4 seqs → 2 batches: 3+1)
TOTAL=${#SEQS[@]}
for ((batch=0; batch<TOTAL; batch+=3)); do
  log ""
  log "==== Batch $((batch/3 + 1)) (seqs $((batch+1))-$((batch+3 < TOTAL ? batch+3 : TOTAL))) ===="
  PIDS=()
  for i in 0 1 2; do
    idx=$((batch + i))
    [ $idx -ge $TOTAL ] && continue
    seq="${SEQS[$idx]}"
    log "    ${seq} on port ${PORTS[$i]}..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "bash /root/catkin_ws/docker/run_avia_exp.sh ${CFG} ${seq} dump/${seq} ${PORTS[$i]} ${RATE} 2>&1" \
      > "${OUT_ROOT}/${seq}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done
done

log ""
log "==== Telemetry analysis ===="

# Run Python analyzer
python3 - <<'PYEOF' "${OUT_ROOT}" "${LOG}" "${SEQS[@]}"
import sys, csv, statistics, json, os

out_root = sys.argv[1]
log_path = sys.argv[2]
seqs = sys.argv[3:]

def log(msg):
    print(msg)
    with open(log_path, 'a') as fp:
        fp.write(f"[telemetry] {msg}\n")

summary = {}
all_rho_l2 = []

for seq in seqs:
    f = f"{out_root}/{seq}/diagnostics.csv"
    if not os.path.exists(f):
        log(f"  {seq}: MISSING diagnostics.csv")
        continue
    rho_l1, rho_l2, rho_full, cos_l1_l2 = [], [], [], []
    with open(f) as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            try:
                rl1 = float(row['dg_a_rho_l1'])
                rl2 = float(row['dg_a_rho_l2'])
                rf = float(row['dg_a_rho_full'])
                cl12 = float(row['dg_a_cos_l1_l2'])
                if rl1 > 0 or rl2 > 0:  # skip frames before DG-A active
                    rho_l1.append(rl1); rho_l2.append(rl2); rho_full.append(rf)
                    cos_l1_l2.append(cl12)
            except (ValueError, KeyError):
                continue
    if not rho_l2:
        log(f"  {seq}: NO non-zero DG-A telemetry")
        continue
    s = {
        'n_frames_active': len(rho_l2),
        'rho_l1_mean': statistics.mean(rho_l1),
        'rho_l1_median': statistics.median(rho_l1),
        'rho_l1_max_min_ratio': max(rho_l1)/max(min(rho_l1), 1e-10),
        'rho_l2_mean': statistics.mean(rho_l2),
        'rho_l2_median': statistics.median(rho_l2),
        'rho_l2_p30': sorted(rho_l2)[int(0.3*len(rho_l2))],
        'rho_l2_p70': sorted(rho_l2)[int(0.7*len(rho_l2))],
        'rho_full_mean': statistics.mean(rho_full),
        'cos_l1_l2_mean': statistics.mean(cos_l1_l2),
        'cos_l1_l2_std': statistics.stdev(cos_l1_l2) if len(cos_l1_l2) > 1 else 0,
        'l1_l2_ratio': statistics.mean(rho_l1) / max(statistics.mean(rho_l2), 1e-10),
    }
    summary[seq] = s
    all_rho_l2.extend(rho_l2)
    log(f"  {seq}: n={s['n_frames_active']}")
    log(f"    ρ_L1: mean={s['rho_l1_mean']:.5e} median={s['rho_l1_median']:.5e} max/min={s['rho_l1_max_min_ratio']:.1f}x")
    log(f"    ρ_L2: mean={s['rho_l2_mean']:.5e} median={s['rho_l2_median']:.5e} p30={s['rho_l2_p30']:.5e} p70={s['rho_l2_p70']:.5e}")
    log(f"    cos²θ_12: mean={s['cos_l1_l2_mean']:.4f} std={s['cos_l1_l2_std']:.4f}")
    log(f"    L1/L2 ratio: {s['l1_l2_ratio']:.1f}x")

# Gate evaluation
log("")
log("==== G1 Gate Evaluation ====")
# (a) ρ_L1 disparity ≥ 50× across seqs (interpret: max-to-min ratio)
all_rho_l1_mean = [s['rho_l1_mean'] for s in summary.values()]
if all_rho_l1_mean:
    disparity = max(all_rho_l1_mean) / max(min(all_rho_l1_mean), 1e-10)
    gate_a = disparity >= 50
    log(f"  G1(a) ρ_L1 cross-seq disparity: {disparity:.1f}x {'PASS' if gate_a else 'FAIL'} (≥50x)")
# (b) cos² std ≥ 0.05 — interpret per-seq
for seq, s in summary.items():
    gate_b = s['cos_l1_l2_std'] >= 0.05
    log(f"  G1(b) {seq} cos² std: {s['cos_l1_l2_std']:.4f} {'PASS' if gate_b else 'FAIL'} (≥0.05)")

# (d) ρ_ref_avia bake candidates
if all_rho_l2:
    bake_median = statistics.median(all_rho_l2)
    bake_p30 = sorted(all_rho_l2)[int(0.3*len(all_rho_l2))]
    bake_p70 = sorted(all_rho_l2)[int(0.7*len(all_rho_l2))]
    log("")
    log("==== ρ_ref_avia bake candidates (across all 4 seqs) ====")
    log(f"  median (P50): {bake_median:.5e}  ← recommended default")
    log(f"  P30:          {bake_p30:.5e}     ← conservative (more L2 contribution)")
    log(f"  P70:          {bake_p70:.5e}     ← aggressive (less L2 contribution)")
    log("")
    log(f"Per-seq ρ_L2 median:")
    for seq, s in summary.items():
        log(f"  {seq}: {s['rho_l2_median']:.5e}")

# Save JSON
json_out = {
    'summary': summary,
    'bake_candidates': {
        'median': statistics.median(all_rho_l2) if all_rho_l2 else None,
        'p30': sorted(all_rho_l2)[int(0.3*len(all_rho_l2))] if all_rho_l2 else None,
        'p70': sorted(all_rho_l2)[int(0.7*len(all_rho_l2))] if all_rho_l2 else None,
    },
}
with open(f"{out_root}/g1_summary.json", 'w') as fp:
    json.dump(json_out, fp, indent=2)
log(f"")
log(f"Saved: {out_root}/g1_summary.json")
PYEOF

log ""
log "==== S13 Path A G1 DONE — $(date) ===="
