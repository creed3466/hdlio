#!/bin/bash
# run_S13_PathB_V3_extended.sh — S13 Path B V3 extended runner.
#
# Architect §4.2: 9-seq × 3-run rate=1.0 on unified avia_outdoor.yaml with
# router=ON. Verifies:
#  - Per-seq CV=0% on class_id (R-A LOCK stability) and trajectory (I-3)
#  - Class assignment matches expected mapping (CLEAN_DENSE for DK01, etc.)
#  - Per-seq ATE at rate=1.0 (deterministic-grade, no rate=3.0 screening tier)
#
# Subsumes B.V0 (V5-DK01-baseline, embedded in Dark01 run) and B.V4
# (V4-PathB-Dark01-sanity, embedded in Dark01 result) per R0.5 §3.
#
# Wallclock: 9 seqs × 3 runs × ~4 min/run / 3 parallel slots ≈ 36 batches → ~80 min.
#
set -e
cd "$(dirname "$0")/.."

LABEL="${1:-S13_PathB_V3_$(date +%Y%m%d_%H%M)}"
OUT_ROOT="dump/${LABEL}"
LOG="${OUT_ROOT}/v3.log"
mkdir -p "${OUT_ROOT}"

HOST_SURFEL="/home/euntae/Project/dataset/ros1/surfel_data"
HOST_INDOOR="/home/euntae/Project/dataset/ros1/indoor"
RATE="1.0"
CFG="avia_outdoor.yaml"
IMAGE="tofslam:ros1"
TIMEOUT_S=900

SEQS=(Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05)

CONTAINERS=(tofslam_PB_V3_1 tofslam_PB_V3_2 tofslam_PB_V3_3)
CPUSETS=("0-3" "4-7" "8-11")
PORTS=(11311 11312 11313)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG}"; }
cleanup() { for c in "${CONTAINERS[@]}"; do docker rm -f "$c" 2>/dev/null || true; done; }
trap cleanup EXIT

log "==== S13 Path B V3 extended — 9-seq × 3-run rate=${RATE} unified ${CFG} ===="

# Build once
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

# Per-seq batch: 3 runs of SAME seq in 3 parallel containers
run_seq_batch() {
  local seq="$1"
  log ""
  log "==== ${seq} × 3 runs ===="
  local PIDS=()
  for i in 0 1 2; do
    out_dir="dump/${seq}_r${i}"
    log "    ${seq} r${i} on port ${PORTS[$i]}..."
    timeout "${TIMEOUT_S}" docker exec "${CONTAINERS[$i]}" bash -lc \
      "bash /root/catkin_ws/docker/run_avia_exp.sh ${CFG} ${seq} ${out_dir} ${PORTS[$i]} ${RATE} 2>&1" \
      > "${OUT_ROOT}/${seq}_r${i}_stdout.log" 2>&1 &
    PIDS+=($!)
  done
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done
  for c in "${CONTAINERS[@]}"; do
    docker exec "$c" bash -lc "killall -9 tofslam_node rosbag rosout rosmaster roscore 2>/dev/null || true" 2>/dev/null || true
  done
}

# 9 seqs in series, each does 3 parallel runs
for seq in "${SEQS[@]}"; do
  run_seq_batch "$seq"
done

# Analysis
log ""
log "==== Analysis ===="

python3 - <<'PYEOF' "${OUT_ROOT}" "${LOG}" "${SEQS[@]}"
import sys, os, csv, statistics

out_root = sys.argv[1]
log_path = sys.argv[2]
seqs = sys.argv[3:]

# Expected class mapping (architect §4.2 / R0.4 §4.2)
EXPECTED = {
    'Dark01':         'CLEAN_DENSE',
    'Dark02':         'OUTDOOR_DRIFT',
    'Dynamic03':      'CLEAN_OUT',
    'Dynamic04':      'DYNAMIC',
    'Occlusion03':    'OCCLUSION_PNCG',
    'Occlusion04':    'OCCLUSION',
    'Varying-illu03': 'CLASS_D',
    'Varying-illu04': 'HIGH_COS2',
    'Varying-illu05': 'CORRIDOR',
}

def log(msg):
    print(msg)
    with open(log_path, 'a') as fp:
        fp.write(f"[analysis] {msg}\n")

def extract_class_from_log(path):
    """Find 'STAGE_B LOCK frame=81 class=<CLASS>' or 'STAGE_A LOCK frame=2 class=<CLASS>'."""
    if not os.path.exists(path):
        return None
    with open(path) as fp:
        for line in fp:
            if 'STAGE_B LOCK frame=81' in line or 'STAGE_A LOCK frame=2' in line:
                # extract class= field
                m = line.split('class=')
                if len(m) >= 2:
                    cls = m[1].split()[0].strip()
                    return cls
    return None

ate_vals = {}
class_assignments = {}

for seq in seqs:
    ates = []
    classes = []
    for r in range(3):
        ate_file = f"{out_root}/{seq}_r{r}/ate_result.txt"
        stdout_log = f"{out_root}/{seq}_r{r}_stdout.log"
        # ATE
        ate = None
        if os.path.exists(ate_file):
            with open(ate_file) as fp:
                for line in fp:
                    if line.startswith('rmse:'):
                        ate = float(line.split()[1])
                        break
        ates.append(ate)
        # Class
        cls = extract_class_from_log(stdout_log)
        classes.append(cls)
    ate_vals[seq] = ates
    class_assignments[seq] = classes

# CV=0% on trajectory
log("")
log("=== CV=0% (trajectory bit-identity) ===")
for seq in seqs:
    a = [x for x in ate_vals[seq] if x is not None]
    if len(a) < 3:
        log(f"  {seq}: INCOMPLETE ({len(a)}/3 runs)")
        continue
    if len(set(a)) == 1:
        log(f"  {seq}: CV=0%% ✓ all 3 runs = {a[0]:.15f}")
    else:
        cv = 100.0 * statistics.stdev(a) / statistics.mean(a)
        log(f"  {seq}: CV={cv:.6f}%% ✗ values={a}")

# Class assignment plausibility + CV
log("")
log("=== Class assignment (per-run + plausibility) ===")
class_pass = 0
for seq in seqs:
    cls = class_assignments[seq]
    expected = EXPECTED.get(seq, '?')
    cls_set = set(c for c in cls if c is not None)
    if len(cls_set) == 0:
        log(f"  {seq}: NO classifier log line found (gate FAIL)")
    elif len(cls_set) > 1:
        log(f"  {seq}: NON-DETERMINISTIC class assignment {cls} (gate FAIL)")
    else:
        actual = list(cls_set)[0]
        match = '✓' if actual == expected else '✗'
        log(f"  {seq}: lock_class={actual}  expected={expected}  {match}")
        if actual == expected:
            class_pass += 1

log("")
log(f"=== Plausibility: {class_pass}/{len(seqs)} match expected ===")

# Per-seq ATE summary (rate=1.0)
log("")
log("=== Per-seq ATE (rate=1.0, run-0) ===")
total = 0.0
n = 0
for seq in seqs:
    a = ate_vals[seq][0]
    if a is not None:
        log(f"  {seq}: {a:.6f}")
        total += a
        n += 1
    else:
        log(f"  {seq}: MISSING")
if n > 0:
    mean = total / n
    log("")
    log(f"=== 9-seq mean (rate=1.0): {mean:.6f} m ===")
    log("    HARD gate (R0.5 §4.2): ≤ 0.314 m")
    log("    PASS-clean: ≤ 0.310 m")
    log("    Falsification: > 0.391 m")

    if mean <= 0.310:
        log("  Verdict: PASS-clean ✓✓✓ proceed to V6 canonical")
    elif mean <= 0.314:
        log("  Verdict: PASS-tight — analyze per-seq drift for graduated abort decision")
    elif mean <= 0.391:
        log("  Verdict: HARD-ABORT (Rule 16)")
    else:
        log("  Verdict: F1 FIRED — Rule 16")

# Save summary JSON
import json
summary = {
    'seqs': seqs,
    'expected_classes': EXPECTED,
    'actual_classes': class_assignments,
    'ate_per_run': ate_vals,
    'mean_ate_run0': sum(ate_vals[s][0] for s in seqs if ate_vals[s][0]) / sum(1 for s in seqs if ate_vals[s][0]),
}
with open(f"{out_root}/v3_summary.json", 'w') as fp:
    json.dump(summary, fp, indent=2, default=str)
log(f"")
log(f"Saved: {out_root}/v3_summary.json")
PYEOF

log ""
log "==== S13 Path B V3 DONE — $(date) ===="
