#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

set -euo pipefail

RATE="${RATE:-4.0}"
ENABLE_LC="${ENABLE_LC:-false}"
DATA_ROOT="${DATA_ROOT:-/root/ros2_ws/data/nrx/nrx_chj_0331}"
OUT_BASE="${OUT_BASE:-/root/ros2_ws/dump/2026-04-02_reval_rate4_nolc}"
CONFIG="${CONFIG:-/root/ros2_ws/install/tof_slam/share/tof_slam/config/nrx_chj.yaml}"
PLAY_TIMEOUT_SEC="${PLAY_TIMEOUT_SEC:-180}"
SETTLE_TIMEOUT_SEC="${SETTLE_TIMEOUT_SEC:-60}"
SEQ_LIST="${SEQ_LIST:-1 2 3 4 5 6 7 8 9 10}"
NODE_EXTRA_ARGS="${NODE_EXTRA_ARGS:-}"

cleanup() {
  pkill -f tofslam_node 2>/dev/null || true
  pkill -f static_transform_publisher 2>/dev/null || true
  pkill -f "ros2 bag play" 2>/dev/null || true
}

wait_for_traj_settle() {
  local traj_csv="$1"
  local prev=-1
  local stable=0
  local cur=0
  local i=0

  for ((i = 0; i < SETTLE_TIMEOUT_SEC; ++i)); do
    if [[ -f "$traj_csv" ]]; then
      cur=$(wc -l < "$traj_csv")
    else
      cur=0
    fi

    if [[ "$cur" -eq "$prev" ]]; then
      stable=$((stable + 1))
    else
      stable=0
      prev="$cur"
    fi

    if [[ "$cur" -gt 10 && "$stable" -ge 5 ]]; then
      return 0
    fi
    sleep 1
  done

  return 1
}

summarize_seq() {
  local seq_id="$1"
  local traj_csv="$2"
  local diag_csv="$3"
  local summary_csv="$4"

  python3 - "$seq_id" "$traj_csv" "$diag_csv" "$summary_csv" <<'PY'
import csv
import math
import pathlib
import sys

seq_id, traj_csv, diag_csv, summary_csv = sys.argv[1:5]
traj_path = pathlib.Path(traj_csv)
diag_path = pathlib.Path(diag_csv)
summary_path = pathlib.Path(summary_csv)

rows = list(csv.DictReader(traj_path.open()))
if len(rows) < 2:
    raise SystemExit(f"{seq_id}: trajectory too short")

def col_float(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default

xs = [col_float(r, "tx") for r in rows]
ys = [col_float(r, "ty") for r in rows]
ox = [col_float(r, "odom_tx") for r in rows]
oy = [col_float(r, "odom_ty") for r in rows]

path_m = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))
s2e_cm = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) * 100.0
odom_s2e_cm = math.hypot(ox[-1] - ox[0], oy[-1] - oy[0]) * 100.0

diag_rows = list(csv.DictReader(diag_path.open())) if diag_path.exists() else []
trust_vals = [col_float(r, "trust_score") for r in diag_rows if "trust_score" in r]
accept_vals = [col_float(r, "accepted_lidar_update") for r in diag_rows if "accepted_lidar_update" in r]
blend_vals = [col_float(r, "blended_lidar_update") for r in diag_rows if "blended_lidar_update" in r]
commit_vals = [col_float(r, "map_commit_allowed") for r in diag_rows if "map_commit_allowed" in r]

def mean(vals):
    return sum(vals) / len(vals) if vals else float("nan")

with summary_path.open("a", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        seq_id,
        f"{s2e_cm:.3f}",
        f"{path_m:.3f}",
        len(rows),
        f"{odom_s2e_cm:.3f}",
        "" if math.isnan(mean(trust_vals)) else f"{mean(trust_vals):.4f}",
        "" if math.isnan(mean(accept_vals)) else f"{mean(accept_vals):.4f}",
        "" if math.isnan(mean(blend_vals)) else f"{mean(blend_vals):.4f}",
        "" if math.isnan(mean(commit_vals)) else f"{mean(commit_vals):.4f}",
    ])

print(
    f"{seq_id}: frames={len(rows)} path_m={path_m:.3f} "
    f"s2e_cm={s2e_cm:.3f} odom_s2e_cm={odom_s2e_cm:.3f}"
)
PY
}

run_seq() {
  local idx="$1"
  local seq_name
  local out_dir
  local bag_dir
  local tf_pid
  local node_pid

  seq_name=$(printf "w9_F4_0331_%d" "$idx")
  out_dir=$(printf "%s/seq%02d" "$OUT_BASE" "$idx")
  bag_dir=$(find "$DATA_ROOT/$seq_name" -mindepth 1 -maxdepth 1 -type d | sort | head -n 1)

  if [[ -z "$bag_dir" ]]; then
    echo "[ERROR] bag dir missing for $seq_name" >&2
    return 1
  fi

  rm -rf "$out_dir"
  mkdir -p "$out_dir"

  cleanup
  sleep 1

  ros2 run tf2_ros static_transform_publisher \
    0.169950 -0.000090 0.060050 -0.500040 0.499920 -0.500022 0.500019 \
    base_link front_spot_pcl >"$out_dir/tf.log" 2>&1 &
  tf_pid=$!

  ros2 run tof_slam tofslam_node --ros-args \
    --params-file "$CONFIG" \
    -p dump_path:="$out_dir" \
    -p trajectory_csv_path:="$out_dir/traj.csv" \
    -p use_sim_time:=true \
    -p enable_loop_closure:="$ENABLE_LC" \
    -p lc_enable_debug_log:=false \
    $NODE_EXTRA_ARGS >"$out_dir/node.log" 2>&1 &
  node_pid=$!

  sleep 3
  if ! kill -0 "$node_pid" 2>/dev/null; then
    echo "[ERROR] node crashed for $seq_name" >&2
    tail -n 40 "$out_dir/node.log" >&2 || true
    cleanup
    return 1
  fi

  echo "[RUN] $seq_name"
  timeout "${PLAY_TIMEOUT_SEC}s" ros2 bag play "$bag_dir" \
    --rate "$RATE" \
    --clock \
    --read-ahead-queue-size 1000 >"$out_dir/bag.log" 2>&1 || true

  wait_for_traj_settle "$out_dir/traj.csv" || true

  kill "$node_pid" 2>/dev/null || true
  wait "$node_pid" 2>/dev/null || true
  kill "$tf_pid" 2>/dev/null || true
  wait "$tf_pid" 2>/dev/null || true

  if [[ ! -f "$out_dir/traj.csv" || ! -f "$out_dir/traj_diag.csv" ]]; then
    echo "[ERROR] missing output for $seq_name" >&2
    return 1
  fi

  summarize_seq "seq$(printf '%02d' "$idx")" \
    "$out_dir/traj.csv" \
    "$out_dir/traj_diag.csv" \
    "$OUT_BASE/summary.csv"
}

trap cleanup EXIT

cleanup
rm -rf "$OUT_BASE"
mkdir -p "$OUT_BASE"

cat > "$OUT_BASE/summary.csv" <<'EOF'
seq,s2e_cm,path_m,frames,odom_s2e_cm,avg_trust,accept_rate,blend_rate,commit_rate
EOF

for idx in $SEQ_LIST; do
  run_seq "$idx"
done

echo "[DONE] results in $OUT_BASE"
