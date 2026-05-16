#!/usr/bin/env python3
"""analyze_debug_multirun.py — bisect H1 (startup race) vs H2 (runtime FP).

Inputs (per run directory):
    debug_imu.csv    — first N IMU samples consumed, with phase tag
    debug_state.csv  — state snapshots at frame 0 (init) + frames 1..6 before/after
    diagnostics.csv  — full run diagnostics (used for frame-1 class tagging)
    ate_result.txt   — final RMSE

Usage:
    python3 analyze_debug_multirun.py <LABEL_DIR> [--seq Dark01]

Output:
    Per-run fingerprints + classification
    Verdict on H1 and H2
"""
import argparse
import csv
import hashlib
import os
import sys
from pathlib import Path


def sha(s: str) -> str:
    return hashlib.sha256(s.encode()).hexdigest()[:12]


def read_imu_prefix(path: Path, n_pre_init: int = 120):
    """Return (pre_init_fp, post_init_fp, n_pre, n_post, first_ts)."""
    if not path.exists():
        return None
    pre, post = [], []
    first_ts = None
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            if first_ts is None:
                first_ts = row["timestamp"]
            line = ",".join(
                [row["timestamp"], row["gx"], row["gy"], row["gz"],
                 row["ax"], row["ay"], row["az"]])
            if row["phase"] == "pre_init":
                pre.append(line)
            else:
                post.append(line)
    pre_fp = sha("\n".join(pre[:n_pre_init]))
    post_fp = sha("\n".join(post[:30]))
    return pre_fp, post_fp, len(pre), len(post), first_ts


def read_state(path: Path):
    """Return dict: {frame_label -> fingerprint}.

    Keys: 'init', 'f1_before', 'f1_after', 'f2_before', 'f2_after', ...
    """
    if not path.exists():
        return None
    out = {}
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = row["frame"]
            phase = row["phase"]
            key = "init" if phase == "init" else f"f{frame}_{phase}"
            # Full state fingerprint
            vals = [row["timestamp"]] + [row[c] for c in
                ("px","py","pz","qw","qx","qy","qz","vx","vy","vz",
                 "gx","gy","gz","bgx","bgy","bgz","bax","bay","baz")]
            out[key] = sha(",".join(vals))
    return out


def read_ate(path: Path):
    if not path.exists():
        return None
    with path.open() as f:
        for line in f:
            if line.startswith("rmse:"):
                return float(line.split()[1])
    return None


def read_frame1_diag(path: Path):
    """Return (corrs, l0, l1, iters, vel) from diagnostics frame 1 row."""
    if not path.exists():
        return None
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["frame"] == "1":
                return (row["corrs"], row["l0_count"], row["l1_count"],
                        row["iekf_iters"], row["vel_norm"])
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("label_dir")
    p.add_argument("--seq", default="Dark01")
    args = p.parse_args()

    base = Path(args.label_dir) / args.seq
    if not base.is_dir():
        print(f"ERROR: {base} does not exist")
        sys.exit(1)

    runs = sorted([d for d in base.iterdir() if d.is_dir() and d.name.startswith("run")],
                  key=lambda d: int(d.name[3:]))

    print(f"Found {len(runs)} runs in {base}")
    print()

    rows = []
    for rd in runs:
        rn = int(rd.name[3:])
        imu = read_imu_prefix(rd / "debug_imu.csv")
        st = read_state(rd / "debug_state.csv")
        ate = read_ate(rd / "ate_result.txt")
        diag = read_frame1_diag(rd / "diagnostics.csv")
        rows.append({
            "run": rn,
            "imu_pre_fp": imu[0] if imu else None,
            "imu_post_fp": imu[1] if imu else None,
            "n_pre": imu[2] if imu else 0,
            "n_post": imu[3] if imu else 0,
            "first_imu_ts": imu[4] if imu else None,
            "init_fp": st.get("init") if st else None,
            "f1_before_fp": st.get("f1_before") if st else None,
            "f1_after_fp": st.get("f1_after") if st else None,
            "f2_before_fp": st.get("f2_before") if st else None,
            "f2_after_fp": st.get("f2_after") if st else None,
            "ate": ate,
            "diag_f1": diag,
        })

    def classify(key: str):
        classes = {}
        for r in rows:
            fp = r.get(key)
            if fp is None:
                continue
            classes.setdefault(fp, []).append(r["run"])
        return classes

    print("=" * 72)
    print("PER-RUN FINGERPRINTS")
    print("=" * 72)
    print(f"{'run':>4} {'pre-init IMU':>14} {'init state':>14} "
          f"{'f1_before':>14} {'f1_after':>14} {'ATE':>10}")
    for r in rows:
        print(f"{r['run']:>4} {str(r['imu_pre_fp']):>14} {str(r['init_fp']):>14} "
              f"{str(r['f1_before_fp']):>14} {str(r['f1_after_fp']):>14} "
              f"{r['ate'] if r['ate'] else 'N/A':>10}")

    print()
    print("=" * 72)
    print("CLASS ANALYSIS")
    print("=" * 72)

    imu_cls = classify("imu_pre_fp")
    init_cls = classify("init_fp")
    f1b_cls = classify("f1_before_fp")
    f1a_cls = classify("f1_after_fp")
    f2b_cls = classify("f2_before_fp")

    def show(name, cls):
        print(f"\n{name}: {len(cls)} class(es)")
        for fp, runs in sorted(cls.items(), key=lambda kv: min(kv[1])):
            print(f"  {fp} -> runs {runs}")

    show("Pre-init IMU window", imu_cls)
    show("Gravity-init state", init_cls)
    show("Frame 1 BEFORE state", f1b_cls)
    show("Frame 1 AFTER state", f1a_cls)
    show("Frame 2 BEFORE state", f2b_cls)

    print()
    print("=" * 72)
    print("VERDICT")
    print("=" * 72)

    n_imu_cls = len(imu_cls)
    n_init_cls = len(init_cls)
    n_f1a_cls = len(f1a_cls)
    n_f2b_cls = len(f2b_cls)

    print(f"- pre-init IMU classes : {n_imu_cls}")
    print(f"- gravity init classes : {n_init_cls}")
    print(f"- frame 1 after classes: {n_f1a_cls}")
    print(f"- frame 2 before class : {n_f2b_cls}")
    print()

    h1_startup_race = (n_imu_cls > 1) or (n_init_cls > 1)
    # H2 = same init but different f1_after, OR same f1_after but different f2_before
    h2_runtime_fp = False
    if n_init_cls >= 1:
        # Within each init class, check if f1_after is consistent
        for fp, runs in init_cls.items():
            f1_fps = {r["f1_after_fp"] for r in rows if r["run"] in runs}
            if len(f1_fps) > 1:
                h2_runtime_fp = True
                break
    # Also: same f1_after but f2_before differs (IMU propagation between frames)
    if not h2_runtime_fp and n_f1a_cls >= 1:
        for fp, runs in f1a_cls.items():
            f2_fps = {r["f2_before_fp"] for r in rows if r["run"] in runs}
            f2_fps.discard(None)
            if len(f2_fps) > 1:
                h2_runtime_fp = True
                break

    print(f"H1 (startup race, gravity-init window differs): "
          f"{'CONFIRMED' if h1_startup_race else 'NOT observed'}")
    print(f"H2 (runtime FP non-determinism with same inputs): "
          f"{'CONFIRMED' if h2_runtime_fp else 'NOT observed'}")

    print()
    if h1_startup_race and h2_runtime_fp:
        print("CONCLUSION: BOTH H1 and H2 are active.")
        print("  -> Need fixes on both startup sync AND runtime FP ordering.")
    elif h1_startup_race:
        print("CONCLUSION: Only H1 (startup race) is active.")
        print("  -> Fix: subscriber-ready handshake before rosbag play.")
    elif h2_runtime_fp:
        print("CONCLUSION: Only H2 (runtime FP) is active.")
        print("  -> Fix: deterministic reductions in Jacobian/correspondence path.")
    else:
        print("CONCLUSION: All classes collapsed to 1 -> fully deterministic.")
        print("  -> The -r 1.0 + fixes achieve bitwise reproducibility on Dark01.")

    # ATE stats
    ates = [r["ate"] for r in rows if r["ate"] is not None]
    if ates:
        import statistics
        m = statistics.mean(ates)
        sd = statistics.stdev(ates) if len(ates) > 1 else 0
        cv = sd / m * 100 if m else 0
        print()
        print(f"ATE RMSE: n={len(ates)} mean={m:.6f} stdev={sd:.6f} CV={cv:.2f}%")
        print(f"  min={min(ates):.6f}  max={max(ates):.6f}  range={max(ates)-min(ates):.6f}")


if __name__ == "__main__":
    main()
