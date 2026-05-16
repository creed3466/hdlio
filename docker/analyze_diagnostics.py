#!/usr/bin/env python3
"""Analyze TofSLAM diagnostic timing CSV for yaw observability."""
import pandas as pd
import sys
import numpy as np

def analyze(csv_path, seq_name):
    df = pd.read_csv(csv_path)
    print(f"\n=== {seq_name} Diagnostic Analysis ===")
    print(f"Total frames: {len(df)}")
    print(f"Mean n_corrs: {df['n_corrs'].mean():.0f}")
    print(f"Min n_corrs: {df['n_corrs'].min()}")
    print(f"Frames with n_degen > 0: {(df['n_degen'] > 0).sum()}")
    print(f"Mean res_rms: {df['res_rms'].mean():.5f}")

    # Eigenvalue analysis (if available)
    if 'eig0' in df.columns:
        print(f"\n--- Eigenvalue Statistics ---")
        for i in range(6):
            col = f'eig{i}'
            print(f"  eig{i}: mean={df[col].mean():.1f}, min={df[col].min():.1f}, "
                  f"max={df[col].max():.1f}, std={df[col].std():.1f}")

        # Yaw-related: eig0 is the smallest eigenvalue
        # Check ratio eig0/eig5 (smallest/largest)
        ratio = df['eig0'] / df['eig5'].clip(lower=1e-6)
        print(f"\n--- Yaw Observability (eig0/eig5 ratio) ---")
        print(f"  Mean ratio: {ratio.mean():.6f}")
        print(f"  Min ratio: {ratio.min():.6f}")
        print(f"  Frames with ratio < 0.001: {(ratio < 0.001).sum()}")
        print(f"  Frames with ratio < 0.01: {(ratio < 0.01).sum()}")

        # Time-windowed analysis for sbs_03 critical segments
        if 'timestamp' in df.columns:
            segments = [
                (25, 45, "First drift onset"),
                (80, 110, "Drift recovery"),
                (190, 250, "Worst drift segment"),
                (285, 295, "Sharp drift spike"),
                (315, 342, "Final unrecovered drift"),
            ]
            t0 = df['timestamp'].iloc[0]
            elapsed = df['timestamp'] - t0

            print(f"\n--- Segment Analysis ---")
            for t_start, t_end, label in segments:
                mask = (elapsed >= t_start) & (elapsed <= t_end)
                if mask.sum() == 0:
                    continue
                seg = df[mask]
                seg_ratio = seg['eig0'] / seg['eig5'].clip(lower=1e-6)
                print(f"\n  [{t_start}-{t_end}s] {label} ({mask.sum()} frames)")
                print(f"    eig0: mean={seg['eig0'].mean():.1f}, min={seg['eig0'].min():.1f}")
                print(f"    eig5: mean={seg['eig5'].mean():.1f}")
                print(f"    ratio: mean={seg_ratio.mean():.6f}, min={seg_ratio.min():.6f}")
                print(f"    n_corrs: mean={seg['n_corrs'].mean():.0f}, min={seg['n_corrs'].min()}")
                print(f"    n_degen: {(seg['n_degen'] > 0).sum()} frames")
                print(f"    res_rms: mean={seg['res_rms'].mean():.5f}")

if __name__ == '__main__':
    csv_path = sys.argv[1]
    seq_name = sys.argv[2] if len(sys.argv) > 2 else "unknown"
    analyze(csv_path, seq_name)
