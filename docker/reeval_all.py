#!/usr/bin/env python3
"""
reeval_all.py — Batch re-evaluation with lever arm correction.

Re-evaluates existing trajectories with and without lever arm correction
and produces a comparison table.
"""

import os
import sys
import csv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_ate_ntu_viral import (
    load_est_traj, load_gt_traj, apply_lever_arm_correction,
    interpolate_gt_at_est_times, compute_ate_from_matched,
    associate_trajectories, T_BODY_PRISM_DEFAULT
)
import numpy as np

SEQUENCES = ['eee_01', 'eee_02', 'eee_03',
             'nya_01', 'nya_02', 'nya_03',
             'sbs_01', 'sbs_02', 'sbs_03']

GT_BASE = '/root/catkin_ws/data/ntu_viral'
DUMP_BASE = '/root/catkin_ws/dump'

EXPERIMENTS = [
    ('fastlio2_ntu_viral', 'FAST-LIO2'),
    ('ntu_viral_opt_v12_full', 'TofSLAM v12'),
    ('ntu_viral_v22', 'TofSLAM v22'),
]

# FAST-LIO2 paper reference values (from RLI-SLAM paper)
PAPER_REF = {
    'eee_01': 0.131, 'eee_02': 0.124, 'eee_03': 0.163,
    'nya_01': 0.122, 'nya_02': 0.142, 'nya_03': 0.144,
    'sbs_01': 0.142, 'sbs_02': 0.140, 'sbs_03': 0.133,
}


def evaluate_one(est_path, gt_path, use_lever_arm=True):
    """Evaluate a single trajectory. Returns ATE RMSE or None on failure."""
    est = load_est_traj(est_path)
    if est is None or len(est) == 0:
        return None, 0

    gt = load_gt_traj(gt_path)
    if gt is None or len(gt) == 0:
        return None, 0

    if use_lever_arm:
        est = apply_lever_arm_correction(est, T_BODY_PRISM_DEFAULT)

    est_matched, gt_matched = interpolate_gt_at_est_times(est, gt)
    if est_matched is None or len(est_matched) < 10:
        return None, 0

    result = compute_ate_from_matched(est_matched[:, 1:4], gt_matched[:, 1:4])
    return result['rmse'], result['n_matches']


def main():
    output_dir = '/root/catkin_ws/dump/reeval_20260403'
    os.makedirs(output_dir, exist_ok=True)

    all_results = {}

    for exp_dir, exp_name in EXPERIMENTS:
        print(f"\n{'='*70}")
        print(f"Evaluating: {exp_name} ({exp_dir})")
        print(f"{'='*70}")

        results_old = {}
        results_new = {}

        for seq in SEQUENCES:
            est_path = os.path.join(DUMP_BASE, exp_dir, seq, 'traj_est.csv')
            gt_path = os.path.join(GT_BASE, seq, 'ground_truth.csv')

            if not os.path.exists(est_path):
                print(f"  {seq}: SKIP (no trajectory)")
                continue

            # Old: no lever arm, nearest-neighbor
            ate_old, n_old = evaluate_one(est_path, gt_path, use_lever_arm=False)
            # New: lever arm + interpolation
            ate_new, n_new = evaluate_one(est_path, gt_path, use_lever_arm=True)

            if ate_old is not None and ate_new is not None:
                results_old[seq] = ate_old
                results_new[seq] = ate_new
                delta = ate_old - ate_new
                paper = PAPER_REF.get(seq, None)
                paper_str = f"{paper:.3f}" if paper else "N/A"
                ratio_str = f"{ate_new/paper:.2f}x" if paper else "N/A"
                print(f"  {seq}: old={ate_old:.4f}  new={ate_new:.4f}  "
                      f"delta={delta:+.4f}  paper={paper_str}  ratio={ratio_str}")

        all_results[exp_name] = (results_old, results_new)

        # Summary
        if results_new:
            seqs_with_data = sorted(results_new.keys())
            avg_old = np.mean([results_old[s] for s in seqs_with_data])
            avg_new = np.mean([results_new[s] for s in seqs_with_data])
            print(f"\n  Average: old={avg_old:.4f}  new={avg_new:.4f}  "
                  f"delta={avg_old-avg_new:+.4f}")

    # ===================================================================
    # Summary table
    # ===================================================================
    print(f"\n\n{'='*100}")
    print("COMPARISON TABLE: Before/After Lever Arm Correction")
    print(f"{'='*100}")

    # Header
    header = f"{'Seq':<8}"
    header += f"{'Paper':>8}"
    for _, exp_name in EXPERIMENTS:
        header += f"  {'Old':>8} {'New':>8} {'Delta':>8}"
    print(header)
    print("-" * len(header))

    for seq in SEQUENCES:
        row = f"{seq:<8}"
        paper = PAPER_REF.get(seq, None)
        row += f"{paper:>8.3f}" if paper else f"{'N/A':>8}"

        for _, exp_name in EXPERIMENTS:
            results_old, results_new = all_results.get(exp_name, ({}, {}))
            old = results_old.get(seq, None)
            new = results_new.get(seq, None)
            if old is not None and new is not None:
                delta = old - new
                row += f"  {old:>8.4f} {new:>8.4f} {delta:>+8.4f}"
            else:
                row += f"  {'---':>8} {'---':>8} {'---':>8}"
        print(row)

    # Averages
    row = f"{'Mean':<8}"
    paper_vals = [v for v in PAPER_REF.values()]
    row += f"{np.mean(paper_vals):>8.3f}"
    for _, exp_name in EXPERIMENTS:
        results_old, results_new = all_results.get(exp_name, ({}, {}))
        common_seqs = sorted(set(results_old.keys()) & set(results_new.keys()))
        if common_seqs:
            avg_old = np.mean([results_old[s] for s in common_seqs])
            avg_new = np.mean([results_new[s] for s in common_seqs])
            row += f"  {avg_old:>8.4f} {avg_new:>8.4f} {avg_old-avg_new:>+8.4f}"
        else:
            row += f"  {'---':>8} {'---':>8} {'---':>8}"
    print("-" * len(header))
    print(row)

    # Save CSV
    csv_path = os.path.join(output_dir, 'reeval_comparison.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        header_row = ['sequence', 'paper_ref']
        for _, exp_name in EXPERIMENTS:
            header_row.extend([f'{exp_name}_old', f'{exp_name}_new', f'{exp_name}_delta'])
        writer.writerow(header_row)

        for seq in SEQUENCES:
            row_data = [seq, PAPER_REF.get(seq, '')]
            for _, exp_name in EXPERIMENTS:
                results_old, results_new = all_results.get(exp_name, ({}, {}))
                old = results_old.get(seq, '')
                new = results_new.get(seq, '')
                delta = (old - new) if isinstance(old, float) and isinstance(new, float) else ''
                row_data.extend([
                    f"{old:.6f}" if isinstance(old, float) else '',
                    f"{new:.6f}" if isinstance(new, float) else '',
                    f"{delta:.6f}" if isinstance(delta, float) else '',
                ])
            writer.writerow(row_data)

    print(f"\nCSV saved to: {csv_path}")

    # ===================================================================
    # FAST-LIO2 vs Paper comparison
    # ===================================================================
    print(f"\n\n{'='*70}")
    print("FAST-LIO2: Corrected vs Paper Values")
    print(f"{'='*70}")
    fl2_old, fl2_new = all_results.get('FAST-LIO2', ({}, {}))
    if fl2_new:
        print(f"{'Seq':<8} {'Paper':>8} {'Corrected':>10} {'Ratio':>8}")
        print("-" * 36)
        for seq in SEQUENCES:
            paper = PAPER_REF.get(seq, None)
            new = fl2_new.get(seq, None)
            if paper and new:
                ratio = new / paper
                mark = "OK" if ratio < 1.5 else "HIGH"
                print(f"{seq:<8} {paper:>8.3f} {new:>10.4f} {ratio:>7.2f}x  {mark}")
        common = [s for s in SEQUENCES if s in fl2_new]
        if common:
            avg_paper = np.mean([PAPER_REF[s] for s in common])
            avg_new = np.mean([fl2_new[s] for s in common])
            print("-" * 36)
            print(f"{'Mean':<8} {avg_paper:>8.3f} {avg_new:>10.4f} {avg_new/avg_paper:>7.2f}x")


if __name__ == '__main__':
    main()
