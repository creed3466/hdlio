#!/usr/bin/env python3
"""Evaluate Start-to-End Error for all sequences in a dump directory."""
import csv, math, os, sys, glob

def quat_to_yaw_deg(qz, qw):
    return 2 * math.atan2(qz, qw) * 180 / math.pi

def evaluate_sequence(traj_path):
    rows = []
    with open(traj_path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    if len(rows) < 2:
        return None

    first, last = rows[0], rows[-1]

    # LWO
    lwo_dx = float(last['tx']) - float(first['tx'])
    lwo_dy = float(last['ty']) - float(first['ty'])
    lwo_2d = math.sqrt(lwo_dx**2 + lwo_dy**2)
    lwo_yaw = quat_to_yaw_deg(float(last['qz']), float(last['qw']))

    # Odom
    odom_dx = float(last['odom_tx']) - float(first['odom_tx'])
    odom_dy = float(last['odom_ty']) - float(first['odom_ty'])
    odom_2d = math.sqrt(odom_dx**2 + odom_dy**2)
    odom_yaw = quat_to_yaw_deg(float(last['odom_qz']), float(last['odom_qw']))

    # Total distance
    total_lwo = sum(math.sqrt(
        (float(rows[i]['tx'])-float(rows[i-1]['tx']))**2 +
        (float(rows[i]['ty'])-float(rows[i-1]['ty']))**2)
        for i in range(1, len(rows)))
    total_odom = sum(math.sqrt(
        (float(rows[i]['odom_tx'])-float(rows[i-1]['odom_tx']))**2 +
        (float(rows[i]['odom_ty'])-float(rows[i-1]['odom_ty']))**2)
        for i in range(1, len(rows)))

    duration = float(last['t_sec']) - float(first['t_sec'])

    return {
        'frames': len(rows),
        'duration': duration,
        'total_lwo': total_lwo,
        'total_odom': total_odom,
        'lwo_2d': lwo_2d,
        'odom_2d': odom_2d,
        'lwo_yaw': lwo_yaw,
        'odom_yaw': odom_yaw,
    }

def main():
    dump_base = sys.argv[1] if len(sys.argv) > 1 else '/root/ros2_ws/dump/nrx_chj_0331'

    seq_dirs = sorted(glob.glob(os.path.join(dump_base, 'w9_F4_0331_*')),
                      key=lambda x: int(x.rstrip('/').split('_')[-1]))

    print()
    print('=' * 100)
    print('TofSLAM v1.0 LWO - Batch Start-to-End Error Report')
    print('=' * 100)
    print()

    header = f"{'Sequence':<18s} {'Dur(s)':>7s} {'Frames':>7s} {'Travel(m)':>10s} {'LWO_STE(m)':>11s} {'Odom_STE(m)':>12s} {'LWO(%)':>7s} {'Odom(%)':>8s} {'Improve':>8s} {'LWO_yaw':>8s} {'Odom_yaw':>9s}"
    print(header)
    print('-' * 100)

    results = []
    for sd in seq_dirs:
        name = os.path.basename(sd)
        traj = os.path.join(sd, 'traj.csv')
        if not os.path.exists(traj):
            print(f'{name:<18s} MISSING traj.csv')
            continue
        r = evaluate_sequence(traj)
        if r is None:
            print(f'{name:<18s} EMPTY traj.csv')
            continue

        lwo_pct = r['lwo_2d'] / max(r['total_lwo'], 0.001) * 100
        odom_pct = r['odom_2d'] / max(r['total_odom'], 0.001) * 100
        if r['odom_2d'] > 0.001:
            improve = (1 - r['lwo_2d'] / r['odom_2d']) * 100
        else:
            improve = 0.0

        print(f"{name:<18s} {r['duration']:>7.1f} {r['frames']:>7d} {r['total_lwo']:>10.2f}"
              f" {r['lwo_2d']:>11.4f} {r['odom_2d']:>12.4f}"
              f" {lwo_pct:>7.2f} {odom_pct:>8.2f} {improve:>+7.1f}%"
              f" {r['lwo_yaw']:>+8.2f} {r['odom_yaw']:>+9.2f}")

        results.append({**r, 'name': name, 'lwo_pct': lwo_pct, 'odom_pct': odom_pct, 'improve': improve})

    if results:
        print('-' * 100)
        # Averages
        n = len(results)
        avg_lwo = sum(r['lwo_2d'] for r in results) / n
        avg_odom = sum(r['odom_2d'] for r in results) / n
        avg_lwo_pct = sum(r['lwo_pct'] for r in results) / n
        avg_odom_pct = sum(r['odom_pct'] for r in results) / n
        avg_improve = sum(r['improve'] for r in results) / n
        total_travel = sum(r['total_lwo'] for r in results)
        total_dur = sum(r['duration'] for r in results)
        total_frames = sum(r['frames'] for r in results)

        print(f"{'AVERAGE':<18s} {total_dur/n:>7.1f} {total_frames//n:>7d} {total_travel/n:>10.2f}"
              f" {avg_lwo:>11.4f} {avg_odom:>12.4f}"
              f" {avg_lwo_pct:>7.2f} {avg_odom_pct:>8.2f} {avg_improve:>+7.1f}%")
        print()

        # Win/Loss
        wins = sum(1 for r in results if r['lwo_2d'] < r['odom_2d'])
        print(f'LWO wins: {wins}/{n}  |  Avg STE: LWO={avg_lwo:.4f}m vs Odom={avg_odom:.4f}m  |  Avg improvement: {avg_improve:+.1f}%')

    print('=' * 100)

if __name__ == '__main__':
    main()
