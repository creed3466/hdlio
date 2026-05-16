import csv, math

rows = []
with open('/root/ros2_ws/dump/nrx_chj_0331_w9/traj.csv') as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

first, last = rows[0], rows[-1]

lwo_dx = float(last['tx']) - float(first['tx'])
lwo_dy = float(last['ty']) - float(first['ty'])
lwo_dz = float(last['tz']) - float(first['tz'])
lwo_dist_2d = math.sqrt(lwo_dx**2 + lwo_dy**2)
lwo_qz, lwo_qw = float(last['qz']), float(last['qw'])
lwo_yaw = 2 * math.atan2(lwo_qz, lwo_qw) * 180 / math.pi

odom_dx = float(last['odom_tx']) - float(first['odom_tx'])
odom_dy = float(last['odom_ty']) - float(first['odom_ty'])
odom_dist_2d = math.sqrt(odom_dx**2 + odom_dy**2)
odom_qz, odom_qw = float(last['odom_qz']), float(last['odom_qw'])
odom_yaw = 2 * math.atan2(odom_qz, odom_qw) * 180 / math.pi

total_lwo = sum(math.sqrt((float(rows[i]['tx'])-float(rows[i-1]['tx']))**2 +
                           (float(rows[i]['ty'])-float(rows[i-1]['ty']))**2)
                for i in range(1, len(rows)))
total_odom = sum(math.sqrt((float(rows[i]['odom_tx'])-float(rows[i-1]['odom_tx']))**2 +
                            (float(rows[i]['odom_ty'])-float(rows[i-1]['odom_ty']))**2)
                 for i in range(1, len(rows)))

duration = float(last['t_sec']) - float(first['t_sec'])

print('=' * 65)
print('TofSLAM v1.0 LWO - Start-to-End Error (TUNED)')
print('=' * 65)
print(f'Dataset:  w9_F4_0331_1  Duration: {duration:.1f}s  Frames: {len(rows)}')
print()
print('--- Total Traveled Distance ---')
print(f'  LWO:  {total_lwo:.3f} m')
print(f'  Odom: {total_odom:.3f} m')
print()
print('--- End Position (vs start) ---')
print(f'  LWO:  ({lwo_dx:+.4f}, {lwo_dy:+.4f}, {lwo_dz:+.4f})  2D: {lwo_dist_2d:.4f} m')
print(f'  Odom: ({odom_dx:+.4f}, {odom_dy:+.4f})              2D: {odom_dist_2d:.4f} m')
print()
print('--- End Yaw (vs start) ---')
print(f'  LWO:  {lwo_yaw:+.3f} deg')
print(f'  Odom: {odom_yaw:+.3f} deg')
print()
print('--- Start-to-End Error ---')
lwo_pct = lwo_dist_2d / max(total_lwo, 0.001) * 100
odom_pct = odom_dist_2d / max(total_odom, 0.001) * 100
print(f'  LWO  2D: {lwo_dist_2d:.4f} m  ({lwo_pct:.2f}%)')
print(f'  Odom 2D: {odom_dist_2d:.4f} m  ({odom_pct:.2f}%)')
if odom_dist_2d > 0:
    if lwo_dist_2d < odom_dist_2d:
        pct = (1 - lwo_dist_2d / odom_dist_2d) * 100
        print(f'  >>> LWO {pct:.1f}% better than odom <<<')
    else:
        pct = (lwo_dist_2d / odom_dist_2d - 1) * 100
        print(f'  >>> LWO {pct:.1f}% worse than odom <<<')
print()
print('--- Before vs After Tuning ---')
print(f'  LWO 2D error:     0.1504 m  ->  {lwo_dist_2d:.4f} m')
print(f'  Odom 2D error:    0.1534 m  ->  {odom_dist_2d:.4f} m')
print(f'  Correspondences:  0-4       ->  35-92')
print(f'  Map points init:  72        ->  134')
print(f'  SKIP_MAP_UPDATE:  49/50     ->  8/~495')
print('=' * 65)
