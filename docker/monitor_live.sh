#!/bin/bash
# Real-time monitoring: speed, position drift, divergence detection
# Usage: bash docker/monitor_live.sh <dump_dir> [interval_sec=5]
set -e

DIR="${1:?Usage: monitor_live.sh <dump_dir> [interval_sec]}"
INTERVAL="${2:-5}"

echo "=== Live Monitor: ${DIR} (every ${INTERVAL}s) ==="
echo "Press Ctrl+C to stop"
echo ""

while true; do
  clear
  echo "=== TofSLAM Live Monitor — $(date '+%H:%M:%S') ==="
  echo "Dir: ${DIR}"
  echo ""

  for traj_file in $(find "${DIR}" -name "traj.csv" 2>/dev/null | sort); do
    label=$(echo "$traj_file" | sed "s|${DIR}/||" | sed 's|/traj.csv||')
    timing_file=$(dirname "$traj_file")/timing.csv

    if [ ! -f "$traj_file" ]; then continue; fi

    # Frame count
    frames=$(awk 'NR>1' "$traj_file" 2>/dev/null | wc -l)

    # Last position (x,y,z from traj.csv: timestamp,x,y,z,qw,qx,qy,qz)
    last_pos=$(tail -1 "$traj_file" 2>/dev/null | awk -F',' '{printf("x=%.1f y=%.1f z=%.1f", $2, $3, $4)}')

    # Position range (max-min for each axis = trajectory extent)
    pos_range=$(awk -F',' 'NR>1{
      if(NR==2||$2<xn)xn=$2; if(NR==2||$2>xx)xx=$2;
      if(NR==2||$3<yn)yn=$3; if(NR==2||$3>yx)yx=$3;
      if(NR==2||$4<zn)zn=$4; if(NR==2||$4>zx)zx=$4;
    } END{printf("Δx=%.1f Δy=%.1f Δz=%.1f", xx-xn, yx-yn, zx-zn)}' "$traj_file" 2>/dev/null)

    # Timing stats (last 100 frames)
    timing_info="N/A"
    if [ -f "$timing_file" ]; then
      col=$(head -1 "$timing_file" | tr ',' '\n' | grep -n "total_ms" | cut -d: -f1)
      [ -z "$col" ] && col=18
      timing_info=$(tail -100 "$timing_file" 2>/dev/null | awk -F',' -v c="$col" '
        {s+=$c; n++; if($c>mx)mx=$c; if(n==1||$c<mn)mn=$c}
        END{if(n>0) printf("avg=%.0fms min=%.0f max=%.0f (last %d)", s/n, mn, mx, n); else print "N/A"}')
    fi

    # Velocity check (last 10 frames for sudden jumps)
    vel_check=""
    if [ "$frames" -gt 10 ]; then
      vel_check=$(tail -11 "$traj_file" | awk -F',' '
        NR>1{
          if(px!=""){
            dx=$2-px; dy=$3-py; dz=$4-pz;
            d=sqrt(dx*dx+dy*dy+dz*dz);
            if(d>2.0) jumps++;
            if(d>maxd) maxd=d;
          }
          px=$2; py=$3; pz=$4;
        }
        END{
          if(jumps>0) printf("⚠️  %d JUMPS (max=%.2fm)", jumps, maxd);
          else if(maxd>0.5) printf("⚡ max_step=%.2fm", maxd);
          else printf("✓ stable (max_step=%.3fm)", maxd);
        }')
    fi

    # Divergence detection: position magnitude
    pos_mag=$(tail -1 "$traj_file" 2>/dev/null | awk -F',' '{d=sqrt($2*$2+$3*$3+$4*$4); if(d>500) printf("🔴 DIVERGED (%.0fm)", d); else if(d>200) printf("🟡 drifting (%.0fm)", d); else printf("🟢 %.1fm", d)}')

    echo "--- ${label} ---"
    echo "  Frames: ${frames} | Pos: ${last_pos} | Range: ${pos_range}"
    echo "  Timing: ${timing_info}"
    echo "  Status: ${vel_check} | Mag: ${pos_mag}"
    echo ""
  done

  # Check for ATE results
  for ate_file in $(find "${DIR}" -name "ate_result.txt" 2>/dev/null | sort); do
    label=$(echo "$ate_file" | sed "s|${DIR}/||" | sed 's|/ate_result.txt||')
    rmse=$(grep "rmse" "$ate_file" 2>/dev/null | awk '{printf("%.4f", $2)}')
    echo "📊 ATE: ${label} = ${rmse}m"
  done

  sleep "${INTERVAL}"
done
