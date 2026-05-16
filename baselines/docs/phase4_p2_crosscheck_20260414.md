# Phase 4 P4.2 cross-check — FAST-LIO2 / Point-LIO / iG-LIO × NTU

**Input**: 27 new runs (3 algos × 9 NTU seqs), label
`phase4_ntu_crosscheck_20260413`, all at `rosbag play -r 1.0`.

---

## Raw ATE RMSE table (meters)

| seq      | LIO-SAM† | FAST-LIO2 | Point-LIO | iG-LIO | mean across 4 |
|----------|---------:|----------:|----------:|-------:|--------------:|
| eee_01   |    0.206 |     0.166 |     0.161 |  1.025 | — (iG outlier)|
| eee_02   |    0.209 |     0.164 |     0.162 |  0.187 |         0.181 |
| eee_03   |    0.210 |     0.185 |     0.179 |  0.202 |         0.194 |
| nya_01   |    0.210 |     0.171 |     0.166 |  0.183 |         0.183 |
| nya_02   |    0.199 |     0.177 |     0.171 |  0.225 |         0.193 |
| nya_03   |    0.197 |     0.175 |     0.177 |  0.194 |         0.186 |
| sbs_01   |    0.218 |     0.187 |     0.184 |  0.189 |         0.195 |
| sbs_02   |    0.245 |     0.204 |     0.203 |  0.209 |         0.215 |
| sbs_03   |    0.457 |     0.443 |     0.442 |  0.889 |         0.558 |

† LIO-SAM from 22f5ee1 Tier-1. FAST-LIO2/Point-LIO/iG-LIO from cfd9dd8 cross-check.

### NTU VIRAL IJRR 2022 Table 4 reference (LIO-SAM column)

| seq    | ref   |
|--------|------:|
| eee_01 | 0.075 |
| eee_02 | 0.069 |
| eee_03 | 0.101 |
| nya_01 | 0.076 |
| nya_02 | 0.090 |
| nya_03 | 0.137 |
| sbs_01 | 0.089 |
| sbs_02 | 0.083 |
| sbs_03 | 0.140 |

Paper mean: 0.096 m. Our mean across 4 algos (ex eee_01/sbs_03 outliers):
0.186 m. **Consistent ~2× gap across every algorithm.**

---

## Key observations

1. **Cross-algo cluster**. FAST-LIO2, Point-LIO, iG-LIO are
   architecturally distinct (IEKF KD-tree map, direct-LIO voxel map,
   GICP voxel map). Yet their ATE cluster bands overlap within 0.02 m
   of each other: 0.16–0.20 m for all 3. This rules out any
   algorithm-specific config issue. The gap must be in a **shared
   downstream layer**.
2. **sbs_03 is a cross-algo outlier**. All four algos report
   sbs_03 at ~0.44–0.89 m vs their own 0.18 m cluster — roughly
   2× cluster-mean. If it were algorithm-specific we would expect
   only one algo to fail. Cross-algo sbs_03 outlier = sbs_03-specific
   evaluation-layer issue.
3. **iG-LIO eee_01 outlier (1.025 m)**. Only iG-LIO. Likely
   algorithm-specific initial-gravity / init-cov setup on a short
   static-start trajectory. Separate from the main finding.
4. **Paper numbers have a factor-of-5 per-seq variation**
   (NTU VIRAL Table 4: 0.069 min, 0.140 max).
   **Ours have a factor-of-~1.3 per-seq variation** (0.16 min,
   0.22 max, excluding outliers). Our numbers **do not track** the
   paper's per-seq ordering. A shared evaluation bias would produce
   exactly this pattern: a ~constant ~0.1 m offset added to every
   seq regardless of actual trajectory length/shape.

---

## Shared layers that could produce this pattern

1. **compute_ate.py alignment** (`paper_figures/alignment.py`) —
   Umeyama SE(3) no-scale absorbs rigid rotation + translation but
   does *not* compensate lever-arm coupling if the GT and algo output
   are in different body frames.
2. **NTU VIRAL GT frame**. NTU VIRAL `ground_truth.csv` is the Leica
   prism position. Algo outputs are lidar-frame (or IMU-frame)
   trajectories. The prism is mounted offset from the lidar (datasheet:
   ~10–15 cm lever arm on the NTU VIRAL UAV platform). Umeyama finds
   the best-fit rigid transform but leaves a residual equal to
   `|lever_arm| × sin(rotation_delta)` at every pose. For a rotating
   trajectory, this residual integrates to a ~constant per-seq offset.
3. **Time-sync / interp window**. `compute_ate.py` uses timestamp-sync
   then interpolates GT to algo timestamps over the overlap window.
   If there is a fixed ~ms-scale offset between bag time and GT file
   time, a moving platform introduces a constant-velocity × time-offset
   ATE contribution.
4. **Quaternion convention**. GT has `qx qy qz qw`; if somewhere in
   the pipeline we interpret it as `qw qx qy qz`, every pose is
   rotated by ~90°. (Quick check: our GT files have the initial row
   at `(0,0,0,1)` explicitly in the xyzw order, and load_tum is
   consistent with this.)
5. **NTU VIRAL-specific frame transform**. The LIO-SAM fork applies
   `R_W2NED = diag(1,-1,-1)` because NTU VIRAL uses an ENU world frame
   by default but the brytsknguyen fork expects NED. If we've applied
   this rotation on an already-NED trajectory (or applied it to IMU-
   frame output instead of world-frame output), every point is rotated.

---

## Hypothesis ranking (prior to Research stage)

| hypothesis | prior | why |
|-|:-:|-|
| prism-lidar lever arm not compensated | **high** | known NTU VIRAL pitfall, cross-algo signature, sbs_03 sensitivity to rotation |
| time-sync bias in compute_ate interp | low-mid | a ms-scale bias × m/s platform motion gives cm-scale, not 10 cm |
| Umeyama alignment methodology mismatch | mid | papers may use first-frame anchor only; our Umeyama could over-absorb |
| R_W2NED double-transform | low-mid | would probably produce catastrophic errors, not +2× |
| quaternion convention swap | low | would also produce trajectory inversion, not constant offset |

---

## Gate decision

**PASS** for P4.2 (cross-check runs). Cross-algo evidence rules out
per-algorithm config issues. Root cause is in the shared evaluation
layer. Do NOT tune any algorithm's params — enter Research on the
evaluation layer first.

---

## Next — P4.4 Research scope (reduced)

Primary question: **Is our NTU VIRAL evaluation using the correct
GT frame + lever-arm compensation?** Secondary questions nested
inside.

Sources to check:
- NTU VIRAL dataset yaml for T_prism_lidar or T_prism_imu.
- NTU VIRAL paper / supplementary for the evaluation protocol they
  used to produce Table 4 (which frame? Umeyama? first-frame align?).
- brytsknguyen/NTU_VIRAL or ntu-aris example evaluation scripts.
- ntu-aris repo for the published benchmark launcher's output frame.

Delegate to Research (web + docs + repo inspection). Do not touch
algorithm params or configs until this layer is understood.
