# Paper Reproduction Validation Targets

Each baseline must reproduce at least one result from its own paper (or a cross-
referenced third-party benchmark) to confirm our integration is correct before
we trust its numbers on M3DGR / NTU VIRAL.

Match criterion: ATE RMSE within **±10%** of paper-reported value.
If off by > 20%, the integration is suspect — revisit build/config.

---

## FAST-LIO2

Paper: Xu et al., "FAST-LIO2: Fast Direct LiDAR-inertial Odometry", T-RO 2022.

Paper evaluation:
- HKU campus (Avia), NCLT, UrbanLoco.
- Paper Table II, III, IV has per-sequence numbers.

Chosen validation target:
- **HKU Avia main_building dataset** (provided by repo, or similar).
- Expected: sub-metre ATE RMSE on closed indoor loops.

Cross-reference (third-party benchmark on NTU VIRAL):
- SLICT paper Table IV reports FAST-LIO2 on NTU VIRAL:
    - eee_01 ≈ 0.22 m, eee_02 ≈ 0.14 m, eee_03 ≈ 0.12 m
    - nya_01 ≈ 0.09 m, nya_02 ≈ 0.07 m, nya_03 ≈ 0.08 m
    - sbs_01 ≈ 0.18 m, sbs_02 ≈ 0.09 m, sbs_03 ≈ 0.09 m

Our run on NTU VIRAL → accept if within ±20% of these published numbers.

---

## Point-LIO

Paper: He, Xu et al., "Point-LIO: Robust High-Bandwidth Lidar-Inertial
Odometry", Advanced Intelligent Systems 2022.

Paper evaluation:
- Racing drone dataset, PULSAR (both in repo OneDrive).
- High-bandwidth acceleration recovery focus.

Chosen validation target:
- **racing_drone.bag** from official OneDrive.
- Expected: convergent trajectory, no divergence.

Cross-reference on NTU VIRAL:
- No direct Point-LIO NTU VIRAL numbers in community tables.
- Fall back to qualitative check: closed-loop trajectory on eee_01, no divergence.

---

## LIO-SAM

Paper: Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry
via Smoothing and Mapping", IROS 2020.

Paper evaluation:
- park_dataset (Velodyne VLP-16), garden_dataset, walking_dataset.

Chosen validation target:
- **park_dataset.bag** from repo (author-provided).
- Expected: closed-loop, final drift < 1 m.

Cross-reference on NTU VIRAL:
- SLICT paper Table IV: LIO-SAM on NTU VIRAL
    - eee_01 ≈ 0.20 m, eee_02 ≈ 0.14 m, eee_03 ≈ 0.10 m
    - nya_01 ≈ 0.09 m, nya_02 ≈ 0.08 m, nya_03 ≈ 0.08 m
    - sbs_01 ≈ 0.05 m, sbs_02 ≈ 0.05 m, sbs_03 ≈ 0.04 m

Note: LIO-SAM needs 9-axis IMU. NTU VIRAL's VectorNav VN100 is 9-axis ✓.
M3DGR is 6-axis → LIO-SAM excluded from M3DGR (or run with magnetometer
disabled via `imu_type: 1` fork — expect degraded performance).

---

## SLICT

Paper: Nguyen et al., "SLICT: Multi-input Multi-scale Surfel-Based
Lidar-Inertial Continuous-Time Odometry and Mapping", RA-L 2023.

Paper evaluation:
- NTU VIRAL (author's own dataset!), MulRan, Hilti2022.

Chosen validation target:
- **NTU VIRAL eee_01** (native).
- Paper Table IV: SLICT eee_01 ≈ 0.07 m, eee_02 ≈ 0.05 m, eee_03 ≈ 0.06 m.
- Accept if we reproduce within ±10% on eee_01.

For M3DGR: no paper numbers, qualitative only (trajectory convergence).

---

## iG-LIO

Paper: Chen et al., "iG-LIO: An Incremental GICP-based Tightly-coupled
LiDAR-inertial Odometry", RA-L 2024.

Paper evaluation:
- Livox AVIA, Newer College, NCLT, ULHK, Botanic Garden.
- Paper Table III reports per-sequence ATE.

Chosen validation target:
- **Newer College Dataset short_experiment.bag**.
- Paper Table III: iG-LIO on NCD short ≈ 0.08–0.10 m.

Cross-reference on NTU VIRAL:
- iG-LIO paper does not evaluate on NTU VIRAL.
- Qualitative check only (closed-loop convergence).

---

## Validation Report Template

After each paper-reproduction run, write `dump/baselines_<DATE>/<algo>/validation.md`:

```markdown
# <ALGO> Paper Reproduction Validation

Target dataset: <name>
Target sequence: <seq>
Paper-reported ATE RMSE: <X> m
Our reproduction ATE RMSE: <Y> m
Delta: <(Y-X)/X*100>%
Verdict: PASS (|delta| ≤ 10%) / WARN (≤ 20%) / FAIL (> 20%)

Notes:
- Build config: ...
- ROS1 rate: -r 1.0
- Any deviations from paper: ...
```
