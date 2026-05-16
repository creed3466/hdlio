# Task #64 — ICP Debug Analysis on sbs_03 (H_B)

**Date**: 2026-04-14  
**Hypothesis under test**: H_B — ICP correspondence degrades in the
sparse/foliage courtyard segment (t=15–45 s) on NTU sbs_03.  
**Data source**: `dump/task64_icp_debug_20260414/ig_lio/ntu/sbs_03/iglio_icp_debug.csv` (3776 lidar frames, duration=377.5 s)  
**Instrumentation**: ig_lio commits `task #64 (a)` + `task #64 (b)`, env-gated via `TOFSLAM_ICP_DEBUG=1`.

## 3-Window Statistics

| Window | t-range | Frames | Inliers (mean ± std) | Mean residual (mean ± std) | Map voxels (mean ± std) |
|--------|---------|-------:|---------------------:|---------------------------:|------------------------:|
| **W1 (0-15s)** | see header | 151 | 2702.7 ± 15.3 | 0.01495 ± 0.001305 | 4606 ± 0 |
| **W2 (15-45s)** | see header | 300 | 1895.7 ± 609.5 | 0.06499 ± 0.03913 | 8405 ± 5045 |
| **W3 (>=45s)** | see header | 3325 | 2948.0 ± 494.3 | 0.2371 ± 0.02987 | 92954 ± 20182 |

Window definitions:  
- **W1**: 0 ≤ t < 15 s (stable prefix)  
- **W2**: 15 ≤ t < 45 s (suspected foliage / error spike)  
- **W3**: t ≥ 45 s (recovery)

## Z-score of W2 vs pooled W1∪W3 baseline

| Metric | W2 z-score | Accept threshold | Criterion met |
|--------|-----------:|:-----------------|:-------------:|
| Inlier count    | -2.14 | ≤ −2.0 | YES |
| Mean residual   | -3.01 | ≥ +2.0 | no |
| Map voxel count | -3.02 | ≤ −2.0 | YES |

## Verdict: **H_B ACCEPT**

W2 inlier count is ≥ 2σ below the W1∪W3 baseline **and** the map-voxel
count criterion fires (z = −3.02). This matches the expected signature
of ICP correspondence collapse from an under-populated map in the
sparse/foliage courtyard segment.

### Caveats / interpretation notes

- **Residual z is negative, not positive** (z = −3.01). This is *not*
  the classical "residual rises in the bad zone" signature. The cause
  is that W3 (recovery) accumulates a much larger mean residual (0.237 m)
  than W2 (0.065 m) as the map grows and the residual distribution
  widens, so the pooled W1∪W3 baseline is dominated by W3. Under the
  pre-registered rule (inlier ≤ −2σ AND (residual ≥ +2σ OR voxels ≤ −2σ)),
  the voxel-count criterion is sufficient for ACCEPT.
- **Map is still warming up through W2**: W1 voxels = 4606, W2 = 8405,
  W3 = 92954. The correspondence collapse in W2 is therefore better
  characterised as *map-under-coverage during early trajectory* rather
  than a pure foliage-sparsity phenomenon. This refines the Architect
  brief: the remediation must target early-trajectory map density
  (e.g. keyframe injection floor, aggressive initial voxel seeding),
  not only foliage-adaptive ICP.
- **AHRS init flag**: during this task the parent commit c54772a had
  set `enable_ahrs_initalization: true` on the stock iG-LIO NTU yaml,
  which caused a SIGABRT at frame ~11 on sbs_03 (independent of task
  #64 instrumentation). The diagnostic run was therefore executed
  against `/tmp/ntu_noahrs.yaml` with `enable_ahrs_initalization: false`.
  Commit 5a0f0f3 (`eval(#63): Option B AHRS init — FAIL (revert c54772a)`)
  has since reverted the stock config back to `false`, so this run is
  consistent with the current stock configuration.

Proceed to Architect stage to design a remediation targeting
early-trajectory map density (keyframe injection / voxel seeding /
multi-resolution ICP warm-up).

## Artifacts

- ICP debug CSV: `dump/task64_icp_debug_20260414/ig_lio/ntu/sbs_03/iglio_icp_debug.csv`
- Plot: `paper_figures/task64_sbs03_icp_timeline.png`
- Research: `docs/reports/task64_sbs03_non_init_research_20260414.md`

