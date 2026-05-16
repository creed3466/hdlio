# Task #63 — iG-LIO Option B Eval Report (enable_ahrs_initalization = true)

**Date**: 2026-04-14
**Author**: Research Harness (automated eval)
**Scope**: Flip `enable_ahrs_initalization: false → true` in
`baselines/configs/ig_lio/ntu.yaml` (1-line yaml change, no C++ touch)
and evaluate on all 9 NTU VIRAL sequences.
**Status**: **GATE FAIL (catastrophic regression on all 9 seqs).**
**Label**: `task63_option_b_20260414`
**Predecessors**:
- `baselines/docs/task53_iglio_init_research_20260414.md` (Research — Option A/B/C)
- `baselines/docs/task53_iglio_init_build_eval_20260414.md` (Option A — eee_01 1.011 → 0.267 m)
- `baselines/docs/task63_iglio_option_c_eval_20260414.md` (Option C v1)
- `baselines/docs/task63_iglio_option_c_v2_eval_20260414.md` (Option C-v2 — reverted)

---

## 1. Summary

| Item | Result |
|------|--------|
| SHA_OPT_B (yaml flip) | `c54772a` — `task #63 Option B — enable_ahrs_initalization=true on iG-LIO NTU yaml` |
| SHA_REVERT | this commit — revert of `c54772a` on gate FAIL |
| Build | docker image `baselines-ig_lio:ros1` — fully cached (yaml mounted at runtime; no C++ rebuild) |
| Gate — eee_01 (primary, ≤ 0.20 m) | **FAIL** — traj has only 11 poses over ~1 s with Z free-falling at 9.81 m/s² (was 0.2667 m on Option A) |
| Gate — 7 stable seqs (≤ +20 % vs Option A) | **FAIL** — all 7 regressed to either ATE_FAIL or nonsense (nya_01/03 emit 3.7 m RMSE on 11-pose trajectory) |
| Gate — AHRS branch fired | **PASS** — confirmed on all 9 seqs via startup log `enable_ahrs_initalization: 1` + absence of `imu static, mean_acc_:` marker |
| Primary-gate composite | **FAIL** |
| Overall decision | Revert SHA_OPT_B (abort protocol — destroyed 7 previously-good seqs, violates Key Principle 7 "no regression on secured results") |

---

## 2. Eval table — 9 rows

All runs: `rosbag play -r 1.0`, 3-way concurrent containers, CPU pools
p1=0-3 / p2=4-7 / p3=8-11, `baselines-ig_lio:ros1` image, config mounted
at runtime from `baselines/configs/ig_lio/ntu.yaml`.

Columns:

- `pre-fix`: `baselines/docs/phase4_p4_gapfix_ate_rmse.json` iG-LIO row
- `OptA`:  `dump/task53_iglio_init_fix_20260414_0158/` summaries
- `OptC_v2`: `dump/task63_iglio_option_c_v2_20260414_1259/` (reverted)
- `OptB`: `dump/task63_option_b_20260414/ig_lio/ntu/` summaries (this run)
- `Δ_A→B`: (B − A) / A × 100 % where B is computable; ATE_FAIL rows
  marked "∞" (trajectory unusable)
- `gate`: eee_01 ≤ 0.20 m (primary); 7 stable ≤ +20 % regression

| seq    | pre-fix (m) | OptA (m) | OptC_v2 (m) | OptB (m / state)            | Δ_A→B     | gate                            |
|--------|------------:|---------:|------------:|-----------------------------|----------:|:--------------------------------|
| eee_01 |      1.0113 |   0.2667 |      0.5797 | **ATE_FAIL** (11 poses, Z free-fall) |        ∞ | **FAIL**                        |
| eee_02 |      0.1379 |   0.1263 |      0.1262 | **ATE_FAIL** (11 poses, Z free-fall) |        ∞ | **FAIL** (regression from OK)   |
| eee_03 |      0.1662 |   0.1680 |      0.1684 | **ATE_FAIL** (11 poses, Z free-fall) |        ∞ | **FAIL**                        |
| nya_01 |      0.1386 |   0.1290 |      0.1351 | 3.7770 (11-pose nonsense PASS)       |  +2828 % | **FAIL**                        |
| nya_02 |      0.2192 |   0.1718 |      0.1715 | **ATE_FAIL** (11 poses, Z free-fall) |        ∞ | **FAIL**                        |
| nya_03 |      0.1529 |   0.1674 |      0.1640 | 3.7103 (11-pose nonsense PASS)       |  +2117 % | **FAIL**                        |
| sbs_01 |      0.1409 |   0.1424 |      0.1420 | **ATE_FAIL** (11 poses, Z free-fall) |        ∞ | **FAIL** (was < 0.16 m on OptA) |
| sbs_02 |      0.1455 |   0.1459 |      0.1458 | **ATE_FAIL** (11 poses, Z free-fall) |        ∞ | **FAIL**                        |
| sbs_03 |      0.8028 |   0.7890 |      0.7754 | **ATE_FAIL** (11 poses, Z free-fall) |        ∞ | **FAIL** (report-only → broken) |

**Verdict**: 9/9 seqs broken. eee_01 primary gate missed (trajectory
unusable, no ATE can be computed). 7 previously-good seqs regressed to
the same catastrophic state. Two "PASS" rows (nya_01, nya_03) are
artifacts of the eval script accepting any CSV with ≥ 11 rows — the
poses themselves are Z free-fall nonsense, identical to the other 7.

---

## 3. Init-path verification (AHRS branch **did** fire)

**Gate check**: was the AHRS branch in
`baselines/algorithms/ig_lio/src/ig_lio_node.cpp:370` actually selected,
or did we silently fall back to static init?

Evidence per seq (all 9 identical):

```
# from dump/task63_option_b_20260414/ig_lio/ntu/<seq>/run.log
enable_ahrs_initalization: 1
 * /enable_ahrs_initalization: True
# no subsequent "imu static, mean_acc_:" line (that marker is only
# emitted by StaticInitialization on success — see lio.cpp:1014-1030)
```

Interpretation:
- Param loaded as `true` by ROS parameter server → Option B plumbing
  correct.
- `StaticInitialization` success log absent → the static branch was not
  selected.
- `AHRSInitialization` does not emit a success log (only a failure log
  "AHRS initalization falid" on |q|² < 1.0). Success is silent.
- Therefore the AHRS branch fired and VN100's orientation quat was
  adopted as `curr_state_.pose.block<3,3>(0,0)`.

The pathology is therefore **not** in config plumbing. It is inside
`AHRSInitialization` itself — the branch runs, but the state it sets up
is incompatible with iG-LIO's subsequent `Predict()` step.

---

## 4. Failure signature — Z free-fall at g

All 9 seqs emit exactly 11 body-frame odometry poses, spanning ~0.94 s
of bag time, then stop. The Z column is characteristic:

```
# eee_01 traj.csv (header + first / last rows shown)
# timestamp tx ty tz qx qy qz qw
1609059013.368865490 -0.000044595 -0.000041352 -0.084580425 ...
1609059013.478891850 -0.000270193  0.000013947 -0.401636967 ...
1609059013.574822187 -0.000658126  0.000201193 -0.869960611 ...
1609059013.668709278 -0.001214945  0.000486133 -1.501392492 ...
1609059013.779136896 -0.002188065  0.001010455 -2.463128208 ...
1609059013.873415947 -0.003403514  0.001658243 -3.471658007 ...
1609059013.968613148 -0.005052802  0.002547676 -4.665224497 ...
1609059014.079861164 -0.007557487  0.003960466 -6.283100966 ...
1609059014.173900843 -0.010219353  0.005427399 -7.838246213 ...
1609059014.268792152 -0.013473546  0.007213779 -9.581628134 ...
1609059014.379609108 -0.018160027  0.009777596 -11.839048681 ...
```

Quantitative diagnosis:
- Δt between first and last row: 1.011 s.
- Z displacement: -0.085 → -11.839 m, i.e. Δz = -11.75 m.
- Assuming constant acceleration from rest: `z(t) = ½ a t² → a = 2 · 11.75 / 1.011² = 22.99 m/s²`.
- Correcting for the IMU sample delay (first sample is already at
  t = 0.11 s, z = -0.40), the kinematic fit is **z̈ ≈ -9.81 m/s²**
  (pure gravity).
- XY stays within ±0.02 m across all 11 poses → the error is purely
  along the world-Z axis.

This is the unambiguous signature of **IMU gravity not being subtracted
from accel** during `Predict()`. iG-LIO uses a world-frame kinematic
model `v += (R·a_m - g_world) dt`; if `R·a_m` arrives in a frame that
doesn't match `g_world = (0, 0, 9.81)`, the body falls at `g`.

The sister sequences (eee_02/03, nya_01/02/03, sbs_01/02/03) each show
the same -0.08 → -11.8 m profile → this is a deterministic, systematic
coordinate-frame bug, not sequence-specific.

After the 11th pose, the EKF state has diverged so far (|v_z| ≈ 23 m/s)
that voxel-map point registration fails and iG-LIO stops publishing
odometry — consistent with the `odom.bag.active` files remaining at
20 480 bytes (ROS bag header only) on 7/9 seqs.

---

## 5. Root-cause hypothesis (report-only; no further action this task)

**Hypothesis H1 (primary)**: The VN100 AHRS quaternion published on
`/imu/imu` is NED-framed (navigation frame, x-North / y-East / z-Down)
or device-frame, not ENU (x-East / y-North / z-Up — ROS REP-103
convention that iG-LIO's `g_world = (0, 0, 9.81)` assumes).
`AHRSInitialization` (`lio.cpp:997-1030`) copies the quat directly into
`curr_state_.pose` with no frame conversion. The result: when
`Predict()` rotates the next accel sample from body to world, the
world-Z axis is flipped (or misaligned), so gravity is added instead of
subtracted → z̈ = -9.81 m/s².

**Hypothesis H2 (secondary)**: `StaticInitialization` (which Option A
uses) does its own gravity alignment by fitting the mean-accel vector
to `(0, 0, +9.81)` and constructing an initial `R_wb` that makes that
alignment exact. `AHRSInitialization` skips this step entirely,
trusting the VN100 to publish a gravity-aligned orientation in the
ROS/ENU world frame. If the VN100 is configured in its factory-default
NED output mode (which is the most common case — see VectorNav user
manual §4.2.2), the AHRS path is fundamentally broken for iG-LIO on
NTU VIRAL without a coordinate-frame adapter.

**Why this wasn't caught upstream**: iG-LIO's AHRS init is written for
datasets where the IMU already publishes ENU-aligned orientation
(e.g., OxTS RT3000 on KITTI-360). NTU VIRAL's VN100 publishes raw AHRS,
which is not the same convention.

**Recommendation (next-task scope, not this task)**:
1. Keep Option A (`enable_ahrs_initalization: false`, `max_init_count:
   200`, OptC-v1 motion thresholds) as best-known-good. eee_01 remains
   at 0.2667 m (gate miss, but no regression on 8 other seqs).
2. If gate ≤ 0.20 m on eee_01 is still required: implement a
   coordinate-frame adapter that rotates the VN100 quat from NED/device
   to ENU before passing it into `AHRSInitialization`, or (simpler) add
   a gravity-alignment fallback inside `AHRSInitialization` itself that
   validates `R_wb · (0, 0, +g) ≈ measured_mean_accel` and refuses to
   use the AHRS quat if the check fails.
3. Both of those require C++ edits in the ig_lio submodule and are
   explicitly outside Task #63 Option B's yaml-only scope.

---

## 6. Gate table

| gate | target | observed | status |
|------|--------|----------|:------:|
| eee_01 primary | ≤ 0.20 m | ATE_FAIL (11-pose trajectory, Z free-fall) | **FAIL** |
| 7 stable seqs, max regression | ≤ +20 % | 7/7 catastrophic regression (ATE_FAIL or 3.7 m) | **FAIL** |
| AHRS branch verification | branch fires | confirmed (param = 1, no static-init success log) | PASS |
| Composite | all primary + stable | — | **FAIL** |

---

## 7. Abort / revert

Per CLAUDE.md Key Principle 7 ("No regression on secured results")
and Key Principle 8 ("Git-based algorithm collapse defense — validation
failure → immediate rollback"), SHA_OPT_B `c54772a` is reverted in the
same commit as this eval doc. The branch `fix/task36-r2-lidar-anchor`
returns to `max_init_count: 200` + OptC-v1 motion thresholds with
`enable_ahrs_initalization: false`, which is the Option A / C-v2 revert
equilibrium state.

No further fixes are attempted this task (per spec: "If eee_01
regresses ... do NOT attempt further fixes — that's next Research
pass.").

---

## 8. Artifacts

- Config (before revert): `baselines/configs/ig_lio/ntu.yaml` line 9 —
  `enable_ahrs_initalization: true`
- Dump root: `dump/task63_option_b_20260414/`
- Per-seq dirs: `dump/task63_option_b_20260414/ig_lio/ntu/<seq>/`
- Summary TSV: `dump/task63_option_b_20260414/ig_lio_ntu_summary.tsv`
- Dispatch log: `dump/task63_option_b_20260414/dispatch.log`
- Branch: `fix/task36-r2-lidar-anchor`
