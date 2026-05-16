# Task #63 — iG-LIO Option C Eval Report (motion-aware static init)

**Date**: 2026-04-14
**Author**: Research Harness (automated eval)
**Scope**: Option C stacked on Option A — reject motion-dirty init windows by
mean gyro norm & per-axis accel std, fall back to least-motion window after a
safety cap.
**Status**: **GATE FAIL — DO NOT COMMIT.** Option C with the spec'd thresholds
regresses Option A. Root cause: thresholds calibrated against motion bounds
but the VN100 per-axis accel noise floor is 2–4× our cap, so every window is
rejected and the safety-cap fallback picks a biased window anyway.
**Label**: `task63_iglio_option_c_20260414_1119`
**Research basis**: `baselines/docs/task53_iglio_init_research_20260414.md` §Options
**Predecessor**: `baselines/docs/task53_iglio_init_build_eval_20260414.md`
(Option A — partial improvement: eee_01 1.011 → 0.267 m)

---

## 1. Summary

| Item | Result |
|------|--------|
| Option A checkpoint commit | `395c77c` super-repo, `d9e336d` `ig_lio` submodule |
| Option C source commit | **skipped** (abort protocol — regression on eee_01 & sbs_01) |
| Build | `baselines-ig_lio:ros1` rebuilt OK (no new warnings from our edits) |
| CPU scheduling | waited ~1 h 23 min for parallel Mid-360 matrix to clear, then acquired all 3 pools at 02:18 UTC |
| eee_01 gate | **FAIL** — 0.418 m observed (gate ≤ 0.20 m; Option A was 0.267 m) |
| Regression ≤20 % vs Option A on 7 previously-OK seqs | **FAIL** — sbs_01 +93 %, nya_02 +12 %, eee_03 +3 %, eee_02 −1 %, nya_01 +8 %, nya_03 −12 %, sbs_02 <0 % |
| sbs_03 | 0.777 m (unchanged mode, report-only) |
| Avia Dy03/Dy04 | Still diverged — report-only, different failure mode (task #65) |

---

## 2. Eval Table — 11 rows

All runs: `rosbag play -r 1.0`, 3-way container concurrency, CPU pools
p1=0-3 / p2=4-7 / p3=8-11, `baselines-ig_lio:ros1` rebuilt with Option C.
`pre-fix` column from `baselines/docs/phase4_p4_gapfix_ate_rmse.json` iG-LIO row;
`Option A` column from `baselines/docs/task53_iglio_init_build_eval_20260414.md`.

| seq         | pre-fix RMSE (m) | Option A RMSE (m) | Option C RMSE (m) | Δ A→C (%) | gate | notes |
|-------------|-----------------:|------------------:|------------------:|----------:|:----:|:------|
| eee_01      | 1.0113           | 0.2667            | **0.4183**        | **+56.9 %** | **FAIL** (>0.20 m) | regression — fallback picked a still-biased window |
| eee_02      | 0.1379           | 0.1263            | 0.1254            | −0.7 %    | OK   | |
| eee_03      | 0.1662           | 0.1680            | 0.1737            | +3.4 %    | OK   | within cap |
| nya_01      | 0.1386           | 0.1290            | 0.1391            | +7.8 %    | OK   | within cap |
| nya_02      | 0.2192           | 0.1718            | 0.1929            | +12.3 %   | OK   | within cap |
| nya_03      | 0.1529           | 0.1674            | 0.1479            | −11.6 %   | OK   | slight improvement |
| sbs_01      | 0.1409           | 0.1424            | **0.2747**        | **+92.9 %** | **FAIL regression** | breaks 20 % cap |
| sbs_02      | 0.1455           | 0.1459            | 0.1457            | −0.1 %    | OK   | |
| sbs_03      | 0.8028           | 0.7890            | 0.7765            | −1.6 %    | FAIL (>0.20 m), report-only — same as Option A |
| Avia Dy03   | n/a              | 8 650.67          | **26 928.09**     | +211 %    | report-only | still diverged (task #65) |
| Avia Dy04   | n/a              | 275 980.68        | **260 340.17**    | −5.7 %    | report-only | still diverged (task #65) |

Artefacts:
- NTU summary: `dump/task63_iglio_option_c_20260414_1119/ig_lio_ntu_summary.tsv`
- Avia summary: `dump/task63_iglio_option_c_20260414_1119/ig_lio_avia_summary.tsv`
- Per-seq ATE: `dump/task63_iglio_option_c_20260414_1119/ig_lio/{ntu,avia}/<seq>/ate.json`
- Per-seq stdout (with init rejection diagnostics): `dump/task63_iglio_option_c_20260414_1119/ig_lio/<dataset>/<seq>/stdout.log`

---

## 3. Code change (not committed; preserved in working tree + docker image)

- `baselines/algorithms/ig_lio/include/ig_lio/lio.h`
  - Added `gyr_motion_threshold`, `acc_std_threshold`, `max_init_wait_samples` to `Config`.
  - Added `init_samples_seen_`, `init_reject_count_`, `best_motion_score_`, `best_init_buff_` members.
- `baselines/algorithms/ig_lio/src/lio.cpp::StaticInitialization`
  - After computing `mean_acc_, mean_gyr_, acc_cov, gyr_cov`, evaluate
    `motion_dirty = (|mean_gyr|>gyr_motion_threshold) || (√max(acc_cov) > acc_std_threshold)`.
  - Track best-so-far by `motion_score = gyr_norm/gyr_th + acc_std/acc_th` (copy buffer on improvement).
  - If dirty and `samples_seen < cap`: clear buffer, return false (keep collecting).
  - If dirty and `samples_seen ≥ cap`: restore best-so-far buffer, recompute mean/cov, proceed (log warning).
- `baselines/algorithms/ig_lio/src/ig_lio_node.cpp`
  - Expose the 3 new ROS params with defaults `(0.01, 0.05, 4000)`.
- `baselines/configs/ig_lio/{ntu,avia}.yaml`
  - `gyr_motion_threshold: 0.01`, `acc_std_threshold: 0.05`, `max_init_wait_samples: 4000`.

Checkpoint commits created (kept on branch main):

- `395c77c [baselines] task #53 Build — iG-LIO max_init_count Option A (preparatory)`
  - super-repo pointer bump + yaml updates for Option A (no Option C yet)
- `d9e336d task #53 Option A — expose max_init_count as ROS param` (submodule `ig_lio`)

Option C edits remain in the working tree (submodule + yamls) but are **not
committed** and will be either reverted or tuned in a follow-up commit
per the recommended next step below.

---

## 4. Root cause of the Option C regression

The thresholds spec'd in the task description (`0.01 rad/s`, `0.05 m/s²`) are
too strict for the NTU VIRAL VN100 IMU. Per-stdout diagnostics from eee_01:

```
|mean_gyr|  = 0.0073–0.0086 rad/s      # BELOW threshold (would pass)
acc_std_max = 0.094–0.106 m/s²         # 2× threshold — ALWAYS rejected
```

Across 9 NTU seqs, observed `acc_std_max` floor:

| seq     | acc_std_max floor (m/s²) | rejections before safety cap | init method used |
|---------|-------------------------:|-----------------------------:|:-----------------|
| eee_01  | ~0.10                    | 17                           | safety-cap fallback |
| eee_02  | ~0.093                   | many                         | safety-cap fallback |
| nya_01  | ~0.19–0.20               | many                         | safety-cap fallback |
| sbs_01  | ~0.17                    | many                         | safety-cap fallback |
| sbs_03  | ~0.13                    | many                         | safety-cap fallback |

Every sequence exhausts the 4 000-sample cap and falls back to the
"least-motion" window — which on `eee_01` is still mid-walk motion because the
whole first 10 s is mid-motion, just at different magnitudes. Consequence:

1. **Fallback window ≠ stationary window** — we pick the *least bad* among
   dirty windows, not a clean one.
2. **Delayed init by ≈10 s** — init now occurs ~10 s later than Option A,
   changing the first-correspondence LiDAR geometry and producing a
   different (here, worse) trajectory.
3. **sbs_01 harder hit** — its acc_std floor (~0.17) is the highest among
   stable seqs, so the fallback window is substantially delayed and happens
   to coincide with a geometrically unfavourable moment.

The gyr threshold (0.01 rad/s) works as intended — NTU IMU at rest sits
around 0.008 rad/s. But the accel-std gate is miscalibrated: raw VN100
per-axis accel RMS noise is in the 0.1–0.2 m/s² range, not 0.05.

---

## 5. Abort decision

Per the task abort protocol ("if any of the 7 Option A-OK seqs regress >20 %,
do NOT commit"):

- sbs_01 regressed **+93 %** → unconditional block.
- eee_01 also regressed vs Option A (+57 %) — worse than the existing
  baseline we were trying to improve.

→ **Option C as spec'd is NOT committed.** Option A checkpoint (`395c77c`)
stays on branch main. Option C source edits remain uncommitted in the
working tree.

---

## 6. Recommended next step

The motion-aware idea is theoretically correct but the acceleration gate
needs re-calibration against observed IMU noise floors. Two concrete options:

### 6.1 Option C′ — Relax `acc_std_threshold` to 0.25 m/s² (NTU VN100)

Observed per-seq acc_std floors: 0.09, 0.13, 0.17, 0.20. Using `0.25 m/s²` would
allow every "stationary" window to pass the gate and let `gyr_motion_threshold
= 0.01` do the heavy lifting — reject only when the platform is actively
turning/accelerating. This matches FAST-LIO2's init heuristic (which uses
`gyr_norm` alone as the motion gate, no accel std).

**Predicted outcome**: no delay on eee_02, eee_03, nya_*, sbs_02 (they pass on
first window). On eee_01 and sbs_03 — same as Option A, since their mid-motion
gyr norms (≈0.008 rad/s) are below the 0.01 gate. *No improvement over
Option A* — but also no regression.

### 6.2 Option C″ — Gate on **accel-magnitude deviation from g**, not per-axis std

Better physics: when stationary, `||mean_acc|| ≈ g` and individual sample
magnitudes sit within one IMU-noise-σ of g. Instead of per-axis covariance,
check:

```
|||mean_acc|| − g| > threshold_g              (e.g. 0.1 m/s²)
std(||acc_i||)  > threshold_mag_std           (e.g. 0.15 m/s²)
```

This is directly tied to the physical invariant we are trying to exploit
(gravity magnitude), not to raw-sample noise. eee_01 mid-walk would give
`|mean_acc|` deviating from g by ≈0.3–0.5 m/s², which would trigger the gate;
stationary windows (if any) would pass.

### 6.3 Option B — AHRS init (VN100 firmware orientation)

Per the Research doc §Options, NTU bags publish `/imu/imu` with non-identity
orientation quaternions from the VN100 AHRS. Setting
`enable_ahrs_initalization: true` and feeding the first orientation directly
as the init attitude bypasses the whole mean-of-accel problem.

Implementation status: the iG-LIO code already has `AHRSInitialization` and
an `enable_ahrs_initalization` ROS param. We set it to `false` on NTU because
Option A was meant to fix static init; AHRS init should be re-evaluated next
as Option B.

### Recommendation

**Run Option C′ (threshold relaxation, `acc_std_threshold: 0.25`)** first —
cheap (no code change, just yaml), validates that the delay-vs-direct-use
mechanism is what broke sbs_01. If that's a no-op (Option A-equivalent), move
to Option B (AHRS).

---

## 7. Separate failure modes (not Task #63 scope)

- **sbs_03** — Option A & Option C both land ~0.78 m. The problem is
  mid-run divergence, not init. Tracked as task #64.
- **Avia Dynamic03 / Dynamic04** — catastrophic divergence on both options.
  Root cause is unrelated to static init (IMU noise params, extrinsic, or
  time_scale mismatch). Tracked as task #65.

---

## 8. Artefacts

- Eval root: `dump/task63_iglio_option_c_20260414_1119/`
- Per-seq stdout with init diagnostics: see `stdout.log` grep for `motion-dirty|safety cap|imu static`
- Runner script: `baselines/scripts/run_task63_iglio_option_c.sh`
- Research doc: `baselines/docs/task53_iglio_init_research_20260414.md`
- Predecessor eval: `baselines/docs/task53_iglio_init_build_eval_20260414.md`
- Baseline JSON: `baselines/docs/phase4_p4_gapfix_ate_rmse.json`
