# Task #53 Research — iG-LIO initialization instability

**Scope**: NTU eee_01 (ATE 1.011 m), NTU sbs_03 (0.803 m), M3DGR Avia
Dynamic03 / Dynamic04 divergence. Other 7/9 NTU seqs are fine.
**Stage**: Research (per CLAUDE.md pipeline).
**Status**: Root cause identified. Fix proposals below. Build + Eval
deferred (iG-LIO re-runs would take ~3 h).

---

## Symptom

Post lever-arm + gap-aware interp (Phase 4 final), iG-LIO on NTU:

| seq    | rmse [m] | max [m] | first err>0.5m | spike region  | peak |
|--------|---------:|--------:|---------------:|---------------|------|
| eee_01 |    1.011 |    4.36 |         t=0.0s | t=0–31.9 s    | 4.36 m |
| sbs_03 |    0.803 |    3.01 |         t=0.0s | t=0–12.6 s + 15.3–43.3 s | 3.01 m, 2.91 m |

All other 7 NTU seqs stay in the 0.13–0.22 m band, same config.
Both failing seqs show the **same signature**: error is large from
frame 0, stays large for ~30–45 s, then self-corrects to the cluster
mean. No other algorithm shows this startup bias.

Velocities in these two seqs are normal (≤2.1 m/s), well within what
the other seqs also hit. **It's startup-specific, not motion-specific.**

---

## Root cause — static init with 20 IMU samples

`baselines/algorithms/ig_lio/include/ig_lio/lio.h:261`:

```cpp
size_t max_init_count_{20};
```

`baselines/algorithms/ig_lio/src/lio.cpp:826`:

```cpp
if (imu_init_buff_.size() < max_init_count_) {
  return false;     // wait for more IMU samples
}
// ... compute mean_acc_, derive z_axis = mean_acc_.normalized()
```

**Mechanism**: iG-LIO's `StaticInitialization` takes the mean of the
first 20 IMU samples and interprets it as the gravity direction to
derive roll + pitch. At VN100's 400 Hz (NTU) that's **50 ms** of
samples. If the platform is moving during those 50 ms (takeoff
accelerations, pose change during the first lidar frame's IMU
batch), the mean no longer equals gravity — it equals gravity +
non-zero motion acceleration — and the derived orientation is wrong
by several degrees.

A wrong init roll/pitch misaligns the gravity vector inside the EKF
prediction step; the filter integrates biased acceleration until map
observations absorb the error (the 30–45 s recovery window we see).

### Why only eee_01 and sbs_03

All 9 NTU seqs have similar stationary prefixes on the spec sheet, but
in practice the rosbags start a variable amount of time *after*
takeoff. The other 7 seqs happen to start during a fully stationary
prefix; eee_01 and sbs_03 happen to start mid-motion.

Contrast with other baselines:
- **LIO-SAM**: pre-integrates ~300 IMU samples during a bag-scoped
  pre-run and uses GTSAM's iterative init.
- **FAST-LIO2**: IEKF init with convergence criteria; absorbs wrong
  orientation in the first few scans.
- **Point-LIO**: state-augmented with gyro bias from the start.

iG-LIO's 20-sample hard gate is the shortest init window of the four
— and the only one that trusts the mean of 20 samples as ground-truth
gravity.

### Supporting evidence

- Both failing seqs start with a **non-identity quaternion** in iG-LIO's
  own output (eee_01: q ≈ (0.999, 0, 0.044, 0) → ~5° pitch residual;
  sbs_03: q ≈ (0.999, 0, 0.034, −0.003)). The other 7 seqs start at
  (1, 0, 0, 0).
- **z-axis trajectory extent** is inflated on both failing seqs:
  eee_01 est z-extent 13.2 m vs gt 10.0 m (+32 %); sbs_03 9.5 m vs
  6.4 m (+48 %). Consistent with gravity-axis misalignment feeding
  bias into vertical integration.
- **Dynamic03 / Dynamic04** (M3DGR Avia) exhibit the same pattern:
  fast-moving handheld / gait-driven trajectories from the first frame
  → same short-stationary problem (tracked in the older thread under
  task #53).

---

## Fix proposals (Architect scope)

### Minimum-diff option A — raise `max_init_count_`

File: `baselines/algorithms/ig_lio/include/ig_lio/lio.h:261`.
Change `{20}` → `{200}` (default) or expose as a ROS param so per-
dataset YAMLs can tune it.

- NTU VN100 at 400 Hz × 200 samples ≈ 0.5 s of IMU — long enough for
  motion to wash out in the mean on short-prefix seqs.
- Avia IMU at 200 Hz × 200 samples ≈ 1.0 s — matches typical
  stationary prefix of the M3DGR bags.

**Downside**: users will see the algo "wait" a second before emitting
the first pose.

### Option B — enable AHRS init

File: `baselines/configs/ig_lio/ntu.yaml` and `.../ig_lio/avia.yaml`.
Flip `enable_ahrs_initalization: false` → `true`.

- `ig_lio_node.cpp:370` branches to `AHRSInitialization()` which uses
  `imu.orientation` (Madgwick / Mahony output from the IMU firmware).
  VN100 publishes its own AHRS orientation on `/imu/imu`.
- Zero-cost if the IMU provides it; falls back to static if not.

**Caveat**: VN100 AHRS yaw is absolute (magnetometer-derived) and can
differ from the lidar map's arbitrary yaw reference. Needs
verification that iG-LIO's `AHRSInitialization` handles this.

### Option C — motion-aware static init

Reject the init batch if `mean_gyr.norm() > 0.01 rad/s` or
`acc_cov > threshold`; keep collecting samples until the platform is
stationary. Would need a code change in `lio.cpp:826`.

---

## Recommendation

**A → validate on NTU eee_01 + sbs_03 + all 9 seqs (regression) →
validate on Avia Dynamic03/04 → commit.**

Concretely:

1. Add `max_init_count` as a ROS param (default 20 for backward
   compat; set 200 in NTU + Avia yamls).
2. Re-run iG-LIO × 9 NTU seqs + 2 M3DGR Avia seqs.
3. Gate: eee_01 and sbs_03 ≤ 0.20 m (into the cluster band); no
   regression on the 7 previously-good seqs.
4. If Dynamic03/04 still diverge, escalate to Option C.

Estimated cost: ~2 h of compute + 1 h of harness work.

---

## Evidence artefacts

- `dump/phase4_ntu_crosscheck_20260413/ig_lio/ntu/eee_01/traj.csv`
- `dump/phase4_ntu_crosscheck_20260413/ig_lio/ntu/sbs_03/traj.csv`
- `baselines/algorithms/ig_lio/src/lio.cpp` §StaticInitialization
- `baselines/algorithms/ig_lio/include/ig_lio/lio.h:261`
- `baselines/configs/ig_lio/ntu.yaml`

---

## What did NOT cause it

Ruled out during Research:

- **Lever arm**: fix already applied. All 5 algorithms use the same
  evaluator — 3/5 are fine on the same inputs.
- **GT dropouts**: sbs_03 GT has a 15.4 s gap at t=249 s. iG-LIO's
  error is concentrated in t=0–45 s, **not** in the gap. The gap-aware
  filter already drops those samples.
- **Extrinsics / topic mismatch**: same yaml is used for all 9 NTU
  seqs; only 2/9 fail.
- **Noise covariance tuning**: same `acc_cov/gyr_cov/ba_cov/bg_cov`
  for all 9 seqs; only 2/9 fail.
- **Aggressive motion during the run**: max velocity in eee_01
  (2.14 m/s) and sbs_03 (1.73 m/s) is comparable to the seqs where
  iG-LIO succeeds. Velocity during the spike window itself is ≤ 1.2 m/s.
