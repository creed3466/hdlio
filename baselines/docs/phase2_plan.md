# Phase 2 Plan — Sensor Adaptation + Tier-1 × 27 Sequence Execution

Precondition: Phase 1 pipeline validated for 4 of 5 algorithms on smoke
sequences (FAST-LIO2, Point-LIO, iG-LIO, LIO-SAM). SLICT remains blocked
at upstream source level (ufomap_ros uses ROS2 C++ API).

Goal: produce per-sequence ATE tables for all 4 pipeline-passing algorithms
across the full Tier-1 matrix — 27 sequences × 4 algorithms = 108 runs —
before any paper-reproduction audit.

---

## Scope

### Algorithms (4)
- FAST-LIO2 (avia, mid360) — dataset-native
- Point-LIO (avia, mid360) — dataset-native
- iG-LIO (avia, mid360) — dataset-native
- LIO-SAM (ntu) — 9-axis IMU only

### Sequences (27)
- **M3DGR Avia** (9): Dark01-02, Dynamic01-04, Occluded01, Varying-illu01,
  Varying-illu02, Varying-illu03
- **M3DGR Mid-360** (9): same scene IDs, different lidar modality
- **NTU VIRAL** (9): eee_{01..03}, nya_{01..03}, sbs_{01..03}

### Run matrix
- Avia algos × 9 avia seqs = 27 runs × 3 algos = 81
- Avia algos × 9 mid360 seqs = requires Mid-360 adaptation (topic remap,
  config swap) — treat as separate sub-phase.
- LIO-SAM × 9 NTU seqs = 9 runs
- Total Tier-1: **108 runs** (81 Avia, 9 Mid-360 bonus if time permits, 9 NTU).

---

## Work items

### P2-A. Config-level sensor adaptation (blocking)

For each of FAST-LIO2 / Point-LIO / iG-LIO:
1. Verify Mid-360 config exists at `baselines/configs/<algo>/mid360.yaml`
   (topic remap, lidar model, Horizon_SCAN, extrinsics).
2. Validate against one Mid-360 smoke sequence before full Tier-1 run.
3. If Mid-360 path is not cleanly supported upstream, scope-reduce to
   Avia-only Tier-1 and move Mid-360 to Phase 3.

For LIO-SAM on NTU VIRAL:
1. Fix `extrinsicRot` / `extrinsicRPY` for VIRAL VN100→Ouster OS1-16
   transform (reference: SLICT paper or upstream LIO-SAM NTU configs).
2. Audit IMU axis convention (VN100 ENU vs LIO-SAM internal).
3. Re-run eee_01 smoke — target 568 m → < 10 m before accepting.
4. Apply validated config to all 9 NTU sequences.

**P2-A LIO-SAM status (2026-04-13, RESOLVED):** fork switch validated.
eee_01 ATE = 0.206 m after switching submodule to `brytsknguyen/LIO-SAM`
master (43b80a0). See `phase1_smoke_results.md` §"Update (2026-04-13,
Phase 2 P2-A resolution)" for full numbers + updated patch list.

Original (pre-fix) status kept for the audit trail:
Two fixes attempted, both falsified on eee_01:
- r1: Z-sign correction on `extrinsicTrans` (`[-0.050, 0, -0.055]` →
  `[-0.050, 0, 0.055]`, canonical SLICT value) → RMSE 568 → 431 m.
- r2: `useImuHeadingInitialization: false` (bypass ENU↔NED confusion) →
  RMSE 431 → 437 m (no improvement).

Research (2026-04-13) confirmed our `ntu.yaml` extrinsics now match the
authoritative `brytsknguyen/LIO-SAM` + `brytsknguyen/slict` canonical values
(identity Rot/RPY, `extrinsicTrans: [-0.050, 0, 0.055]`). The remaining
divergence is not a config issue — Z integrates to +25 m within 1 s, +141 m
within 2 s, i.e. ~5g upward acceleration → gravity-sign or world-frame bug
in vanilla TixiaoShan master.

Root cause (per Research): vanilla TixiaoShan/LIO-SAM lacks the
`R_W2NED = diag(1, -1, -1)` correction that `brytsknguyen/LIO-SAM` master
carries specifically for NTU VIRAL's VN100 ENU-framed IMU. No config flag
in vanilla can substitute for this source-level patch.

**Decision options for P2-A:**
1. **Fork switch** — replace `baselines/algorithms/lio_sam` submodule with
   `brytsknguyen/LIO-SAM` (NTU-validated VN100 pipeline). New Docker build,
   new 5-patch audit. Estimated effort: 2–3 hours.
2. **Patch-port** — cherry-pick `R_W2NED` + `imuConverter` diff from
   brytsknguyen fork into our vanilla build. Smaller diff but needs careful
   testing. Estimated effort: 1–2 hours.
3. **Drop LIO-SAM NTU** — ship Phase 2 with Avia Tier-1 (81 runs across
   FAST-LIO2 / Point-LIO / iG-LIO) only. Mark LIO-SAM NTU divergence as a
   known blocker; revisit in Phase 3.

Recommendation: **option 1 (fork switch)**. The brytsknguyen fork is the
canonical NTU VIRAL LIO-SAM path per the dataset authors themselves, so
Phase 4 paper-reproduction numbers will also need it. Front-loading the
switch now avoids double work.

### P2-B. Runner harness hardening

1. **Parallel execution** — extend `run_baseline.sh` or add `run_tier1.sh`
   that drives N containers in parallel (respecting `docs/0_docker_container.md`
   port/CPU pinning: p1=0-3 / 11311, p2=4-7 / 11312, p3=8-11 / 11313).
2. **Retry on transient failure** — Docker build hiccups, roscore port
   collision → auto-retry once before marking FAIL.
3. **Artifact hygiene** — `dump/baselines_<date>/<algo>/<dataset>/<seq>/`
   with `{traj.csv, ate.json, stdout.log, odom.bag}` — same layout as Phase 1.
4. **TUM-format GT caching** — one-time convert NTU `ground_truth.csv` +
   M3DGR GT into cached `<dataset>_<seq>_gt.tum` files (shared across runs).

### P2-C. Aggregation + reporting

1. `baselines/scripts/summarize_tier1.py` — walks `dump/baselines_<date>/`,
   collects all `ate.json`, produces:
   - Master table (algo × sequence → RMSE, mean, max, n_poses, status).
   - Per-algo CSV for downstream figures.
   - Pass/fail grading (no divergence, ATE within plausible range).
2. Commit the aggregated table to `docs/reports/phase2_tier1_<date>.md`.

### P2-D. SLICT decision

Per Phase 1 note: port `ufomap_ros/conversions.{h,cpp}` to ROS1 API (1–2
days) **OR** drop SLICT and substitute another surfel-LIO (e.g.,
CT-ICP-ROS1 if available). Decision point:
- If Phase 2 budget allows and port is ≤ 2 days → attempt port.
- Otherwise defer SLICT to Phase 3 or drop.

---

## Success criteria (Phase 2 gate → Phase 3)

1. ≥ 3 of 4 algorithms run end-to-end on all 9 sequences of their primary
   dataset (Avia for FAST-LIO2/Point-LIO/iG-LIO, NTU for LIO-SAM).
2. No catastrophic divergence (RMSE < 10 m) on any sequence that the
   algorithm's paper claims to handle.
3. Aggregated table committed under `docs/reports/`.
4. Known blockers (Mid-360 config gaps, SLICT decision) explicitly
   documented with next-step owners.

Phase 3 begins once Phase 2 gate passes. Phase 4 (paper reproduction per
`paper_reproduction_targets.md`) requires Phase 2 + per-algorithm config
tuning.

---

## Risks & mitigations

- **Mid-360 topic/config drift**: upstream configs assume Avia. Mitigation
  — treat Mid-360 as optional sub-phase; ship Avia-only if blocked.
- **LIO-SAM NTU divergence** (Phase 1): without correct extrinsics, any
  NTU number is noise. Block Phase 2 NTU rollout until eee_01 post-fix
  ATE < 10 m.
- **Container port collisions** in parallel runs: enforce per-container
  ROS port pinning via env (11311/11312/11313) as specified in
  `docs/0_docker_container.md`.
- **Determinism regression**: run each sequence once at `-r 1.0` for the
  published Phase 2 table. Re-run spot-checks (3 seqs × 3 repeats) to
  validate determinism envelope.
