# HDLIO Project Overlay

> Reboot date: 2026-05-16.
> This workspace is now used for new HDLIO algorithm development. Historical
> TofSLAM names may remain in package paths until an explicit rename task.

## Project Scope

HDLIO is a fresh LiDAR-Inertial Odometry research project. The current codebase
is an implementation substrate, not a source of mandatory algorithmic
dependency. Existing methods, sprint conclusions, per-sequence tuning results,
and prior paper/SOTA numbers are legacy reference only.

Local work is limited to research/analysis, design, and implementation. Build
and evaluation run on the server.

## Server Boundary

| Item | Value |
|------|-------|
| SSH target | `eutae@192.168.0.42` |
| Project path | `~/Project/hdlio/` |
| Dataset root | `~/Project/data/` |

Do not store passwords in repo files, memory, shell scripts, SSH config, or
logs. Use SSH key auth or prompt-based auth.

Server run artifacts must be copied back and verified before cleanup. Delete
only the specific run directory under `~/Project/hdlio/dump/<run_label>/` after
the copied artifact manifest passes locally. Never bulk-delete datasets,
repository source, `build/`, `devel/`, Docker images, or the whole `dump/`
directory unless the user explicitly asks for that cleanup.

## Algorithm Development Rules

1. Use exactly one active algorithm config per sensor family:
   - Avia: `src/tof_slam/config/avia.yaml`
   - Mid360: `src/tof_slam/config/mid360.yaml`
   - NTU/Ouster: `src/tof_slam/config/ntu.yaml`
2. Per-sequence configs are deprecated for active development. Do not add,
   tune, route, or evaluate with sequence-specific YAMLs.
3. Dataset paths, topic remaps, bag names, and output folders belong in launch
   or runner arguments, not in per-sequence algorithm configs.
4. Do not preserve previous algorithm choices for compatibility. Replace or
   delete old mechanisms when a new design justifies it.
5. New work must aim for novelty. A change that only replays old sprint logic,
   classifier templates, or per-sequence tuning is out of scope.

## Research Directions

Explore novelty under two primary categories:

- Hierarchical surfel mapping: map representation, multi-resolution update
  rules, surfel uncertainty, cross-level correspondence, map aging, and
  information flow between levels.
- Degeneracy: detection, observability-aware updates, information projection,
  regularization, motion priors, and failure-mode-aware estimator behavior.

Existing implementations under these categories are editable and replaceable.
Treat them as candidate code, not fixed methodology.

## Workflow

1. Research: write the hypothesis, evidence, and falsification criteria under
   `docs/research/`.
2. Design: before non-trivial implementation, write or update a short design
   under `docs/specs/`.
3. Build: implement locally with narrow source changes.
4. Eval: run build/evaluation only on `eutae@192.168.0.42:~/Project/hdlio/`.
5. Results: record server results under `docs/results/`.
6. Artifact cleanup: copy server artifacts to local ignored storage, verify the
   manifest/checksums, then remove only the verified remote run directory.

If implementation diverges from the design, update the design and state the
drift explicitly.

## Active Documentation

- `docs/README.md` — active documentation index.
- `docs/hdlio_onboarding.md` — current workspace and architecture overview.
- `docs/specs/hdlio_reboot_20260516.md` — reboot design rules.
- `docs/canonical_config_map.md` — active sensor config policy.
- `docs/0_docker_container.md` — server build/eval operating notes.

Historical sprint/report/research documents were removed from the active repo.
They must not be restored as active constraints or success criteria.

## Codebase Notes

- Current ROS package paths still use `tof_slam`; do not rename broadly unless
  requested.
- Primary implementation surface is `src/tof_slam/`.
- ROS 1 remains the main execution path unless a task explicitly targets ROS 2.
- Keep code changes scoped to the research/design objective.

## Git Conventions

- Commit format: `<type>: <description>`.
- Never commit credentials, datasets, large dumps, or generated build products.
- Before handing off to server eval, summarize the intended command, config,
  dataset subset, and expected falsification criteria.
