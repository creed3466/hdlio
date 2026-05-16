# HDLIO Onboarding

## Overview

This repository is now the HDLIO development workspace. The project is being
restarted from a clean research direction while reusing the existing ROS/C++
codebase as a substrate. Historical TofSLAM naming remains in paths such as
`src/tof_slam/`; that naming does not imply that old algorithm choices are
mandatory.

Local work covers research/analysis, design, and implementation. Build and
evaluation happen on the remote server.

## Operating Model

| Stage | Location | Output |
|-------|----------|--------|
| Research / analysis | Local | `docs/research/` |
| Design | Local | `docs/specs/` |
| Implementation | Local | `src/tof_slam/` |
| Build | Server | `eutae@192.168.0.42:~/Project/hdlio/` |
| Evaluation | Server | `docs/results/` summaries after server runs |

Dataset root on the server is `~/Project/data/`.

## Current Development Rules

- Use one active algorithm config per sensor: Avia, Mid360, NTU/Ouster.
- Do not use per-sequence configs for active development.
- Do not derive new hypotheses from old sprint conclusions as constraints.
- New research must target novelty, mainly in hierarchical surfel mapping and
  degeneracy handling.
- Existing implementations in those areas may be rewritten or removed.

## Architecture Map

- `src/tof_slam/include/tof_slam/common/`, `src/tof_slam/src/common/`:
  ROS-independent math, Lie groups, state, and point types.
- `src/tof_slam/include/tof_slam/frontend/`, `src/tof_slam/src/frontend/`:
  core LIO pipeline, estimator, IEKF, correspondence, surfel/PV maps, and
  degeneracy-related logic.
- `src/tof_slam/include/tof_slam/ros1/`, `src/tof_slam/src/ros1/`:
  ROS 1 node wrappers, deterministic event queue, Livox/PointCloud2 ingestion,
  and parameter loading.
- `src/tof_slam/config/`: active config surface should converge to
  `avia.yaml`, `mid360.yaml`, and `ntu.yaml`. Existing sequence/tuning configs
  are legacy unless a migration task says otherwise.
- `docker/`: server-side build/eval scripts and metrics utilities.
- `baselines/`: baseline algorithm infrastructure; use only when a design or
  evaluation plan explicitly needs it.

## Key Entry Points

- ROS 1 executable: `src/tof_slam/src/ros1/main.cpp`
- ROS 1 wrapper: `src/tof_slam/src/ros1/slam_node.cpp`
- Core estimator: `src/tof_slam/include/tof_slam/frontend/estimator/lio_estimator.hpp`
- Core estimator implementation: `src/tof_slam/src/frontend/estimator/lio_estimator.cpp`
- Build graph: `src/tof_slam/CMakeLists.txt`
- Config policy: `docs/canonical_config_map.md`
- Reboot design: `docs/specs/hdlio_reboot_20260516.md`

## Legacy Boundary

The following legacy material was removed or reduced to README notices:

- `docs/closed-sprints/`
- `docs/reports/`
- old sprint files under `docs/research/`
- old per-sequence config reports and canonical result maps

Do not recreate them as active requirements. If a legacy idea is reused, restate
it as a new hypothesis with a fresh design and evaluation criterion.
