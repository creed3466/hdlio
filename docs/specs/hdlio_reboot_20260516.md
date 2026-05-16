# HDLIO Reboot Design

## Objective

Restart HDLIO algorithm development from a clean research direction while
reusing the existing ROS/C++ codebase as an implementation substrate.

## Hard Rules

1. One active config per sensor family: Avia, Mid360, NTU/Ouster.
2. No per-sequence algorithm configs.
3. Old sprint results and old per-sequence tuning are legacy only.
4. New work must target novelty rather than preserving previous methodology.
5. Local work covers research, design, and implementation only.
6. Server build/eval uses `eutae@192.168.0.42:~/Project/hdlio/` and datasets
   under `~/Project/data/`.

## Research Axes

### Hierarchical Surfel Mapping

Questions to explore:

- How should surfel statistics be represented across levels?
- What information should flow upward/downward between levels?
- Can cross-level uncertainty prevent local map contamination?
- Should correspondence selection be a map-query problem or an estimator-state
  problem?
- How should map aging, locking, and replacement be formulated without
  sequence-specific tuning?

### Degeneracy

Questions to explore:

- How should observability be measured from active correspondences?
- Which state subspaces should be trusted, projected, regularized, or deferred?
- Can degeneracy handling be continuous rather than a brittle mode switch?
- How should degeneracy interact with hierarchical surfel updates?
- What failure criteria identify map contamination versus estimator
  under-observability?

## Implementation Boundary

Existing code can be changed or removed. The main implementation surfaces are:

- `frontend/map/`
- `frontend/estimator/`
- `frontend/policy/`
- `ros1/` parameter and runner integration

Avoid broad package renames until explicitly requested.

## Evaluation Boundary

Evaluation is not a local completion criterion. A local implementation is ready
for server evaluation when:

- the design document states the hypothesis,
- the active sensor config is identified,
- no per-sequence config is introduced,
- the expected metric and falsification condition are clear,
- the server command can resolve datasets under `~/Project/data/`.

## Initial Migration Tasks

- Update runners to select configs by sensor.
- Add a guard against active runner references to deprecated config dirs.
- Remove old sprint/report conclusions from active documentation paths.
