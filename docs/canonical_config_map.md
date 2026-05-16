# HDLIO Active Config Policy

> The filename is kept for compatibility with existing links. Its content is
> now the HDLIO reboot config policy, not the old per-sequence canonical map.

## Rule

HDLIO uses exactly one active algorithm config per sensor family.

| Sensor family | Active target config | Scope |
|---------------|----------------------|-------|
| Avia | `src/tof_slam/config/avia.yaml` | All Avia sequences/datasets |
| Mid360 | `src/tof_slam/config/mid360.yaml` | All Mid360 sequences/datasets |
| NTU/Ouster | `src/tof_slam/config/ntu.yaml` | All NTU VIRAL Ouster sequences |

Dataset path, bag name, sequence name, topic remap, output directory, and run
rate are runner/launch concerns. They must not create algorithm-level
per-sequence YAMLs.

## Deprecated Config Surfaces

These paths are legacy and must not be used for active HDLIO development:

- `src/tof_slam/config/avia_v6_seq/`
- `src/tof_slam/config/avia_indoor_seq/`
- `src/tof_slam/config/mid360_seq/`
- `src/tof_slam/config/tuning/`
- `src/tof_slam/config/ablation/`
- `src/tof_slam/config/_ablation_merged_*.yaml`

Existing files may remain until a cleanup task removes or archives them. Do not
add new files under these surfaces.

## Current State

The active config surface has been reduced to the three sensor configs plus
supporting non-algorithm assets such as RViz and this reference file. Historical
per-sequence, tuning, ablation, indoor/outdoor split, and merged-ablation YAMLs
were removed from the active repo.

## Design Constraints

- Config values should describe sensor-level assumptions, not sequence identity.
- Runtime classifiers may be researched, but they must not become hidden
  per-sequence routing tables.
- A sensor config can expose feature flags for an experiment only when the
  corresponding design document states the hypothesis and falsification gate.
- Config changes are part of the algorithm. Record them in the design or result
  document for the run.

## Expected Cleanup

1. Update server runners to select configs by sensor, not sequence.
2. Add a guard that fails if active eval scripts reference deprecated config
   directories.
