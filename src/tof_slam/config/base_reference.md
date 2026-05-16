# HDLIO Config Reference

This file describes the active HDLIO config policy. Historical values from old
per-sequence tuning campaigns are not active requirements.

## Active Configs

| Sensor family | Target file | Rule |
|---------------|-------------|------|
| Avia | `avia.yaml` | One config for all Avia runs |
| Mid360 | `mid360.yaml` | One config for all Mid360 runs |
| NTU/Ouster | `ntu.yaml` | One config for all NTU VIRAL runs |

Historical per-sequence, tuning, ablation, and indoor/outdoor split config files
were removed from the active repo.

## Deprecated

Do not add or route active runs through:

- `avia_v6_seq/`
- `avia_indoor_seq/`
- `mid360_seq/`
- `tuning/`
- `ablation/`
- `_ablation_merged_*.yaml`

## What Belongs in Sensor Config

- Sensor-level noise assumptions.
- Sensor-level preprocessing parameters.
- Algorithm feature flags under active research.
- Parameters for hierarchical surfel mapping.
- Parameters for degeneracy detection/handling.

## What Does Not Belong in Sensor Config

- Sequence name.
- Dataset path.
- Bag filename.
- Output directory.
- Per-sequence overrides.
- Hidden scene-class routing tables that reproduce per-sequence tuning.

## Migration Target

Update runners so sensor selection determines the config. Active runners should
fail if they reference removed per-sequence config paths.
