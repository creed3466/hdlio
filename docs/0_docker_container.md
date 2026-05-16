# Server Build and Evaluation Notes

Build and evaluation for HDLIO run on the server, not on the local workstation.
Local work stops at research/analysis, design, and implementation.

## Server

| Item | Value |
|------|-------|
| SSH target | `eutae@192.168.0.42` |
| Project path | `~/Project/hdlio/` |
| Dataset root | `~/Project/data/` |

Do not store passwords in scripts, config files, docs, memory, or shell history.
Use SSH key auth or prompt-based auth.

## Dataset Policy

All server-side dataset paths should resolve under:

```bash
~/Project/data/
```

Do not hardcode the old `~/Project/dataset/...` paths in new scripts. If a
legacy runner still uses old paths, update the runner as part of the migration
before using it for active HDLIO evaluation.

## Config Policy

Evaluation must use one config per sensor family:

- Avia: `src/tof_slam/config/avia.yaml`
- Mid360: `src/tof_slam/config/mid360.yaml`
- NTU/Ouster: `src/tof_slam/config/ntu.yaml`

Per-sequence configs and old sprint tuning configs are legacy-only.

## Remote Workflow

```bash
ssh eutae@192.168.0.42
cd ~/Project/hdlio/
```

Before running evaluation, confirm:

- repository state or patch matches the local implementation,
- selected sensor config is one of the three active configs,
- dataset path is under `~/Project/data/`,
- output directory is unique for the run,
- falsification criteria are defined in `docs/specs/` or `docs/research/`.

## Result Recording

After a server run, record a concise result under `docs/results/` with:

- git hash or patch identifier,
- server command,
- sensor/config used,
- dataset subset,
- metric output,
- pass/fail against the stated criterion,
- observed drift from the design.

## Legacy Scripts

The `docker/` directory contains many historical runners. Treat them as
implementation references until they are migrated to the new server dataset root
and one-config-per-sensor policy. Do not treat old canonical/per-sequence runner
behavior as active HDLIO policy.
