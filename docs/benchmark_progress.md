# HDLIO Benchmark Policy

This document replaces the old benchmark-progress narrative for the HDLIO
reboot.

## Current Status

No active HDLIO benchmark claim is established yet. Prior TofSLAM/Sprint/SOTA
numbers are legacy archaeology and must not be used as current success criteria.

## Benchmark Principles

- Benchmark only after a hypothesis and design define the expected mechanism.
- Use one active config per sensor family.
- Do not compare a new algorithm against per-sequence tuned HDLIO outputs.
- Dataset paths on the server must resolve under `~/Project/data/`.
- Record new results under `docs/results/`, not `docs/reports/`.

## Required Result Metadata

Every benchmark result must state:

- git hash or patch identifier,
- server command,
- sensor config (`avia.yaml`, `mid360.yaml`, or `ntu.yaml`),
- dataset subset,
- metric implementation,
- pass/fail criterion,
- whether any runner or evaluator migration was required.

## Baseline Role

External baselines may be used to frame results, but they are not the first
development driver. The first driver is whether the new hierarchical surfel map
or degeneracy design behaves according to its stated mechanism.

## Deprecated

Old benchmark tables, old paper-canonical numbers, and old sprint closure
metrics are not active HDLIO requirements.
