# HDLIO Refactoring Policy

This document replaces the old release-refactoring plan for the HDLIO reboot.

## Goal

Refactor only when it supports the new research direction or removes active
confusion from the one-config-per-sensor workflow. Do not perform broad cleanup
only to preserve or explain old sprint architecture.

## Priorities

1. Protect the active research surface:
   - hierarchical surfel mapping,
   - degeneracy handling,
   - estimator/map interfaces,
   - ROS 1 parameter and runner integration.
2. Remove or isolate legacy per-sequence config paths from active runners.
3. Keep old package names until an explicit rename task.
4. Prefer small, reviewable changes tied to a design document.

## What Can Change

Existing methods are not protected for backward compatibility. Old classifier
logic, old feature gates, and old surfel/degen mechanisms can be replaced when a
new design justifies the change.

## What Should Not Drive Refactoring

- old sprint closure reports,
- old canonical result preservation,
- old per-sequence tuning parity,
- release polish unrelated to the current research design.

## Required Before Large Refactors

- Write or update a design under `docs/specs/`.
- State which old behavior is intentionally abandoned.
- State the server-side build/eval plan.
- Confirm no credentials or dataset files are touched.
