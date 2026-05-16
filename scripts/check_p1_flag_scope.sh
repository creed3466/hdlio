#!/bin/bash
# S13-B.A.1 — CI hook: enforce P1 anisotropic_iekf_* keys are sensor-global only.
#
# Per sprint13_architecture §3.6 + §5: the 6 new YAML keys must NOT appear in
# per-seq overlay files. Architect §3.6 invariant I-4 (NTU bit-identical) and
# I-3 (deterministic ρ_ref_avia) require sensor-global scope.
#
# Forbidden file patterns (per-seq overlays):
#   - src/tof_slam/config/avia_v6_seq/*.yaml
#   - src/tof_slam/config/avia_indoor_seq/*.yaml
#   - src/tof_slam/config/mid360_seq/*.yaml
#   - src/tof_slam/config/ntu*.yaml (any per-seq NTU overlay)
#
# Allowed (sensor-global only):
#   - src/tof_slam/config/avia_outdoor.yaml
#   - src/tof_slam/config/avia_indoor.yaml
#   - src/tof_slam/config/unified_mid360_v3c.yaml (Mid-360 stays default-OFF)
#   - src/tof_slam/config/ros1_ntu_viral.yaml (NTU stays default-OFF)
#
# Usage:
#   bash scripts/check_p1_flag_scope.sh
#   Exit 0 = clean; Exit 1 = violation (fail build).

set -euo pipefail

cd "$(dirname "$0")/.."

KEYS=(
  "frontend_anisotropic_iekf_enable"
  "frontend_anisotropic_iekf_scalar_shim"
  "frontend_anisotropic_iekf_epsilon"
  "frontend_anisotropic_iekf_rho_ref_avia"
  "frontend_anisotropic_iekf_chi2_threshold"
  "frontend_anisotropic_iekf_sigma_theta_sq"
  # S13-B.A.5 Path B router master gate (sensor-global only, Avia outdoor only)
  "frontend_anisotropic_iekf_router_enable"
)

FORBIDDEN_GLOBS=(
  "src/tof_slam/config/avia_v6_seq"
  "src/tof_slam/config/avia_indoor_seq"
  "src/tof_slam/config/mid360_seq"
)

VIOLATIONS=0

for d in "${FORBIDDEN_GLOBS[@]}"; do
  [ -d "$d" ] || continue
  for key in "${KEYS[@]}"; do
    hits=$(grep -l "${key}" "${d}"/*.yaml 2>/dev/null || true)
    if [ -n "${hits}" ]; then
      echo "[FAIL] S13 P1 sensor-global violation: '${key}' found in per-seq overlay:" >&2
      echo "${hits}" | sed 's/^/        /' >&2
      VIOLATIONS=$((VIOLATIONS+1))
    fi
  done
done

# NTU per-seq YAML check (loose glob — covers any future per-seq NTU overlay)
for key in "${KEYS[@]}"; do
  hits=$(find src/tof_slam/config -maxdepth 2 -name 'ntu_*.yaml' -exec grep -l "${key}" {} + 2>/dev/null || true)
  if [ -n "${hits}" ]; then
    echo "[FAIL] S13 P1 sensor-global violation: '${key}' found in NTU per-seq overlay:" >&2
    echo "${hits}" | sed 's/^/        /' >&2
    VIOLATIONS=$((VIOLATIONS+1))
  fi
done

if [ ${VIOLATIONS} -eq 0 ]; then
  echo "[OK] check_p1_flag_scope: 7 P1 keys are sensor-global only (no per-seq violations)"
  exit 0
else
  echo "[FAIL] check_p1_flag_scope: ${VIOLATIONS} violation(s)" >&2
  echo "       Per sprint13_architecture §3.6/§5, anisotropic_iekf_* keys" >&2
  echo "       MUST live in sensor-global YAML only (avia_outdoor.yaml," >&2
  echo "       avia_indoor.yaml, unified_mid360_v3c.yaml, ros1_ntu_viral.yaml)." >&2
  exit 1
fi
