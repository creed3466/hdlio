#!/usr/bin/env bash
# Run all Faster-LIO experiments sequentially (one at a time).
# Usage: bash baselines/scripts/run_all_faster_lio.sh
set -euo pipefail

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
cd "${REPO_ROOT}"

LABEL="faster_lio_paper"
ALGO="faster_lio"

run_one() {
    local dataset="$1" seq="$2"
    local out_dir="dump/${LABEL}/${ALGO}/${dataset}/${seq}"
    if [[ -f "${out_dir}/traj.csv" ]]; then
        echo "[SKIP] ${dataset}/${seq} — already done"
        return 0
    fi
    echo "========================================"
    echo "[START] ${ALGO} ${dataset}/${seq}"
    echo "========================================"
    bash baselines/scripts/run_baseline.sh "${ALGO}" "${dataset}" "${seq}" "${LABEL}" || {
        echo "[FAIL] ${dataset}/${seq} — exit $?"
        return 0  # continue to next
    }
    echo "[OK] ${dataset}/${seq}"
}

echo "=== M3DGR Avia (9 sequences) ==="
for seq in Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05; do
    run_one avia "$seq"
done

echo "=== M3DGR Mid-360 (9 sequences) ==="
for seq in Dark01 Dark02 Dynamic03 Dynamic04 Occlusion03 Occlusion04 Varying-illu03 Varying-illu04 Varying-illu05; do
    run_one mid360 "$seq"
done

echo "=== NTU VIRAL (9 sequences) ==="
for seq in eee_01 eee_02 eee_03 nya_01 nya_02 nya_03 sbs_01 sbs_02 sbs_03; do
    run_one ntu "$seq"
done

echo "=== M3DGR Indoor (8 sequences) ==="
for seq in Dark03 Dark04 Dynamic01 Dynamic02 Occlusion01 Occlusion02 Varying-illu01 Varying-illu02; do
    run_one indoor "$seq"
done

echo "========================================"
echo "ALL DONE. Results in dump/${LABEL}/"
echo "========================================"
find "dump/${LABEL}/" -name "traj.csv" | wc -l
echo "trajectory files produced"
