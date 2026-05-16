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
- output directory is unique for the run and uses a safe label matching
  `[A-Za-z0-9._-]+`,
- falsification criteria are defined in `docs/specs/` or `docs/research/`.

## Artifact Copy and Cleanup Policy

Every server run writes artifacts to one run-specific directory:

```bash
~/Project/hdlio/dump/<run_label>/
```

Use a short label with only letters, numbers, dot, underscore, or dash. Do not
use spaces, shell metacharacters, absolute paths, or `..` in labels.

Required artifact lifecycle:

1. Generate a manifest on the server before copying.
2. Copy the run directory to local ignored storage.
3. Verify the manifest/checksums locally.
4. Record the result summary under `docs/results/`.
5. Delete only the verified remote run directory.

Manifest command on the server:

```bash
cd ~/Project/hdlio/dump/<run_label>
du -sh . > artifact-size.txt
find . -type f ! -name artifact-manifest.sha256 -print0 \
  | sort -z \
  | xargs -0 sha256sum > artifact-manifest.sha256
```

Copy command from the local workstation:

```bash
mkdir -p dump/server_runs/<run_label>
rsync -a --partial --human-readable \
  eutae@192.168.0.42:~/Project/hdlio/dump/<run_label>/ \
  dump/server_runs/<run_label>/
```

Local verification:

```bash
cd dump/server_runs/<run_label>
if command -v sha256sum >/dev/null 2>&1; then
  sha256sum -c artifact-manifest.sha256
else
  shasum -a 256 -c artifact-manifest.sha256
fi
```

Remote cleanup is allowed only after local verification succeeds. Use a guarded
delete pattern and never delete broad paths:

```bash
ssh eutae@192.168.0.42 'set -euo pipefail
label="<run_label>"
case "$label" in ""|*[!A-Za-z0-9._-]*)
  echo "unsafe run label: $label" >&2
  exit 2
  ;;
esac
base="$HOME/Project/hdlio/dump"
target="$base/$label"
[ -d "$target" ] || { echo "already absent: $target"; exit 0; }
base_real="$(realpath "$base")"
target_real="$(realpath "$target")"
case "$target_real" in
  "$base_real"/*) rm -rf --one-file-system -- "$target_real" ;;
  *) echo "refusing to delete outside $base_real: $target_real" >&2; exit 3 ;;
esac
'
```

Forbidden cleanup targets unless the user explicitly requests them:

- `~/Project/data/`
- `~/Project/hdlio/src/`
- `~/Project/hdlio/build/`
- `~/Project/hdlio/devel/`
- `~/Project/hdlio/dump/` as a whole
- Docker images, volumes, build cache, or `docker system prune`

## Result Recording

After a server run, record a concise result under `docs/results/` with:

- git hash or patch identifier,
- server command,
- sensor/config used,
- dataset subset,
- local artifact copy path and verification status,
- remote cleanup status,
- metric output,
- pass/fail against the stated criterion,
- observed drift from the design.

## Legacy Scripts

The `docker/` directory contains many historical runners. Treat them as
implementation references until they are migrated to the new server dataset root
and one-config-per-sensor policy. Do not treat old canonical/per-sequence runner
behavior as active HDLIO policy.
