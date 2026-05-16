---
name: dataset-versioning
description: |
  Pin research dataset paths with sha256 in research-state.json and
  guard against silent dataset drift via the dataset-lock-check hook.
  Supports DVC, git-lfs, git-annex, or plain checksum lock.
  TRIGGER when: user adds/changes a dataset path, asks "how do I version
  this data", or `/reproduce` reports a checksum mismatch.
origin: rcc
---

# dataset-versioning

Dataset reproducibility is the silent #1 cause of non-reproducible
research. This skill pins datasets by content hash and integrates with
RCC's pipeline state so `/reproduce` can detect silent drift.

## Why content hashing > file timestamp

A dataset is a *value*, not a *path*. If the file at
`data/raw/train.h5` differs between two runs of the "same" experiment,
the experiments are not the same. We pin by `sha256(content)`.

## Pin a dataset

```bash
# Single file
sha256sum data/raw/train.h5

# Directory recursive (deterministic)
find data/raw -type f -print0 | sort -z | xargs -0 sha256sum
```

Record in `.claude/research-state.json`:

```json
"dataset_locks": [
  {
    "path": "data/raw/train.h5",
    "sha256": "<64-hex-chars>",
    "size_bytes": 134217728,
    "recorded_at": "2026-05-11T..."
  }
]
```

For large datasets (> 1 GB), checksum once at pin time and store the
result; don't recompute on every run. The hook compares stat (size +
mtime) first for fast-path; only re-hashes if those differ.

## Choose the right backend

| Tool | When |
|---|---|
| **plain checksum lock** (RCC default) | small datasets in-repo, simple workflows |
| **git-lfs** | binary files versioned alongside code, single git remote |
| **DVC** | large datasets with remote storage (S3/GCS) and lineage tracking |
| **git-annex** | very large files, distributed teams, dropbox-like syncing |

### git-lfs setup

```bash
git lfs install
git lfs track "data/raw/*.h5"
git add .gitattributes
git add data/raw/*.h5 && git commit -m "data: pin train.h5"
```

### DVC setup

```bash
pip install dvc[s3]
dvc init
dvc add data/raw/train.h5
git add data/raw/train.h5.dvc .gitignore
dvc remote add -d myremote s3://bucket/path
dvc push
```

After DVC add: still record the `.dvc` file's `md5` field in
`research-state.json.dataset_locks` so RCC's checks work without
invoking DVC.

## research-config.json extension

```json
{
  "dataset_lock_required_for": [
    "data/raw/**",
    "datasets/**/*.zip"
  ]
}
```

The `dataset-lock-check` hook reads this and warns/blocks edits to
matching paths if checksum has changed without explicit unlock.

## Updating a lock

```bash
# Compute new checksum
sha256sum data/raw/train.h5

# Patch research-state.json (the skill provides a helper)
```

```python
# scripts/lib/research-state.js exports pinDataset()
const rs = require('./scripts/lib/research-state');
const state = rs.ensureState(process.cwd());
const buf = require('fs').readFileSync('data/raw/train.h5');
rs.pinDataset(state, 'data/raw/train.h5', buf);
rs.writeState(process.cwd(), state);
```

## Anti-patterns

- **Don't** `rm` and re-pull a dataset without re-pinning — `/reproduce`
  will report drift even though you "didn't change anything".
- **Don't** symlink `data/raw/` to a network drive — the checksum will
  appear stable while the underlying content changes.
- **Don't** check `mtime` only — file copies preserve mtime in many
  filesystems; hash is the source of truth.

## Related

- command: `/baseline`, `/reproduce`
- hook: `dataset-lock-check`
- state field: `dataset_locks[]`
- skill: `experiment-tracking` (logs dataset hash to tracker tags)
