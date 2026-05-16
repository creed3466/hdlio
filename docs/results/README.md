# Results

Record new HDLIO server build/evaluation outcomes here.

## Entries

- `server_docker_build_20260516.md` - server Docker image/build verification
  and hardware-based parallel container plan.

Each result should include:

- git hash or patch identifier,
- server command,
- sensor/config used,
- dataset subset under `~/Project/data/`,
- server artifact directory under `~/Project/hdlio/dump/<run_label>/`,
- local artifact copy path, usually `dump/server_runs/<run_label>/`,
- artifact manifest/checksum verification status,
- remote cleanup status after verified copy,
- metrics,
- pass/fail against the stated criterion,
- design drift or open uncertainty.

Do not keep large artifacts in git. Copy them into ignored local `dump/`
storage, summarize only the reproducibility metadata here, then delete only the
verified server run directory.
