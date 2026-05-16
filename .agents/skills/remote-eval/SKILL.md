---
name: remote-eval
description: |
  Probe configured SSH/Docker research servers, run project evals inside a
  remote Docker container when resources are available, collect artifacts back
  into dump/<label>, and record reproducibility metadata in research-state.json.
  TRIGGER when: user asks to use another server, GPU box, SSH host, Docker
  remote eval, canonical eval, benchmark, or "server resources if available".
  DO NOT TRIGGER for ordinary local unit tests or app-dev Docker workflows.
origin: rcc
tools: Read, Write, Bash
---

# remote-eval

Remote eval is a deterministic execution adapter. Ben and the research agents
decide what should be evaluated; this skill decides whether a locally registered
remote resource is reachable and runs the already-defined eval command there.

## When To Use

- The project has a heavy canonical eval, benchmark, training run, or GPU test.
- The user wants Claude main to continue research locally while a server runs
  Dockerized evals.
- A configured SSH host may be available and should be used opportunistically.
- The result must be collected as artifacts and compared against a baseline.

Do not use this skill as a substitute for Research, Architect, or Codex Review.
It only executes and records the eval contract those stages already defined.

## Configuration

Keep project research requirements in `.claude/research-config.json` first, then
`research-config.json`. Keep machine-specific server details in the local
registry at `$RCC_REMOTE_REGISTRY`, or `~/.config/rcc/remote-resources.json` by
default. Tests and docs must not write this file directly.

```json
{
  "remote_eval": {
    "enabled": true,
    "resource_profile": "gpu-large",
    "requires": ["docker", "gpu"],
    "host_allowlist": ["gpu-a"],
    "probe_cache_ttl_sec": 300,
    "default_timeout_sec": 14400,
    "artifact_dir": "dump"
  }
}
```

Local registry example:

```json
{
  "version": 1,
  "profiles": {
    "gpu-large": {
      "requires": ["docker", "gpu"],
      "labels": ["cuda"]
    }
  },
  "hosts": [
    {
      "id": "gpu-a",
      "ssh": "rcc-gpu-a",
      "labels": ["docker", "cuda"],
      "workdir": "~/rcc-runs",
      "max_parallel": 2,
      "gpu_slots": ["0", "1"],
      "docker": {
        "allowed_images": ["project-eval:*", "ghcr.io/my-org/*"],
        "default_run_args": ["--ipc=host"]
      }
    }
  ]
}
```

Use an SSH config alias in `ssh`, not a raw password or private key path. Keep
credentials in the user's SSH agent or `~/.ssh/config`.

Concurrent sessions coordinate through `$RCC_REMOTE_STATE_DIR`, or
`~/.local/state/rcc/remote-eval` by default. This is a machine-local lease store,
not project state.

Legacy project-local host definitions are still accepted for compatibility:

```json
{
  "remote_eval": {
    "enabled": true,
    "hosts": [
      {
        "id": "gpu-a",
        "ssh": "rcc-gpu-a",
        "labels": ["docker", "cuda"],
        "workdir": "~/rcc-runs",
        "requires": {
          "docker": true,
          "gpu": true
        }
      }
    ]
  }
}
```

## Workflow

1. Run a local smoke test first. Remote heavy evals should not spend server time
   on obviously broken code.
2. Resolve the installed RCC remote runner path:

   - RCC source checkout: `scripts/remote/`
   - Claude Code install: `~/.claude/scripts/remote/`
   - Codex project install: `.codex/scripts/remote/`

3. Probe resources:

   ```bash
   node scripts/remote/probe-resource.js --json --requires docker,gpu
   ```

   Replace `scripts/remote/` with the installed path when not running from the
   RCC source checkout.

4. If a host is selected, run the eval:

   ```bash
   node scripts/remote/remote-eval-run.js \
     --host auto \
     --label S13_V5_canonical \
     --image project-eval@sha256:<digest> \
     --cmd "bash run_canonical_eval.sh S13_V5_canonical" \
     --timeout 14400
   ```

   For C++/SLAM build-and-eval workflows, prefer the wrapper:

   ```bash
   node scripts/remote/remote-cpp-eval.js \
     --host auto \
     --label kitti_00 \
     --image slam-dev:ros-humble-cuda \
     --configure "cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Release" \
     --build "cmake --build build -j8" \
     --test "ctest --test-dir build --output-on-failure" \
     --eval "./build/run_kitti_eval --seq 00 --out dump/kitti_00" \
     --requires docker,gpu \
     --timeout 14400
   ```

5. Inspect `dump/<label>/remote-run.json`, `stdout.log`, `stderr.log`, and
   `artifact-manifest.json`.
6. Run the normal RCC eval gate:

   ```bash
   node scripts/hooks/eval-regression-check.js
   ```

## Output Contract

The local project receives:

```text
dump/<label>/
  configure.log
  build.log
  test.log
  eval.log
  stdout.log
  stderr.log
  remote-run.json
  artifact-manifest.json
  <remote eval artifacts>
```

`remote-run.json` must include the selected host, run id, base git SHA when
available, patch hash when the worktree is dirty, Docker image, command, exit
code, and artifact path.

## Safety Rules

- Never write to `~/.ssh`, `~/.docker`, `~/.claude`, `~/.codex`, or global git
  hooks. Server registry and leases are machine-local and user-controlled.
- Treat `ping` as a hint only. SSH plus Docker probe is the resource gate.
- Reject host ids, SSH aliases, labels, image names, and Docker args that contain
  shell metacharacters or option-injection patterns.
- Prefer pinned Docker image digests for research conclusions.
- If no remote host is reachable, fall back to local eval and report reduced
  resource coverage.
- Do not promote remote results unless artifacts include enough metadata to
  reproduce the run.

## Related RCC Surface

- `eval-harness` defines evals and success criteria.
- `docker-research-discipline` covers ROS/ML Docker determinism traps.
- `benchmark` records before/after performance baselines.
- `hpo-sweep` handles long-running search and polling.
- `canonical-eval-detect` and `eval-regression-check` consume the collected
  artifacts and metrics.
