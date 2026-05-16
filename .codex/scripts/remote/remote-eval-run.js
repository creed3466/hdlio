#!/usr/bin/env node
'use strict';

const crypto = require('crypto');
const fs = require('fs');
const os = require('os');
const path = require('path');
const { spawnSync } = require('child_process');

const { collectArtifacts } = require('./collect-artifacts');
const { acquireLease, releaseLease } = require('./lease-store');
const { writeArtifactManifest } = require('./artifact-manifest');
const {
  loadRemoteEvalConfig,
  probeResources,
  runSsh,
  splitCsv,
  validateHostEntry,
} = require('./probe-resource');

let rs = null;
try {
  rs = require('../lib/research-state');
} catch {
  rs = null;
}

const SAFE_LABEL = /^[A-Za-z0-9._-]+$/;
const SAFE_IMAGE = /^[A-Za-z0-9._/@:-]+$/;
const SAFE_DOCKER_ARG = /^[A-Za-z0-9._/@:=,+-]+$/;

function shellQuote(value) {
  return `'${String(value).replace(/'/g, `'\\''`)}'`;
}

function parseArgs(argv) {
  const options = {
    cwd: process.cwd(),
    configPath: '',
    host: 'auto',
    label: '',
    image: '',
    command: '',
    timeoutSec: null,
    requires: ['docker'],
    json: false,
    dryRun: false,
    noCache: false,
    keepRemote: false,
    registryPath: '',
    stateDir: '',
  };

  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === '--cwd') {
      options.cwd = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--config') {
      options.configPath = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--host') {
      options.host = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--label') {
      options.label = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--image') {
      options.image = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--cmd') {
      options.command = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--timeout') {
      options.timeoutSec = Number(argv[index + 1] || 0);
      index += 1;
    } else if (arg === '--requires') {
      options.requires = splitCsv(argv[index + 1] || '');
      index += 1;
    } else if (arg === '--json') {
      options.json = true;
    } else if (arg === '--dry-run') {
      options.dryRun = true;
    } else if (arg === '--no-cache') {
      options.noCache = true;
    } else if (arg === '--keep-remote') {
      options.keepRemote = true;
    } else if (arg === '--registry') {
      options.registryPath = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--state-dir') {
      options.stateDir = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--help' || arg === '-h') {
      options.help = true;
    } else {
      throw new Error(`Unknown argument: ${arg}`);
    }
  }
  return options;
}

function usage() {
  console.log(`Usage:
  remote-eval-run --label <label> --image <image> --cmd <command>
                  [--host auto|<id>] [--requires docker,gpu] [--timeout <sec>]
                  [--registry <path>] [--state-dir <path>]
                  [--json] [--dry-run] [--no-cache]
`);
}

function validateOptions(options) {
  if (!SAFE_LABEL.test(options.label)) {
    throw new Error('--label must contain only letters, numbers, dot, underscore, or dash');
  }
  if (!SAFE_IMAGE.test(options.image) || String(options.image || '').startsWith('-')) {
    throw new Error('--image must be a Docker image reference without shell metacharacters');
  }
  if (!options.command || /[\u0000]/.test(options.command)) {
    throw new Error('--cmd is required');
  }
  if (options.timeoutSec !== null && (!Number.isFinite(options.timeoutSec) || options.timeoutSec <= 0)) {
    throw new Error('--timeout must be a positive number of seconds');
  }
}

function validateDockerRunArgs(args) {
  const values = Array.isArray(args) ? args : [];
  for (const arg of values) {
    if (!SAFE_DOCKER_ARG.test(String(arg || ''))) {
      throw new Error(`unsafe Docker run arg in local registry: ${arg}`);
    }
  }
  return values.map(arg => String(arg));
}

function patternToRegex(pattern) {
  const escaped = String(pattern)
    .replace(/[.+?^${}()|[\]\\]/g, '\\$&')
    .replace(/\*/g, '.*');
  return new RegExp(`^${escaped}$`);
}

function assertImageAllowed(host, image) {
  const patterns = host.docker && Array.isArray(host.docker.allowed_images)
    ? host.docker.allowed_images
    : [];
  if (patterns.length === 0) return true;
  const allowed = patterns.some(pattern => patternToRegex(pattern).test(image));
  if (!allowed) {
    throw new Error(`Docker image is not allowed by local registry policy for host ${host.id}`);
  }
  return true;
}

function run(command, args, options = {}) {
  return spawnSync(command, args, {
    cwd: options.cwd || process.cwd(),
    input: options.input,
    encoding: options.encoding || 'utf8',
    timeout: options.timeoutMs || 120000,
    stdio: options.stdio || ['pipe', 'pipe', 'pipe'],
    maxBuffer: options.maxBuffer || 50 * 1024 * 1024,
  });
}

function safeName(value, fallback = 'project') {
  const normalized = String(value || fallback)
    .toLowerCase()
    .replace(/[^a-z0-9._-]+/g, '-')
    .replace(/^-+|-+$/g, '');
  return normalized || fallback;
}

function gitOutput(cwd, args) {
  const result = run('git', args, { cwd, timeoutMs: 30000 });
  return result.status === 0 ? String(result.stdout || '').trim() : '';
}

function currentGitSha(cwd) {
  return gitOutput(cwd, ['rev-parse', 'HEAD']) || null;
}

function currentPatchSha(cwd) {
  const result = run('git', ['diff', '--binary', 'HEAD'], { cwd, timeoutMs: 30000 });
  if (result.status !== 0 || !result.stdout) return null;
  const hash = crypto.createHash('sha256');
  hash.update(result.stdout);
  return hash.digest('hex');
}

function sha256File(filePath) {
  const hash = crypto.createHash('sha256');
  hash.update(fs.readFileSync(filePath));
  return hash.digest('hex');
}

function createSourceArchive(cwd) {
  const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'rcc-remote-eval-'));
  const archivePath = path.join(tempDir, 'source.tar');
  const tarArgs = [
    '--exclude', './.git',
    '--exclude', './node_modules',
    '--exclude', './dump',
    '--exclude', './.claude/remote-eval',
    '--exclude', './.codex',
    '--exclude', './.agents',
    '-cf', archivePath,
    '.',
  ];
  const result = run('tar', tarArgs, { cwd, timeoutMs: 120000 });
  if (result.status !== 0) {
    throw new Error(`failed to create source archive: ${String(result.stderr || result.stdout || '').trim()}`);
  }
  return { tempDir, archivePath };
}

function cleanupTemp(tempDir) {
  if (tempDir) fs.rmSync(tempDir, { recursive: true, force: true });
}

function remoteRunDirExpression(workdir, projectName, runId) {
  return [
    `base=${shellQuote(workdir || '~/rcc-runs')}`,
    'case "$base" in "~") base="$HOME" ;; "~/"*) base="$HOME/${base#\\~/}" ;; esac',
    `run_dir="$base/${projectName}/${runId}"`,
  ].join('\n');
}

function prepareRemoteSource(host, archivePath, projectName, runId, timeoutSec) {
  const archive = fs.readFileSync(archivePath);
  const command = [
    'set -eu',
    remoteRunDirExpression(host.workdir, projectName, runId),
    'rm -rf "$run_dir"',
    'mkdir -p "$run_dir/src" "$run_dir/artifacts"',
    'tar -xf - -C "$run_dir/src"',
    'printf "%s\\n" "$run_dir"',
  ].join('\n');
  const result = runSsh(host, command, {
    input: archive,
    timeoutSec,
  });
  if (result.status !== 0) {
    throw new Error(`remote source upload failed: ${String(result.stderr || result.stdout || '').trim()}`);
  }
  return String(result.stdout || '').trim().split(/\r?\n/).filter(Boolean).pop();
}

function dockerRunArgs(host, options) {
  const needsGpu = new Set(options.requires || []).has('gpu') || new Set(options.requires || []).has('cuda');
  const args = validateDockerRunArgs(host.docker && host.docker.default_run_args);
  if (needsGpu) {
    const lease = options.lease || {};
    const slot = lease.slot_type === 'gpu' && /^[0-9,]+$/.test(String(lease.slot_id || ''))
      ? `device=${lease.slot_id}`
      : 'all';
    args.push('--gpus', slot);
  }
  return args.map(shellQuote).join(' ');
}

function runRemoteDocker(host, remoteRunDir, options, timeoutSec) {
  const extraArgs = dockerRunArgs(host, options);
  const renderedExtraArgs = extraArgs ? `${extraArgs} ` : '';
  const containerCommand = [
    'set +e',
    `sh -lc ${shellQuote(options.command)}`,
    'status=$?',
    'chown -R "$RCC_HOST_UID:$RCC_HOST_GID" /workspace 2>/dev/null || true',
    'exit "$status"',
  ].join('\n');
  const command = [
    'set +e',
    `run_dir=${shellQuote(remoteRunDir)}`,
    'cd "$run_dir/src"',
    'host_uid=$(id -u)',
    'host_gid=$(id -g)',
    `mkdir -p ${shellQuote(path.posix.join('dump', options.label))}`,
    `docker run --rm ${renderedExtraArgs}-e RCC_HOST_UID="$host_uid" -e RCC_HOST_GID="$host_gid" -v "$PWD:/workspace" -w /workspace ${shellQuote(options.image)} sh -lc ${shellQuote(containerCommand)}`,
    'status=$?',
    'mkdir -p "$run_dir/artifacts"',
    `if [ -d ${shellQuote(path.posix.join('dump', options.label))} ]; then cp -R ${shellQuote(path.posix.join('dump', options.label))}/. "$run_dir/artifacts/"; fi`,
    'exit "$status"',
  ].join('\n');

  return runSsh(host, command, {
    timeoutSec,
  });
}

function writeJson(filePath, value) {
  fs.mkdirSync(path.dirname(filePath), { recursive: true });
  fs.writeFileSync(filePath, `${JSON.stringify(value, null, 2)}\n`);
}

function recordResearchState(cwd, runRecord) {
  if (!rs) return false;
  if (!rs.isValidProjectCwd(cwd)) return false;
  try {
    rs.withStateLock(cwd, () => {
      const state = rs.ensureState(cwd);
      state.active_experiment = {
        ...(state.active_experiment || {}),
        run_id: runRecord.run_id,
        runner: 'remote-docker',
        host: runRecord.host,
        base_sha: runRecord.base_sha,
        patch_sha256: runRecord.patch_sha256,
        image: runRecord.image,
        command: runRecord.command,
        status: runRecord.status,
        exit_code: runRecord.exit_code,
        artifact_path: runRecord.artifact_path,
        started_at: runRecord.started_at,
        ended_at: runRecord.ended_at,
      };
      rs.writeState(cwd, state);
    });
    return true;
  } catch {
    return false;
  }
}

function buildDryRunPlan(options, config, selected) {
  const runId = `${safeName(options.label)}-${new Date().toISOString().replace(/[:.]/g, '-')}`;
  const projectName = safeName(path.basename(path.resolve(options.cwd)));
  return {
    dry_run: true,
    selected_host: selected ? selected.id : null,
    ssh_alias: selected ? selected.ssh_alias : null,
    run_id: runId,
    project: projectName,
    label: options.label,
    image: options.image,
    command: options.command,
    artifact_path: path.join(config.artifactDir, options.label),
  };
}

function runRemoteEval(options) {
  validateOptions(options);
  const cwd = path.resolve(options.cwd || process.cwd());
  const config = loadRemoteEvalConfig({
    cwd,
    configPath: options.configPath,
    registryPath: options.registryPath,
  });
  const timeoutSec = options.timeoutSec || config.defaultTimeoutSec || 14400;
  const requirements = [...new Set([...(config.requires || []), ...(options.requires || [])])];
  const probe = probeResources({
    cwd,
    configPath: options.configPath,
    registryPath: options.registryPath,
    stateDir: options.stateDir,
    host: options.host,
    requires: requirements,
    noCache: options.noCache,
    timeoutSec: Math.min(10, timeoutSec),
  });

  if (!probe.selected) {
    return {
      status: 'unavailable',
      selected_host: null,
      reason: 'no configured remote host matched requirements',
      probe,
    };
  }

  const selectedProbe = probe.selected;
  const hostConfig = validateHostEntry(config.hosts.find(host => host.id === selectedProbe.id));
  assertImageAllowed(hostConfig, options.image);
  if (options.dryRun) {
    return buildDryRunPlan(options, config, selectedProbe);
  }

  const startedAt = new Date().toISOString();
  const projectName = safeName(path.basename(cwd));
  const runId = `${safeName(options.label)}-${startedAt.replace(/[:.]/g, '-')}`;
  const outDir = path.join(cwd, config.artifactDir || 'dump', options.label);
  fs.mkdirSync(outDir, { recursive: true });

  const baseSha = currentGitSha(cwd);
  const patchSha = currentPatchSha(cwd);
  let tempDir = '';
  let remoteRunDir = '';
  let dockerResult = null;
  let sourceArchiveSha = null;
  let lease = null;
  try {
    lease = acquireLease({
      host: hostConfig,
      requirements,
      project: cwd,
      label: options.label,
      runId,
      leaseId: runId,
      ttlSec: timeoutSec + 600,
      stateDir: options.stateDir,
    });
    if (!lease) {
      return {
        status: 'unavailable',
        selected_host: selectedProbe.id,
        reason: 'selected host has no available local lease capacity',
        probe,
      };
    }
    const archive = createSourceArchive(cwd);
    tempDir = archive.tempDir;
    sourceArchiveSha = sha256File(archive.archivePath);
    remoteRunDir = prepareRemoteSource(hostConfig, archive.archivePath, projectName, runId, Math.min(timeoutSec, 300));
    dockerResult = runRemoteDocker(hostConfig, remoteRunDir, { ...options, requires: requirements, lease }, timeoutSec);
    try {
      collectArtifacts({
        host: hostConfig.id,
        ssh: hostConfig.ssh,
        remotePath: `${remoteRunDir}/artifacts`,
        outDir,
        timeoutSec: Math.min(300, timeoutSec),
      });
    } catch (error) {
      fs.writeFileSync(path.join(outDir, 'collect-warning.txt'), `${error.message}\n`);
    }
  } finally {
    cleanupTemp(tempDir);
    if (lease) releaseLease(lease.id, { stateDir: options.stateDir });
  }

  const endedAt = new Date().toISOString();
  fs.writeFileSync(path.join(outDir, 'stdout.log'), String(dockerResult.stdout || ''));
  fs.writeFileSync(path.join(outDir, 'stderr.log'), String(dockerResult.stderr || ''));

  const runRecord = {
    version: 1,
    run_id: runId,
    label: options.label,
    runner: 'remote-docker',
    host: hostConfig.id,
    ssh_alias: hostConfig.ssh,
    remote_run_dir: remoteRunDir,
    image: options.image,
    command: options.command,
    requirements,
    source_archive_sha256: sourceArchiveSha,
    lease: lease ? {
      id: lease.id,
      slot_type: lease.slot_type,
      slot_id: lease.slot_id,
      acquired_at: lease.acquired_at,
    } : null,
    base_sha: baseSha,
    patch_sha256: patchSha,
    status: dockerResult.status === 0 ? 'completed' : 'failed',
    exit_code: dockerResult.status,
    signal: dockerResult.signal || null,
    started_at: startedAt,
    ended_at: endedAt,
    artifact_path: path.relative(cwd, outDir).replace(/\\/g, '/'),
  };
  writeJson(path.join(outDir, 'remote-run.json'), runRecord);
  const manifest = writeArtifactManifest(outDir).manifest;
  const state_recorded = recordResearchState(cwd, runRecord);

  if (!options.keepRemote && remoteRunDir) {
    runSsh(hostConfig, `rm -rf ${shellQuote(remoteRunDir)}`, { timeoutSec: 30 });
  }

  return {
    status: runRecord.status,
    run: runRecord,
    state_recorded,
    artifact_manifest: {
      file_count: manifest.file_count,
      path: path.join(runRecord.artifact_path, 'artifact-manifest.json').replace(/\\/g, '/'),
    },
  };
}

function main(argv = process.argv.slice(2)) {
  const options = parseArgs(argv);
  if (options.help) {
    usage();
    return 0;
  }
  const result = runRemoteEval(options);
  if (options.json) {
    console.log(JSON.stringify(result, null, 2));
  } else if (result.status === 'unavailable') {
    console.log('remote-eval: no matching remote host available');
  } else if (result.dry_run) {
    console.log(`remote-eval dry-run: ${result.selected_host} ${result.run_id}`);
  } else {
    console.log(`remote-eval ${result.status}: ${result.run.run_id}`);
  }
  if (result.status === 'unavailable') return 1;
  if (result.run && result.run.exit_code !== 0) return result.run.exit_code || 1;
  return 0;
}

if (require.main === module) {
  try {
    process.exit(main());
  } catch (error) {
    console.error(`remote-eval-run: ${error.message}`);
    process.exit(1);
  }
}

module.exports = {
  createSourceArchive,
  currentGitSha,
  currentPatchSha,
  main,
  parseArgs,
  recordResearchState,
  remoteRunDirExpression,
  runRemoteDocker,
  runRemoteEval,
  safeName,
  shellQuote,
  validateOptions,
};
