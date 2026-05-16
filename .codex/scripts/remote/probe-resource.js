#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');
const { getHostLeaseSummary } = require('./lease-store');
const {
  buildRemotePolicy,
  getRegistryProfile,
  hostAllowedByPolicy,
  loadRemoteResourceRegistry,
  mergeHosts,
} = require('./resource-registry');

const DEFAULT_CONNECT_TIMEOUT_SEC = 3;
const DEFAULT_CACHE_TTL_SEC = 300;
const SAFE_HOST_ID = /^[A-Za-z0-9._-]+$/;
const SAFE_SSH_ALIAS = /^[A-Za-z0-9._@:-]+$/;

function splitCsv(value) {
  return String(value || '')
    .split(',')
    .map(item => item.trim().toLowerCase())
    .filter(Boolean);
}

function parseArgs(argv) {
  const options = {
    configPath: '',
    cwd: process.cwd(),
    host: 'auto',
    requires: [],
    json: false,
    noCache: false,
    ping: false,
    timeoutSec: DEFAULT_CONNECT_TIMEOUT_SEC,
    cacheTtlSec: null,
    registryPath: '',
    stateDir: '',
  };

  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === '--config') {
      options.configPath = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--cwd') {
      options.cwd = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--host') {
      options.host = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--requires') {
      options.requires = splitCsv(argv[index + 1] || '');
      index += 1;
    } else if (arg === '--json') {
      options.json = true;
    } else if (arg === '--no-cache') {
      options.noCache = true;
    } else if (arg === '--ping') {
      options.ping = true;
    } else if (arg === '--timeout') {
      options.timeoutSec = Number(argv[index + 1] || DEFAULT_CONNECT_TIMEOUT_SEC);
      index += 1;
    } else if (arg === '--cache-ttl') {
      options.cacheTtlSec = Number(argv[index + 1] || DEFAULT_CACHE_TTL_SEC);
      index += 1;
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

  if (!Number.isFinite(options.timeoutSec) || options.timeoutSec <= 0) {
    throw new Error('--timeout must be a positive number of seconds');
  }
  if (options.cacheTtlSec !== null && (!Number.isFinite(options.cacheTtlSec) || options.cacheTtlSec < 0)) {
    throw new Error('--cache-ttl must be a non-negative number of seconds');
  }
  return options;
}

function resolveConfigPath(cwd, explicitPath = '') {
  if (explicitPath) {
    return path.isAbsolute(explicitPath) ? explicitPath : path.join(cwd, explicitPath);
  }
  const candidates = [
    path.join(cwd, '.claude', 'research-config.json'),
    path.join(cwd, 'research-config.json'),
  ];
  return candidates.find(candidate => fs.existsSync(candidate)) || candidates[0];
}

function loadRemoteEvalConfig(options = {}) {
  const cwd = path.resolve(options.cwd || process.cwd());
  const configPath = resolveConfigPath(cwd, options.configPath || '');
  const registry = loadRemoteResourceRegistry({
    registryPath: options.registryPath || '',
  });
  let raw = {};
  if (fs.existsSync(configPath)) {
    raw = JSON.parse(fs.readFileSync(configPath, 'utf8'));
  }
  const configured = Boolean(raw.remote_eval && typeof raw.remote_eval === 'object');
  const remote = configured
    ? raw.remote_eval
    : {};
  const projectHosts = Array.isArray(remote.hosts) ? remote.hosts : [];
  const profile = getRegistryProfile(registry, remote.resource_profile || '');
  const policy = buildRemotePolicy(remote, profile);
  const hosts = mergeHosts(registry.hosts, projectHosts);
  return {
    cwd,
    configPath,
    configured,
    enabled: configured && remote.enabled !== false,
    registryConfigured: registry.configured,
    registryPath: registry.registryPath,
    resourceProfile: policy.resourceProfile,
    requires: policy.requires,
    hostAllowlist: policy.hostAllowlist,
    labels: policy.labels,
    artifactDir: remote.artifact_dir || 'dump',
    probeCacheTtlSec: Number.isFinite(Number(remote.probe_cache_ttl_sec))
      ? Number(remote.probe_cache_ttl_sec)
      : DEFAULT_CACHE_TTL_SEC,
    defaultTimeoutSec: Number.isFinite(Number(remote.default_timeout_sec))
      ? Number(remote.default_timeout_sec)
      : 14400,
    hosts,
  };
}

function validateHostEntry(host) {
  if (!host || typeof host !== 'object') {
    throw new Error('remote_eval host entry must be an object');
  }
  if (!SAFE_HOST_ID.test(String(host.id || ''))) {
    throw new Error(`Invalid remote_eval host id: ${host.id || '(missing)'}`);
  }
  if (!SAFE_SSH_ALIAS.test(String(host.ssh || '')) || String(host.ssh || '').startsWith('-')) {
    throw new Error(`Invalid remote_eval ssh alias for host ${host.id}`);
  }
  const workdir = String(host.workdir || '~/rcc-runs');
  if (!/^[A-Za-z0-9_./~:-]+$/.test(workdir)) {
    throw new Error(`Invalid remote_eval workdir for host ${host.id}`);
  }
  return {
    ...host,
    id: String(host.id),
    ssh: String(host.ssh),
    workdir,
    labels: Array.isArray(host.labels) ? host.labels.map(label => String(label).toLowerCase()) : [],
    max_parallel: host.max_parallel,
    cpu_slots: host.cpu_slots,
    gpu_slots: Array.isArray(host.gpu_slots) ? host.gpu_slots.map(slot => String(slot)) : [],
    docker: host.docker && typeof host.docker === 'object' ? host.docker : {},
    source: host.source || 'unknown',
  };
}

function run(command, args, options = {}) {
  return spawnSync(command, args, {
    cwd: options.cwd || process.cwd(),
    encoding: options.encoding || 'utf8',
    input: options.input,
    env: options.env || process.env,
    timeout: options.timeoutMs || 30000,
    stdio: options.stdio || ['pipe', 'pipe', 'pipe'],
    maxBuffer: options.maxBuffer || 10 * 1024 * 1024,
  });
}

function sshArgs(host, command, timeoutSec) {
  return [
    '-o', 'BatchMode=yes',
    '-o', `ConnectTimeout=${Math.max(1, Math.floor(timeoutSec || DEFAULT_CONNECT_TIMEOUT_SEC))}`,
    host.ssh,
    command,
  ];
}

function runSsh(host, command, options = {}) {
  return run('ssh', sshArgs(host, command, options.timeoutSec), {
    encoding: options.encoding || 'utf8',
    input: options.input,
    timeoutMs: (options.timeoutSec || DEFAULT_CONNECT_TIMEOUT_SEC) * 1000 + 5000,
    stdio: options.stdio,
  });
}

function maybePing(host, timeoutSec) {
  const target = String(host.ping || host.ssh).replace(/^.*@/, '').replace(/:\d+$/, '');
  if (!SAFE_SSH_ALIAS.test(target) || target.startsWith('-')) {
    return { attempted: false, ok: false };
  }
  const result = run('ping', ['-c', '1', '-W', String(Math.max(1, Math.floor(timeoutSec || 1))), target], {
    timeoutMs: Math.max(2, timeoutSec || 1) * 1000,
  });
  return { attempted: true, ok: result.status === 0 };
}

function parseGpuCsv(stdout) {
  return String(stdout || '')
    .split(/\r?\n/)
    .map(line => line.trim())
    .filter(Boolean)
    .map(line => {
      const [name, total, free] = line.split(',').map(part => part.trim());
      return {
        name,
        memory_total_mib: Number(total),
        memory_free_mib: Number(free),
      };
    })
    .filter(gpu => gpu.name);
}

function cachePath(cwd) {
  return path.join(cwd, '.claude', 'remote-eval', 'probe-cache.json');
}

function readCache(cwd, ttlSec) {
  const filePath = cachePath(cwd);
  if (!fs.existsSync(filePath)) return null;
  try {
    const parsed = JSON.parse(fs.readFileSync(filePath, 'utf8'));
    const ageMs = Date.now() - Date.parse(parsed.generated_at || '');
    if (!Number.isFinite(ageMs) || ageMs > ttlSec * 1000) return null;
    return parsed;
  } catch {
    return null;
  }
}

function writeCache(cwd, payload) {
  const filePath = cachePath(cwd);
  fs.mkdirSync(path.dirname(filePath), { recursive: true });
  fs.writeFileSync(filePath, `${JSON.stringify(payload, null, 2)}\n`);
}

function hostMatchesRequirements(probe, requires) {
  const req = new Set(requires || []);
  if (req.has('ssh') && !probe.ssh) return false;
  if (req.has('docker') && !probe.docker) return false;
  if (req.has('gpu') && !probe.gpu) return false;
  if (req.has('cuda') && !probe.gpu) return false;
  if (probe.capacity && probe.capacity.available <= 0) return false;
  return probe.reachable;
}

function selectHost(probes, options = {}) {
  const requested = options.host || 'auto';
  const requires = options.requires || [];
  const eligible = probes.filter(probe => {
    if (requested !== 'auto' && probe.id !== requested) return false;
    return hostMatchesRequirements(probe, requires);
  });
  eligible.sort((left, right) => {
    const leftFree = Math.max(0, ...left.gpus.map(gpu => Number(gpu.memory_free_mib) || 0));
    const rightFree = Math.max(0, ...right.gpus.map(gpu => Number(gpu.memory_free_mib) || 0));
    return rightFree - leftFree || left.id.localeCompare(right.id);
  });
  return eligible[0] || null;
}

function probeHost(hostInput, options = {}) {
  const host = validateHostEntry(hostInput);
  const timeoutSec = options.timeoutSec || DEFAULT_CONNECT_TIMEOUT_SEC;
  const ping = options.ping ? maybePing(host, timeoutSec) : { attempted: false, ok: false };
  const ssh = runSsh(host, 'true', { timeoutSec });
  const sshOk = ssh.status === 0;

  let dockerOk = false;
  let gpuOk = false;
  let gpus = [];
  let dockerVersion = '';
  const errors = [];

  if (!sshOk) {
    errors.push((ssh.stderr || ssh.stdout || 'ssh probe failed').trim());
  } else {
    const docker = runSsh(host, 'docker version --format "{{.Server.Version}}"', { timeoutSec });
    dockerOk = docker.status === 0;
    dockerVersion = dockerOk ? String(docker.stdout || '').trim() : '';
    if (!dockerOk) errors.push((docker.stderr || docker.stdout || 'docker probe failed').trim());

    const gpu = runSsh(
      host,
      'nvidia-smi --query-gpu=name,memory.total,memory.free --format=csv,noheader,nounits',
      { timeoutSec }
    );
    gpuOk = gpu.status === 0;
    gpus = gpuOk ? parseGpuCsv(gpu.stdout) : [];
  }

  return {
    id: host.id,
    ssh_alias: host.ssh,
    labels: host.labels,
    workdir: host.workdir,
    reachable: sshOk,
    ping,
    ssh: sshOk,
    docker: dockerOk,
    docker_version: dockerVersion,
    gpu: gpuOk && gpus.length > 0,
    gpus,
    errors: errors.filter(Boolean),
  };
}

function probeResources(options = {}) {
  const config = loadRemoteEvalConfig(options);
  const requirements = [...new Set([...(config.requires || []), ...(options.requires || [])])];
  if (!config.enabled) {
    return {
      status: 'disabled',
      config_path: config.configPath,
      configured: config.configured,
      registry_configured: config.registryConfigured,
      registry_path: config.registryPath,
      requirements,
      selected: null,
      probes: [],
    };
  }

  if (config.hosts.length === 0) {
    return {
      status: 'unavailable',
      generated_at: new Date().toISOString(),
      config_path: config.configPath,
      configured: config.configured,
      registry_configured: config.registryConfigured,
      registry_path: config.registryPath,
      requirements,
      selected: null,
      probes: [],
      cache_hit: false,
    };
  }

  const cacheTtlSec = options.cacheTtlSec == null ? config.probeCacheTtlSec : options.cacheTtlSec;
  if (!options.noCache && cacheTtlSec > 0) {
    const cached = readCache(config.cwd, cacheTtlSec);
    if (cached) {
      const hostById = new Map(config.hosts
        .map(validateHostEntry)
        .filter(host => hostAllowedByPolicy(host, config))
        .map(host => [host.id, host]));
      const probes = (cached.probes || []).map(probe => {
        const host = hostById.get(probe.id);
        if (!host) return probe;
        return {
          ...probe,
          capacity: getHostLeaseSummary(host, requirements, { stateDir: options.stateDir }),
        };
      });
      const selected = selectHost(probes, { ...options, requires: requirements });
      return {
        ...cached,
        status: selected ? 'available' : 'unavailable',
        selected,
        requirements,
        probes,
        cache_hit: true,
      };
    }
  }

  const requested = options.host || 'auto';
  const hosts = config.hosts
    .map(validateHostEntry)
    .filter(host => requested === 'auto' || host.id === requested)
    .filter(host => hostAllowedByPolicy(host, config));
  const probes = hosts.map(host => {
    const probe = probeHost(host, options);
    return {
      ...probe,
      source: host.source,
      capacity: getHostLeaseSummary(host, requirements, { stateDir: options.stateDir }),
    };
  });
  const selected = selectHost(probes, { ...options, requires: requirements });
  const payload = {
    status: selected ? 'available' : 'unavailable',
    generated_at: new Date().toISOString(),
    config_path: config.configPath,
    configured: config.configured,
    registry_configured: config.registryConfigured,
    registry_path: config.registryPath,
    resource_profile: config.resourceProfile,
    requirements,
    selected,
    probes,
    cache_hit: false,
  };
  if (!options.noCache) {
    writeCache(config.cwd, payload);
  }
  return payload;
}

function usage() {
  console.log(`Usage:
  probe-resource [--json] [--host auto|<id>] [--requires docker,gpu] [--no-cache]
                 [--config <path>] [--registry <path>] [--state-dir <path>]
                 [--cwd <project>] [--timeout <sec>] [--ping]
`);
}

function main(argv = process.argv.slice(2)) {
  const options = parseArgs(argv);
  if (options.help) {
    usage();
    return 0;
  }
  const result = probeResources(options);
  if (options.json) {
    console.log(JSON.stringify(result, null, 2));
  } else if (result.selected) {
    console.log(`remote-eval: selected ${result.selected.id}`);
  } else {
    console.log('remote-eval: no matching remote resource available');
  }
  return result.selected ? 0 : 1;
}

if (require.main === module) {
  try {
    process.exit(main());
  } catch (error) {
    console.error(`probe-resource: ${error.message}`);
    process.exit(2);
  }
}

module.exports = {
  hostMatchesRequirements,
  loadRemoteEvalConfig,
  main,
  parseArgs,
  probeHost,
  probeResources,
  runSsh,
  selectHost,
  splitCsv,
  validateHostEntry,
};
