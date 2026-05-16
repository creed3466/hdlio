#!/usr/bin/env node
'use strict';

const crypto = require('crypto');
const fs = require('fs');
const os = require('os');
const path = require('path');

const DEFAULT_LEASE_TTL_SEC = 6 * 60 * 60;
const LOCK_STALE_MS = 30 * 1000;

function resolveStateDir(options = {}) {
  if (options.stateDir) return path.resolve(options.stateDir);
  const env = options.env || process.env;
  if (env.RCC_REMOTE_STATE_DIR) return path.resolve(env.RCC_REMOTE_STATE_DIR);
  const stateHome = env.XDG_STATE_HOME
    ? path.resolve(env.XDG_STATE_HOME)
    : path.join(os.homedir(), '.local', 'state');
  return path.join(stateHome, 'rcc', 'remote-eval');
}

function leasePath(options = {}) {
  return path.join(resolveStateDir(options), 'leases.json');
}

function nowMs() {
  return Date.now();
}

function readLeaseFile(options = {}) {
  const filePath = leasePath(options);
  if (!fs.existsSync(filePath)) return { version: 1, leases: [] };
  try {
    const parsed = JSON.parse(fs.readFileSync(filePath, 'utf8'));
    return {
      version: 1,
      leases: Array.isArray(parsed.leases) ? parsed.leases : [],
    };
  } catch {
    return { version: 1, leases: [] };
  }
}

function writeLeaseFile(payload, options = {}) {
  const filePath = leasePath(options);
  fs.mkdirSync(path.dirname(filePath), { recursive: true });
  const tmp = `${filePath}.${process.pid}.tmp`;
  fs.writeFileSync(tmp, `${JSON.stringify(payload, null, 2)}\n`);
  fs.renameSync(tmp, filePath);
}

function pruneExpiredLeases(leases, atMs = nowMs()) {
  return leases.filter(lease => {
    const expiresAt = Date.parse(lease.expires_at || '');
    return Number.isFinite(expiresAt) && expiresAt > atMs;
  });
}

function sleepBusy(ms) {
  const sab = new SharedArrayBuffer(4);
  const view = new Int32Array(sab);
  Atomics.wait(view, 0, 0, Math.max(1, ms));
}

function withLeaseLock(options, fn) {
  const stateDir = resolveStateDir(options);
  fs.mkdirSync(stateDir, { recursive: true });
  const lockPath = path.join(stateDir, 'leases.lock');
  for (let attempt = 0; attempt < 80; attempt += 1) {
    let fd = null;
    try {
      fd = fs.openSync(lockPath, 'wx');
      fs.writeFileSync(fd, `${process.pid}\n${new Date().toISOString()}\n`);
      return fn();
    } catch (error) {
      if (error.code !== 'EEXIST') throw error;
      try {
        const stat = fs.statSync(lockPath);
        if (Date.now() - stat.mtimeMs > LOCK_STALE_MS) fs.unlinkSync(lockPath);
      } catch {
        // Lock disappeared between checks; retry.
      }
      sleepBusy(Math.min(250, 5 + attempt * 5));
    } finally {
      if (fd !== null) {
        try { fs.closeSync(fd); } catch {}
        try { fs.unlinkSync(lockPath); } catch {}
      }
    }
  }
  throw new Error(`Timed out acquiring remote eval lease lock: ${lockPath}`);
}

function positiveInt(value, fallback) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) return fallback;
  return Math.max(0, Math.floor(parsed));
}

function needsGpu(requirements = []) {
  const req = new Set(requirements.map(item => String(item).toLowerCase()));
  return req.has('gpu') || req.has('cuda');
}

function buildHostSlots(host, requirements = []) {
  if (needsGpu(requirements)) {
    const gpuSlots = Array.isArray(host.gpu_slots) ? host.gpu_slots.map(slot => String(slot)) : [];
    if (gpuSlots.length > 0) {
      return gpuSlots.map(slot => ({ type: 'gpu', id: slot }));
    }
    const capacity = positiveInt(host.max_parallel, 1);
    return Array.from({ length: capacity }, (_, index) => ({ type: 'gpu', id: `gpu-${index}` }));
  }

  const capacity = positiveInt(
    host.cpu_slots == null ? host.max_parallel : host.cpu_slots,
    1
  );
  return Array.from({ length: capacity }, (_, index) => ({ type: 'cpu', id: `cpu-${index}` }));
}

function getHostLeaseSummary(host, requirements = [], options = {}) {
  const payload = readLeaseFile(options);
  const activeLeases = pruneExpiredLeases(payload.leases);
  const slots = buildHostSlots(host, requirements);
  const used = new Set(activeLeases
    .filter(lease => lease.host === host.id)
    .map(lease => `${lease.slot_type}:${lease.slot_id}`));
  const availableSlots = slots.filter(slot => !used.has(`${slot.type}:${slot.id}`));
  return {
    capacity: slots.length,
    active: slots.length - availableSlots.length,
    available: availableSlots.length,
    next_slot: availableSlots[0] || null,
  };
}

function acquireLease(input = {}) {
  const host = input.host;
  if (!host || !host.id) throw new Error('host is required for lease acquisition');
  const ttlSec = positiveInt(input.ttlSec, DEFAULT_LEASE_TTL_SEC);
  return withLeaseLock(input, () => {
    const payload = readLeaseFile(input);
    const activeLeases = pruneExpiredLeases(payload.leases);
    const slots = buildHostSlots(host, input.requirements || []);
    const used = new Set(activeLeases
      .filter(lease => lease.host === host.id)
      .map(lease => `${lease.slot_type}:${lease.slot_id}`));
    const slot = slots.find(candidate => !used.has(`${candidate.type}:${candidate.id}`));
    if (!slot) {
      writeLeaseFile({ version: 1, leases: activeLeases }, input);
      return null;
    }

    const acquiredAt = new Date().toISOString();
    const lease = {
      id: input.leaseId || crypto.randomUUID(),
      host: host.id,
      ssh_alias: host.ssh,
      slot_type: slot.type,
      slot_id: slot.id,
      project: input.project || '',
      label: input.label || '',
      run_id: input.runId || '',
      pid: process.pid,
      requirements: input.requirements || [],
      acquired_at: acquiredAt,
      expires_at: new Date(Date.now() + ttlSec * 1000).toISOString(),
    };
    writeLeaseFile({ version: 1, leases: [...activeLeases, lease] }, input);
    return lease;
  });
}

function releaseLease(leaseId, options = {}) {
  if (!leaseId) return false;
  return withLeaseLock(options, () => {
    const payload = readLeaseFile(options);
    const activeLeases = pruneExpiredLeases(payload.leases);
    const nextLeases = activeLeases.filter(lease => lease.id !== leaseId);
    writeLeaseFile({ version: 1, leases: nextLeases }, options);
    return nextLeases.length !== activeLeases.length;
  });
}

function main(argv = process.argv.slice(2)) {
  const options = { stateDir: '', json: false };
  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === '--state-dir') {
      options.stateDir = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--json') {
      options.json = true;
    } else if (arg === '--help' || arg === '-h') {
      console.log(`Usage:
  lease-store [--state-dir <path>] [--json]
`);
      return 0;
    } else {
      throw new Error(`Unknown argument: ${arg}`);
    }
  }
  const payload = readLeaseFile(options);
  const activeLeases = pruneExpiredLeases(payload.leases);
  const result = {
    state_dir: resolveStateDir(options),
    active_count: activeLeases.length,
    leases: activeLeases,
  };
  if (options.json) console.log(JSON.stringify(result, null, 2));
  else console.log(`remote leases: ${result.active_count}`);
  return 0;
}

if (require.main === module) {
  try {
    process.exit(main());
  } catch (error) {
    console.error(`lease-store: ${error.message}`);
    process.exit(1);
  }
}

module.exports = {
  acquireLease,
  buildHostSlots,
  getHostLeaseSummary,
  leasePath,
  needsGpu,
  pruneExpiredLeases,
  readLeaseFile,
  releaseLease,
  resolveStateDir,
};
