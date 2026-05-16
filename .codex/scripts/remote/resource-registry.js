#!/usr/bin/env node
'use strict';

const fs = require('fs');
const os = require('os');
const path = require('path');

function splitCsv(value) {
  return String(value || '')
    .split(',')
    .map(item => item.trim().toLowerCase())
    .filter(Boolean);
}

function normalizeStringArray(value) {
  if (Array.isArray(value)) return value.map(item => String(item)).filter(Boolean);
  if (typeof value === 'string') return splitCsv(value);
  return [];
}

function resolveRegistryPath(options = {}) {
  if (options.registryPath) {
    return path.resolve(options.registryPath);
  }
  const env = options.env || process.env;
  if (env.RCC_REMOTE_REGISTRY) {
    return path.resolve(env.RCC_REMOTE_REGISTRY);
  }
  const configHome = env.XDG_CONFIG_HOME
    ? path.resolve(env.XDG_CONFIG_HOME)
    : path.join(os.homedir(), '.config');
  return path.join(configHome, 'rcc', 'remote-resources.json');
}

function normalizeProfiles(rawProfiles) {
  if (!rawProfiles || typeof rawProfiles !== 'object') return {};
  if (Array.isArray(rawProfiles)) {
    return Object.fromEntries(rawProfiles
      .filter(profile => profile && profile.id)
      .map(profile => [String(profile.id), profile]));
  }
  return Object.fromEntries(Object.entries(rawProfiles)
    .filter(([, profile]) => profile && typeof profile === 'object')
    .map(([id, profile]) => [String(id), { id, ...profile }]));
}

function loadRemoteResourceRegistry(options = {}) {
  const registryPath = resolveRegistryPath(options);
  if (!fs.existsSync(registryPath)) {
    return {
      configured: false,
      registryPath,
      hosts: [],
      profiles: {},
      raw: {},
    };
  }

  const raw = JSON.parse(fs.readFileSync(registryPath, 'utf8'));
  return {
    configured: true,
    registryPath,
    hosts: Array.isArray(raw.hosts) ? raw.hosts : [],
    profiles: normalizeProfiles(raw.profiles),
    raw,
  };
}

function getRegistryProfile(registry, profileId) {
  if (!profileId || !registry || !registry.profiles) return null;
  return registry.profiles[String(profileId)] || null;
}

function mergeUnique(left = [], right = []) {
  const seen = new Set();
  const out = [];
  for (const value of [...left, ...right]) {
    const normalized = String(value || '').trim();
    if (!normalized || seen.has(normalized)) continue;
    seen.add(normalized);
    out.push(normalized);
  }
  return out;
}

function mergeHosts(localHosts = [], projectHosts = []) {
  const byId = new Map();
  for (const host of localHosts) {
    if (!host || !host.id) continue;
    byId.set(String(host.id), { ...host, source: host.source || 'local-registry' });
  }
  for (const host of projectHosts) {
    if (!host || !host.id || byId.has(String(host.id))) continue;
    byId.set(String(host.id), { ...host, source: host.source || 'project-config' });
  }
  return [...byId.values()];
}

function buildRemotePolicy(remote = {}, profile = null) {
  return {
    resourceProfile: remote.resource_profile || null,
    requires: mergeUnique(
      normalizeStringArray(profile && profile.requires),
      normalizeStringArray(remote.requires)
    ).map(item => item.toLowerCase()),
    hostAllowlist: mergeUnique(
      normalizeStringArray(profile && profile.host_allowlist),
      normalizeStringArray(remote.host_allowlist)
    ),
    labels: mergeUnique(
      normalizeStringArray(profile && profile.labels),
      normalizeStringArray(remote.labels)
    ).map(item => item.toLowerCase()),
  };
}

function hostAllowedByPolicy(host, policy = {}) {
  if (policy.hostAllowlist && policy.hostAllowlist.length > 0 && !policy.hostAllowlist.includes(host.id)) {
    return false;
  }
  if (policy.labels && policy.labels.length > 0) {
    const hostLabels = new Set((host.labels || []).map(label => String(label).toLowerCase()));
    return policy.labels.every(label => hostLabels.has(label));
  }
  return true;
}

function main(argv = process.argv.slice(2)) {
  const options = { registryPath: '', json: false };
  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === '--registry') {
      options.registryPath = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--json') {
      options.json = true;
    } else if (arg === '--help' || arg === '-h') {
      console.log(`Usage:
  resource-registry [--registry <path>] [--json]
`);
      return 0;
    } else {
      throw new Error(`Unknown argument: ${arg}`);
    }
  }
  const registry = loadRemoteResourceRegistry(options);
  const payload = {
    configured: registry.configured,
    registry_path: registry.registryPath,
    host_count: registry.hosts.length,
    profile_count: Object.keys(registry.profiles).length,
  };
  if (options.json) console.log(JSON.stringify(payload, null, 2));
  else console.log(`remote registry: ${payload.configured ? payload.registry_path : 'not configured'}`);
  return registry.configured ? 0 : 1;
}

if (require.main === module) {
  try {
    process.exit(main());
  } catch (error) {
    console.error(`resource-registry: ${error.message}`);
    process.exit(1);
  }
}

module.exports = {
  buildRemotePolicy,
  getRegistryProfile,
  hostAllowedByPolicy,
  loadRemoteResourceRegistry,
  mergeHosts,
  mergeUnique,
  normalizeStringArray,
  resolveRegistryPath,
  splitCsv,
};
