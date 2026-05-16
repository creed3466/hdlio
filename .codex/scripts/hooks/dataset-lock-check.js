#!/usr/bin/env node
'use strict';

/**
 * pre:bash:dataset-lock-check (RCC v2.1 Phase α hook)
 *
 * Trigger: PreToolUse on Bash.
 *
 * Detect commands that would modify pinned dataset paths
 * (rm / git rm / mv / sed -i / dd / curl > / wget) and:
 *   minimal  → no-op
 *   standard → warn (exit 0)
 *   strict   → block (exit 2)
 *
 * Reads dataset_locks[] from .claude/research-state.json AND
 * dataset_lock_required_for[] globs from .claude/research-config.json.
 *
 * No state / config present → graceful no-op.
 */

const fs = require('fs');
const path = require('path');
const { matchesGlob } = require('../lib/research-glob.js');
const rs = require('../lib/research-state.js');

const MAX_STDIN = 1024 * 1024;

// Commands that can modify files. Operates on argv tokens.
const MUTATING_CMDS = [
  /\brm\s+/,
  /\bgit\s+rm\b/,
  /\bmv\s+/,
  /\bdd\s+(?:if|of)=/,
  /\bcurl\s+.*-o\s+/,
  /\bwget\s+/,
  /\bsed\s+-i\s+/,
  /\brsync\s+.*--delete\b/,
];

function readStdin() {
  return new Promise(resolve => {
    let raw = '';
    process.stdin.setEncoding('utf8');
    process.stdin.on('data', chunk => {
      if (raw.length < MAX_STDIN) {
        const remaining = MAX_STDIN - raw.length;
        raw += chunk.substring(0, remaining);
      }
    });
    process.stdin.on('end', () => resolve(raw));
    process.stdin.on('error', () => resolve(raw));
  });
}

function getProfile() {
  const raw = String(
    process.env.RCC_HOOK_PROFILE || process.env.ECC_HOOK_PROFILE || 'standard'
  ).trim().toLowerCase();
  return ['minimal', 'standard', 'strict'].includes(raw) ? raw : 'standard';
}

function loadConfig(cwd) {
  for (const p of [
    path.join(cwd, '.claude', 'research-config.json'),
    path.join(cwd, 'research-config.json'),
  ]) {
    if (fs.existsSync(p)) {
      try { return JSON.parse(fs.readFileSync(p, 'utf8')); } catch { return null; }
    }
  }
  return null;
}

// Heuristic: extract file paths from a Bash command string.
// Catches typical `rm -rf data/raw/x`, `mv a/b c/d`, etc.
function extractPathArgs(cmd) {
  const tokens = cmd.split(/\s+/).filter(Boolean);
  // Drop the program name and flag-like tokens; keep things that look
  // like paths (contain / or end with common dataset extensions).
  return tokens.filter(t => /\/|\.(h5|hdf5|npy|npz|pkl|parquet|csv|json|zip|tar\.gz|pt|pth|onnx|safetensors|bin|dat|csv\.gz)$/i.test(t));
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  const toolName = input.tool_name || input.toolName || '';
  if (toolName !== 'Bash') { passthrough(); return; }

  const cmd = String(input.tool_input?.command || '');
  if (!cmd) { passthrough(); return; }

  // Fast-fail: mutating command?
  if (!MUTATING_CMDS.some(re => re.test(cmd))) { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  const cfg = loadConfig(cwd);
  const state = rs.readState(cwd);
  if (!cfg && (!state || !state.dataset_locks || state.dataset_locks.length === 0)) {
    passthrough();
    return;
  }

  // Build the set of "protected" paths from config globs + pinned locks
  const required = (cfg && Array.isArray(cfg.dataset_lock_required_for))
    ? cfg.dataset_lock_required_for
    : [];
  const pinnedPaths = ((state && state.dataset_locks) || []).map(d => d.path);

  // Extract candidate paths from the command
  const candidates = extractPathArgs(cmd);
  const hits = [];
  for (const c of candidates) {
    const norm = c.replace(/^['"]|['"]$/g, '');
    // Match against pinned paths exactly
    if (pinnedPaths.includes(norm)) { hits.push({ path: norm, reason: 'pinned' }); continue; }
    // Match against glob patterns
    for (const pat of required) {
      if (matchesGlob(norm, pat)) {
        hits.push({ path: norm, reason: `matches ${pat}` });
        break;
      }
    }
  }

  if (hits.length === 0) { passthrough(); return; }

  const profile = getProfile();
  const header = profile === 'strict'
    ? '[dataset-lock-check] BLOCKED: would mutate pinned dataset path'
    : '[dataset-lock-check] WARNING: command may mutate pinned dataset path';

  process.stderr.write(`${header}\n`);
  process.stderr.write(`  command: ${cmd.slice(0, 120)}\n`);
  for (const h of hits.slice(0, 5)) {
    process.stderr.write(`  • ${h.path}  (${h.reason})\n`);
  }
  process.stderr.write('  Use the dataset-versioning skill to re-pin after intentional changes.\n');

  passthrough();
  process.exit(profile === 'strict' ? 2 : 0);
}

main().catch(() => process.exit(0));
