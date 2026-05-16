#!/usr/bin/env node
'use strict';

/**
 * pre:edit:checkpoint-gate (RCC research hook)
 *
 * PreToolUse hook on Edit/Write/MultiEdit. When the target file matches any
 * of the project's `checkpoint_required_for` globs (algorithm files), checks
 * that the working tree has either (a) a clean state for that file, or
 * (b) a recent commit whose message starts with `checkpoint:`.
 *
 * Rationale: research projects modify performance-critical algorithm code.
 * Without checkpoint commits before each change, rolling back a regression
 * is much harder. This hook enforces Rule 8 (Algorithm checkpoint discipline)
 * from ben.md.
 *
 * Behaviour:
 *   - No `.claude/research-config.json` → no-op
 *   - Target not in `checkpoint_required_for` → no-op
 *   - Target file has uncommitted changes AND no recent `checkpoint:` commit:
 *       standard → warn (exit 0)
 *       strict   → block (exit 2)
 *   - Otherwise → no-op
 */

const fs = require('fs');
const path = require('path');
const { matchesGlob } = require('../lib/research-glob.js');
const { execSync } = require('child_process');

const MAX_STDIN = 1024 * 1024;
const CHECKPOINT_LOOKBACK = 10; // last N commits

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

function loadConfig(projectRoot) {
  const candidates = [
    path.join(projectRoot, '.claude', 'research-config.json'),
    path.join(projectRoot, 'research-config.json'),
  ];
  for (const p of candidates) {
    if (fs.existsSync(p)) {
      try { return JSON.parse(fs.readFileSync(p, 'utf8')); } catch { return null; }
    }
  }
  return null;
}


function gitFileStatus(cwd, file) {
  try {
    const out = execSync(`git status --porcelain -- ${JSON.stringify(file)}`, {
      cwd, encoding: 'utf8', stdio: ['ignore', 'pipe', 'ignore']
    }).trim();
    return out; // empty = clean
  } catch {
    return null; // not a git repo or other error
  }
}

// Look for a recent checkpoint marker. Recognizes:
//   (a) commits with "checkpoint:" message prefix (canonical)
//   (b) annotated/lightweight tags matching pre-step-*, pre-<word>-*,
//       or checkpoint-* (real RCC users — e.g. TofSLAM — discipline via
//       `git tag pre-step-S<N>-<step>` not via commit prefix)
function hasRecentCheckpointMarker(cwd, lookback) {
  // (a) commit message scan
  try {
    const out = execSync(`git log --oneline -${lookback}`, {
      cwd, encoding: 'utf8', stdio: ['ignore', 'pipe', 'ignore']
    });
    if (out.split('\n').some(line => /\bcheckpoint:/i.test(line))) return true;
  } catch { /* not a git repo or no commits — fall through */ }

  // (b) recent tags scan
  try {
    const tags = execSync(
      `git for-each-ref --sort=-creatordate --count=${lookback} --format='%(refname:short)' refs/tags`,
      { cwd, encoding: 'utf8', stdio: ['ignore', 'pipe', 'ignore'] }
    );
    const tagPattern = /^(?:pre-step-|checkpoint-|pre-[a-z]+-).+/i;
    if (tags.split('\n').some(t => tagPattern.test(t.trim()))) return true;
  } catch { /* no tags or no git — fine */ }

  return false;
}

// Legacy name kept for any external caller; prefer hasRecentCheckpointMarker.
function hasRecentCheckpointCommit(cwd, lookback) {
  return hasRecentCheckpointMarker(cwd, lookback);
}

function getProfile() {
  const raw = String(process.env.RCC_HOOK_PROFILE || process.env.ECC_HOOK_PROFILE || 'standard').trim().toLowerCase();
  return ['minimal', 'standard', 'strict'].includes(raw) ? raw : 'standard';
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  const toolName = input.tool_name || input.toolName || '';
  if (!['Write', 'Edit', 'MultiEdit'].includes(toolName)) { passthrough(); return; }

  const filePath = input.tool_input?.file_path || input.tool_input?.path;
  if (!filePath) { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  const config = loadConfig(cwd);
  if (!config) { passthrough(); return; }

  const requiredGlobs = config.checkpoint_required_for || [];
  if (requiredGlobs.length === 0) { passthrough(); return; }

  let relPath = filePath;
  if (path.isAbsolute(filePath)) {
    const rel = path.relative(cwd, filePath);
    if (!rel.startsWith('..')) relPath = rel;
  }

  if (!requiredGlobs.some(g => matchesGlob(relPath, g))) { passthrough(); return; }

  // Algorithm file is being modified. Check checkpoint discipline.
  const status = gitFileStatus(cwd, relPath);
  if (status === null) { passthrough(); return; }   // not a git repo

  // Brand new file (Write creates from nothing) — proceed silently.
  if (status === '' && !fs.existsSync(path.join(cwd, relPath))) {
    passthrough();
    return;
  }

  const hasCheckpoint = hasRecentCheckpointMarker(cwd, CHECKPOINT_LOOKBACK);
  if (hasCheckpoint) { passthrough(); return; }     // discipline observed

  // Rule 8 intent: "before modifying core algorithm code, create a git
  // checkpoint commit OR tag". We therefore enforce on BOTH:
  //   - dirty file + no marker (mid-edit without preparing)
  //   - clean file + no marker (first edit of a new step without preparing)
  // The latter was the previously-silent path that masked TofSLAM's
  // tag-based discipline going unenforced.
  const profile = getProfile();
  const dirty = status !== '';
  const header = profile === 'strict'
    ? `[checkpoint-gate] BLOCKED: algorithm file edit without recent checkpoint marker`
    : `[checkpoint-gate] WARNING: algorithm file edit without recent checkpoint marker`;

  process.stderr.write(`${header}\n`);
  process.stderr.write(`  file: ${relPath}\n`);
  process.stderr.write(`  status: ${dirty ? status : 'clean (pre-first-edit)'}\n`);
  process.stderr.write(`  Recommended (either form):\n`);
  process.stderr.write(`    git commit -am "checkpoint: pre-<change-name>"\n`);
  process.stderr.write(`    git tag pre-step-<sprint>-<step>          # tag-based discipline\n`);
  process.stderr.write(`  Then retry. Rule 8 — ben.md\n`);

  passthrough();
  process.exit(profile === 'strict' ? 2 : 0);
}

main().catch(() => {
  process.stdout.write('');
  process.exit(0);
});
