#!/usr/bin/env node
'use strict';

/**
 * pre:compact:save-state (RCC v2.1 Phase α hook)
 *
 * Trigger: PreCompact (Claude Code native event before context compression).
 *
 * Flush a human-readable summary of .claude/research-state.json to
 * ~/.claude/memory/<project-slug>_research_state.md so the pipeline
 * survives context compression. The full JSON state is already on disk
 * — this hook just makes it discoverable to the memory subsystem after
 * compact.
 *
 * Profile gating:
 *   minimal  → no-op
 *   standard → write summary
 *   strict   → write summary
 */

const fs = require('fs');
const path = require('path');
const os = require('os');
const rs = require('../lib/research-state');

const MAX_STDIN = 1024 * 1024;

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

function projectSlug(cwd) {
  const base = path.basename(path.resolve(cwd));
  return base.replace(/[^a-zA-Z0-9._-]/g, '_').toLowerCase();
}

function renderSummary(state, cwd) {
  const lines = [];
  lines.push('---');
  lines.push('name: RCC pipeline state');
  lines.push(`description: Saved by precompact-save-state hook. Project: ${path.basename(cwd)}. Stage: ${state.current_stage}.`);
  lines.push('type: project');
  lines.push('---');
  lines.push('');
  lines.push(`# RCC pipeline state — ${path.basename(cwd)}`);
  lines.push('');
  lines.push(`**Current stage**: \`${state.current_stage}\`  `);
  lines.push(`**Started**: ${state.started_at}  `);
  if (state.active_baseline_ref) {
    lines.push(`**Active baseline**: ${state.active_baseline_ref}` +
      (state.active_baseline_sha256 ? ` (sha256: ${state.active_baseline_sha256.slice(0, 12)}…)` : ''));
  }
  if (state.active_experiment?.name) {
    lines.push(`**Active experiment**: ${state.active_experiment.name} (tracker: ${state.active_experiment.tracker || '—'})`);
  }
  lines.push('');
  lines.push('## Stage history (last 10)');
  lines.push('');
  const hist = (state.stage_history || []).slice(-10);
  if (hist.length === 0) {
    lines.push('_(empty)_');
  } else {
    lines.push('| Stage | Agent | Verdict | Output ref |');
    lines.push('|---|---|---|---|');
    for (const h of hist) {
      lines.push(`| ${h.stage} | ${h.agent || '—'} | **${h.verdict || 'PENDING'}** | ${h.output_ref || '—'} |`);
    }
  }
  lines.push('');
  if (state.regression_history && state.regression_history.length) {
    lines.push('## Regression history (last 5)');
    lines.push('');
    lines.push('| Metric | Value | Baseline | Δ% | Verdict |');
    lines.push('|---|---:|---:|---:|---|');
    for (const r of state.regression_history.slice(-5)) {
      lines.push(`| ${r.metric} | ${r.value} | ${r.baseline ?? '—'} | ${r.delta_pct?.toFixed?.(2) ?? '—'} | ${r.verdict} |`);
    }
    lines.push('');
  }
  if (state.locks) {
    lines.push('## Pipeline locks');
    lines.push('');
    for (const [stage, reqs] of Object.entries(state.locks)) {
      const missing = rs.blockedPrerequisites(state, stage);
      const status = missing.length === 0 ? '[unlocked]' : `[blocked by: ${missing.join(', ')}]`;
      lines.push(`- **${stage}** ← requires \`${reqs.join(', ')}\` — ${status}`);
    }
  }
  lines.push('');
  lines.push(`> Restored after compact from ${rs.STATE_RELPATH}`);
  return lines.join('\n') + '\n';
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  if (getProfile() === 'minimal') { passthrough(); return; }

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  if (!rs.isValidProjectCwd(cwd)) {
    process.stderr.write(
      `[rcc:precompact-save-state] WARNING: invalid project cwd: ${cwd}\n`,
    );
    passthrough();
    return;
  }
  const state = rs.readState(cwd);
  if (!state) { passthrough(); return; }

  try {
    const memoryDir = path.join(os.homedir(), '.claude', 'memory');
    fs.mkdirSync(memoryDir, { recursive: true });
    const slug = projectSlug(cwd);
    const memFile = path.join(memoryDir, `${slug}_research_state.md`);
    const summary = renderSummary(state, cwd);
    fs.writeFileSync(memFile, summary);
    process.stderr.write(`[precompact-save-state] flushed → ${memFile}\n`);
    pruneStaleMemoryFiles(memoryDir);
  } catch (e) {
    process.stderr.write(`[precompact-save-state] error: ${e.message}\n`);
  }

  passthrough();
}

// Best-effort: drop *_research_state.md files older than PRUNE_AGE_MS.
// Memory directory auto-loads every session, so stale project markdowns
// (project deleted, archived, dormant) keep adding token cost forever.
// 30 days of inactivity is the threshold — re-running precompact on an
// active project refreshes its mtime and survives prune.
const PRUNE_AGE_MS = 30 * 24 * 60 * 60 * 1000;

function pruneStaleMemoryFiles(memoryDir) {
  let files;
  try { files = fs.readdirSync(memoryDir); } catch { return; }
  const cutoff = Date.now() - PRUNE_AGE_MS;
  let pruned = 0;
  for (const f of files) {
    if (!f.endsWith('_research_state.md')) continue;
    const full = path.join(memoryDir, f);
    try {
      const mtime = fs.statSync(full).mtimeMs;
      if (mtime < cutoff) {
        fs.unlinkSync(full);
        pruned++;
      }
    } catch { /* skip on stat/unlink errors */ }
  }
  if (pruned > 0) {
    process.stderr.write(
      `[precompact-save-state] pruned ${pruned} stale memory file(s) >30d old\n`,
    );
  }
}

main().catch(() => process.exit(0));
