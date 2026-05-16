#!/usr/bin/env node
'use strict';

/**
 * post:bash:canonical-eval-detect (RCC research hook)
 *
 * Closes the "long-running eval invisibility" gap described in
 * 20260513_hook_issue §5: when ben runs a canonical evaluation via
 * direct Bash + Monitor pattern (e.g. 4h `nohup bash run_canonical_eval.sh ...`),
 * stage-record never fires (it is gated on Task/Agent tool, not Bash).
 *
 * This hook scans the Bash command and the post-command artefacts on disk
 * for canonical-eval completion markers. When detected:
 *   - recordStage(build, direct-impl, PASS|BLOCK) into stage_history
 *   - appendRegression(metric, value, baseline) when metrics file exists
 *
 * Detection inputs (any of):
 *   - command pattern matches `run_canonical_eval`, `run_S\\d+_canonical`,
 *     `run_canonical`, or the user-provided `canonical_eval_patterns`
 *     in `.claude/research-config.json`
 *   - dump/<label>/.done sentinel file modified within last 5 minutes
 *   - dump/<label>/grand_mean.json or dump/<label>/summary.json present
 *     and modified within last 5 minutes
 *
 * Profile gating:
 *   minimal   → no-op
 *   standard  → record + alert (exit 0)
 *   strict    → record + alert (exit 0; never blocks Bash post-hook)
 *
 * Behaviour intentionally tolerant — false positives (recording extra
 * PASS entries) are recoverable via rcc-state-amend; false negatives
 * (missed eval) are the failure mode this hook exists to prevent.
 */

const fs = require('fs');
const path = require('path');
const rs = require('../lib/research-state');

const MAX_STDIN = 1024 * 1024;
const RECENT_WINDOW_MS = 5 * 60 * 1000;   // 5 minutes
const DEFAULT_PATTERNS = [
  /\brun_canonical_eval\b/i,
  /\brun_S\d+[\w]*_canonical\b/i,    // matches run_S12_V5_canonical (V5 has digit)
  /\brun_canonical\b/i,
  /\bcanonical[_-]?eval\b/i,
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
    process.env.RCC_HOOK_PROFILE || process.env.ECC_HOOK_PROFILE || 'standard',
  ).trim().toLowerCase();
  return ['minimal', 'standard', 'strict'].includes(raw) ? raw : 'standard';
}

function loadConfig(cwd) {
  const candidates = [
    path.join(cwd, '.claude', 'research-config.json'),
    path.join(cwd, 'research-config.json'),
  ];
  for (const p of candidates) {
    if (fs.existsSync(p)) {
      try { return JSON.parse(fs.readFileSync(p, 'utf8')); } catch { return null; }
    }
  }
  return null;
}

function compileExtraPatterns(config) {
  const out = [];
  const fromConfig = config && Array.isArray(config.canonical_eval_patterns)
    ? config.canonical_eval_patterns : [];
  for (const p of fromConfig) {
    try { out.push(new RegExp(p, 'i')); } catch { /* skip malformed */ }
  }
  return out;
}

// Find canonical-eval label argument in command. Recognizes shell forms:
//   run_canonical_eval.sh S12_V5_canonical
//   bash docker/run_canonical_eval.sh --label=S13_B6 ...
//   run_canonical_eval.sh -l "S13 sweep"
function extractEvalLabel(cmd) {
  const flag = cmd.match(/--label[=\s]+(['"]?)([\w.-]+)\1/);
  if (flag) return flag[2];
  const dashL = cmd.match(/\s-l\s+(['"])([^'"]+)\1/);
  if (dashL) return dashL[2];
  // last bare token after run_canonical_eval(.sh)? typically the label
  const tail = cmd.match(/run_(?:canonical_eval|canonical|S\d+[A-Za-z_]*_canonical)(?:\.sh)?\s+([\w.-]+)/i);
  if (tail) return tail[1];
  return null;
}

// Commands that *mention* the runner name in passing but don't actually
// invoke it. Excluded so a `git log --grep="run_canonical_eval"` doesn't
// fake a PASS record.
function looksLikeRealInvocation(cmd) {
  const trimmed = cmd.trim();
  if (!trimmed) return false;

  // Help / dry-run flags → user is inspecting, not running
  if (/(?:^|\s)(-h|--help|--dry-run|--list)\b/i.test(trimmed)) return false;

  // Common false-positive command shapes that only *reference* the runner
  // - grep / rg / ag / git-grep / git-log --grep
  // - echo / printf / cat / less / head / tail (passive reads)
  // - find / fd (search)
  // - which / type / where (binary lookup)
  const passive = /^(?:grep|rg|ag|fgrep|egrep|echo|printf|cat|less|more|head|tail|find|fd|which|type|where|ls|stat|file|md5sum|sha\d+sum)\b/;
  if (passive.test(trimmed)) return false;
  if (/^git\s+(?:log|grep|show)/.test(trimmed)) return false;

  // Heuristic: real invocation usually starts with bash/sh/python/zsh/
  // nohup/timeout/./{path}/abs path, or directly with the runner name.
  // Accept anything that LOOKS like execution rather than text inspection.
  if (/^(?:bash|sh|zsh|fish|python|python3|node|nohup|timeout|env|nice|ionice)\b/.test(trimmed)) return true;
  if (/^\.?\/[\w./_-]+/.test(trimmed)) return true;
  if (/^run_[\w./_-]+/.test(trimmed)) return true;
  // Allow if it looks like an absolute path execution
  if (/^\/[\w./_-]+/.test(trimmed)) return true;
  // Default: not clearly an invocation; do not fire
  return false;
}

function matchesCanonicalPattern(cmd, extraPatterns) {
  if (!looksLikeRealInvocation(cmd)) return false;
  const patterns = DEFAULT_PATTERNS.concat(extraPatterns);
  return patterns.some(re => re.test(cmd));
}

// Look for completion markers in cwd. Returns the most recently-touched
// matching path, or null. A marker is recent only if mtime is within
// RECENT_WINDOW_MS of "now" — older artefacts are stale and probably
// from a previous run.
function findRecentMarker(cwd, label) {
  const dump = path.join(cwd, 'dump');
  if (!fs.existsSync(dump)) return null;
  const now = Date.now();
  const candidates = [];
  const dirs = label
    ? [path.join(dump, label)]
    : safeReaddir(dump).map(d => path.join(dump, d));
  for (const dir of dirs) {
    if (!fs.existsSync(dir)) continue;
    for (const fname of ['.done', 'grand_mean.json', 'summary.json', '_done.txt']) {
      const f = path.join(dir, fname);
      if (!fs.existsSync(f)) continue;
      try {
        const m = fs.statSync(f).mtimeMs;
        if (now - m <= RECENT_WINDOW_MS) candidates.push({ path: f, mtime: m });
      } catch { /* ignore */ }
    }
  }
  if (candidates.length === 0) return null;
  candidates.sort((a, b) => b.mtime - a.mtime);
  return candidates[0];
}

function safeReaddir(dir) {
  try { return fs.readdirSync(dir); } catch { return []; }
}

// Pull ATE/score from a marker file. Tolerant of multiple shapes:
//   - JSON: { "ate_grand_mean": ..., "ate": ..., "metric": ..., "value": ... }
//   - plain text: lines like "Grand mean: 0.3947" or "Mean (9/9): 0.3096"
function extractMetricFromMarker(markerPath) {
  let body = '';
  try { body = fs.readFileSync(markerPath, 'utf8'); } catch { return null; }
  if (markerPath.endsWith('.json')) {
    try {
      const j = JSON.parse(body);
      for (const key of ['ate_grand_mean', 'grand_mean', 'ate', 'metric', 'value']) {
        if (typeof j[key] === 'number') return { name: 'ATE', value: j[key], source: key };
      }
    } catch { /* fall through */ }
  }
  const patterns = [
    /\bgrand\s*mean\s*[:=]\s*([0-9.]+)/i,
    /\bmean\s*\([^)]*\)\s*[:=]\s*([0-9.]+)/i,
    /\bate\s*rmse\s*[:=]\s*([0-9.]+)/i,
    /\brmse\s*[:=]\s*([0-9.]+)/i,
  ];
  for (const re of patterns) {
    const m = body.match(re);
    if (m) return { name: 'ATE', value: Number(m[1]), source: re.source };
  }
  return null;
}

function loadBaseline(cwd, config) {
  const metric = (config && Array.isArray(config.metrics))
    ? config.metrics.find(m => m && m.name === 'ATE')
    : null;
  if (!metric || !metric.baseline_path) return null;
  const baselinePath = path.isAbsolute(metric.baseline_path)
    ? metric.baseline_path
    : path.join(cwd, metric.baseline_path);
  try {
    const body = JSON.parse(fs.readFileSync(baselinePath, 'utf8'));
    if (typeof body === 'number') return { value: body, tolerance_pct: metric.tolerance_pct || 0 };
    if (body && typeof body.metrics === 'object' && typeof body.metrics.ATE === 'number') {
      return { value: body.metrics.ATE, tolerance_pct: metric.tolerance_pct || 0 };
    }
    if (typeof body.ATE === 'number') {
      return { value: body.ATE, tolerance_pct: metric.tolerance_pct || 0 };
    }
  } catch { /* missing or unparseable */ }
  return null;
}

function classifyVerdict(value, baseline) {
  if (!baseline) return { verdict: 'PASS', delta_pct: null };
  const delta = ((value - baseline.value) / baseline.value) * 100;
  if (delta > baseline.tolerance_pct) {
    return { verdict: 'BLOCK', delta_pct: Number(delta.toFixed(2)) };
  }
  return { verdict: 'PASS', delta_pct: Number(delta.toFixed(2)) };
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  if (getProfile() === 'minimal') { passthrough(); return; }

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  const toolName = input.tool_name || input.toolName || '';
  if (toolName !== 'Bash') { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  if (!rs.isValidProjectCwd(cwd)) { passthrough(); return; }

  const cmd = String(input.tool_input?.command || '');
  if (!cmd) { passthrough(); return; }

  const config = loadConfig(cwd);
  const extraPatterns = compileExtraPatterns(config);

  // Conservative gate: require command pattern match. Marker files alone
  // are not sufficient — a stray `.done` from an unrelated workflow would
  // otherwise trigger false-positive PASS records. Once the command
  // matches, marker presence upgrades the entry with verdict + metric.
  const matched = matchesCanonicalPattern(cmd, extraPatterns);
  if (!matched) { passthrough(); return; }

  const label = extractEvalLabel(cmd);
  const marker = findRecentMarker(cwd, label);

  // Gather metric if available
  const metric = marker ? extractMetricFromMarker(marker.path) : null;
  const baseline = metric ? loadBaseline(cwd, config) : null;
  const { verdict, delta_pct } = classifyVerdict(metric ? metric.value : NaN, baseline);

  try {
    rs.withStateLock(cwd, () => {
      const state = rs.ensureState(cwd);
      const entry = {
        stage: 'build',
        agent: 'direct-impl',
        verdict: metric ? verdict : 'PENDING',
        detection_source: marker ? 'canonical-marker' : 'cmd-pattern',
      };
      if (label) entry.notes = `canonical eval auto-recorded: label=${label}`;
      else entry.notes = 'canonical eval auto-recorded (command match)';
      if (metric) {
        entry.notes += ` | ${metric.name}=${metric.value}` +
          (delta_pct !== null ? ` Δ=${delta_pct}%` : '');
      }
      if (marker) entry.output_ref = path.relative(cwd, marker.path);

      rs.recordStage(state, entry);

      if (metric && baseline) {
        rs.appendRegression(state, {
          metric: metric.name,
          value: metric.value,
          baseline: baseline.value,
          delta_pct,
          verdict: verdict === 'BLOCK' ? 'REGRESSION' : 'OK',
          command: cmd.slice(0, 200),
        });
      }

      rs.writeState(cwd, state);
    });

    const metricPart = metric
      ? ` ${metric.name}=${metric.value}` + (delta_pct !== null ? ` Δ=${delta_pct}%` : '')
      : '';
    process.stderr.write(
      `[rcc:canonical-eval-detect] auto-recorded ` +
      `verdict=${metric ? verdict : 'PENDING'}` +
      ` label=${label || '(unknown)'}${metricPart}\n` +
      `  → state.json updated. Amend if misclassified: ` +
      `node scripts/rcc-state-amend.js <verdict> "reason"\n`,
    );
  } catch (e) {
    process.stderr.write(`[rcc:canonical-eval-detect] error: ${e.message}\n`);
  }

  passthrough();
}

main().catch(() => process.exit(0));
