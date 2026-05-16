#!/usr/bin/env node
'use strict';

/**
 * post:bash:eval-regression-check (RCC research hook)
 *
 * PostToolUse hook on Bash. Scans command stdout for evaluation metric
 * patterns defined in `.claude/research-config.json`. When a metric value
 * is detected and exceeds tolerance vs the project baseline, emits an
 * informational alert. Always exits 0 — never blocks (informational only).
 *
 * Generic across domains:
 *   SLAM:  ATE RMSE, RPE  (lower_is_better)
 *   3DGS:  PSNR, SSIM     (higher_is_better)
 *          LPIPS          (lower_is_better)
 *   ML:    accuracy, F1, mAP, IoU (configurable direction)
 *
 * Config schema:
 *   metrics: [{ name, direction, trigger_patterns[], extract_regex,
 *               baseline_path, tolerance_pct }]
 *
 *   - direction: "lower_is_better" | "higher_is_better"
 *   - trigger_patterns: regex[] — at least one must match the output
 *     to enable extraction (avoids false positives on unrelated commands)
 *   - extract_regex: regex with one capture group (the numeric value)
 *   - baseline_path: relative path to baseline JSON containing { metrics: { <name>: <value> } }
 *                    or a flat number; supports plain JSON only
 *   - tolerance_pct: percent over baseline (or under, depending on direction)
 *                    triggering an alert
 */

const fs = require('fs');
const path = require('path');

const MAX_STDIN = 4 * 1024 * 1024; // larger to capture eval output

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

function loadBaselineValue(cwd, baselinePath, metricName) {
  if (!baselinePath) return null;
  const abs = path.isAbsolute(baselinePath) ? baselinePath : path.join(cwd, baselinePath);
  if (!fs.existsSync(abs)) return null;
  try {
    const content = fs.readFileSync(abs, 'utf8').trim();
    // Attempt JSON parse first
    try {
      const data = JSON.parse(content);
      if (typeof data === 'number') return data;
      if (data && typeof data === 'object') {
        if (typeof data.metrics === 'object' && metricName in data.metrics) {
          const v = data.metrics[metricName];
          return typeof v === 'number' ? v : Number(v);
        }
        if (metricName in data) {
          const v = data[metricName];
          return typeof v === 'number' ? v : Number(v);
        }
      }
    } catch {
      // Maybe a plain text file like "ate: 0.293"
      const m = content.match(new RegExp(`${metricName}\\s*[:=]\\s*([-+]?[0-9]*\\.?[0-9]+)`, 'i'));
      if (m) return Number(m[1]);
    }
  } catch { /* fall through */ }
  return null;
}

// Extract leading inline flag group like (?i), (?im), (?ims) and return
// { source, flags }. Node 18 does not support inline flag groups inside
// the regex literal, so we convert them to constructor flags manually.
// This lets users author case-insensitive ATE/PSNR patterns naturally.
function splitInlineFlags(src) {
  const m = String(src || '').match(/^\(\?([imsux]+)\)(.*)$/s);
  if (!m) return { source: String(src || ''), flags: '' };
  // Filter out flags Node doesn't understand on RegExp constructor
  const supported = m[1].split('').filter(f => 'imsuy'.includes(f)).join('');
  return { source: m[2], flags: supported };
}

function compileRegex(src, extraFlags = '') {
  try {
    const { source, flags } = splitInlineFlags(src);
    const merged = [...new Set((flags + extraFlags).split(''))].join('');
    return new RegExp(source, merged);
  } catch {
    return null;
  }
}

function extractMetricValues(output, extractRegexSrc) {
  const re = compileRegex(extractRegexSrc);
  if (!re) return [];
  const values = [];
  // Use a global variant to capture all occurrences. Honor inline flags
  // declared on the source so (?i)... and similar continue to work.
  const global = compileRegex(extractRegexSrc, 'g');
  if (!global) return values;
  let m;
  while ((m = global.exec(output)) !== null) {
    if (m[1] !== undefined) {
      const v = Number(m[1]);
      if (Number.isFinite(v)) values.push(v);
    }
    if (global.lastIndex === m.index) global.lastIndex++;
  }
  return values;
}

function isRegression(value, baseline, direction, tolerancePct) {
  if (!Number.isFinite(value) || !Number.isFinite(baseline)) return false;
  const tol = (tolerancePct || 0) / 100;
  if (direction === 'higher_is_better') {
    // regression = value drops below baseline by more than tolerance
    return value < baseline * (1 - tol);
  }
  // default lower_is_better
  return value > baseline * (1 + tol);
}

function summarizeValues(values, direction) {
  if (values.length === 0) return null;
  // Pick the worst-case value as representative
  if (direction === 'higher_is_better') {
    return Math.min(...values);
  }
  return Math.max(...values);
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  const toolName = input.tool_name || input.toolName || '';
  if (toolName !== 'Bash') { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  const config = loadConfig(cwd);
  if (!config || !Array.isArray(config.metrics)) { passthrough(); return; }

  const output = String(input.tool_response?.stdout || '') +
                 '\n' +
                 String(input.tool_response?.stderr || '');

  const alerts = [];

  for (const metric of config.metrics) {
    if (!metric.name || !metric.extract_regex) continue;

    // Trigger gate
    const triggers = metric.trigger_patterns || [];
    if (triggers.length > 0) {
      const triggered = triggers.some(p => {
        const re = compileRegex(p);
        return re && re.test(output);
      });
      if (!triggered) continue;
    }

    const values = extractMetricValues(output, metric.extract_regex);
    if (values.length === 0) continue;

    const baseline = loadBaselineValue(cwd, metric.baseline_path, metric.name);
    if (baseline === null) {
      // Inform but no comparison possible
      const repr = summarizeValues(values, metric.direction);
      alerts.push({
        kind: 'INFO',
        msg: `${metric.name}=${repr.toFixed(6)} (no baseline at ${metric.baseline_path || '(unset)'})`,
      });
      continue;
    }

    const repr = summarizeValues(values, metric.direction);
    const regressed = isRegression(repr, baseline, metric.direction || 'lower_is_better',
                                    metric.tolerance_pct || 0);

    if (regressed) {
      const dir = metric.direction === 'higher_is_better' ? '↓' : '↑';
      const pct = ((repr - baseline) / baseline * 100).toFixed(2);
      alerts.push({
        kind: 'REGRESSION',
        msg: `${metric.name}=${repr} (baseline=${baseline}, ${dir} ${pct}%, tol=${metric.tolerance_pct}%)`,
      });
    } else {
      const dir = metric.direction === 'higher_is_better' ? '↑' : '↓';
      const pct = ((repr - baseline) / baseline * 100).toFixed(2);
      alerts.push({
        kind: 'OK',
        msg: `${metric.name}=${repr} (baseline=${baseline}, ${dir} ${pct}%)`,
      });
    }
  }

  if (alerts.length > 0) {
    const regressed = alerts.filter(a => a.kind === 'REGRESSION');
    if (regressed.length > 0) {
      process.stderr.write('[eval-regression-check] REGRESSION DETECTED:\n');
      for (const a of regressed) process.stderr.write(`  - ${a.msg}\n`);
      const ok = alerts.filter(a => a.kind === 'OK');
      if (ok.length > 0) {
        process.stderr.write('[eval-regression-check] OK:\n');
        for (const a of ok) process.stderr.write(`  - ${a.msg}\n`);
      }
    } else {
      process.stderr.write('[eval-regression-check] metrics observed:\n');
      for (const a of alerts) process.stderr.write(`  - [${a.kind}] ${a.msg}\n`);
    }
  }

  passthrough();
  process.exit(0); // always informational
}

main().catch(() => {
  process.stdout.write('');
  process.exit(0);
});
