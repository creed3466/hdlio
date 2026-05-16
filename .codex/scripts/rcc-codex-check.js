#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');

const SECRET_PATTERNS = [
  ['OpenAI key', /sk-[A-Za-z0-9]{20,}/],
  ['GitHub classic token', /ghp_[A-Za-z0-9]{36}/],
  ['GitHub fine-grained token', /github_pat_[A-Za-z0-9_]{20,}/],
  ['AWS access key', /AKIA[0-9A-Z]{16}/],
  ['private key block', /-----BEGIN (RSA|EC|OPENSSH|DSA|PRIVATE) KEY-----/],
  ['generic credential assignment', /\b(api[_-]?key|secret|password|token)\b\s*[:=]\s*['"][^'"]{12,}['"]/i],
];

const CITATION_PATTERNS = [
  /\bhttps?:\/\/\S+/i,
  /\b[A-Z][a-z]+(?:\s+(?:and|&)\s+[A-Z][a-z]+)?\s+et\s+al\.?\b/,
  /\[(?:[A-Z][a-zA-Z]+,?\s*)+(?:19|20)\d{2}[a-z]?\]/,
  /\barXiv:\s*\d{4}\.\d{4,5}\b/i,
  /\bdoi:\s*10\.\d{4,9}\/[-._;()/:A-Z0-9]+/i,
  /\b\d+(?:\.\d+)?\s*%\s+(?:improvement|reduction|gain|increase|decrease|better|worse)/i,
];

const VERIFICATION_TAG = /\[(verified(?::[^\]]*)?|unverified(?::[^\]]*)?)\]/i;
const TEXT_EXTENSIONS = new Set([
  '.c', '.cc', '.cpp', '.cs', '.css', '.go', '.h', '.hpp', '.html', '.java',
  '.js', '.jsx', '.json', '.kt', '.md', '.mjs', '.py', '.rs', '.sh', '.sql',
  '.swift', '.toml', '.ts', '.tsx', '.txt', '.yaml', '.yml',
]);
const CODE_EXTENSIONS = new Set([
  '.c', '.cc', '.cpp', '.cs', '.go', '.java', '.js', '.jsx', '.kt', '.mjs',
  '.py', '.rs', '.swift', '.ts', '.tsx',
]);
const CONSOLE_EXTENSIONS = new Set(['.js', '.jsx', '.mjs', '.cjs', '.ts', '.tsx', '.vue', '.svelte']);
const PYTHON_EXTENSIONS = new Set(['.py', '.pyi']);
const CPP_EXTENSIONS = new Set(['.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx']);

function usage() {
  console.log(`Usage:
  rcc-codex-check <command> [options]

Commands:
  research --changed|--staged|--all     Check research citations for [verified]/[unverified] tags
  checkpoint --changed|--staged         Check configured algorithm checkpoint discipline
  quality --changed|--staged            Run project-native lint/typecheck/test when available
  console --changed|--staged|--all      Scan changed code for console.log/debugger
  secrets --staged|--changed|--all      Scan files or staged additions for secrets
  mcp                                  Verify Codex MCP config is loadable
  eval --log <file>                    Compare eval log metrics to configured baselines
  all                                  Run research, checkpoint, console, secrets, quality, mcp

Options:
  --changed     Use changed/untracked files
  --staged      Use staged files
  --all         Use all git-tracked files
  --log <file>  Eval log file
  --json        Emit JSON summary
`);
}

function run(command, args, options = {}) {
  return spawnSync(command, args, {
    cwd: options.cwd || process.cwd(),
    encoding: 'utf8',
    stdio: options.stdio || ['ignore', 'pipe', 'pipe'],
    env: process.env,
    timeout: options.timeout || 120000,
  });
}

function git(args) {
  return run('git', args);
}

function isGitRepo() {
  return git(['rev-parse', '--is-inside-work-tree']).status === 0;
}

function gitRoot() {
  const result = git(['rev-parse', '--show-toplevel']);
  return result.status === 0 ? result.stdout.trim() : process.cwd();
}

function listFiles(mode) {
  if (!isGitRepo()) return [];
  if (mode === 'staged') {
    return lines(git(['diff', '--cached', '--name-only', '--diff-filter=ACMR']).stdout);
  }
  if (mode === 'all') {
    return lines(git(['ls-files']).stdout);
  }

  const tracked = lines(git(['diff', '--name-only', '--diff-filter=ACMR', 'HEAD']).stdout);
  const untracked = lines(git(['ls-files', '--others', '--exclude-standard']).stdout);
  return unique([...tracked, ...untracked]);
}

function stagedAddedLines(file) {
  const result = git(['diff', '--cached', '-U0', '--', file]);
  if (result.status !== 0) return '';
  return result.stdout
    .split('\n')
    .filter(line => line.startsWith('+') && !line.startsWith('+++ '))
    .map(line => line.slice(1))
    .join('\n');
}

function lines(text) {
  return String(text || '').split(/\r?\n/).map(v => v.trim()).filter(Boolean);
}

function unique(values) {
  return [...new Set(values)];
}

function commandExists(command) {
  return run('sh', ['-c', `command -v ${JSON.stringify(command)} >/dev/null 2>&1`], { timeout: 15000 }).status === 0;
}

function isSkippedFile(file) {
  return /(^|\/)(node_modules|\.git|\.codex|\.agents|dist|build|coverage|\.next)\//.test(file)
    || /\.(png|jpg|jpeg|gif|webp|pdf|zip|gz|lock)$/i.test(file)
    || ['pnpm-lock.yaml', 'package-lock.json', 'yarn.lock', 'bun.lockb'].includes(path.basename(file));
}

function readTextFile(root, file) {
  if (isSkippedFile(file)) return null;
  const abs = path.join(root, file);
  const ext = path.extname(file).toLowerCase();
  if (!TEXT_EXTENSIONS.has(ext) && !fs.existsSync(abs)) return null;
  try {
    const stat = fs.statSync(abs);
    if (!stat.isFile() || stat.size > 2 * 1024 * 1024) return null;
    const buf = fs.readFileSync(abs);
    if (buf.includes(0)) return null;
    return buf.toString('utf8');
  } catch {
    return null;
  }
}

function loadResearchConfig(root) {
  for (const candidate of [
    path.join(root, '.claude', 'research-config.json'),
    path.join(root, 'research-config.json'),
  ]) {
    if (!fs.existsSync(candidate)) continue;
    try {
      return JSON.parse(fs.readFileSync(candidate, 'utf8'));
    } catch {
      return null;
    }
  }
  return {
    research_paths: ['docs/research/**/*.md', 'research/**/*.md', 'docs/results/**/*.md'],
    checkpoint_required_for: [],
    metrics: [],
  };
}

function globToRegExp(glob) {
  let out = '^';
  for (let i = 0; i < glob.length; i += 1) {
    const ch = glob[i];
    if (ch === '*') {
      if (glob[i + 1] === '*') {
        if (glob[i + 2] === '/') {
          i += 2;
          out += '(?:.*/)?';
        } else {
          i += 1;
          out += '.*';
        }
      } else {
        out += '[^/]*';
      }
    } else if (ch === '?') {
      out += '.';
    } else {
      out += ch.replace(/[.+^${}()|[\]\\]/g, '\\$&');
    }
  }
  return new RegExp(`${out}$`);
}

function matchesAny(file, patterns) {
  return Array.isArray(patterns) && patterns.some(pattern => globToRegExp(pattern).test(file));
}

function result(name) {
  return { name, status: 'pass', findings: [], notes: [] };
}

function addFinding(res, severity, message, file = null) {
  res.findings.push({ severity, message, file });
  if (severity === 'error') res.status = 'fail';
  else if (res.status === 'pass') res.status = 'warn';
}

function checkResearch(options) {
  const res = result('research');
  const root = gitRoot();
  const config = loadResearchConfig(root);
  const files = listFiles(options.mode).filter(file => matchesAny(file, config.research_paths || []));
  if (files.length === 0) {
    res.notes.push('No changed research files matched research_paths.');
    return res;
  }

  for (const file of files) {
    const content = readTextFile(root, file);
    if (!content) continue;
    const fileLines = content.split(/\r?\n/);
    for (let i = 0; i < fileLines.length; i += 1) {
      const line = fileLines[i];
      const hit = CITATION_PATTERNS.find(pattern => pattern.test(line));
      if (!hit) continue;
      let tagged = false;
      for (let j = Math.max(0, i - 3); j <= Math.min(fileLines.length - 1, i + 3); j += 1) {
        if (VERIFICATION_TAG.test(fileLines[j])) {
          tagged = true;
          break;
        }
      }
      if (!tagged) {
        addFinding(res, 'error', `line ${i + 1}: citation/claim lacks [verified] or [unverified] tag`, file);
      }
    }
  }
  if (res.findings.length === 0) res.notes.push(`Checked ${files.length} research file(s).`);
  return res;
}

function hasRecentCheckpoint(root) {
  const out = git(['log', '--oneline', '-10']).stdout;
  return /\bcheckpoint:/i.test(out);
}

function checkCheckpoint(options) {
  const res = result('checkpoint');
  const root = gitRoot();
  const config = loadResearchConfig(root);
  const patterns = config.checkpoint_required_for || [];
  if (patterns.length === 0) {
    res.notes.push('No checkpoint_required_for patterns configured.');
    return res;
  }
  const files = listFiles(options.mode).filter(file => matchesAny(file, patterns));
  if (files.length === 0) {
    res.notes.push('No changed files require checkpoint discipline.');
    return res;
  }
  if (!hasRecentCheckpoint(root)) {
    for (const file of files) {
      addFinding(res, 'error', 'algorithm/research-critical file changed without recent checkpoint: commit', file);
    }
  } else {
    res.notes.push(`Checkpoint commit found for ${files.length} critical file(s).`);
  }
  return res;
}

function checkConsole(options) {
  const res = result('console');
  const root = gitRoot();
  const files = listFiles(options.mode).filter(file => CONSOLE_EXTENSIONS.has(path.extname(file).toLowerCase()));
  for (const file of files) {
    const content = readTextFile(root, file);
    if (!content) continue;
    const fileLines = content.split(/\r?\n/);
    fileLines.forEach((line, index) => {
      if (/\bconsole\.(log|debug|trace)\s*\(/.test(line) || /\bdebugger\s*;?/.test(line)) {
        addFinding(res, 'error', `line ${index + 1}: console/debug statement`, file);
      }
    });
  }
  if (res.findings.length === 0) res.notes.push(`Checked ${files.length} console-capable file(s).`);
  return res;
}

function checkSecrets(options) {
  const res = result('secrets');
  const root = gitRoot();
  const files = listFiles(options.mode);
  for (const file of files) {
    const content = options.mode === 'staged' ? stagedAddedLines(file) : readTextFile(root, file);
    if (!content) continue;
    for (const [label, pattern] of SECRET_PATTERNS) {
      const match = content.match(pattern);
      if (match) {
        addFinding(res, 'error', `${label} detected`, file);
      }
    }
  }
  if (res.findings.length === 0) res.notes.push(`Checked ${files.length} file(s) for high-signal secrets.`);
  return res;
}

function detectPackageManager(root) {
  if (fs.existsSync(path.join(root, 'pnpm-lock.yaml'))) return ['pnpm', ['run']];
  if (fs.existsSync(path.join(root, 'bun.lockb'))) return ['bun', ['run']];
  if (fs.existsSync(path.join(root, 'yarn.lock'))) return ['yarn', []];
  return ['npm', ['run']];
}

function packageScripts(root) {
  try {
    const pkg = JSON.parse(fs.readFileSync(path.join(root, 'package.json'), 'utf8'));
    return pkg.scripts && typeof pkg.scripts === 'object' ? pkg.scripts : {};
  } catch {
    return {};
  }
}

function checkQuality(options = {}) {
  const res = result('quality');
  const root = gitRoot();
  let ran = 0;

  function runCheck(label, command, args, options = {}) {
    if (!commandExists(command)) {
      res.notes.push(`Skipped ${label}: ${command} not found.`);
      return;
    }
    ran += 1;
    const child = run(command, args, { cwd: root, timeout: options.timeout || 300000 });
    if (child.status !== 0) {
      addFinding(res, 'error', `${label} failed`);
      res.notes.push((child.stdout || child.stderr || '').trim().slice(0, 1200));
    }
  }

  if (fs.existsSync(path.join(root, 'package.json'))) {
    const scripts = packageScripts(root);
    const [pm, prefix] = detectPackageManager(root);
    for (const script of ['lint', 'typecheck', 'test']) {
      if (!scripts[script]) continue;
      ran += 1;
      const child = run(pm, [...prefix, script], { cwd: root, timeout: 300000 });
      if (child.status !== 0) {
        addFinding(res, 'error', `${pm} ${prefix.concat(script).join(' ')} failed`);
        res.notes.push((child.stdout || child.stderr || '').trim().slice(0, 1200));
      }
    }
  }

  if (fs.existsSync(path.join(root, 'go.mod')) && run('go', ['version']).status === 0) {
    ran += 1;
    const child = run('go', ['test', './...'], { cwd: root, timeout: 300000 });
    if (child.status !== 0) addFinding(res, 'error', 'go test ./... failed');
  }

  const modeFiles = listFiles(options.mode || 'changed').filter(file => !isSkippedFile(file));
  const pythonFiles = modeFiles.filter(file => PYTHON_EXTENSIONS.has(path.extname(file).toLowerCase()));
  const cppFiles = modeFiles.filter(file => CPP_EXTENSIONS.has(path.extname(file).toLowerCase()));
  const hasPythonMarker = fs.existsSync(path.join(root, 'pyproject.toml'))
    || fs.existsSync(path.join(root, 'requirements.txt'))
    || fs.existsSync(path.join(root, 'setup.py'));
  const hasCppMarker = fs.existsSync(path.join(root, 'CMakeLists.txt'))
    || fs.existsSync(path.join(root, 'compile_commands.json'));
  const hasPythonProject = hasPythonMarker || pythonFiles.length > 0;
  const hasCppProject = hasCppMarker || cppFiles.length > 0;

  if (hasPythonProject) {
    runCheck('ruff check', 'ruff', ['check', '.']);
    runCheck('black --check', 'black', ['--check', '.']);
    runCheck('isort --check-only', 'isort', ['--check-only', '.']);
    runCheck('mypy', 'mypy', ['.']);
    runCheck('bandit', 'bandit', pythonFiles.length > 0 ? ['-q', ...pythonFiles] : ['-q', '-r', '.']);
    runCheck('pytest -q', 'pytest', ['-q']);
  }

  if (hasCppProject) {
    if (cppFiles.length > 0) {
      runCheck('clang-format --dry-run --Werror', 'clang-format', ['--dry-run', '--Werror', ...cppFiles]);
      const tidyTargets = cppFiles.filter(file => ['.c', '.cc', '.cpp', '.cxx'].includes(path.extname(file).toLowerCase()));
      if (tidyTargets.length > 0) {
        runCheck('clang-tidy', 'clang-tidy', [...tidyTargets, '--', '-std=c++17']);
      } else {
        res.notes.push('Skipped clang-tidy: no changed or tracked C++ source files.');
      }
      runCheck('cppcheck', 'cppcheck', ['--enable=all', '--suppress=missingIncludeSystem', ...cppFiles]);
    } else {
      res.notes.push('Skipped C++ file checks: no C++ files found.');
    }
    if (fs.existsSync(path.join(root, 'CMakeLists.txt'))) {
      if (fs.existsSync(path.join(root, 'build'))) {
        runCheck('cmake --build build', 'cmake', ['--build', 'build']);
        runCheck('ctest --test-dir build', 'ctest', ['--test-dir', 'build', '--output-on-failure']);
      } else {
        res.notes.push('Skipped CMake build/CTest: build directory not found.');
      }
    }
  }

  if (ran === 0) res.notes.push('No supported project-native quality checks found.');
  else if (res.findings.length === 0) res.notes.push(`Ran ${ran} quality check(s).`);
  return res;
}

function checkMcp() {
  const res = result('mcp');
  const child = run('codex', ['mcp', 'list'], { timeout: 30000 });
  if (child.status !== 0) {
    addFinding(res, 'error', 'codex mcp list failed');
    res.notes.push((child.stderr || child.stdout || '').trim().slice(0, 1200));
  } else {
    res.notes.push('codex mcp list succeeded.');
  }
  return res;
}

function compileRegex(value) {
  try {
    return new RegExp(value, 'g');
  } catch {
    return null;
  }
}

function loadBaseline(root, baselinePath, metricName) {
  if (!baselinePath) return null;
  const file = path.isAbsolute(baselinePath) ? baselinePath : path.join(root, baselinePath);
  if (!fs.existsSync(file)) return null;
  try {
    const raw = fs.readFileSync(file, 'utf8').trim();
    const parsed = JSON.parse(raw);
    if (typeof parsed === 'number') return parsed;
    if (parsed.metrics && parsed.metrics[metricName] !== undefined) return Number(parsed.metrics[metricName]);
    if (parsed[metricName] !== undefined) return Number(parsed[metricName]);
  } catch {
    const raw = fs.readFileSync(file, 'utf8');
    const match = raw.match(new RegExp(`${metricName}\\s*[:=]\\s*([-+]?[0-9]*\\.?[0-9]+)`, 'i'));
    if (match) return Number(match[1]);
  }
  return null;
}

function checkEval(options) {
  const res = result('eval');
  const root = gitRoot();
  if (!options.logFile) {
    addFinding(res, 'error', '--log <file> is required for eval checks');
    return res;
  }
  const logFile = path.isAbsolute(options.logFile) ? options.logFile : path.join(root, options.logFile);
  if (!fs.existsSync(logFile)) {
    addFinding(res, 'error', `eval log not found: ${options.logFile}`);
    return res;
  }
  const config = loadResearchConfig(root);
  const metrics = Array.isArray(config.metrics) ? config.metrics : [];
  if (metrics.length === 0) {
    res.notes.push('No metrics configured; eval log exists but no comparison was possible.');
    return res;
  }
  const output = fs.readFileSync(logFile, 'utf8');
  for (const metric of metrics) {
    if (!metric.name || !metric.extract_regex) continue;
    const triggers = metric.trigger_patterns || [];
    if (triggers.length > 0 && !triggers.some(pattern => new RegExp(pattern).test(output))) continue;
    const re = compileRegex(metric.extract_regex);
    if (!re) continue;
    const values = [];
    let match;
    while ((match = re.exec(output)) !== null) {
      const value = Number(match[1]);
      if (Number.isFinite(value)) values.push(value);
      if (re.lastIndex === match.index) re.lastIndex += 1;
    }
    if (values.length === 0) continue;
    const observed = metric.direction === 'higher_is_better' ? Math.min(...values) : Math.max(...values);
    const baseline = loadBaseline(root, metric.baseline_path, metric.name);
    if (baseline === null || !Number.isFinite(baseline)) {
      addFinding(res, 'warn', `${metric.name}=${observed} has no baseline`);
      continue;
    }
    const tolerance = Number(metric.tolerance_pct || 0) / 100;
    const regressed = metric.direction === 'higher_is_better'
      ? observed < baseline * (1 - tolerance)
      : observed > baseline * (1 + tolerance);
    if (regressed) addFinding(res, 'error', `${metric.name} regressed: observed=${observed}, baseline=${baseline}`);
  }
  if (res.findings.length === 0) res.notes.push('Eval log checked against configured metrics.');
  return res;
}

function parseArgs(argv) {
  const [command, ...rest] = argv;
  const options = { command, mode: 'changed', json: false, logFile: '' };
  for (let i = 0; i < rest.length; i += 1) {
    const arg = rest[i];
    if (arg === '--changed') options.mode = 'changed';
    else if (arg === '--staged') options.mode = 'staged';
    else if (arg === '--all') options.mode = 'all';
    else if (arg === '--json') options.json = true;
    else if (arg === '--log') {
      options.logFile = rest[i + 1] || '';
      i += 1;
    } else if (arg === '--help' || arg === '-h') {
      options.help = true;
    }
  }
  return options;
}

function runCommand(options) {
  switch (options.command) {
    case 'research': return [checkResearch(options)];
    case 'checkpoint': return [checkCheckpoint(options)];
    case 'quality': return [checkQuality(options)];
    case 'console': return [checkConsole(options)];
    case 'secrets': return [checkSecrets(options)];
    case 'mcp': return [checkMcp(options)];
    case 'eval': return [checkEval(options)];
    case 'all':
      return [
        checkResearch({ ...options, mode: 'changed' }),
        checkCheckpoint({ ...options, mode: 'changed' }),
        checkConsole({ ...options, mode: 'changed' }),
        checkSecrets({ ...options, mode: 'staged' }),
        checkQuality({ ...options, mode: 'changed' }),
        checkMcp(options),
      ];
    default:
      return null;
  }
}

function printResults(results, json) {
  const failed = results.some(item => item.status === 'fail');
  if (json) {
    console.log(JSON.stringify({ status: failed ? 'fail' : 'pass', results }, null, 2));
    return;
  }
  console.log(`RCC CODEX CHECK: ${failed ? 'FAIL' : 'PASS'}`);
  for (const item of results) {
    console.log(`\n[${item.status.toUpperCase()}] ${item.name}`);
    for (const note of item.notes.filter(Boolean)) console.log(`  - ${note}`);
    for (const finding of item.findings) {
      const file = finding.file ? `${finding.file}: ` : '';
      console.log(`  - ${finding.severity.toUpperCase()}: ${file}${finding.message}`);
    }
  }
}

function main() {
  const options = parseArgs(process.argv.slice(2));
  if (options.help || !options.command) {
    usage();
    process.exit(options.help ? 0 : 1);
  }
  const results = runCommand(options);
  if (!results) {
    usage();
    process.exit(1);
  }
  printResults(results, options.json);
  process.exit(results.some(item => item.status === 'fail') ? 1 : 0);
}

main();
