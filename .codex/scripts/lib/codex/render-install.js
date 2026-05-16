'use strict';

const fs = require('fs');
const path = require('path');
const TOML = require('@iarna/toml');

const DEFAULT_CODEX_MODEL = 'gpt-5.5';

const HIGH_REASONING_NAME_PARTS = [
  'architect',
  'ben',
  'evaluator',
  'harness',
  'optimizer',
  'planner',
  'reviewer',
  'security',
];

const SAFE_CODEX_HOOK_COMMANDS = Object.freeze({
  'pre:bash:block-no-verify': null,
  'pre:bash:tmux-reminder': 'scripts/hooks/pre-bash-tmux-reminder.js',
  'pre:bash:git-push-reminder': 'scripts/hooks/pre-bash-git-push-reminder.js',
  'pre:bash:commit-quality': 'scripts/hooks/pre-bash-commit-quality.js',
  'pre:write:doc-file-warning': 'scripts/hooks/doc-file-warning.js',
  'pre:config-protection': 'scripts/hooks/config-protection.js',
  'post:edit:design-quality-check': 'scripts/hooks/design-quality-check.js',
  'post:edit:console-warn': 'scripts/hooks/post-edit-console-warn.js',
});

function normalizeRelativePath(value) {
  return String(value || '').replace(/\\/g, '/').replace(/^\.\/+/, '').replace(/\/+$/, '');
}

function readUtf8(repoRoot, relativePath) {
  return fs.readFileSync(path.join(repoRoot, relativePath), 'utf8');
}

function listRelativeFiles(dirPath, prefix = '') {
  if (!fs.existsSync(dirPath)) {
    return [];
  }

  const entries = fs.readdirSync(dirPath, { withFileTypes: true }).sort((left, right) => (
    left.name.localeCompare(right.name)
  ));
  const files = [];

  for (const entry of entries) {
    const entryPrefix = prefix ? path.join(prefix, entry.name) : entry.name;
    const absolutePath = path.join(dirPath, entry.name);

    if (entry.isDirectory()) {
      files.push(...listRelativeFiles(absolutePath, entryPrefix));
    } else if (entry.isFile()) {
      files.push(normalizeRelativePath(entryPrefix));
    }
  }

  return files;
}

function listMarkdownFiles(repoRoot, relativeDir) {
  const dirPath = path.join(repoRoot, relativeDir);
  if (!fs.existsSync(dirPath)) {
    return [];
  }

  return fs.readdirSync(dirPath, { withFileTypes: true })
    .filter(entry => entry.isFile() && entry.name.endsWith('.md'))
    .map(entry => normalizeRelativePath(path.join(relativeDir, entry.name)))
    .sort();
}

function parseScalar(value) {
  const trimmed = String(value || '').trim();
  if (!trimmed) {
    return '';
  }

  if (
    (trimmed.startsWith('"') && trimmed.endsWith('"'))
    || (trimmed.startsWith("'") && trimmed.endsWith("'"))
  ) {
    return trimmed.slice(1, -1);
  }

  if (trimmed.startsWith('[') && trimmed.endsWith(']')) {
    try {
      return JSON.parse(trimmed);
    } catch {
      return trimmed;
    }
  }

  if (trimmed === 'true') {
    return true;
  }
  if (trimmed === 'false') {
    return false;
  }

  return trimmed;
}

function parseFrontmatter(markdown) {
  const source = String(markdown || '').replace(/^\uFEFF/, '');
  const match = /^---\r?\n([\s\S]*?)\r?\n---\r?\n?/.exec(source);

  if (!match) {
    return {
      data: {},
      body: source,
    };
  }

  const data = {};
  for (const line of match[1].split(/\r?\n/)) {
    const keyMatch = /^([A-Za-z0-9_-]+):\s*(.*)$/.exec(line);
    if (!keyMatch) {
      continue;
    }
    data[keyMatch[1]] = parseScalar(keyMatch[2]);
  }

  return {
    data,
    body: source.slice(match[0].length),
  };
}

function sanitizeCodexName(value, fallback) {
  const normalized = String(value || fallback || '')
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9_-]+/g, '-')
    .replace(/^-+|-+$/g, '');

  return normalized || 'agent';
}

function determineReasoningEffort(agentName, description, claudeModel) {
  const model = String(claudeModel || '').toLowerCase();
  if (model === 'opus') {
    return 'high';
  }

  const haystack = `${agentName} ${description}`.toLowerCase();
  return HIGH_REASONING_NAME_PARTS.some(part => haystack.includes(part)) ? 'high' : 'medium';
}

function determineSandboxMode(tools) {
  const normalizedTools = Array.isArray(tools)
    ? tools.map(tool => String(tool).toLowerCase())
    : [];

  return normalizedTools.some(tool => ['edit', 'write', 'multiedit'].includes(tool))
    ? 'workspace-write'
    : 'read-only';
}

function renderCombinedAgentsMd(repoRoot) {
  const rootAgents = readUtf8(repoRoot, 'AGENTS.md').trimEnd();
  const codexSupplement = readUtf8(repoRoot, '.codex/AGENTS.md').trimEnd();

  return [
    '<!-- BEGIN RCC -->',
    rootAgents,
    '',
    '---',
    '',
    '# Codex Supplement (From RCC .codex/AGENTS.md)',
    '',
    codexSupplement,
    '<!-- END RCC -->',
    '',
  ].join('\n');
}

function stripTomlArray(lines, startIndex) {
  let index = startIndex;
  let depth = (lines[index].match(/\[/g) || []).length - (lines[index].match(/\]/g) || []).length;

  while (depth > 0 && index + 1 < lines.length) {
    index += 1;
    depth += (lines[index].match(/\[/g) || []).length - (lines[index].match(/\]/g) || []).length;
  }

  return index;
}

function renderCodexProjectConfigToml(repoRoot) {
  const source = readUtf8(repoRoot, '.codex/config.toml')
    .replace('Everything Claude Code (ECC) \u2014 Codex Reference Configuration', 'Research Claude Code (RCC) - Codex Project Configuration')
    .replace('Research Claude Code (RCC) - Codex Reference Configuration', 'Research Claude Code (RCC) - Codex Project Configuration')
    .replace('Copy this file to ~/.codex/config.toml for global defaults, or keep it in\n# the project root as .codex/config.toml for project-local settings.',
      'Installed by RCC as project-local .codex/config.toml.\n# Keep user-specific Codex preferences in ~/.codex/config.toml.')
    .replace('Codex discovers it from ~/.codex/hooks.json.', 'Codex discovers it from project .codex/hooks.json.')
    .replace('ECC installs a Codex-native hooks.json', 'RCC installs a Codex-native hooks.json')
    .replace('Codex ECC', 'Codex RCC')
    .replace(/\n# External notifications receive a JSON payload on stdin\.\n/g, '\n')
    .replace(/\n# Profiles .*\n/g, '\n');
  const lines = source.split(/\r?\n/);
  const out = [];

  for (let index = 0; index < lines.length; index += 1) {
    const line = lines[index];

    if (/^notify\s*=\s*\[/.test(line)) {
      index = stripTomlArray(lines, index);
      continue;
    }

    if (/^\[profiles(?:\.|\])/.test(line)) {
      while (index + 1 < lines.length && !/^\[/.test(lines[index + 1])) {
        index += 1;
      }
      continue;
    }

    out.push(line);
  }

  return `${out.join('\n').replace(/\n{3,}/g, '\n\n').trimEnd()}\n`;
}

function renderCommandPrompt(sourceRelativePath, markdown) {
  const commandName = path.basename(sourceRelativePath, '.md');
  const parsed = parseFrontmatter(markdown);
  const body = parsed.body.trimEnd();

  return [
    `# RCC Command Prompt: /${commandName}`,
    '',
    `Source: ${normalizeRelativePath(sourceRelativePath)}`,
    '',
    `Use this prompt to run the RCC \`${commandName}\` workflow.`,
    '',
    body,
    '',
  ].join('\n');
}

function renderCodexAgentToml(sourceRelativePath, markdown) {
  const parsed = parseFrontmatter(markdown);
  const fallbackName = path.basename(sourceRelativePath, '.md');
  const name = sanitizeCodexName(parsed.data.name, fallbackName);
  const description = String(parsed.data.description || `${name} agent`).trim();
  const developerInstructions = parsed.body.trim();
  const config = {
    name,
    description,
    model: DEFAULT_CODEX_MODEL,
    model_reasoning_effort: determineReasoningEffort(name, description, parsed.data.model),
    sandbox_mode: determineSandboxMode(parsed.data.tools),
    developer_instructions: developerInstructions ? `${developerInstructions}\n` : '',
  };

  return {
    name,
    fileName: `${name}.toml`,
    content: `${TOML.stringify(config).trimEnd()}\n`,
  };
}

function shellQuote(value) {
  return `"${String(value).replace(/(["\\$`])/g, '\\$1')}"`;
}

function translateCodexMatcher(matcher) {
  const normalized = String(matcher || '*');
  if (normalized === '*') {
    return normalized;
  }

  return normalized
    .split('|')
    .map(part => part.trim())
    .filter(Boolean)
    .map(part => (['Edit', 'Write', 'MultiEdit'].includes(part) ? 'apply_patch' : part))
    .filter((part, index, parts) => parts.indexOf(part) === index)
    .join('|') || '*';
}

function renderCodexHookEntry(entry, codexRoot) {
  const scriptRelativePath = SAFE_CODEX_HOOK_COMMANDS[entry.id];
  const sourceHook = Array.isArray(entry.hooks) ? entry.hooks[0] : null;
  const hook = {
    type: 'command',
    command: scriptRelativePath === null
      ? sourceHook && sourceHook.command
      : `node ${shellQuote(path.join(codexRoot, scriptRelativePath))}`,
  };

  if (sourceHook && Number.isInteger(sourceHook.timeout)) {
    hook.timeout = sourceHook.timeout;
  }
  if (sourceHook && sourceHook.async === true) {
    hook.async = true;
  }
  if (entry.description) {
    hook.statusMessage = entry.description;
  }

  return {
    matcher: translateCodexMatcher(entry.matcher),
    hooks: [hook],
    description: entry.description,
    id: entry.id,
  };
}

function renderCodexHooks(repoRoot, codexRoot) {
  const source = JSON.parse(readUtf8(repoRoot, 'hooks/hooks.json'));
  const hooks = {};

  for (const eventName of ['PreToolUse', 'PostToolUse']) {
    const entries = Array.isArray(source.hooks && source.hooks[eventName])
      ? source.hooks[eventName]
      : [];
    const safeEntries = entries
      .filter(entry => entry && Object.prototype.hasOwnProperty.call(SAFE_CODEX_HOOK_COMMANDS, entry.id))
      .map(entry => renderCodexHookEntry(entry, codexRoot));

    if (safeEntries.length > 0) {
      hooks[eventName] = safeEntries;
    }
  }

  return `${JSON.stringify({ hooks }, null, 2)}\n`;
}

function renderRulesPackPrompt(repoRoot, namespace) {
  const rulesDir = path.join(repoRoot, 'rules', namespace);
  const files = listRelativeFiles(rulesDir);
  const title = namespace === 'common'
    ? 'RCC Rule Pack: common'
    : `RCC Rule Pack: ${namespace}`;
  const intro = namespace === 'common'
    ? 'Apply RCC common engineering rules for this session.'
    : `Apply RCC common rules plus ${namespace}-specific rules for this session.`;
  const lines = files.map(file => `- \`ecc/rules/${namespace}/${file}\``);

  return [
    `# ${title}`,
    '',
    intro,
    '',
    'Installed rule sources:',
    '',
    ...lines,
    '',
    'Treat these as source-of-truth guidance for planning, implementation, review, and verification. Resolve installed paths under the target project `.codex` directory.',
    '',
  ].join('\n');
}

function renderRunTestsPrompt() {
  return [
    '# RCC Tool Prompt: run-tests',
    '',
    'Run the repository test suite with package-manager autodetection and concise reporting.',
    '',
    '1. Detect the package manager from lock files.',
    '2. Detect available scripts or test commands for this repo.',
    '3. Execute tests with the best project-native command.',
    '4. If tests fail, report failing files/tests first, then the smallest likely fix list.',
    '5. Do not change code unless explicitly asked.',
    '',
  ].join('\n');
}

function renderCoveragePrompt() {
  return [
    '# RCC Tool Prompt: check-coverage',
    '',
    'Analyze coverage and compare it to an 80% threshold unless a different threshold is provided.',
    '',
    'Find existing coverage artifacts first. If missing, run the project coverage command with the detected package manager. Report total coverage, top under-covered files, and recommended focus areas.',
    '',
  ].join('\n');
}

function renderSecurityAuditPrompt() {
  return [
    '# RCC Tool Prompt: security-audit',
    '',
    'Run a practical security audit: dependency vulnerabilities, secret scan, and high-risk code patterns.',
    '',
    'Prioritize findings by CRITICAL, HIGH, MEDIUM, LOW. Do not auto-fix unless explicitly asked.',
    '',
  ].join('\n');
}

function renderCodexCheckPrompt() {
  return [
    '# RCC Codex Gate Prompt: codex-check',
    '',
    'Run the RCC Codex stage-gate checker for the current project.',
    '',
    'Default command:',
    '',
    '```bash',
    'node .codex/scripts/rcc-codex-check.js all',
    '```',
    '',
    'Report:',
    '- checks run',
    '- pass/fail',
    '- blocking findings',
    '- skipped checks and why',
    '',
  ].join('\n');
}

function renderCodexResearchGatePrompt() {
  return [
    '# RCC Codex Gate Prompt: research-gate',
    '',
    'Before promoting Research to Architect, run:',
    '',
    '```bash',
    'node .codex/scripts/rcc-codex-check.js research --changed',
    '```',
    '',
    'Then invoke or simulate the `codex-reviewer` stage. Do not promote unsupported claims. External claims need `[verified: <source>]` or `[unverified]`.',
    '',
  ].join('\n');
}

function renderCodexQualityGatePrompt() {
  return [
    '# RCC Codex Gate Prompt: quality-gate',
    '',
    'Before finalizing code changes, run:',
    '',
    '```bash',
    'node .codex/scripts/rcc-codex-check.js quality --changed',
    'node .codex/scripts/rcc-codex-check.js console --changed',
    'node .codex/scripts/rcc-codex-check.js secrets --staged',
    '```',
    '',
    'Report pass/fail, files checked, and unresolved risk.',
    '',
  ].join('\n');
}

function renderCodexEvalGatePrompt() {
  return [
    '# RCC Codex Gate Prompt: eval-gate',
    '',
    'Before claiming an evaluation win, compare against the baseline or state that no baseline exists.',
    '',
    'If an eval log exists, run:',
    '',
    '```bash',
    'node .codex/scripts/rcc-codex-check.js eval --log <file>',
    '```',
    '',
    'Report metric, baseline, tolerance, result, and residual uncertainty.',
    '',
  ].join('\n');
}

function renderCodexExtensionPromptsManifest(fileNames) {
  return `${[...new Set(fileNames)].sort().join('\n')}\n`;
}

module.exports = {
  listMarkdownFiles,
  normalizeRelativePath,
  parseFrontmatter,
  renderCodexAgentToml,
  renderCodexCheckPrompt,
  renderCodexEvalGatePrompt,
  renderCodexHooks,
  renderCodexProjectConfigToml,
  renderCodexQualityGatePrompt,
  renderCodexResearchGatePrompt,
  renderCombinedAgentsMd,
  renderCommandPrompt,
  renderCoveragePrompt,
  renderCodexExtensionPromptsManifest,
  renderRulesPackPrompt,
  renderRunTestsPrompt,
  renderSecurityAuditPrompt,
};
