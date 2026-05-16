'use strict';

const fs = require('fs');
const path = require('path');

const {
  listMarkdownFiles,
  normalizeRelativePath,
  parseFrontmatter,
} = require('../codex/render-install');

const LANGUAGE_GLOBS = Object.freeze({
  cpp: '**/*.{c,cc,cpp,cxx,h,hpp,hxx}',
  csharp: '**/*.cs',
  dart: '**/*.dart',
  go: '**/*.go',
  golang: '**/*.go',
  java: '**/*.java',
  javascript: '**/*.{js,jsx,mjs,cjs}',
  kotlin: '**/*.{kt,kts}',
  perl: '**/*.{pl,pm,t}',
  php: '**/*.php',
  python: '**/*.py',
  rust: '**/*.rs',
  swift: '**/*.swift',
  typescript: '**/*.{ts,tsx,js,jsx,mts,cts}',
  web: '**/*.{html,css,scss,sass,less,ts,tsx,js,jsx,vue,svelte}',
});

function sanitizeFileStem(value, fallback = 'item') {
  const normalized = String(value || fallback)
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9_-]+/g, '-')
    .replace(/^-+|-+$/g, '');

  return normalized || fallback;
}

function yamlString(value) {
  return JSON.stringify(String(value || '').replace(/\r?\n/g, ' ').trim());
}

function firstHeading(markdown, fallback) {
  const match = /^#\s+(.+)$/m.exec(String(markdown || ''));
  return match ? match[1].trim() : fallback;
}

function listMarkdownFilesRecursive(repoRoot, relativeDir) {
  const root = path.join(repoRoot, relativeDir);
  if (!fs.existsSync(root)) {
    return [];
  }

  const files = [];
  const walk = (dirPath, prefix = '') => {
    const entries = fs.readdirSync(dirPath, { withFileTypes: true }).sort((left, right) => (
      left.name.localeCompare(right.name)
    ));

    for (const entry of entries) {
      const entryPrefix = prefix ? path.join(prefix, entry.name) : entry.name;
      const absolutePath = path.join(dirPath, entry.name);
      if (entry.isDirectory()) {
        walk(absolutePath, entryPrefix);
      } else if (entry.isFile() && entry.name.endsWith('.md')) {
        files.push(normalizeRelativePath(path.join(relativeDir, entryPrefix)));
      }
    }
  };

  walk(root);
  return files.sort();
}

function inferRuleNamespace(sourceRelativePath) {
  const parts = normalizeRelativePath(sourceRelativePath).split('/');
  return parts[0] === 'rules' && parts.length > 2 ? parts[1] : 'common';
}

function inferCursorGlobs(sourceRelativePath) {
  return LANGUAGE_GLOBS[inferRuleNamespace(sourceRelativePath)] || '';
}

function renderMdc({ description, globs = '', alwaysApply = false, body }) {
  return [
    '---',
    `description: ${yamlString(description)}`,
    `globs: ${globs ? yamlString(globs) : ''}`,
    `alwaysApply: ${alwaysApply ? 'true' : 'false'}`,
    '---',
    '',
    String(body || '').trimEnd(),
    '',
  ].join('\n');
}

function renderCursorRuleMdc(sourceRelativePath, markdown) {
  const parsed = parseFrontmatter(markdown);
  const source = normalizeRelativePath(sourceRelativePath);
  const body = parsed.body.trimEnd();
  const title = firstHeading(body, path.basename(source, '.md'));
  const namespace = inferRuleNamespace(source);

  return renderMdc({
    description: parsed.data.description || `ECC ${namespace} rule: ${title}`,
    globs: inferCursorGlobs(source),
    alwaysApply: false,
    body: [
      `# ECC Rule: ${title}`,
      '',
      `Source: \`${source}\``,
      '',
      body,
    ].join('\n'),
  });
}

function renderCursorAgentRule(sourceRelativePath, markdown) {
  const parsed = parseFrontmatter(markdown);
  const source = normalizeRelativePath(sourceRelativePath);
  const fallbackName = path.basename(source, '.md');
  const name = sanitizeFileStem(parsed.data.name, fallbackName);
  const description = parsed.data.description || `${name} agent`;

  return {
    fileName: `ecc-agent-${name}.mdc`,
    content: renderMdc({
      description: `ECC agent ${name}: ${description}`,
      alwaysApply: false,
      body: [
        `# ECC Agent: ${name}`,
        '',
        `Source: \`${source}\``,
        '',
        parsed.body.trimEnd(),
      ].join('\n'),
    }),
  };
}

function renderCursorCommandRule(sourceRelativePath, markdown) {
  const parsed = parseFrontmatter(markdown);
  const source = normalizeRelativePath(sourceRelativePath);
  const commandName = sanitizeFileStem(path.basename(source, '.md'), 'command');
  const body = parsed.body.trimEnd();

  return {
    fileName: `ecc-command-${commandName}.mdc`,
    content: renderMdc({
      description: parsed.data.description || `ECC command /${commandName}`,
      alwaysApply: false,
      body: [
        `# ECC Command: /${commandName}`,
        '',
        `Source: \`${source}\``,
        '',
        `Use this workflow when the user asks for \`/${commandName}\` or an equivalent task.`,
        '',
        body,
      ].join('\n'),
    }),
  };
}

function renderCursorAgentsIndex(repoRoot) {
  const body = fs.existsSync(path.join(repoRoot, 'AGENTS.md'))
    ? fs.readFileSync(path.join(repoRoot, 'AGENTS.md'), 'utf8').trimEnd()
    : '';

  return renderMdc({
    description: 'ECC project agent guidance and Research -> Architect -> Build -> Eval routing.',
    alwaysApply: true,
    body: [
      '# ECC Agent Guidance',
      '',
      'Cursor CLI should use the ECC research-development workflow and route non-trivial work through the installed agent rules when relevant.',
      '',
      body,
    ].filter(Boolean).join('\n'),
  });
}

function renderCursorSkillRule(sourceRelativePath, markdown) {
  const parsed = parseFrontmatter(markdown);
  const source = normalizeRelativePath(sourceRelativePath);
  const skillName = sanitizeFileStem(path.basename(path.dirname(source)), 'skill');
  const description = parsed.data.description || firstHeading(parsed.body, `${skillName} skill`);

  return {
    fileName: `ecc-skill-${skillName}.mdc`,
    content: renderMdc({
      description: `ECC skill ${skillName}: ${description}`,
      alwaysApply: false,
      body: [
        `# ECC Skill: ${skillName}`,
        '',
        `Source: \`${source}\``,
        `Installed copy: \`.cursor/skills/${skillName}/SKILL.md\``,
        '',
        parsed.body.trimEnd(),
      ].join('\n'),
    }),
  };
}

function renderCursorCliHooks(repoRoot) {
  const hooksPath = path.join(repoRoot, '.cursor', 'hooks.json');
  const source = JSON.parse(fs.readFileSync(hooksPath, 'utf8'));
  const hooks = {};

  for (const [eventName, entries] of Object.entries(source.hooks || {})) {
    if (!Array.isArray(entries)) {
      continue;
    }

    const safeEntries = entries.filter(entry => {
      const command = String(entry && entry.command || '');
      return command.startsWith('node .cursor/hooks/') || command.startsWith('npx block-no-verify@');
    });

    if (safeEntries.length > 0) {
      hooks[eventName] = safeEntries;
    }
  }

  return `${JSON.stringify({ hooks }, null, 2)}\n`;
}

module.exports = {
  inferCursorGlobs,
  listMarkdownFiles,
  listMarkdownFilesRecursive,
  renderCursorAgentRule,
  renderCursorAgentsIndex,
  renderCursorCliHooks,
  renderCursorCommandRule,
  renderCursorRuleMdc,
  renderCursorSkillRule,
  sanitizeFileStem,
};
