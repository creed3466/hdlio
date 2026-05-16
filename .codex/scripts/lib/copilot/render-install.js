'use strict';

const fs = require('fs');
const path = require('path');

const {
  listMarkdownFiles,
  normalizeRelativePath,
  parseFrontmatter,
} = require('../codex/render-install');

const COPILOT_LANGUAGE_GLOBS = Object.freeze({
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

function inferCopilotApplyTo(sourceRelativePath) {
  return COPILOT_LANGUAGE_GLOBS[inferRuleNamespace(sourceRelativePath)] || '**';
}

function renderInstruction({ applyTo = '**', body }) {
  return [
    '---',
    `applyTo: ${yamlString(applyTo)}`,
    '---',
    '',
    String(body || '').trimEnd(),
    '',
  ].join('\n');
}

function renderCopilotRepositoryInstructions(repoRoot) {
  const agentsPath = path.join(repoRoot, 'AGENTS.md');
  const body = fs.existsSync(agentsPath) ? fs.readFileSync(agentsPath, 'utf8').trimEnd() : '';

  return [
    '# ECC Repository Instructions',
    '',
    'Source: `AGENTS.md`',
    '',
    body,
    '',
  ].join('\n');
}

function renderCopilotRuleInstruction(sourceRelativePath, markdown) {
  const parsed = parseFrontmatter(markdown);
  const source = normalizeRelativePath(sourceRelativePath);
  const body = parsed.body.trimEnd();
  const title = firstHeading(body, path.basename(source, '.md'));

  return renderInstruction({
    applyTo: inferCopilotApplyTo(source),
    body: [
      `# ECC Rule: ${title}`,
      '',
      `Source: \`${source}\``,
      '',
      body,
    ].join('\n'),
  });
}

function renderCopilotPrompt(sourceRelativePath, markdown, label) {
  const parsed = parseFrontmatter(markdown);
  const source = normalizeRelativePath(sourceRelativePath);
  const name = path.basename(source, '.md');
  const body = parsed.body.trimEnd();

  return [
    `# ECC ${label}: ${name}`,
    '',
    `Source: \`${source}\``,
    '',
    body,
    '',
  ].join('\n');
}

function renderCopilotAgentIndex(agentSummaries) {
  const lines = agentSummaries.map(agent => (
    `- \`${agent.name}\`: ${agent.description} (prompt: \`.github/prompts/ecc-agent-${agent.name}.prompt.md\`)`
  ));

  return renderInstruction({
    applyTo: '**',
    body: [
      '# ECC Agents',
      '',
      'Use these ECC agent prompt files when the task benefits from specialized review, planning, research, or validation.',
      '',
      ...lines,
    ].join('\n'),
  });
}

function renderCopilotCommandIndex(commandSummaries) {
  const lines = commandSummaries.map(command => (
    `- \`/${command.name}\`: ${command.description} (prompt: \`.github/prompts/ecc-${command.name}.prompt.md\`)`
  ));

  return renderInstruction({
    applyTo: '**',
    body: [
      '# ECC Commands',
      '',
      'Reusable ECC workflows are installed as Copilot prompt files. Use the matching prompt when the user invokes a slash-command name or asks for the equivalent workflow.',
      '',
      ...lines,
    ].join('\n'),
  });
}

function renderCopilotSkillIndex(skillSummaries) {
  const lines = skillSummaries.map(skill => (
    `- \`${skill.name}\`: ${skill.description} (source: \`.github/ecc/skills/${skill.name}/SKILL.md\`, prompt: \`.github/prompts/ecc-skill-${skill.name}.prompt.md\`)`
  ));

  return renderInstruction({
    applyTo: '**',
    body: [
      '# ECC Skills',
      '',
      'Selected ECC skills are installed under `.github/ecc/skills`. Use a skill when the user request matches its domain or when the corresponding prompt file is invoked.',
      '',
      ...lines,
    ].join('\n'),
  });
}

function summarizeMarkdown(sourceRelativePath, markdown) {
  const parsed = parseFrontmatter(markdown);
  const source = normalizeRelativePath(sourceRelativePath);
  const name = sanitizeFileStem(parsed.data.name || path.basename(source, '.md'));
  const description = parsed.data.description || firstHeading(parsed.body, `${name} workflow`);

  return {
    name,
    description: String(description).replace(/\s+/g, ' ').trim(),
  };
}

function summarizeSkill(sourceRelativePath, markdown) {
  const parsed = parseFrontmatter(markdown);
  const source = normalizeRelativePath(sourceRelativePath);
  const name = sanitizeFileStem(path.basename(path.dirname(source)), 'skill');
  const description = parsed.data.description || firstHeading(parsed.body, `${name} skill`);

  return {
    name,
    description: String(description).replace(/\s+/g, ' ').trim(),
  };
}

module.exports = {
  inferCopilotApplyTo,
  listMarkdownFiles,
  listMarkdownFilesRecursive,
  renderCopilotAgentIndex,
  renderCopilotCommandIndex,
  renderCopilotPrompt,
  renderCopilotRepositoryInstructions,
  renderCopilotRuleInstruction,
  renderCopilotSkillIndex,
  sanitizeFileStem,
  summarizeMarkdown,
  summarizeSkill,
};
