'use strict';

const fs = require('fs');
const path = require('path');

const {
  createInstallTargetAdapter,
  createManagedOperation,
  isForeignPlatformPath,
  normalizeRelativePath,
} = require('./helpers');
const {
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
} = require('../copilot/render-install');

function createRenderOperation({ moduleId, sourceRelativePath, destinationPath, content, strategy }) {
  const operation = createManagedOperation({
    kind: 'render-template',
    moduleId,
    sourceRelativePath,
    destinationPath,
    strategy: strategy || 'render-copilot',
    ownership: 'managed',
    scaffoldOnly: false,
  });

  Object.defineProperty(operation, 'renderedContent', {
    value: content,
    enumerable: false,
    configurable: true,
  });

  return operation;
}

function createCopyOperation({ moduleId, sourceRelativePath, destinationPath, strategy }) {
  return createManagedOperation({
    kind: 'copy-path',
    moduleId,
    sourceRelativePath,
    destinationPath,
    strategy: strategy || 'preserve-relative-path',
  });
}

function readUtf8(repoRoot, sourceRelativePath) {
  return fs.readFileSync(path.join(repoRoot, sourceRelativePath), 'utf8');
}

function pathExists(repoRoot, sourceRelativePath) {
  return fs.existsSync(path.join(repoRoot, sourceRelativePath));
}

function planRuleInstructions(moduleId, repoRoot, githubRoot) {
  return listMarkdownFilesRecursive(repoRoot, 'rules')
    .filter(file => normalizeRelativePath(file).split('/').length >= 3)
    .filter(file => !normalizeRelativePath(file).startsWith('rules/zh/'))
    .filter(file => pathExists(repoRoot, file))
    .map(file => {
      const destinationName = `ecc-${file.replace(/^rules\//, '').replace(/\//g, '-').replace(/\.md$/, '.instructions.md')}`;
      return createRenderOperation({
        moduleId,
        sourceRelativePath: file,
        destinationPath: path.join(githubRoot, 'instructions', destinationName),
        content: renderCopilotRuleInstruction(file, readUtf8(repoRoot, file)),
        strategy: 'render-copilot-rule-instruction',
      });
    });
}

function planAgentAssets(moduleId, repoRoot, githubRoot) {
  const operations = [];
  const agentSummaries = [];

  if (pathExists(repoRoot, 'AGENTS.md')) {
    operations.push(createRenderOperation({
      moduleId,
      sourceRelativePath: 'AGENTS.md',
      destinationPath: path.join(githubRoot, 'copilot-instructions.md'),
      content: renderCopilotRepositoryInstructions(repoRoot),
      strategy: 'render-copilot-repository-instructions',
    }));
  }

  for (const file of listMarkdownFiles(repoRoot, 'agents')) {
    if (!pathExists(repoRoot, file)) {
      continue;
    }

    const markdown = readUtf8(repoRoot, file);
    const summary = summarizeMarkdown(file, markdown);
    const name = sanitizeFileStem(summary.name, path.basename(file, '.md'));
    agentSummaries.push({ ...summary, name });
    operations.push(createRenderOperation({
      moduleId,
      sourceRelativePath: file,
      destinationPath: path.join(githubRoot, 'prompts', `ecc-agent-${name}.prompt.md`),
      content: renderCopilotPrompt(file, markdown, 'Agent Prompt'),
      strategy: 'render-copilot-agent-prompt',
    }));
  }

  if (agentSummaries.length > 0) {
    operations.push(createRenderOperation({
      moduleId,
      sourceRelativePath: 'agents',
      destinationPath: path.join(githubRoot, 'instructions', 'ecc-agents.instructions.md'),
      content: renderCopilotAgentIndex(agentSummaries),
      strategy: 'render-copilot-agent-index',
    }));
  }

  return operations;
}

function planCommandAssets(moduleId, repoRoot, githubRoot) {
  const operations = [];
  const commandSummaries = [];

  for (const file of listMarkdownFiles(repoRoot, 'commands')) {
    if (!pathExists(repoRoot, file)) {
      continue;
    }

    const markdown = readUtf8(repoRoot, file);
    const name = sanitizeFileStem(path.basename(file, '.md'), 'command');
    const summary = summarizeMarkdown(file, markdown);
    commandSummaries.push({ ...summary, name });
    operations.push(createRenderOperation({
      moduleId,
      sourceRelativePath: file,
      destinationPath: path.join(githubRoot, 'prompts', `ecc-${name}.prompt.md`),
      content: renderCopilotPrompt(file, markdown, 'Command Prompt'),
      strategy: 'render-copilot-command-prompt',
    }));
  }

  if (commandSummaries.length > 0) {
    operations.push(createRenderOperation({
      moduleId,
      sourceRelativePath: 'commands',
      destinationPath: path.join(githubRoot, 'instructions', 'ecc-commands.instructions.md'),
      content: renderCopilotCommandIndex(commandSummaries),
      strategy: 'render-copilot-command-index',
    }));
  }

  return operations;
}

function planSkillAssets(moduleId, repoRoot, sourceRelativePath, githubRoot) {
  const normalized = normalizeRelativePath(sourceRelativePath);
  const skillMd = path.join(normalized, 'SKILL.md');

  if (!pathExists(repoRoot, skillMd)) {
    return {
      operations: [],
      summary: null,
    };
  }

  const markdown = readUtf8(repoRoot, skillMd);
  const summary = summarizeSkill(skillMd, markdown);
  const skillName = sanitizeFileStem(summary.name, path.basename(normalized));

  return {
    summary: { ...summary, name: skillName },
    operations: [
      createCopyOperation({
        moduleId,
        sourceRelativePath: normalized,
        destinationPath: path.join(githubRoot, 'ecc', 'skills', skillName),
        strategy: 'copilot-skill-reference-copy',
      }),
      createRenderOperation({
        moduleId,
        sourceRelativePath: skillMd,
        destinationPath: path.join(githubRoot, 'prompts', `ecc-skill-${skillName}.prompt.md`),
        content: renderCopilotPrompt(skillMd, markdown, 'Skill Prompt'),
        strategy: 'render-copilot-skill-prompt',
      }),
    ],
  };
}

function planReferenceCopy(moduleId, repoRoot, sourceRelativePath, githubRoot) {
  const normalized = normalizeRelativePath(sourceRelativePath);
  if (!pathExists(repoRoot, normalized)) {
    return [];
  }

  return [createCopyOperation({
    moduleId,
    sourceRelativePath: normalized,
    destinationPath: path.join(githubRoot, 'ecc', normalized),
    strategy: 'copilot-reference-copy',
  })];
}

function dedupeOperationsByDestination(operations) {
  const lastIndexByDestination = new Map();
  operations.forEach((operation, index) => {
    lastIndexByDestination.set(operation.destinationPath, index);
  });

  return operations.filter((operation, index) => lastIndexByDestination.get(operation.destinationPath) === index);
}

function planCopilotOperations(input = {}, adapter) {
  const modules = Array.isArray(input.modules) ? input.modules : [];
  const repoRoot = input.repoRoot;
  const githubRoot = adapter.resolveRoot(input);
  const operations = [];
  const skillSummaries = [];
  let firstSkillModuleId = null;

  for (const module of modules) {
    if (module.id === 'rules-core') {
      operations.push(...planRuleInstructions(module.id, repoRoot, githubRoot));
      continue;
    }

    if (module.id === 'agents-core') {
      operations.push(...planAgentAssets(module.id, repoRoot, githubRoot));
      continue;
    }

    if (module.id === 'commands-core') {
      operations.push(...planCommandAssets(module.id, repoRoot, githubRoot));
      continue;
    }

    if (module.id === 'hooks-runtime') {
      continue;
    }

    const paths = Array.isArray(module.paths) ? module.paths : [];
    for (const sourceRelativePath of paths) {
      const normalized = normalizeRelativePath(sourceRelativePath);
      if (isForeignPlatformPath(normalized, adapter.target)) {
        continue;
      }

      if (normalized === 'rules') {
        operations.push(...planRuleInstructions(module.id, repoRoot, githubRoot));
      } else if (normalized === 'commands') {
        operations.push(...planCommandAssets(module.id, repoRoot, githubRoot));
      } else if (normalized === 'agents' || normalized === 'AGENTS.md' || normalized === '.agents') {
        operations.push(...planAgentAssets(module.id, repoRoot, githubRoot));
      } else if (normalized.startsWith('skills/') || normalized.startsWith('.agents/skills/')) {
        const planned = planSkillAssets(module.id, repoRoot, normalized, githubRoot);
        operations.push(...planned.operations);
        if (planned.summary) {
          firstSkillModuleId = firstSkillModuleId || module.id;
          skillSummaries.push(planned.summary);
        }
      } else if (module.id === 'platform-configs') {
        if (!['.claude-plugin', '.codex', '.cursor', '.gemini', '.opencode'].includes(normalized)) {
          operations.push(...planReferenceCopy(module.id, repoRoot, normalized, githubRoot));
        }
      } else {
        operations.push(...planReferenceCopy(module.id, repoRoot, normalized, githubRoot));
      }
    }
  }

  if (skillSummaries.length > 0) {
    operations.push(createRenderOperation({
      moduleId: firstSkillModuleId,
      sourceRelativePath: 'skills',
      destinationPath: path.join(githubRoot, 'instructions', 'ecc-skills.instructions.md'),
      content: renderCopilotSkillIndex(skillSummaries),
      strategy: 'render-copilot-skill-index',
    }));
  }

  return dedupeOperationsByDestination(operations);
}

module.exports = createInstallTargetAdapter({
  id: 'copilot-project',
  target: 'copilot',
  kind: 'project',
  rootSegments: ['.github'],
  installStatePathSegments: ['ecc-install-state.json'],
  nativeRootRelativePath: '.github',
  planOperations: planCopilotOperations,
});
