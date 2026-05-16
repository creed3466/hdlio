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
  renderCursorAgentRule,
  renderCursorAgentsIndex,
  renderCursorCliHooks,
  renderCursorCommandRule,
  renderCursorRuleMdc,
  renderCursorSkillRule,
} = require('../cursor/render-install');

function createRenderOperation({ moduleId, sourceRelativePath, destinationPath, content, strategy }) {
  const operation = createManagedOperation({
    kind: 'render-template',
    moduleId,
    sourceRelativePath,
    destinationPath,
    strategy: strategy || 'render-cursor-cli',
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

function skillNameFromPath(sourceRelativePath) {
  const normalized = normalizeRelativePath(sourceRelativePath);
  return path.basename(normalized);
}

function planRuleOperations(moduleId, repoRoot, cursorRoot) {
  return listMarkdownFilesRecursive(repoRoot, 'rules')
    .filter(file => normalizeRelativePath(file).split('/').length >= 3)
    .filter(file => !normalizeRelativePath(file).startsWith('rules/zh/'))
    .filter(file => pathExists(repoRoot, file))
    .map(file => {
      const destinationName = `ecc-${file.replace(/^rules\//, '').replace(/\//g, '-').replace(/\.md$/, '.mdc')}`;
      return createRenderOperation({
        moduleId,
        sourceRelativePath: file,
        destinationPath: path.join(cursorRoot, 'rules', destinationName),
        content: renderCursorRuleMdc(file, readUtf8(repoRoot, file)),
        strategy: 'render-cursor-cli-rule-mdc',
      });
    });
}

function planCommandOperations(moduleId, repoRoot, cursorRoot) {
  return listMarkdownFiles(repoRoot, 'commands')
    .filter(file => pathExists(repoRoot, file))
    .map(file => {
      const rendered = renderCursorCommandRule(file, readUtf8(repoRoot, file));
      return createRenderOperation({
        moduleId,
        sourceRelativePath: file,
        destinationPath: path.join(cursorRoot, 'rules', rendered.fileName),
        content: rendered.content,
        strategy: 'render-cursor-cli-command-mdc',
      });
    });
}

function planAgentOperations(moduleId, repoRoot, cursorRoot) {
  const operations = [];

  if (pathExists(repoRoot, 'AGENTS.md')) {
    operations.push(createRenderOperation({
      moduleId,
      sourceRelativePath: 'AGENTS.md',
      destinationPath: path.join(cursorRoot, 'rules', 'ecc-agents.mdc'),
      content: renderCursorAgentsIndex(repoRoot),
      strategy: 'render-cursor-cli-agents-index',
    }));
  }

  for (const file of listMarkdownFiles(repoRoot, 'agents')) {
    if (!pathExists(repoRoot, file)) {
      continue;
    }

    const rendered = renderCursorAgentRule(file, readUtf8(repoRoot, file));
    operations.push(createRenderOperation({
      moduleId,
      sourceRelativePath: file,
      destinationPath: path.join(cursorRoot, 'rules', rendered.fileName),
      content: rendered.content,
      strategy: 'render-cursor-cli-agent-mdc',
    }));
  }

  return operations;
}

function planSkillOperations(moduleId, repoRoot, sourceRelativePath, cursorRoot) {
  const normalized = normalizeRelativePath(sourceRelativePath);
  const skillName = skillNameFromPath(normalized);
  const skillMd = path.join(normalized, 'SKILL.md');

  if (!pathExists(repoRoot, skillMd)) {
    return [];
  }

  const rendered = renderCursorSkillRule(skillMd, readUtf8(repoRoot, skillMd));
  return [
    createCopyOperation({
      moduleId,
      sourceRelativePath: normalized,
      destinationPath: path.join(cursorRoot, 'skills', skillName),
      strategy: 'cursor-cli-skill-copy',
    }),
    createRenderOperation({
      moduleId,
      sourceRelativePath: skillMd,
      destinationPath: path.join(cursorRoot, 'rules', rendered.fileName),
      content: rendered.content,
      strategy: 'render-cursor-cli-skill-mdc',
    }),
  ];
}

function planHooksRuntime(moduleId, repoRoot, cursorRoot) {
  const operations = [];

  for (const sourceRelativePath of ['.cursor/hooks', 'scripts/hooks', 'scripts/lib']) {
    if (!pathExists(repoRoot, sourceRelativePath)) {
      continue;
    }
    operations.push(createCopyOperation({
      moduleId,
      sourceRelativePath,
      destinationPath: path.join(cursorRoot, normalizeRelativePath(sourceRelativePath).replace(/^\.cursor\//, '')),
      strategy: 'cursor-cli-hook-runtime-copy',
    }));
  }

  if (pathExists(repoRoot, '.cursor/hooks.json')) {
    operations.push(createRenderOperation({
      moduleId,
      sourceRelativePath: '.cursor/hooks.json',
      destinationPath: path.join(cursorRoot, 'hooks.json'),
      content: renderCursorCliHooks(repoRoot),
      strategy: 'render-cursor-cli-hooks',
    }));
  }

  return operations;
}

function planPlatformConfig(moduleId, repoRoot, sourceRelativePath, cursorRoot) {
  const normalized = normalizeRelativePath(sourceRelativePath);

  if (normalized === '.cursor') {
    return [];
  }

  if (!pathExists(repoRoot, normalized)) {
    return [];
  }

  return [createCopyOperation({
    moduleId,
    sourceRelativePath: normalized,
    destinationPath: path.join(cursorRoot, 'ecc', normalized),
    strategy: 'cursor-cli-platform-reference-copy',
  })];
}

function planGenericPath(moduleId, repoRoot, sourceRelativePath, cursorRoot) {
  const normalized = normalizeRelativePath(sourceRelativePath);

  if (normalized === 'rules') {
    return planRuleOperations(moduleId, repoRoot, cursorRoot);
  }

  if (normalized === 'commands') {
    return planCommandOperations(moduleId, repoRoot, cursorRoot);
  }

  if (normalized === 'agents' || normalized === 'AGENTS.md' || normalized === '.agents') {
    return planAgentOperations(moduleId, repoRoot, cursorRoot);
  }

  if (normalized.startsWith('skills/') || normalized.startsWith('.agents/skills/')) {
    return planSkillOperations(moduleId, repoRoot, normalized, cursorRoot);
  }

  if (!pathExists(repoRoot, normalized)) {
    return [];
  }

  return [createCopyOperation({
    moduleId,
    sourceRelativePath: normalized,
    destinationPath: path.join(cursorRoot, 'ecc', normalized),
    strategy: 'cursor-cli-reference-copy',
  })];
}

function dedupeOperationsByDestination(operations) {
  const lastIndexByDestination = new Map();
  operations.forEach((operation, index) => {
    lastIndexByDestination.set(operation.destinationPath, index);
  });

  return operations.filter((operation, index) => lastIndexByDestination.get(operation.destinationPath) === index);
}

function planCursorCliOperations(input = {}, adapter) {
  const modules = Array.isArray(input.modules) ? input.modules : [];
  const repoRoot = input.repoRoot;
  const cursorRoot = adapter.resolveRoot(input);
  const operations = [];

  for (const module of modules) {
    if (module.id === 'rules-core') {
      operations.push(...planRuleOperations(module.id, repoRoot, cursorRoot));
      continue;
    }

    if (module.id === 'agents-core') {
      operations.push(...planAgentOperations(module.id, repoRoot, cursorRoot));
      continue;
    }

    if (module.id === 'commands-core') {
      operations.push(...planCommandOperations(module.id, repoRoot, cursorRoot));
      continue;
    }

    if (module.id === 'hooks-runtime') {
      operations.push(...planHooksRuntime(module.id, repoRoot, cursorRoot));
      continue;
    }

    const paths = Array.isArray(module.paths) ? module.paths : [];
    for (const sourceRelativePath of paths) {
      if (isForeignPlatformPath(sourceRelativePath, adapter.target)) {
        continue;
      }

      if (module.id === 'platform-configs') {
        operations.push(...planPlatformConfig(module.id, repoRoot, sourceRelativePath, cursorRoot));
      } else {
        operations.push(...planGenericPath(module.id, repoRoot, sourceRelativePath, cursorRoot));
      }
    }
  }

  return dedupeOperationsByDestination(operations);
}

module.exports = createInstallTargetAdapter({
  id: 'cursor-cli-project',
  target: 'cursor-cli',
  kind: 'project',
  rootSegments: ['.cursor'],
  installStatePathSegments: ['ecc-cli-install-state.json'],
  nativeRootRelativePath: '.cursor',
  planOperations: planCursorCliOperations,
});
