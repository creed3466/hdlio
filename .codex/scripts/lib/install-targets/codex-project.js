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
  renderCodexAgentToml,
  renderCodexCheckPrompt,
  renderCodexEvalGatePrompt,
  renderCodexExtensionPromptsManifest,
  renderCodexHooks,
  renderCodexProjectConfigToml,
  renderCodexQualityGatePrompt,
  renderCodexResearchGatePrompt,
  renderCombinedAgentsMd,
  renderCommandPrompt,
  renderCoveragePrompt,
  renderRulesPackPrompt,
  renderRunTestsPrompt,
  renderSecurityAuditPrompt,
} = require('../codex/render-install');

function createRenderOperation({ moduleId, sourceRelativePath, destinationPath, content, strategy }) {
  const operation = createManagedOperation({
    kind: 'render-template',
    moduleId,
    sourceRelativePath,
    destinationPath,
    strategy: strategy || 'render-codex',
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

function createManagedBlockOperation({ moduleId, sourceRelativePath, destinationPath, content, strategy }) {
  return createManagedOperation({
    kind: 'merge-managed-block',
    moduleId,
    sourceRelativePath,
    destinationPath,
    strategy: strategy || 'merge-codex-project-agents-md',
    ownership: 'managed',
    scaffoldOnly: false,
    managedContent: content,
    beginMarker: '<!-- BEGIN RCC -->',
    endMarker: '<!-- END RCC -->',
  });
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

function getProjectRoot(input = {}) {
  const projectRoot = input.projectRoot || input.repoRoot;
  if (!projectRoot) {
    throw new Error('projectRoot or repoRoot is required for Codex project install');
  }
  return projectRoot;
}

function getAgentsHome(input = {}) {
  return path.join(getProjectRoot(input), '.agents');
}

function createSkillCopyOperation(moduleId, sourceRelativePath, input) {
  const normalized = normalizeRelativePath(sourceRelativePath);
  const skillName = path.basename(normalized);
  return createCopyOperation({
    moduleId,
    sourceRelativePath: normalized,
    destinationPath: path.join(getAgentsHome(input), 'skills', skillName),
    strategy: 'codex-project-skill',
  });
}

function planCommandPrompts(moduleId, repoRoot, sourceRelativePath, codexRoot) {
  const normalized = normalizeRelativePath(sourceRelativePath);
  const commandFiles = normalized === 'commands'
    ? listMarkdownFiles(repoRoot, 'commands')
    : [normalized].filter(file => file.startsWith('commands/') && file.endsWith('.md'));

  return commandFiles
    .filter(file => pathExists(repoRoot, file))
    .map(file => {
      const commandName = path.basename(file, '.md');
      return createRenderOperation({
        moduleId,
        sourceRelativePath: file,
        // Keep ecc-* filenames stable for existing Codex prompt references.
        destinationPath: path.join(codexRoot, 'prompts', `ecc-${commandName}.md`),
        content: renderCommandPrompt(file, readUtf8(repoRoot, file)),
        strategy: 'render-codex-command-prompt',
      });
    });
}

function planAgentTomls(moduleId, repoRoot, sourceRelativePath, codexRoot) {
  const normalized = normalizeRelativePath(sourceRelativePath);
  const agentFiles = normalized === 'agents'
    ? listMarkdownFiles(repoRoot, 'agents')
    : [normalized].filter(file => file.startsWith('agents/') && file.endsWith('.md'));

  return agentFiles
    .filter(file => pathExists(repoRoot, file))
    .map(file => {
      const rendered = renderCodexAgentToml(file, readUtf8(repoRoot, file));
      return createRenderOperation({
        moduleId,
        sourceRelativePath: file,
        destinationPath: path.join(codexRoot, 'agents', rendered.fileName),
        content: rendered.content,
        strategy: 'render-codex-agent-toml',
      });
    });
}

function planRuleOperations(moduleId, repoRoot, codexRoot) {
  const operations = [
    createCopyOperation({
      moduleId,
      sourceRelativePath: 'rules',
      destinationPath: path.join(codexRoot, 'ecc', 'rules'),
      strategy: 'codex-rules-reference-copy',
    }),
  ];

  const rulesRoot = path.join(repoRoot, 'rules');
  const namespaces = fs.existsSync(rulesRoot)
    ? fs.readdirSync(rulesRoot, { withFileTypes: true })
      .filter(entry => entry.isDirectory())
      .filter(entry => !entry.name.startsWith('.') && entry.name !== 'node_modules')
      .map(entry => entry.name)
      .sort()
    : [];

  for (const namespace of namespaces) {
    operations.push(createRenderOperation({
      moduleId,
      sourceRelativePath: `rules/${namespace}`,
      // Keep ecc-* filenames stable for existing Codex prompt references.
      destinationPath: path.join(codexRoot, 'prompts', `ecc-rules-pack-${namespace}.md`),
      content: renderRulesPackPrompt(repoRoot, namespace),
      strategy: 'render-codex-rules-prompt',
    }));
  }

  return operations;
}

function planAgentsCore(module, input, adapter) {
  const repoRoot = input.repoRoot;
  const codexRoot = adapter.resolveRoot(input);
  const projectRoot = getProjectRoot(input);
  const agentsHome = getAgentsHome(input);
  const operations = [];

  if (pathExists(repoRoot, 'AGENTS.md') && pathExists(repoRoot, '.codex/AGENTS.md')) {
    operations.push(createManagedBlockOperation({
      moduleId: module.id,
      sourceRelativePath: 'AGENTS.md + .codex/AGENTS.md',
      destinationPath: path.join(projectRoot, 'AGENTS.md'),
      content: renderCombinedAgentsMd(repoRoot),
      strategy: 'merge-codex-project-agents-md',
    }));
  }

  operations.push(...planAgentTomls(module.id, repoRoot, 'agents', codexRoot));

  if (pathExists(repoRoot, '.agents/skills')) {
    operations.push(createCopyOperation({
      moduleId: module.id,
      sourceRelativePath: '.agents/skills',
      destinationPath: path.join(agentsHome, 'skills'),
      strategy: 'codex-project-agent-skills',
    }));
  }

  if (pathExists(repoRoot, '.agents/plugins')) {
    operations.push(createCopyOperation({
      moduleId: module.id,
      sourceRelativePath: '.agents/plugins',
      destinationPath: path.join(agentsHome, 'plugins'),
      strategy: 'codex-project-agent-plugins',
    }));
  }

  return operations;
}

function planPlatformConfigs(module, input, adapter) {
  const repoRoot = input.repoRoot;
  const codexRoot = adapter.resolveRoot(input);
  const operations = [];

  if (pathExists(repoRoot, '.codex/config.toml')) {
    operations.push(createRenderOperation({
      moduleId: module.id,
      sourceRelativePath: '.codex/config.toml',
      destinationPath: path.join(codexRoot, 'config.toml'),
      content: renderCodexProjectConfigToml(repoRoot),
      strategy: 'render-codex-project-config',
    }));
  }

  if (pathExists(repoRoot, '.codex/agents')) {
    operations.push(createCopyOperation({
      moduleId: module.id,
      sourceRelativePath: '.codex/agents',
      destinationPath: path.join(codexRoot, 'agents'),
      strategy: 'codex-native-agent-samples',
    }));
  }

  for (const sourceRelativePath of ['mcp-configs', 'scripts/setup-package-manager.js']) {
    if (!pathExists(repoRoot, sourceRelativePath)) {
      continue;
    }
    operations.push(createCopyOperation({
      moduleId: module.id,
      sourceRelativePath,
      destinationPath: path.join(codexRoot, normalizeRelativePath(sourceRelativePath)),
    }));
  }

  return operations;
}

function planHooksRuntime(module, input, adapter) {
  const repoRoot = input.repoRoot;
  const codexRoot = adapter.resolveRoot(input);
  const operations = [];

  for (const sourceRelativePath of ['scripts/hooks', 'scripts/lib']) {
    if (!pathExists(repoRoot, sourceRelativePath)) {
      continue;
    }
    operations.push(createCopyOperation({
      moduleId: module.id,
      sourceRelativePath,
      destinationPath: path.join(codexRoot, normalizeRelativePath(sourceRelativePath)),
      strategy: 'codex-hook-runtime-copy',
    }));
  }

  if (pathExists(repoRoot, 'scripts/codex/rcc-codex-check.js')) {
    operations.push(createCopyOperation({
      moduleId: module.id,
      sourceRelativePath: 'scripts/codex/rcc-codex-check.js',
      destinationPath: path.join(codexRoot, 'scripts', 'rcc-codex-check.js'),
      strategy: 'codex-project-stage-gate-checker',
    }));
  }

  if (pathExists(repoRoot, 'hooks/hooks.json')) {
    operations.push(createRenderOperation({
      moduleId: module.id,
      sourceRelativePath: 'hooks/hooks.json',
      destinationPath: path.join(codexRoot, 'hooks.json'),
      content: renderCodexHooks(repoRoot, codexRoot),
      strategy: 'render-codex-safe-hooks',
    }));
  }

  return operations;
}

function planExtensionPrompts(moduleId, codexRoot) {
  const prompts = [
    ['ecc-tool-run-tests.md', renderRunTestsPrompt()],
    ['ecc-tool-check-coverage.md', renderCoveragePrompt()],
    ['ecc-tool-security-audit.md', renderSecurityAuditPrompt()],
    ['ecc-codex-check.md', renderCodexCheckPrompt()],
    ['ecc-codex-research-gate.md', renderCodexResearchGatePrompt()],
    ['ecc-codex-quality-gate.md', renderCodexQualityGatePrompt()],
    ['ecc-codex-eval-gate.md', renderCodexEvalGatePrompt()],
  ];
  const operations = prompts.map(([fileName, content]) => createRenderOperation({
    moduleId,
    sourceRelativePath: `codex/generated-prompts/${fileName}`,
    // Keep ecc-* filenames stable for existing Codex prompt references.
    destinationPath: path.join(codexRoot, 'prompts', fileName),
    content,
    strategy: 'render-codex-extension-prompt',
  }));

  operations.push(createRenderOperation({
    moduleId,
    sourceRelativePath: 'codex/generated-prompts/ecc-extension-prompts-manifest.txt',
    destinationPath: path.join(codexRoot, 'prompts', 'ecc-extension-prompts-manifest.txt'),
    content: renderCodexExtensionPromptsManifest(prompts.map(([fileName]) => fileName)),
    strategy: 'render-codex-extension-prompt-manifest',
  }));

  return operations;
}

function planGenericPath(module, sourceRelativePath, input, adapter) {
  const repoRoot = input.repoRoot;
  const codexRoot = adapter.resolveRoot(input);
  const normalized = normalizeRelativePath(sourceRelativePath);

  if (isForeignPlatformPath(normalized, 'codex')) {
    return [];
  }

  if (normalized === '.codex') {
    return planPlatformConfigs(module, input, adapter);
  }

  if (normalized === 'commands' || (normalized.startsWith('commands/') && normalized.endsWith('.md'))) {
    return planCommandPrompts(module.id, repoRoot, normalized, codexRoot);
  }

  if (normalized === 'agents' || (normalized.startsWith('agents/') && normalized.endsWith('.md'))) {
    return planAgentTomls(module.id, repoRoot, normalized, codexRoot);
  }

  if (normalized === 'rules') {
    return planRuleOperations(module.id, repoRoot, codexRoot);
  }

  if (normalized.startsWith('skills/')) {
    return [createSkillCopyOperation(module.id, normalized, input)];
  }

  if (normalized.startsWith('.agents/skills/')) {
    return [createSkillCopyOperation(module.id, normalized, input)];
  }

  if (normalized.startsWith('.agents/plugins')) {
    return [createCopyOperation({
      moduleId: module.id,
      sourceRelativePath: normalized,
      destinationPath: path.join(getAgentsHome(input), normalized.replace(/^\.agents\//, '')),
      strategy: 'codex-project-agent-plugins',
    })];
  }

  if (!pathExists(repoRoot, normalized)) {
    return [];
  }

  return [createCopyOperation({
    moduleId: module.id,
    sourceRelativePath: normalized,
    destinationPath: path.join(codexRoot, normalized),
  })];
}

function dedupeOperationsByDestination(operations) {
  const lastIndexByDestination = new Map();
  operations.forEach((operation, index) => {
    lastIndexByDestination.set(operation.destinationPath, index);
  });

  return operations.filter((operation, index) => lastIndexByDestination.get(operation.destinationPath) === index);
}

function planCodexOperations(input = {}, adapter) {
  const operations = [];
  const modules = Array.isArray(input.modules) ? input.modules : [];
  const codexRoot = adapter.resolveRoot(input);

  for (const module of modules) {
    if (module.id === 'agents-core') {
      operations.push(...planAgentsCore(module, input, adapter));
      continue;
    }

    if (module.id === 'platform-configs') {
      operations.push(...planPlatformConfigs(module, input, adapter));
      continue;
    }

    if (module.id === 'commands-core') {
      operations.push(...planCommandPrompts(module.id, input.repoRoot, 'commands', codexRoot));
      operations.push(...planExtensionPrompts(module.id, codexRoot));
      continue;
    }

    if (module.id === 'rules-core') {
      operations.push(...planRuleOperations(module.id, input.repoRoot, codexRoot));
      continue;
    }

    if (module.id === 'hooks-runtime') {
      operations.push(...planHooksRuntime(module, input, adapter));
      continue;
    }

    const paths = Array.isArray(module.paths) ? module.paths : [];
    for (const sourceRelativePath of paths) {
      operations.push(...planGenericPath(module, sourceRelativePath, input, adapter));
    }
  }

  return dedupeOperationsByDestination(operations);
}

module.exports = createInstallTargetAdapter({
  id: 'codex-project',
  target: 'codex',
  kind: 'project',
  rootSegments: ['.codex'],
  installStatePathSegments: ['ecc-install-state.json'],
  nativeRootRelativePath: '.codex',
  planOperations: planCodexOperations,
});
