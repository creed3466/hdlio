<!-- BEGIN RCC -->
# Everything Claude Code (ECC) — Agent Instructions

This repository is used as a **research-development harness**, providing 22 specialized agents, 78 skills, 73 commands, and automated hook workflows for research-driven software development on Python and C++ projects.

The default operating mode is research-driven. For non-trivial work, route through the Research → Architect → Build → Eval pipeline via the `ben` orchestrator.

**Version:** 1.10.0

## Core Principles

1. **Research Before Implementation** — For algorithms, models, architecture, or theoretical claims, start with research, not code
2. **Design Artifacts Are Durable** — Before building, produce or update a short design document under `docs/specs/`
3. **Evidence Over Assertion** — Keep facts, assumptions, and open questions separate; prefer primary sources
4. **Validation > Tests** — Passing tests is not completion; validation means the result supports the intended research or design claim
5. **Agent-First** — Delegate to specialized agents; use `ben` as the default orchestrator for multi-stage work
6. **Immutability** — Always create new objects, never mutate existing ones
7. **Explicit Drift** — If implementation diverges from the design, surface the drift and update the design

> The production-engineering principles (TDD 80%+, security-first hardening, OWASP checklist) still apply **when building production components**. See `## When Building Production Components` below.

## Available Agents

RCC v2 ships 22 agents — research orchestration, research-stage specialists,
cross-model adversarial reviewers, paper review, software-engineering
review/build helpers for the supported languages (Python + C++), and
generic engineering utilities.

### R&D Orchestration (3)

| Agent | Purpose | When to Use |
|-------|---------|-------------|
| ben | Research-development orchestrator (Research → Codex Review → Architect → Build → Eval) | Default for non-trivial research-driven work; enforces stage gates |
| researcher | Research-stage specialist (hypothesis, mathematical analysis, literature verification, theoretical-bottleneck enforcement) | Before any non-trivial implementation |
| codex-reviewer | Adversarial cross-model reviewer via Codex CLI (GPT-5.5) | Stage 2 of the 5-stage pipeline; model-independent critique |

### Design + Planning (3)

| Agent | Purpose | When to Use |
|-------|---------|-------------|
| architect | System design and scalability — turns research into implementation-ready design | Architectural decisions, design contracts |
| code-architect | Designs feature architectures from existing patterns | New features with concrete implementation blueprint |
| planner | Implementation planning, dependencies, risks | Complex features, refactoring |

### Code Review (4)

| Agent | Purpose | When to Use |
|-------|---------|-------------|
| code-reviewer | General code quality and maintainability | After writing/modifying code |
| security-reviewer | Vulnerability detection (OWASP Top 10, secrets, SSRF, injection) | Before commits, sensitive code |
| python-reviewer | Python — PEP 8, type hints, security, performance | Python projects |
| cpp-reviewer | C++ — memory safety, modern idioms, concurrency, performance | C++ projects |

### Build + Performance (3)

| Agent | Purpose | When to Use |
|-------|---------|-------------|
| build-error-resolver | Generic build/type error resolution | When build fails |
| cpp-build-resolver | C++/CMake/linker error resolution | C++ build failures |
| pytorch-build-resolver | PyTorch runtime/CUDA/training error resolution | PyTorch training crashes |

### Code Analysis (3)

| Agent | Purpose | When to Use |
|-------|---------|-------------|
| code-explorer | Deep codebase analysis — traces execution paths, maps architecture | Understanding unfamiliar code before changes |
| refactor-cleaner | Dead code cleanup, unused imports, duplicates | Code maintenance |
| tdd-guide | Test-driven development enforcement | New features, bug fixes |

### Docs + Database (3)

| Agent | Purpose | When to Use |
|-------|---------|-------------|
| docs-lookup | Documentation lookup via Context7 MCP | API/docs questions |
| doc-updater | Documentation and codemap updates | Doc/codemap maintenance |
| database-reviewer | PostgreSQL specialist (schema, queries, indexing) | Schema design, query optimization |

### Paper Review (1)

| Agent | Purpose | When to Use |
|-------|---------|-------------|
| paper-reviewer | Adversarial paper reviewer with strict context isolation (RA-L/ICRA/IROS/CVPR/TRO/IJRR) | Pre-submission audit; takes only `{pdf_path, venue, reviewer, role}` |

## Agent Orchestration

Use agents proactively without user prompt:
- Non-trivial research-driven work → **ben** (routes through 5-stage pipeline)
- Pre-implementation research → **researcher**
- Adversarial cross-model review → **codex-reviewer**
- Complex feature requests → **planner**
- Architectural decision → **architect** or **code-architect**
- Unfamiliar codebase exploration → **code-explorer**
- Code just written/modified → **code-reviewer** (+ language reviewer)
- Bug fix or new feature → **tdd-guide**
- Security-sensitive code → **security-reviewer**
- Pre-submission paper audit → **paper-reviewer**

Use parallel execution for independent operations — launch multiple agents simultaneously.

## Research Workflow (Default)

For algorithmic work, model changes, architecture decisions, or any theoretical claim, operate in four stages:

1. **Research** — Establish problem, baseline, evidence, and candidate proposal. Prefer primary sources (papers, official docs, reference implementations). Output goes to `docs/research/`.
2. **Architect** — Turn research into an implementation-ready design: objective, baseline, proposal, interfaces, risks, validation plan. Output goes to `docs/specs/`.
3. **Build** — Implement against the design. Surface design drift explicitly. Keep implementation notes in `docs/experiments/`.
4. **Eval** — Compare the result to the baseline or success criteria defined in Architect. Record outcomes in `docs/results/`.

Do not skip directly from idea to code unless the user asks for a fast prototype. The `ben` agent enforces these stage gates.

**Completion standard:** Work is complete when the design intent is clear, the implementation matches intent, the result has been evaluated at the right level, and remaining uncertainty is stated explicitly.

## When Building Production Components

The guidelines below apply to production-bound code (services, APIs, shipped software). For pure research/experimentation code, scale these expectations to context.

### Security Guidelines

**Before ANY commit:**
- No hardcoded secrets (API keys, passwords, tokens)
- All user inputs validated
- SQL injection prevention (parameterized queries)
- XSS prevention (sanitized HTML)
- CSRF protection enabled
- Authentication/authorization verified
- Rate limiting on all endpoints
- Error messages don't leak sensitive data

**Secret management:** NEVER hardcode secrets. Use environment variables or a secret manager. Validate required secrets at startup. Rotate any exposed secrets immediately.

**If security issue found:** STOP → use security-reviewer agent → fix CRITICAL issues → rotate exposed secrets → review codebase for similar issues.

### Coding Style

**Immutability (CRITICAL):** Always create new objects, never mutate. Return new copies with changes applied.

**File organization:** Many small files over few large ones. 200-400 lines typical, 800 max. Organize by feature/domain, not by type. High cohesion, low coupling.

**Error handling:** Handle errors at every level. Provide user-friendly messages in UI code. Log detailed context server-side. Never silently swallow errors.

**Input validation:** Validate all user input at system boundaries. Use schema-based validation. Fail fast with clear messages. Never trust external data.

**Code quality checklist:**
- Functions small (<50 lines), files focused (<800 lines)
- No deep nesting (>4 levels)
- Proper error handling, no hardcoded values
- Readable, well-named identifiers

### Testing Requirements

**Minimum coverage: 80%** (production components)

Test types (all required):
1. **Unit tests** — Individual functions, utilities, components
2. **Integration tests** — API endpoints, database operations
3. **E2E tests** — Critical user flows

**TDD workflow (mandatory):**
1. Write test first (RED) — test should FAIL
2. Write minimal implementation (GREEN) — test should PASS
3. Refactor (IMPROVE) — verify coverage 80%+

Troubleshoot failures: check test isolation → verify mocks → fix implementation (not tests, unless tests are wrong).

## Development Workflow

1. **Plan** — Use planner agent, identify dependencies and risks, break into phases
2. **TDD** — Use tdd-guide agent, write tests first, implement, refactor
3. **Review** — Use code-reviewer agent immediately, address CRITICAL/HIGH issues
4. **Capture knowledge in the right place**
   - Personal debugging notes, preferences, and temporary context → auto memory
   - Team/project knowledge (architecture decisions, API changes, runbooks) → the project's existing docs structure
   - If the current task already produces the relevant docs or code comments, do not duplicate the same information elsewhere
   - If there is no obvious project doc location, ask before creating a new top-level file
5. **Commit** — Conventional commits format, comprehensive PR summaries

## Workflow Surface Policy

- `skills/` is the canonical workflow surface.
- New workflow contributions should land in `skills/` first.
- `commands/` is a legacy slash-entry compatibility surface and should only be added or updated when a shim is still required for migration or cross-harness parity.

## Git Workflow

**Commit format:** `<type>: <description>` — Types: feat, fix, refactor, docs, test, chore, perf, ci

**PR workflow:** Analyze full commit history → draft comprehensive summary → include test plan → push with `-u` flag.

## Architecture Patterns

**API response format:** Consistent envelope with success indicator, data payload, error message, and pagination metadata.

**Repository pattern:** Encapsulate data access behind standard interface (findAll, findById, create, update, delete). Business logic depends on abstract interface, not storage mechanism.

**Skeleton projects:** Search for battle-tested templates, evaluate with parallel agents (security, extensibility, relevance), clone best match, iterate within proven structure.

## Performance

**Context management:** Avoid last 20% of context window for large refactoring and multi-file features. Lower-sensitivity tasks (single edits, docs, simple fixes) tolerate higher utilization.

**Build troubleshooting:** Use build-error-resolver agent → analyze errors → fix incrementally → verify after each fix.

## Project Structure

```
agents/          — 22 specialized subagents
skills/          — 78 workflow skills and domain knowledge
commands/        — 73 slash commands
hooks/           — Trigger-based automations
rules/           — Always-follow guidelines (common + python + cpp)
scripts/         — Cross-platform Node.js utilities
mcp-configs/     — 15 MCP server configurations
tests/           — Test suite
```

`commands/` remains in the repo for compatibility, but the long-term direction is skills-first.

## Success Metrics

- All tests pass with 80%+ coverage
- No security vulnerabilities
- Code is readable and maintainable
- Performance is acceptable
- User requirements are met

---

# Codex Supplement (From RCC .codex/AGENTS.md)

# RCC for Codex CLI

This supplements the root `AGENTS.md` with Codex-specific guidance. The root
RCC instructions remain the canonical research workflow; this file describes how
that workflow is rendered into Codex-native project files.

## Codex Is Not Claude Code

Claude Code RCC is a runtime orchestration system:

- Ben orchestrator running inside Claude Code
- Agent/Task subagents
- Claude hook events
- `.claude/research-state.json`
- automatic stage-gate and stage-record updates

Codex RCC is a project-local rendered kit:

- `AGENTS.md` instructions
- `.codex/config.toml`
- `.codex/agents/*.toml`
- `.codex/prompts/*.md`
- `.agents/skills/*`
- explicit `.codex/scripts/rcc-codex-check.js` gates

Do not claim Codex has full Claude Code stage-gate or stage-record parity. In
Codex, gates are explicit prompt/checker steps unless a future state-lite layer
is added.

## Model Policy

Use the current Codex default for simple interaction unless a task needs a
specific model. RCC-rendered Codex agents default to GPT-5.5 so research,
architecture, adversarial review, and security work keep a strong reasoning
baseline.

| Task Type | Recommended Model |
|---|---|
| Routine coding, tests, formatting | Current Codex default, or GPT-5.4 when intentionally cost-optimized |
| Debugging and local refactors | Current Codex default or GPT-5.5 for broad/ambiguous failures |
| Complex architecture and research | GPT-5.5 |
| Adversarial review and security review | GPT-5.5 |

## Skills Discovery

Codex loads project skills from `.agents/skills/`. Each selected RCC skill is
installed as a folder with a `SKILL.md` entrypoint. The active profile controls
which skills are rendered; avoid hard-coding skill counts in user-facing output.

## Project Config

Treat the target project's `.codex/config.toml` as the Codex baseline for RCC.
The project config contains repo-safe defaults such as sandbox policy, web
search policy, MCP server definitions, multi-agent enablement, and role file
references.

User-specific credentials, auth files, and personal global preferences stay in
the user's own Codex configuration. RCC project install must not write
`~/.codex`, `~/.agents`, or `~/.claude`.

RCC's canonical Codex section name for Context7 is
`[mcp_servers.context7]`. The launcher package remains
`@upstash/context7-mcp`; only the TOML section name is normalized for
consistency with `codex mcp list` and the reference config.

## Multi-Agent Support

Codex supports multi-agent workflows through `[features] multi_agent = true`
and standalone role files under `.codex/agents/*.toml`.

- Enable it in `.codex/config.toml` with `[features] multi_agent = true`
- Define project-local role files under `.codex/agents/*.toml`
- Include `name`, `description`, and `developer_instructions` in each role file
- Use `/agent` inside Codex CLI to inspect and steer child agents

RCC renders Claude agent markdown into Codex TOML roles. The TOML roles are
Codex-native artifacts, not Claude Code Agent/Task definitions.

## Commands And Prompts

Claude slash commands are rendered into Codex prompt files under
`.codex/prompts/`. The installed filenames intentionally keep the historical
`ecc-*` prefix for compatibility with existing references, but the generated
prompt copy should describe RCC.

## Key Differences From Claude Code

| Feature | Claude Code | Codex CLI |
|---|---|---|
| Context | `CLAUDE.md` plus `AGENTS.md` | `AGENTS.md` plus `.codex/config.toml` |
| Agents | Agent/Task subagents | `/agent` and `.codex/agents/*.toml` roles |
| Commands | Claude slash commands | Prompt files in `.codex/prompts/` |
| Hooks | Full Claude hook runtime | Conservative Codex-safe subset plus explicit checker gates |
| State | `.claude/research-state.json` automation | Explicit gates/checkers unless future state-lite is added |
| Skills | Claude plugin skills | Project `.agents/skills/` |
| MCP | Claude Code MCP config | Project `.codex/config.toml` and Codex MCP commands |

## Security With Codex Hooks

RCC installs a conservative Codex-native `.codex/hooks.json` generated from the
Claude hook catalog. The installed subset avoids Claude-specific state paths and
only uses commands that are safe inside the target project. Do not add
Claude-specific stage-record or stage-gate hooks to Codex without proving they
work under Codex hook events.

Before finalizing work in Codex, prefer explicit gates:

```bash
node .codex/scripts/rcc-codex-check.js all
node .codex/scripts/rcc-codex-check.js research --changed
node .codex/scripts/rcc-codex-check.js quality --changed
node .codex/scripts/rcc-codex-check.js eval --log <file>
```
<!-- END RCC -->
