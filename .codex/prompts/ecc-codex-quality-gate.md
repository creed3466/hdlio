# RCC Codex Gate Prompt: quality-gate

Before finalizing code changes, run:

```bash
node .codex/scripts/rcc-codex-check.js quality --changed
node .codex/scripts/rcc-codex-check.js console --changed
node .codex/scripts/rcc-codex-check.js secrets --staged
```

Report pass/fail, files checked, and unresolved risk.
