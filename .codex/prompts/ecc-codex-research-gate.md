# RCC Codex Gate Prompt: research-gate

Before promoting Research to Architect, run:

```bash
node .codex/scripts/rcc-codex-check.js research --changed
```

Then invoke or simulate the `codex-reviewer` stage. Do not promote unsupported claims. External claims need `[verified: <source>]` or `[unverified]`.
