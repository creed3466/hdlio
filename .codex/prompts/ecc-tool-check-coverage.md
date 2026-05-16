# RCC Tool Prompt: check-coverage

Analyze coverage and compare it to an 80% threshold unless a different threshold is provided.

Find existing coverage artifacts first. If missing, run the project coverage command with the detected package manager. Report total coverage, top under-covered files, and recommended focus areas.
