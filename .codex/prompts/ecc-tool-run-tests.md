# RCC Tool Prompt: run-tests

Run the repository test suite with package-manager autodetection and concise reporting.

1. Detect the package manager from lock files.
2. Detect available scripts or test commands for this repo.
3. Execute tests with the best project-native command.
4. If tests fail, report failing files/tests first, then the smallest likely fix list.
5. Do not change code unless explicitly asked.
