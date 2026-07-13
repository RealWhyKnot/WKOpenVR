# Contributing

Thanks for helping improve WKOpenVR. Keep changes focused and easy to review.

## Before opening an issue

- Search existing issues and the README first.
- Include the version, operating system, relevant VRChat/SteamVR details, and logs when applicable.
- Keep security reports private; use GitHub Security Advisories instead of public issues.

## Developer loop

- Build: `powershell -ExecutionPolicy Bypass -File build.ps1`
- Test: `powershell -ExecutionPolicy Bypass -File test.ps1`. Four phases: build, the GoogleTest binaries, the in-process driver harness (`WKOpenVR.exe --test-harness`), and download + load checks. Narrow a run with `-Suite <name>` (for example `-Suite calibration`), `-Filter <gtest filter>`, `-SkipHarness`, or `-SkipDownload`.
- Lint: `powershell -ExecutionPolicy Bypass -File lint.ps1 -Check -ChangedOnly` runs clang-format and clang-tidy over changed files. The pre-push hook runs the same check.
- Calibration changes: also run `tools\Run-SessionReplayGate.ps1 -Quick -Baseline` for each scenario (default, `-Scenario upstream_parity`, `-Scenario v2_math`). Replay counters must match the stored baselines in `tools\replay-baselines` exactly. Run the gate without `-Quick` before pushing solver or recovery changes.

## Code style

- clang-format and clang-tidy settings live at the repo root; `lint.ps1` applies both.
- Member naming follows the module you are in: `m_` prefixes in core and calibration, trailing underscores in captions. Match the surrounding file.
- Plain ASCII in code, comments, and docs.

## Pull requests

- Open one PR per behavior change or documentation update.
- Describe why the change is needed, not only what files changed.
- Update README or CHANGELOG when behavior, setup, diagnostics, or release notes change.

## Commit style

Use conventional commit subjects: `feat:`, `fix:`, `docs:`, `ci:`, `chore:`, `refactor:`, `test:`, `build:`, with an optional scope such as `fix(calibration):`. Use `[skip changelog]` for changes that should not produce user-facing release notes. The commit hooks append a build stamp to the subject; never type one by hand.
