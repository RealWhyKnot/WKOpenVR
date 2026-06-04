# Contributing

Thanks for helping improve WKOpenVR. Keep changes focused and easy to review.

## Before opening an issue

- Search existing issues and the README first.
- Include the version, operating system, relevant VRChat/SteamVR details, and logs when applicable.
- Keep security reports private; use GitHub Security Advisories instead of public issues.

## Pull requests

- Open one PR per behavior change or documentation update.
- Describe why the change is needed, not only what files changed.
- Update README or CHANGELOG when behavior, setup, diagnostics, or release notes change.
- Run powershell -ExecutionPolicy Bypass -File build.ps1 where applicable.

## Commit style

Prefer conventional commit subjects such as eat:, ix:, docs:, ci:, and chore:. Use [skip changelog] for changes that should not produce user-facing release notes.
