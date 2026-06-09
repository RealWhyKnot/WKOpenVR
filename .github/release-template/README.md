# Release-body templates

`Generate-ReleaseNotes.ps1` reads each `.md` file in this directory and emits its content as a section of a WKOpenVR release body. The tag workflow publishes end-user installers from the module repositories; this generator stays covered by tests so source-repo release notes can still be composed if needed. The order is fixed: title, auto-changelog slice, file integrity, then `links.md`, `install.md`, `uninstall.md`, `what-you-need-to-do.md`, then optional extras from `.github/release-extras/<tag>.md`.

## Tokens

Each template runs through token substitution before the body composes. Any of these strings in a template gets replaced with the corresponding value at compose time:

| Token | Example value |
|---|---|
| `{tag}` | `v2026.5.6.0` |
| `{version}` | `2026.5.6.0` |
| `{owner}` | `RealWhyKnot` |
| `{repo}` | `WKOpenVR` |
| `{full-repo}` | `RealWhyKnot/WKOpenVR` |
| `{commit-sha}` | full 40-char hash of the tag's commit |
| `{commit-sha-short}` | first 12 chars of the same hash |
| `{prior-tag}` | `v2026.5.5.0` (empty on first release) |
| `{zip-name}` | `WKOpenVR-v2026.5.6.0.zip` |

Tokens that the resolver could not compute render as the literal token string. A missing token is visible to a reader so they can file an operator fix.

## Optional release-specific extras

Templates here are evergreen content, the same on every release. For one-off prose tied to a single release, put a markdown file at `.github/release-extras/<tag>.md`. The composer appends its content verbatim below the templated sections, separated by `---` and an `## Additional notes` heading.

## Editing existing templates

Templates are read verbatim and pass through the same scrub gates as commit subjects. Avoid marketing puffery, internal-tooling vocabulary, AI-shaped phrasing, and any character outside printable ASCII. The list of forbidden patterns lives in `Generate-ReleaseNotes.ps1` near the bottom.
