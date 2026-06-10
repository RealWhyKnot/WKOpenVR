# Release workflow

Releases are tag-driven and fully automated. Pushing a `v*` tag triggers
[release.yml](workflows/release.yml), which builds the shared WKOpenVR
binary and driver DLL, packages the local artifacts, tests the shared
installer round trip, builds one `*-Setup.exe` per selected public module,
and publishes those installers to the module repositories. Stable tags
publish every active public module. Beta tags publish only modules whose
module files or shared release inputs changed since that module's latest
release.

The WKOpenVR source repository is not the end-user release surface. End
users install from the module repositories:

- Space Calibrator: https://github.com/RealWhyKnot/WKOpenVR-SpaceCalibrator
- Smoothing: https://github.com/RealWhyKnot/WKOpenVR-Smoothing
- Input Health: https://github.com/RealWhyKnot/WKOpenVR-InputHealth
- OSC Router: https://github.com/RealWhyKnot/WKOpenVR-OSCRouter
- Face Tracking: https://github.com/RealWhyKnot/WKOpenVR-FaceTracking
- Quest App: https://github.com/RealWhyKnot/WKOpenVR-QuestApp
- Captions: https://github.com/RealWhyKnot/WKOpenVR-Captions

The workflow still promotes `CHANGELOG.md` for stable tags. Stable release
tags use the previous stable tag as the base, so beta release notes since
the last stable are included when the stable release is published. Beta and
dev tags use the nearest previous tag.

[nightly-beta.yml](workflows/nightly-beta.yml) runs once per night and can
also be started manually. It checks the latest published release in each
module repository, compares that tag to the source tree, and pushes a new
`vYYYY.M.D.N-beta` tag only when at least one module has releasable changes.
That tag starts the normal release workflow.

## Tag shapes

| Form | When | Example |
|---|---|---|
| `vYYYY.M.D.N` | Release. `.N` is the release iteration for that calendar day, starting at 0. | `v2026.5.6.0` |
| `vYYYY.M.D.N-beta` | Nightly or manual prerelease. `.N` is the beta iteration for that calendar day, starting at 0. | `v2026.5.6.0-beta` |
| `vYYYY.M.D.N-XXXX` | Dev. `.N` is local build count; `XXXX` is a 4-hex UID. Rare on the release stream. | `v2026.5.6.0-A1B2` |

Beta tags must use exactly the `-beta` suffix. If another prerelease is
needed on the same day, increment `.N` instead of adding a numbered beta
suffix.

## Module release body

Each module release gets a small generated body:

```
# <Module Name> <tag>

<Module Name> installer for WKOpenVR <version>.
Run this Setup.exe to install WKOpenVR and enable <Module Name>.
Other WKOpenVR modules can be enabled from the Modules tab or by running another module installer.

SHA256: <hash>
```

The source repo release-note generator remains tested for source-repo notes
and changelog composition, but tag publishing does not create a WKOpenVR
GitHub release.

The tested source-note generator deliberately skips beta tags when picking
the stable compare base. For example, if `v2026.5.5.0-beta` ships after
`v2026.5.1.0`, then `v2026.5.7.0` compares from `v2026.5.1.0` so the beta
changes appear in generated source notes too. CHANGELOG.md keeps
accumulating in-repo for browsing.

## Source-note policy

Commits in the tag range get bucketed by the prefix on their subject line:

| Prefix | Section |
|---|---|
| `feat(...)?:` | Features |
| `fix(...)?:` | Bug Fixes |
| `perf(...)?:` | Performance |
| `refactor(...)?:` | Refactors |
| `revert(...)?:` | Reverts |
| `docs(...)?:` | Documentation |
| `style(...)?:` | Style |
| `test(...)?:` | Tests |
| `ci(...)?:` | CI |
| `build(...)?:` | Build |
| `chore(...)?:` | Chores |
| anything else | Other Changes |

Subjects starting with `<prefix>!:` (the breaking-change marker) match
the same section. Trailing version-stamps like ` (2026.5.6.0-A1B2)` are
stripped before grouping. Commits with `[skip changelog]` in the subject
are excluded entirely; merge commits are excluded by `--no-merges`.

## Author handle remap

The generator emits `by @<author>` from the commit's `%an` field. The
local git config uses the brand "WhyKnot"; the GitHub login is
"RealWhyKnot". The generator carries an `$AuthorHandleMap` hashtable
that remaps known git authors to the right login before emit.

## Scrub gates

When [Generate-ReleaseNotes.ps1](scripts/Generate-ReleaseNotes.ps1) composes
source notes, three gates run before output is accepted:

1. **ASCII normalisation.** Common typographic patterns (em-dash,
   en-dash, ellipsis, smart quotes, etc.) are substituted to ASCII
   equivalents. One-way, silent.

2. **Non-ASCII fail.** Anything left outside printable ASCII (0x20-0x7E
   plus tab) after normalisation fails with the offending line, column,
   and Unicode code point.

3. **Voice + internal-only-vocabulary check.** Pattern groups match
   case-insensitively against the composed body. Any match fails the
   script. Fix by amending the offending commit subject (or extras
   file) to use plainer language, OR mark the commit `[skip changelog]`.

The scrub list lives in
[Generate-ReleaseNotes.ps1](scripts/Generate-ReleaseNotes.ps1).

## Required secrets

`MODULE_RELEASE_TOKEN` must have release-write access to the seven module
repositories and contents-write access to the source repository for pushing
nightly beta tags. The repository `GITHUB_TOKEN` is still used for checkout
and the signed changelog promote-back commit, but it cannot publish releases
in sibling repositories and tag pushes made with it do not start the tag
release workflow.

## Promote-back to main

The workflow pushes the promoted `CHANGELOG.md` back to `main` via the
GraphQL `createCommitOnBranch` mutation. The resulting commit is signed
server-side with GitHub's bot key, so it lands as verified=true.

## Failure modes + remediations

| Symptom | Fix |
|---|---|
| `No commits found in range` | Check the tag's parent reachability. Either prev-tag detection failed or every commit is `[skip changelog]`. |
| `Non-ASCII characters in release body after normalisation` | Amend the offending commit subject, force-push the tag at the new SHA. Or add the char to `$asciiSubs`. |
| `Voice or internal-only-vocabulary patterns in release body` | Amend the offending commit subject. Or `[skip changelog]` it if unavoidable. |
| `MODULE_RELEASE_TOKEN secret is required` | Add or refresh the token with release-write access to the module repositories. |
| `Beta tag did not select any module installers` | Check that the tag follows `vYYYY.M.D.N-beta` and that the module repositories have reachable previous release tags. |
| `createCommitOnBranch returned GraphQL errors` | Usually a stale `expectedHeadOid`. Release itself is fine; re-run the promote step or re-run the workflow. |

## Updating the workflow

The workflow + scripts are versioned alongside the code. Workflow
plumbing fixes MUST ride along with a genuine customer release; never
tag a release purely to test workflow plumbing. Validate YAML and scripts
locally, then let the next real release exercise the publish path.
