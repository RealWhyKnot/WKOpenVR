#!/usr/bin/env pwsh
<#
.SYNOPSIS
  Generate the GitHub release body for a tag from the git-log slice plus
  per-repo template sections plus a per-file integrity table.

.DESCRIPTION
  Composes a multi-section markdown body for a source-repo release note. Section order:

    1. Title (h1: "<repo> <tag>")
    2. What's Changed (auto-changelog from the commit slice between prev tag
       and this tag; bucketed by conventional-commit prefix)
    3. File integrity (two-row Markdown table covering the umbrella zip and
       the matching Setup.exe; per-file hashes inside the zip are deliberately
       not listed any more -- the body had grown past one screen of scroll
       and the inner hashes were rarely consulted)
    4. More (from .github/release-template/links.md, with token substitution)
    5. Install (fresh) (from .github/release-template/install.md)
    6. Uninstall (from .github/release-template/uninstall.md)
    7. What you need to do (from .github/release-template/what-you-need-to-do.md)
    8. Optional extras (from .github/release-extras/<tag>.md if present;
       appended below with `---` separator and `## Additional notes` heading)

  Slice composition: walks commits between prev tag and current tag. Stable
  release tags use the previous stable tag as their base, so any beta notes
  since the last stable release are included in the stable release body. Beta
  and dev tags keep using the nearest previous tag. Skips merge commits and
  commits containing "[skip changelog]". Strips trailing version-stamp noise
  of the form " (YYYY.M.D.N-XXXX)" that some repos append to subjects. Groups
  by conventional-commit prefix when at least one entry has one, otherwise
  emits a flat bullet list.

  Prev-tag resolution is layered for resilience against history rewrites that
  orphan the prior tag (rebase + force-push of main): describe + sanity gate,
  then subject-match against the most recent published GitHub release, then
  root-walk fallback. See Resolve-PrevTagForSlice for details.

  Templates and the optional extras file run through an ASCII normalisation
  pass, then any remaining non-ASCII characters fail the workflow with a
  remediation hint.

  Outputs the markdown body to stdout. Throws on:
    * empty slice (no qualifying commits between prev and current tag)
    * non-ASCII characters in the final body (after a normalisation pass)

  Each failure prints a clear remediation hint so the operator knows whether
  to amend a commit, mark one [skip changelog], or fix a template or the
  extras file.

  Requires the checkout step to have used fetch-depth: 0 (or otherwise have
  the full history + tags available).

.PARAMETER Tag
  The release tag being built (e.g. "v2026.4.28.0"). Defaults to env:TAG_NAME
  or env:GITHUB_REF_NAME.

.PARAMETER Repo
  GitHub "owner/repo" slug for the compare link. Defaults to env:GITHUB_REPOSITORY.

.PARAMETER Extras
  Optional path to a markdown file whose contents get appended verbatim
  below all auto sections. Used for release-specific narrative that does
  not fit a commit subject (migration steps, server-side coordination
  notes, etc.). Default: ".github/release-extras/<tag>.md".

.PARAMETER TemplateDir
  Directory containing the per-section evergreen templates: links.md,
  install.md, uninstall.md, what-you-need-to-do.md. Templates undergo
  token substitution (see README in this directory) before emission.
  Default: ".github/release-template".

.PARAMETER ZipPath
  Path to the release zip artifact. Used to derive the zip name when
  -ZipName is not set, and as a presence check for the File integrity
  section.

.PARAMETER ZipName
  Override for the zip's display name in the File integrity section.
  Defaults to the leaf of -ZipPath.

.PARAMETER ZipSize
  Size in bytes of the release zip. Used by the File integrity section.

.PARAMETER ZipSha256
  SHA256 of the release zip. Used by the File integrity section.

.PARAMETER SetupPath
  Path to the Setup.exe installer. Used to derive the display name when
  -SetupName is not set, and as a presence check for the second row of
  the File integrity section.

.PARAMETER SetupName
  Override for the installer's display name in the File integrity section.
  Defaults to the leaf of -SetupPath.

.PARAMETER SetupSize
  Size in bytes of the installer. Used by the File integrity section.

.PARAMETER SetupSha256
  SHA256 of the installer. Used by the File integrity section.

.PARAMETER AllowEmpty
  Skip the empty-slice guard. Use only for the very first release on a repo
  where there is no prior tag and the tag-range trick yields nothing
  meaningful. Default: $false.

.PARAMETER SkipScrub
  Skip the ASCII scrub. Escape hatch for unblocking edge cases; the
  workflow should never set this. Default: $false.
#>
[CmdletBinding()]
param(
    [string] $Tag         = $(if ($env:TAG_NAME) { $env:TAG_NAME } else { $env:GITHUB_REF_NAME }),
    [string] $Repo        = $env:GITHUB_REPOSITORY,
    [string] $Extras      = $null,
    [string] $TemplateDir = $null,
    [string] $ZipPath     = $null,
    [string] $ZipName     = $null,
    [long]   $ZipSize     = 0,
    [string] $ZipSha256   = $null,
    [string] $SetupPath   = $null,
    [string] $SetupName   = $null,
    [long]   $SetupSize   = 0,
    [string] $SetupSha256 = $null,
    [switch] $AllowEmpty,
    [switch] $SkipScrub
)

$ErrorActionPreference = 'Stop'

if (-not $Tag) { throw "No tag provided (pass -Tag or set TAG_NAME / GITHUB_REF_NAME)." }

# Default extras path conventionally lives under .github/release-extras/<tag>.md
# next to the workflow that consumes it. Resolve relative to the caller's CWD,
# not the script dir, so the workflow can invoke the script from repo root.
if (-not $Extras) {
    $Extras = Join-Path -Path (Get-Location) -ChildPath ".github/release-extras/$Tag.md"
}

# Default template dir for the per-section evergreen content (More, Install,
# Uninstall, What you need to do). Each repo curates its own content here;
# the composer reads `<TemplateDir>/<section>.md` and runs token substitution
# on the contents before emitting it as a section of the release body.
if (-not $TemplateDir) {
    $TemplateDir = Join-Path -Path (Get-Location) -ChildPath ".github/release-template"
}

# Resolve previous tag. Layered fallback so a history rewrite that orphans the
# prior tag does not produce a giant slice walking from root:
#   1. `git describe --tags --abbrev=0 <tag>^` finds the most recent tag
#      reachable from $Tag^ along the current line of history. Stable tags
#      exclude prerelease/dev tags from this probe so a stable release after a
#      beta includes the beta's changes in the public release body. Sanity
#      gate: if the slice is more than 50 commits, treat the describe result as
#      stale (likely orphaned by a rewrite) and fall through.
#   2. Ask GitHub for the most recent published non-prerelease release. Look
#      up its tag in the local repo (the SHA may be orphaned in current
#      history but the object still exists in git's DB). Read the orphan
#      commit's subject. Walk current $Tag history looking for that exact
#      subject; use the matched SHA as the slice anchor. This works because
#      a typical rebase preserves commit subjects even when SHAs change.
#   3. Date anchoring is NOT used: a force-push rebase rewrites every
#      commit's committer-date, so --since against the prev release's
#      publishedAt walks the full rewritten history instead of the slice.
#   4. If layers 1+2 yield nothing, walk from the repo's root commit.
#      First-release fallback.
# Surfaces a ::warning:: when layer 2 or 4 fires so the operator sees in
# workflow logs that a fallback was used.
function Test-IsPrereleaseTag([string]$Tag) {
    return $Tag -match '^v?\d{4}\.\d+\.\d+\.\d+-.+'
}

function Resolve-PrevTagForSlice([string]$Tag, [string]$Repo) {
    # Function-local relaxation of EAP. The script's outer Stop is intact
    # for non-git logic, but the prev-tag probes legitimately fail in
    # several cases (no tags yet, orphaned tags after rewrite, gh not
    # authed) and need to be soft-failures here.
    $ErrorActionPreference = 'Continue'

    # Layer 1: describe + sanity gate.
    $describeArgs = @('describe', '--tags', '--abbrev=0')
    if (-not (Test-IsPrereleaseTag -Tag $Tag)) {
        $describeArgs += @('--exclude', '*-*')
    }
    $describeArgs += "$Tag^"
    $prevRef = & git @describeArgs 2>$null
    if ($LASTEXITCODE -eq 0 -and $prevRef) {
        $prevTag = $prevRef.Trim()
        $count = & git rev-list --count "$prevTag..$Tag" 2>$null
        if ($LASTEXITCODE -eq 0 -and [int]$count -le 50) {
            return @{
                Tag     = $prevTag
                LogArgs = @("$prevTag..$Tag")
                Display = "$prevTag..$Tag"
                Source  = 'describe'
            }
        }
        Write-Host "::warning::Slice from $prevTag..$Tag is $count commits (>50 cap). Falling back to subject-match against the most recent published release."
    }

    # Layer 2: subject-match the most recent published GitHub release.
    if ($Repo) {
        $listJson = & gh release list --repo $Repo --limit 20 --json tagName,publishedAt,isPrerelease 2>$null
        if ($LASTEXITCODE -eq 0 -and $listJson) {
            $candidatePrevTag = $null
            try {
                $releases = $listJson | ConvertFrom-Json
                $candidate = $releases |
                    Where-Object { $_.tagName -ne $Tag -and -not $_.isPrerelease } |
                    Sort-Object publishedAt -Descending |
                    Select-Object -First 1
                if ($candidate) { $candidatePrevTag = $candidate.tagName }
            } catch {
                Write-Host "::warning::Failed to parse 'gh release list' output: $_."
            }

            if ($candidatePrevTag) {
                $orphanSha = & git rev-parse $candidatePrevTag 2>$null
                if ($LASTEXITCODE -eq 0 -and $orphanSha) {
                    $orphanSha = $orphanSha.Trim()
                    $orphanSubject = & git show -s --format=%s $orphanSha 2>$null
                    if ($LASTEXITCODE -eq 0 -and $orphanSubject) {
                        $rebasedSha = $null
                        $logLines = & git log $Tag --format='%H%x09%s' 2>$null
                        if ($LASTEXITCODE -eq 0 -and $logLines) {
                            $lineArr = if ($logLines -is [array]) { $logLines } else { ,$logLines }
                            foreach ($line in $lineArr) {
                                if (-not $line) { continue }
                                $parts = $line -split "`t", 2
                                if ($parts.Count -eq 2 -and $parts[1] -eq $orphanSubject) {
                                    $rebasedSha = $parts[0]
                                    break
                                }
                            }
                        }
                        if ($rebasedSha) {
                            $shortSha = $rebasedSha.Substring(0, 12)
                            Write-Host "::warning::Subject-matched slice: prev tag $candidatePrevTag (orphan sha $($orphanSha.Substring(0,12))) matches current-history sha $shortSha by subject; using $shortSha..$Tag."
                            return @{
                                Tag     = $candidatePrevTag
                                LogArgs = @("$rebasedSha..$Tag")
                                Display = "$candidatePrevTag..$Tag (subject-matched at $shortSha)"
                                Source  = 'subject-match'
                            }
                        }
                        Write-Host "::warning::Prev tag $candidatePrevTag subject '$orphanSubject' not found in current $Tag history. Falling back to root walk."
                    }
                }
            }
        } else {
            Write-Host "::warning::'gh release list' produced no usable output (gh not authed or no releases yet). Falling back to root walk."
        }
    }

    # Layer 3: root walk.
    $root = (& git rev-list --max-parents=0 HEAD | Select-Object -First 1).Trim()
    Write-Host "::warning::No prior tag matched; walking from root $root."
    return @{
        Tag     = $null
        LogArgs = @("$root..$Tag")
        Display = "$root..$Tag (root walk)"
        Source  = 'root'
    }
}

$prevInfo = Resolve-PrevTagForSlice -Tag $Tag -Repo $Repo
$prevTag  = $prevInfo.Tag
$logArgs  = $prevInfo.LogArgs
$range    = $prevInfo.Display

# %H = full sha, %h = short sha, %an = author name, %s = subject. Tabs as field
# separator are safe -- git rejects literal tabs in author names and subjects
# don't contain them in any of these repos.
$raw = & git log @logArgs --no-merges --pretty=format:"%H`t%h`t%an`t%s" 2>$null
if ($LASTEXITCODE -ne 0) { $raw = @() }

$lines = @()
if ($raw) { $lines = $raw -split "`r?`n" | Where-Object { $_ } }

# Git author.name to GitHub @-handle. Auto-changelog emits "by @<author>" and
# GitHub @-mentions only resolve when the handle is the actual login. Local
# git config uses the brand "WhyKnot" but the GitHub login is "RealWhyKnot".
# Any commit author not in this map passes through unchanged.
$AuthorHandleMap = @{
    'WhyKnot' = 'RealWhyKnot'
}

$entries = foreach ($line in $lines) {
    if ($line -match '\[skip changelog\]') { continue }
    $parts = $line -split "`t", 4
    if ($parts.Count -lt 4) { continue }
    $sha     = $parts[0]
    $short   = $parts[1]
    $author  = $parts[2]
    if ($AuthorHandleMap.ContainsKey($author)) { $author = $AuthorHandleMap[$author] }
    $subject = $parts[3]

    # Strip embedded version-stamp noise like " (2026.4.30.13-EB4B)". Some
    # repos append it mid-subject before a trailing PR ref like " (#42)".
    $subject = $subject -replace '\s*\(\d{4}\.\d+\.\d+\.\d+-[A-Fa-f0-9]+\)\s*',' '
    $subject = $subject.Trim() -replace '\s{2,}',' '

    [pscustomobject]@{
        Sha     = $sha
        Short   = $short
        Author  = $author
        Subject = $subject
    }
}

# Empty-slice guard. A release with no qualifying commits in the tag range
# means either the prev-tag detection is wrong, every commit was skipped,
# or the tag was pushed from an empty branch. All three are operator
# mistakes worth catching before publish.
if (-not $entries -or $entries.Count -eq 0) {
    if ($AllowEmpty) {
        # First-release escape hatch. Emit a stub body the workflow can still
        # publish; downstream consumers can reformat or replace.
        return "## What's Changed`n`n_First release; see commit log for details._`n"
    }
    throw "No commits found in range $range. " +
          "Either the previous tag is misdetected, every commit in the range " +
          "carries [skip changelog], or the tag points at an empty branch. " +
          "Pass -AllowEmpty for a first release. Otherwise amend the offending " +
          "commits or push a real change before tagging."
}

function Get-Category([string] $subject) {
    if ($subject -match '^feat(\(.+?\))?!?:')     { return @{ Order = 1;  Name = 'Features' } }
    if ($subject -match '^fix(\(.+?\))?!?:')      { return @{ Order = 2;  Name = 'Bug Fixes' } }
    if ($subject -match '^perf(\(.+?\))?!?:')     { return @{ Order = 3;  Name = 'Performance' } }
    if ($subject -match '^refactor(\(.+?\))?!?:') { return @{ Order = 4;  Name = 'Refactors' } }
    if ($subject -match '^revert(\(.+?\))?!?:')   { return @{ Order = 5;  Name = 'Reverts' } }
    if ($subject -match '^docs(\(.+?\))?!?:')     { return @{ Order = 6;  Name = 'Documentation' } }
    if ($subject -match '^style(\(.+?\))?!?:')    { return @{ Order = 7;  Name = 'Style' } }
    if ($subject -match '^test(\(.+?\))?!?:')     { return @{ Order = 8;  Name = 'Tests' } }
    if ($subject -match '^ci(\(.+?\))?!?:')       { return @{ Order = 9;  Name = 'CI' } }
    if ($subject -match '^build(\(.+?\))?!?:')    { return @{ Order = 10; Name = 'Build' } }
    if ($subject -match '^chore(\(.+?\))?!?:')    { return @{ Order = 11; Name = 'Chores' } }
    return @{ Order = 99; Name = 'Other Changes' }
}

# Conventional-commit coverage warning. Don't fail -- 'Other Changes' is the
# documented bucket for non-conforming subjects -- but log to stderr so the
# operator sees them in the workflow output and can amend if desired.
$nonConforming = @($entries | Where-Object {
    $_.Subject -notmatch '^(feat|fix|perf|refactor|revert|docs|style|test|ci|build|chore)(\(.+?\))?!?:'
})
if ($nonConforming.Count -gt 0) {
    Write-Host "::warning::$($nonConforming.Count) commit(s) in range $range do not follow conventional-commit prefixes; bucketed under 'Other Changes':"
    foreach ($e in $nonConforming) {
        Write-Host "::warning::  $($e.Short)  $($e.Subject)"
    }
}

$useGroups = $false
foreach ($e in $entries) {
    if ($e.Subject -match '^(feat|fix|perf|refactor|revert|docs|style|test|ci|build|chore)(\(.+?\))?!?:') {
        $useGroups = $true
        break
    }
}

# Token substitution map. Templates and emitted sections reference these by
# {key} string (literal curly braces in the template). Any value the resolver
# could not compute is left as the literal {key} in the output so the operator
# sees the omission rather than blank space.
$ownerOnly = ''
$repoShort = ''
if ($Repo -and ($Repo -match '/')) {
    $parts = $Repo -split '/', 2
    $ownerOnly = $parts[0]
    $repoShort = $parts[1]
} elseif ($Repo) {
    $repoShort = $Repo
}
$tagCommitSha = ''
$tagCommitShort = ''
$tagSha = & git rev-parse $Tag 2>$null
if ($LASTEXITCODE -eq 0 -and $tagSha) {
    $tagCommitSha = $tagSha.Trim()
    if ($tagCommitSha.Length -ge 12) { $tagCommitShort = $tagCommitSha.Substring(0, 12) }
}
$priorTagToken = if ($prevTag) { $prevTag } else { '' }
$zipNameToken  = if ($ZipName) { $ZipName } elseif ($ZipPath) { (Split-Path -Leaf $ZipPath) } else { '' }
$tokens = @{
    '{tag}'              = $Tag
    '{version}'          = ($Tag -replace '^v', '')
    '{owner}'            = $ownerOnly
    '{repo}'             = $repoShort
    '{full-repo}'        = $Repo
    '{commit-sha}'       = $tagCommitSha
    '{commit-sha-short}' = $tagCommitShort
    '{prior-tag}'        = $priorTagToken
    '{zip-name}'         = $zipNameToken
}

function Expand-Tokens([string] $text, [hashtable] $map) {
    if (-not $text) { return $text }
    foreach ($key in $map.Keys) {
        $val = $map[$key]
        if ($null -eq $val) { $val = '' }
        $text = $text.Replace($key, $val)
    }
    return $text
}

function Format-Bytes([long] $bytes) {
    if ($bytes -ge 1MB) { return ('{0:F2} MB' -f ($bytes / 1MB)) }
    if ($bytes -ge 1KB) { return ('{0:F2} KB' -f ($bytes / 1KB)) }
    return ('{0} B' -f $bytes)
}

function Read-TemplateSection([string] $name, [string] $dir, [hashtable] $tokenMap) {
    $path = Join-Path -Path $dir -ChildPath "$name.md"
    if (-not (Test-Path -LiteralPath $path)) {
        Write-Host "::warning::Release-body template missing: $path. Section '$name' will not render."
        return $null
    }
    $rawContent = Get-Content -LiteralPath $path -Raw -Encoding UTF8
    if ($null -eq $rawContent) { return $null }
    $content = $rawContent.Trim()
    if (-not $content) { return $null }
    return (Expand-Tokens -text $content -map $tokenMap)
}

$sb = [System.Text.StringBuilder]::new()
if ($repoShort) {
    [void]$sb.AppendLine("# $repoShort $Tag")
    [void]$sb.AppendLine()
}
[void]$sb.AppendLine("## What's Changed")
[void]$sb.AppendLine()

if ($useGroups) {
    $tagged = foreach ($e in $entries) {
        $cat = Get-Category $e.Subject
        [pscustomobject]@{ Order = $cat.Order; Name = $cat.Name; Entry = $e }
    }
    $groups = $tagged | Group-Object Name | Sort-Object { ($_.Group | Select-Object -First 1).Order }
    foreach ($g in $groups) {
        [void]$sb.AppendLine("### $($g.Name)")
        foreach ($t in $g.Group) {
            $e = $t.Entry
            [void]$sb.AppendLine("- $($e.Subject) by @$($e.Author) in $($e.Short)")
        }
        [void]$sb.AppendLine()
    }
} else {
    foreach ($e in $entries) {
        [void]$sb.AppendLine("- $($e.Subject) by @$($e.Author) in $($e.Short)")
    }
    [void]$sb.AppendLine()
}

if ($Repo -and $prevTag) {
    [void]$sb.AppendLine("**Full Changelog**: https://github.com/$Repo/compare/$prevTag...$Tag")
}

# --- File integrity ---
# Two-row Markdown table covering the umbrella zip + Setup.exe. The zip itself
# is hashed by the Package step in release.yml; the Setup.exe is hashed by the
# Build NSIS installer step. Both pairs (path/size/sha) are passed in. If any
# value is missing (running locally without a build, or the workflow wiring is
# incomplete), the section is skipped with a warning so the operator notices.
$includeIntegrity = $ZipPath -and $ZipSha256 -and $ZipSize -gt 0 `
    -and $SetupPath -and $SetupSha256 -and $SetupSize -gt 0
if ($includeIntegrity) {
    [void]$sb.AppendLine()
    [void]$sb.AppendLine("## File integrity")
    [void]$sb.AppendLine()
    [void]$sb.AppendLine("Verify with ``Get-FileHash <file> -Algorithm SHA256`` on PowerShell.")
    [void]$sb.AppendLine()
    [void]$sb.AppendLine("| File | Size | SHA256 |")
    [void]$sb.AppendLine("|---|---|---|")
    $zipNameForLine   = if ($zipNameToken) { $zipNameToken } else { Split-Path -Leaf $ZipPath }
    $setupNameForLine = if ($SetupName)    { $SetupName }    else { Split-Path -Leaf $SetupPath }
    [void]$sb.AppendLine(("| ``{0}`` | {1} | ``{2}`` |" -f $zipNameForLine,   (Format-Bytes $ZipSize),   $ZipSha256.ToUpper()))
    [void]$sb.AppendLine(("| ``{0}`` | {1} | ``{2}`` |" -f $setupNameForLine, (Format-Bytes $SetupSize), $SetupSha256.ToUpper()))
} elseif ($ZipPath -or $ZipSha256 -or $SetupPath -or $SetupSha256) {
    Write-Host "::warning::File-integrity section skipped: -ZipPath, -ZipSize, -ZipSha256, -SetupPath, -SetupSize, and -SetupSha256 must all be set. Got ZipPath='$ZipPath' ZipSize=$ZipSize ZipSha256='$ZipSha256' SetupPath='$SetupPath' SetupSize=$SetupSize SetupSha256='$SetupSha256'."
}

# --- Templated evergreen sections ---
# Each template is repo-curated content under .github/release-template/<name>.md.
# Read in this fixed order; missing templates emit a warning and skip without
# failing the build. Token substitution happens inside Read-TemplateSection.
$templateOrder = @('links', 'install', 'uninstall', 'what-you-need-to-do', 'please-read')
foreach ($name in $templateOrder) {
    $section = Read-TemplateSection -name $name -dir $TemplateDir -tokenMap $tokens
    if ($section) {
        [void]$sb.AppendLine()
        [void]$sb.AppendLine($section)
    }
}

# Optional extras append. Free-form prose for the rare case where a release
# needs to communicate something that isn't captured by commit subjects --
# server-side coordination notes, migration steps, etc. The file is read
# verbatim so the author has full markdown control; the same ASCII scrub runs
# on the final composed body so non-ASCII characters in the extras fail the
# workflow just as if they were in commit subjects.
if (Test-Path -LiteralPath $Extras) {
    $extrasContent = (Get-Content -LiteralPath $Extras -Raw -Encoding UTF8).Trim()
    if ($extrasContent) {
        [void]$sb.AppendLine()
        [void]$sb.AppendLine("---")
        [void]$sb.AppendLine()
        [void]$sb.AppendLine("## Additional notes")
        [void]$sb.AppendLine()
        [void]$sb.AppendLine($extrasContent)
    }
}

$body = $sb.ToString().TrimEnd()

# ASCII normalisation. Common typographic patterns get substituted to
# their plain-ASCII equivalents. The substitution is one-way and silent;
# a commit subject that contains an em-dash gets emitted with `--` instead.
# After normalisation, anything still non-ASCII fails the scrub gate (so
# we surface the offending byte rather than letting it ship).
# Pattern is stored as a single-char string (not [char]) so the (string,string)
# overload of String.Replace binds cleanly. The (char,char) overload would
# reject a multi-char replacement like '--' or '...'.
$asciiSubs = @(
    @{ Pattern = [string][char]0x2014; Replacement = '--' }        # em-dash
    @{ Pattern = [string][char]0x2013; Replacement = '-'  }        # en-dash
    @{ Pattern = [string][char]0x2026; Replacement = '...' }       # ellipsis
    @{ Pattern = [string][char]0x201C; Replacement = '"'  }        # left double quote
    @{ Pattern = [string][char]0x201D; Replacement = '"'  }        # right double quote
    @{ Pattern = [string][char]0x2018; Replacement = "'"  }        # left single quote
    @{ Pattern = [string][char]0x2019; Replacement = "'"  }        # right single quote
    @{ Pattern = [string][char]0x00A0; Replacement = ' '  }        # non-breaking space
    @{ Pattern = [string][char]0x2022; Replacement = '*'  }        # bullet
    @{ Pattern = [string][char]0x00D7; Replacement = 'x'  }        # multiplication sign
    @{ Pattern = [string][char]0x2192; Replacement = '->' }        # right arrow
    @{ Pattern = [string][char]0x2190; Replacement = '<-' }        # left arrow
    @{ Pattern = [string][char]0x21D2; Replacement = '=>' }        # double right arrow
    @{ Pattern = [string][char]0x21D0; Replacement = '<=' }        # double left arrow
    @{ Pattern = [string][char]0x00A7; Replacement = 'section' }   # section sign
    @{ Pattern = [string][char]0x00B6; Replacement = 'paragraph' } # pilcrow
)
foreach ($sub in $asciiSubs) {
    $body = $body.Replace($sub.Pattern, $sub.Replacement)
}

if (-not $SkipScrub) {
    # Anything outside printable ASCII (plus tab/newline) after the
    # substitution pass fails. Prints the offending line + char code so the
    # operator can find and fix.
    $lineNumber = 0
    $offenders = foreach ($line in ($body -split "`r?`n")) {
        $lineNumber++
        for ($i = 0; $i -lt $line.Length; $i++) {
            $ch = $line[$i]
            $code = [int][char]$ch
            $isAllowed = ($code -ge 0x20 -and $code -le 0x7E) -or $code -eq 9
            if (-not $isAllowed) {
                [pscustomobject]@{
                    Line = $lineNumber
                    Col  = $i + 1
                    Char = $ch
                    Code = ('U+{0:X4}' -f $code)
                    Text = $line
                }
            }
        }
    }
    if ($offenders) {
        $report = $offenders | ForEach-Object { "  line $($_.Line) col $($_.Col): $($_.Code) in: $($_.Text)" }
        throw "Non-ASCII characters in release body after normalisation:`n$($report -join "`n")`n" +
              "Fix: amend the offending commit subject (or extras file) to use ASCII equivalents. " +
              "Common substitutes are pre-mapped in Generate-ReleaseNotes.ps1; if a new character " +
              "trips this, add it to `$asciiSubs and try again."
    }
}

# Single trimmed string so $(...) capture in calling scripts gets clean text.
$body
