#!/usr/bin/env pwsh
<#
.SYNOPSIS
  Static PowerShell-syntax check across every script the release / CI
  pipelines execute.

.DESCRIPTION
  Runs at the very start of release.yml + ci.yml so PowerShell parser
  errors (like the latent `$MaxAttempts:` bug that took down the
  v2026.5.13.0 release after the umbrella had already published) fail
  the workflow in seconds, before the build step runs.

  Two passes:

    1. Every .ps1 under .github/scripts/ is parsed via
       [System.Management.Automation.Language.Parser]::ParseFile.
    2. Every workflow .yml under .github/workflows/ and every composite
       action.yml under .github/actions/ is scanned for `run: |` blocks
       running under `shell: pwsh`. Each block is extracted, ${{ ... }}
       expressions are stubbed with a literal sentinel (so the parser
       sees valid PowerShell), and the result is parsed via ParseInput.

  Either pass reports the file, the step name, the offending line, and
  the parser's own diagnostic. Exit 1 on any error.

  Designed to be cheap: no build dependencies, no network, completes
  in a few seconds on a cold runner.
#>
[CmdletBinding()]
param(
    [string] $Root = (Get-Location).Path
)

$ErrorActionPreference = 'Stop'

$errors = [System.Collections.Generic.List[string]]::new()

function Add-ParserErrors {
    param(
        [string] $Source,
        [Parameter(Mandatory)] $ParseErrors
    )
    if (-not $ParseErrors) { return }
    foreach ($e in $ParseErrors) {
        $line = $e.Extent.StartLineNumber
        $col  = $e.Extent.StartColumnNumber
        $msg  = $e.Message
        $errors.Add("$Source (line ${line}:${col}): $msg") | Out-Null
    }
}

# ---- Pass 1: every committed .ps1 -----------------------------------------

$scriptDir = Join-Path $Root '.github/scripts'
if (Test-Path -LiteralPath $scriptDir) {
    Get-ChildItem -LiteralPath $scriptDir -Filter '*.ps1' -Recurse |
        ForEach-Object {
            $parseErrors = $null
            [void][System.Management.Automation.Language.Parser]::ParseFile(
                $_.FullName, [ref]$null, [ref]$parseErrors)
            Add-ParserErrors -Source $_.FullName -ParseErrors $parseErrors
        }
}

# ---- Pass 2: inline pwsh blocks inside workflow YAML ----------------------
#
# YAML in this repo always uses the literal block scalar `|` for `run:` and
# always indents two spaces per nesting level (the workflows are hand-
# authored to that style). That lets a small state machine extract each
# block without pulling in a YAML parser dependency. The state machine
# tracks the indent of `run: |` and consumes following lines until the
# indent drops to <= that level.
#
# Each block is then sanitised: ${{ ... }} expressions are GitHub Actions
# substitutions evaluated before the runner sees them, so at parse time
# they are not valid PowerShell. Replace each with a quoted sentinel so
# the rest of the block parses normally.

$workflowDir = Join-Path $Root '.github/workflows'
if (Test-Path -LiteralPath $workflowDir) {
    $ghaPattern = [regex]'\$\{\{[^}]*\}\}'
    $retiredReleaseSecret = 'MODULE_RELEASE_TOKEN'
    $nightlyBetaWorkflow = Join-Path $workflowDir 'nightly-beta.yml'

    # Also scan composite actions (.github/actions/**/action.yml): they embed
    # `shell: pwsh` `run: |` blocks too and CI invokes them, so the same parser
    # gate must cover them or a syntax bug there surfaces only at runtime.
    $yamlFiles = @(Get-ChildItem -LiteralPath $workflowDir -Filter '*.yml')
    $actionsDir = Join-Path $Root '.github/actions'
    if (Test-Path -LiteralPath $actionsDir) {
        $yamlFiles += @(Get-ChildItem -LiteralPath $actionsDir -Filter 'action.yml' -Recurse)
    }

    $yamlFiles | ForEach-Object {
        $wfPath  = $_.FullName

        # The retired-secret and beta-tag credential checks are workflow policy;
        # they do not apply to composite actions.
        if ($wfPath.StartsWith($workflowDir, [System.StringComparison]::OrdinalIgnoreCase)) {
            $retiredSecretMatches = Select-String -LiteralPath $wfPath -SimpleMatch $retiredReleaseSecret
            foreach ($match in $retiredSecretMatches) {
                $errors.Add("$wfPath (line $($match.LineNumber)): $retiredReleaseSecret is not a configured repository secret; use MIRROR_RELEASE_TOKEN.") | Out-Null
            }

            if ([string]::Equals($wfPath, $nightlyBetaWorkflow, [System.StringComparison]::OrdinalIgnoreCase)) {
                $workflowText = Get-Content -LiteralPath $wfPath -Raw
                if ($workflowText -match 'git push \$remote' -and $workflowText -notmatch 'persist-credentials:\s*false') {
                    $errors.Add("${wfPath}: checkout must set persist-credentials: false before pushing the beta tag with MIRROR_RELEASE_TOKEN.") | Out-Null
                }
            }
        }

        $lines   = Get-Content -LiteralPath $wfPath
        $stepName    = '<unnamed>'
        $isPwsh      = $false
        $inRun       = $false
        $runIndent   = -1
        $runStartLn  = 0
        $blockLines  = [System.Collections.Generic.List[string]]::new()

        $flushBlock = {
            if ($blockLines.Count -eq 0) { return }
            # Strip the common leading indent so the parser sees the script
            # the way the runner does. Use the indent of the first non-blank
            # line as the baseline.
            $baseline = -1
            foreach ($bl in $blockLines) {
                if ($bl.Trim().Length -eq 0) { continue }
                $strip = $bl.Length - $bl.TrimStart(' ').Length
                $baseline = $strip
                break
            }
            if ($baseline -lt 0) { return }
            $body = ($blockLines | ForEach-Object {
                if ($_.Length -gt $baseline) { $_.Substring($baseline) } else { '' }
            }) -join "`n"
            # GHA expressions are almost always nested inside a quoted PS
            # string (single or double), so the placeholder must be a
            # bareword -- wrapping it in quotes would unbalance the
            # surrounding quotes and trip the parser.
            $stubbed = $ghaPattern.Replace($body, '__GHA_EXPR__')
            $parseErrors = $null
            [void][System.Management.Automation.Language.Parser]::ParseInput(
                $stubbed, [ref]$null, [ref]$parseErrors)
            Add-ParserErrors -Source "$wfPath step '$stepName' (run: starting at line $runStartLn)" -ParseErrors $parseErrors
        }

        for ($i = 0; $i -lt $lines.Count; $i++) {
            $line = $lines[$i]
            $indent = $line.Length - $line.TrimStart(' ').Length
            $trimmed = $line.Trim()

            if ($inRun) {
                if ($trimmed.Length -gt 0 -and $indent -le $runIndent) {
                    & $flushBlock
                    $blockLines.Clear()
                    $inRun = $false
                    # Fall through to re-process this line as a new directive.
                } else {
                    $blockLines.Add($line) | Out-Null
                    continue
                }
            }

            if ($trimmed -match '^- name:\s*(.+?)\s*$') {
                # Starting a new step. Reset shell state and stash the name.
                if ($blockLines.Count -gt 0) { & $flushBlock; $blockLines.Clear(); $inRun = $false }
                $stepName = $Matches[1]
                $isPwsh = $false
                continue
            }

            if ($trimmed -match '^shell:\s*(.+?)\s*$') {
                $isPwsh = ($Matches[1] -eq 'pwsh')
                continue
            }

            if ($isPwsh -and $trimmed -match '^run:\s*\|\s*$') {
                $inRun = $true
                $runIndent = $indent
                $runStartLn = $i + 1
                $blockLines.Clear()
                continue
            }
        }
        if ($inRun) { & $flushBlock }
    }
}

if ($errors.Count -gt 0) {
    Write-Host 'PowerShell syntax errors:'
    foreach ($e in $errors) { Write-Host "  $e" }
    throw "Found $($errors.Count) syntax error(s) across release / CI scripts."
}

Write-Host "All PowerShell scripts and workflow inline pwsh blocks parsed cleanly."
