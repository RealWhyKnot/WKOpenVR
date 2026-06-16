#!/usr/bin/env pwsh
<#
.SYNOPSIS
  Updates a module release-page CHANGELOG.md for a published installer.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string] $ChangelogPath,

    [Parameter(Mandatory = $true)]
    [string] $DisplayName,

    [Parameter(Mandatory = $true)]
    [string] $Tag,

    [Parameter(Mandatory = $true)]
    [string] $Version,

    [Parameter(Mandatory = $true)]
    [string] $InstallerName,

    [Parameter(Mandatory = $true)]
    [string] $Sha256,

    [Parameter(Mandatory = $true)]
    [string] $IntegrityName,

    [Parameter(Mandatory = $true)]
    [string] $ReleaseUrl,

    [string] $ReleaseDate = '',
    [datetime] $NowUtc = ([datetime]::UtcNow)
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$Utf8NoBom = New-Object System.Text.UTF8Encoding($false)

function Read-TextUtf8 {
    param([string] $Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        return ''
    }
    return [System.IO.File]::ReadAllText($Path, $Utf8NoBom)
}

function Write-TextUtf8 {
    param([string] $Path, [string] $Content)

    $parent = Split-Path -Parent $Path
    if ($parent -and -not (Test-Path -LiteralPath $parent)) {
        New-Item -ItemType Directory -Force -Path $parent | Out-Null
    }
    [System.IO.File]::WriteAllText($Path, $Content, $Utf8NoBom)
}

function Get-CentralTimeZone {
    foreach ($id in @('Central Standard Time', 'America/Chicago')) {
        try { return [System.TimeZoneInfo]::FindSystemTimeZoneById($id) }
        catch { }
    }
    throw 'Could not resolve the America/Chicago release time zone.'
}

function Get-ReleaseDateStamp {
    param([datetime] $NowUtc, [string] $Format)

    $utc = $NowUtc
    if ($utc.Kind -ne [System.DateTimeKind]::Utc) {
        $utc = $utc.ToUniversalTime()
    }
    $central = [System.TimeZoneInfo]::ConvertTimeFromUtc($utc, (Get-CentralTimeZone))
    return $central.ToString($Format, [System.Globalization.CultureInfo]::InvariantCulture)
}

function New-SeedChangelog {
    param([string] $Name)

    return @(
        '# Changelog',
        '',
        "All notable user-visible releases for WKOpenVR $Name. Release entries are published by the WKOpenVR source repository when installer artifacts are created.",
        '',
        '## Unreleased',
        '',
        '_No notable changes since the last release._',
        '',
        '---',
        ''
    ) -join "`n"
}

function New-ReleaseSection {
    return @(
        "## $Tag -- $ReleaseDate",
        '',
        '### Released',
        "- Published $DisplayName installer for WKOpenVR $Version.",
        "- Installer: ``$InstallerName``",
        "- Integrity: ``$IntegrityName``",
        "- Release: <$ReleaseUrl>",
        '',
        '---',
        ''
    )
}

function Find-UnreleasedSeparatorIndex {
    param([string[]] $Lines)

    $unreleased = -1
    for ($i = 0; $i -lt $Lines.Length; $i++) {
        if ($Lines[$i] -match '^##\s+Unreleased\s*$') {
            $unreleased = $i
            break
        }
    }
    if ($unreleased -lt 0) { return -1 }

    for ($i = $unreleased + 1; $i -lt $Lines.Length; $i++) {
        if ($Lines[$i] -match '^---\s*$') {
            return $i
        }
        if ($Lines[$i] -match '^##\s+') {
            return ($i - 1)
        }
    }
    return ($Lines.Length - 1)
}

function Find-TaggedSectionRange {
    param([string[]] $Lines, [string] $ReleaseTag)

    $escapedTag = [regex]::Escape($ReleaseTag)
    $start = -1
    for ($i = 0; $i -lt $Lines.Length; $i++) {
        if ($Lines[$i] -match "^##\s+$escapedTag(?:\s+--.*)?\s*$") {
            $start = $i
            break
        }
    }
    if ($start -lt 0) { return $null }

    $end = $Lines.Length - 1
    for ($i = $start + 1; $i -lt $Lines.Length; $i++) {
        if ($Lines[$i] -match '^##\s+') {
            $end = $i - 1
            break
        }
    }

    return @{ Start = $start; End = $end }
}

if ([string]::IsNullOrWhiteSpace($ReleaseDate)) {
    $ReleaseDate = Get-ReleaseDateStamp -NowUtc $NowUtc -Format 'yyyy-MM-dd'
}

$content = Read-TextUtf8 -Path $ChangelogPath
if ($content -notmatch '(?m)^##\s+Unreleased\s*$') {
    $content = New-SeedChangelog -Name $DisplayName
}

$lines = @($content -replace "`r`n", "`n" -replace "`r", "`n" -split "`n")
$sectionLines = New-ReleaseSection
$existing = Find-TaggedSectionRange -Lines $lines -ReleaseTag $Tag

if ($existing) {
    $before = if ($existing.Start -gt 0) { $lines[0..($existing.Start - 1)] } else { @() }
    $after = if ($existing.End -lt ($lines.Length - 1)) { $lines[($existing.End + 1)..($lines.Length - 1)] } else { @() }
    $lines = @($before + $sectionLines + $after)
} else {
    $insertAfter = Find-UnreleasedSeparatorIndex -Lines $lines
    if ($insertAfter -lt 0) {
        $content = New-SeedChangelog -Name $DisplayName
        $lines = @($content -split "`n")
        $insertAfter = Find-UnreleasedSeparatorIndex -Lines $lines
    }

    $before = if ($insertAfter -ge 0) { $lines[0..$insertAfter] } else { $lines }
    $after = if ($insertAfter -lt ($lines.Length - 1)) { $lines[($insertAfter + 1)..($lines.Length - 1)] } else { @() }
    $lines = @($before + '' + $sectionLines + $after)
}

$newContent = (($lines -join "`n") -replace "(`n){3,}", "`n`n").TrimEnd() + "`n"
Write-TextUtf8 -Path $ChangelogPath -Content $newContent
Write-Host "Updated $ChangelogPath for $DisplayName $Tag."
