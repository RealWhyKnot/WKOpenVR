#!/usr/bin/env pwsh

$ErrorActionPreference = 'Stop'

$ScriptRoot = Split-Path -Parent $PSCommandPath
$Updater = Join-Path $ScriptRoot 'Update-ModuleReleaseChangelog.ps1'
$TempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("WKOpenVR.ModuleReleaseChangelog." + [Guid]::NewGuid().ToString('N'))
$Utf8NoBom = New-Object System.Text.UTF8Encoding($false)

function Read-TestText {
    param([string] $Path)
    return [System.IO.File]::ReadAllText($Path, $Utf8NoBom)
}

function Assert-Contains {
    param([string] $Text, [string] $Expected)
    if (-not $Text.Contains($Expected)) {
        throw "Expected text to contain '$Expected'."
    }
}

function Assert-NotContains {
    param([string] $Text, [string] $Unexpected)
    if ($Text.Contains($Unexpected)) {
        throw "Expected text not to contain '$Unexpected'."
    }
}

try {
    New-Item -ItemType Directory -Force -Path $TempRoot | Out-Null
    $changelog = Join-Path $TempRoot 'CHANGELOG.md'

    & $Updater `
        -ChangelogPath $changelog `
        -DisplayName 'Smoothing' `
        -Tag 'v2026.6.15.0' `
        -Version '2026.6.15.0' `
        -InstallerName 'WKOpenVR-Smoothing-v2026.6.15.0-Setup.exe' `
        -Sha256 'ABCDEF1234' `
        -IntegrityName 'WKOpenVR-Smoothing-v2026.6.15.0.integrity.tsv' `
        -ReleaseUrl 'https://github.com/RealWhyKnot/WKOpenVR-Smoothing/releases/tag/v2026.6.15.0' `
        -NowUtc ([datetime]::Parse('2026-06-16T01:30:00Z'))
    if (-not $?) { throw "Initial changelog update failed." }

    $text = Read-TestText -Path $changelog
    Assert-Contains -Text $text -Expected '# Changelog'
    Assert-Contains -Text $text -Expected '## Unreleased'
    Assert-Contains -Text $text -Expected '## v2026.6.15.0 -- 2026-06-15'
    Assert-Contains -Text $text -Expected 'WKOpenVR-Smoothing-v2026.6.15.0-Setup.exe'
    Assert-Contains -Text $text -Expected 'WKOpenVR-Smoothing-v2026.6.15.0.integrity.tsv'
    Assert-NotContains -Text $text -Unexpected 'ABCDEF1234'

    & $Updater `
        -ChangelogPath $changelog `
        -DisplayName 'Smoothing' `
        -Tag 'v2026.6.15.0' `
        -Version '2026.6.15.0' `
        -InstallerName 'WKOpenVR-Smoothing-v2026.6.15.0-Setup.exe' `
        -Sha256 '9999999999' `
        -IntegrityName 'WKOpenVR-Smoothing-v2026.6.15.0.integrity.tsv' `
        -ReleaseUrl 'https://github.com/RealWhyKnot/WKOpenVR-Smoothing/releases/tag/v2026.6.15.0' `
        -NowUtc ([datetime]::Parse('2026-06-16T01:30:00Z'))
    if (-not $?) { throw "Idempotent changelog update failed." }

    $text = Read-TestText -Path $changelog
    Assert-NotContains -Text $text -Unexpected '9999999999'
    Assert-NotContains -Text $text -Unexpected 'abcdef1234'

    & $Updater `
        -ChangelogPath $changelog `
        -DisplayName 'Smoothing' `
        -Tag 'v2026.6.16.0' `
        -Version '2026.6.16.0' `
        -InstallerName 'WKOpenVR-Smoothing-v2026.6.16.0-Setup.exe' `
        -Sha256 '1111111111' `
        -IntegrityName 'WKOpenVR-Smoothing-v2026.6.16.0.integrity.tsv' `
        -ReleaseUrl 'https://github.com/RealWhyKnot/WKOpenVR-Smoothing/releases/tag/v2026.6.16.0' `
        -ReleaseDate '2026-06-16'
    if (-not $?) { throw "Second changelog update failed." }

    $text = Read-TestText -Path $changelog
    if ($text.IndexOf('## v2026.6.16.0 -- 2026-06-16') -gt $text.IndexOf('## v2026.6.15.0 -- 2026-06-15')) {
        throw 'Latest release section should be inserted above older releases.'
    }

    Write-Host 'Module release changelog tests passed.'
}
finally {
    if (Test-Path -LiteralPath $TempRoot) {
        Remove-Item -LiteralPath $TempRoot -Recurse -Force
    }
}
