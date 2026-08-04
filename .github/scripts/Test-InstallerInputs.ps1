#!/usr/bin/env pwsh
<#
.SYNOPSIS
  Verifies every build artifact installer.nsi packages is present.

.DESCRIPTION
  makensis errors with an obscure "no files found" when a File line points at
  a missing artifact, and a stale build cache can mask a removed artifact for
  weeks. This list is the single source of truth for the installer's inputs:
  release.yml runs it before makensis on tags, and ci.yml's release leg runs
  it on every push so a payload gap fails before a tag ever exists.

  When installer.nsi gains or loses a File line, update this list in the same
  commit. The installer round-trip test then verifies the installed result.
#>
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$required = [System.Collections.Generic.List[string]]@(
    'build/artifacts/Release/WKOpenVR.exe',
    'build/artifacts/Release/openvr_api.dll',
    'build/artifacts/Release/manifest.vrmanifest',
    'build/artifacts/Release/dashboard_icon.png',
    'build/artifacts/Release/resources/face-module-sync.ps1',
    'build/artifacts/Release/resources/questapp/install-platform-tools.ps1',
    'build/artifacts/Release/resources/questapp/uninstall-questapp.ps1',
    'build/artifacts/Release/resources/questapp/WKOpenVRQuestCompanion.apk',
    'build/driver_wkopenvr/driver.vrdrivermanifest',
    'build/driver_wkopenvr/resources/driver.vrresources',
    'build/driver_wkopenvr/resources/settings/default.vrsettings',
    'build/driver_wkopenvr/bin/win64/driver_wkopenvr.dll'
)

# Per-module artifacts: only verify the ones whose modules are actually built
# into this release (the disabled-in-release marker tells the CMake helper to
# skip add_subdirectory for those, so their host exes / input profiles never
# get produced).
if (-not (Test-Path -LiteralPath 'modules/facetracking/disabled-in-release.flag')) {
    $required.Add('build/driver_wkopenvr/resources/facetracking/host/WKOpenVR.FaceModuleHost.exe')
}
if (-not (Test-Path -LiteralPath 'modules/captions/disabled-in-release.flag')) {
    $required.Add('build/driver_wkopenvr/resources/captions/host/WKOpenVR.CaptionsHost.exe')
    $required.Add('build/driver_wkopenvr/resources/captions/host/openvr_api.dll')
    $required.Add('build/driver_wkopenvr/resources/captions/host/resources/captions-packs.json')
    $required.Add('build/driver_wkopenvr/resources/captions/host/resources/install-captions-pack.ps1')
}
if (-not (Test-Path -LiteralPath 'modules/phantom/disabled-in-release.flag')) {
    $required.Add('build/driver_wkopenvr/resources/phantom/host/WKOpenVRPhantomSidecar.exe')
    $required.Add('build/driver_wkopenvr/resources/input/vive_tracker_waist_profile.json')
    $required.Add('build/driver_wkopenvr/resources/input/vive_tracker_left_foot_profile.json')
    $required.Add('build/driver_wkopenvr/resources/input/vive_tracker_right_foot_profile.json')
}

$missing = @()
foreach ($p in $required) {
    if (-not (Test-Path -LiteralPath $p)) { $missing += $p }
}
if ($missing.Count -gt 0) {
    Write-Host "Missing artifacts required by installer.nsi:"
    $missing | ForEach-Object { Write-Host "  - $_" }
    throw "NSIS input artifacts are not fully staged. The CMake build did not produce one or more required files."
}
Write-Host "All NSIS input artifacts present."
