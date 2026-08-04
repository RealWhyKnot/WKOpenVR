#!/usr/bin/env pwsh
<#
.SYNOPSIS
  Single source of truth for the installer payload: build artifact -> installed path.

.DESCRIPTION
  Default mode verifies every build artifact installer.nsi packages is
  present. makensis errors with an obscure "no files found" when a File line
  points at a missing artifact, and a stale build cache can mask a removed
  artifact for weeks: release.yml runs this before makensis on tags, and
  ci.yml's release leg runs it on every push so a payload gap fails before a
  tag ever exists.

  -Manifest emits the payload table instead of checking the filesystem.
  Test-InstallerRoundTrip.ps1 derives its installed-state expectations from
  it, so the artifact list, the installer inputs, and the round-trip
  expectations cannot drift apart -- a payload change is one edit here plus
  the matching installer.nsi File/Delete lines.

  Entry fields:
    Artifact  - repo-relative build output installer.nsi packages.
    Installed - installed location as '<root>:<relative path>', where root is
                'app' (the $INSTDIR tree) or 'driver' (the SteamVR driver
                dir). $null when the round-trip does not assert it (module
                host trees whose presence depends on install-time flags).
    Group     - 'core' or 'questapp'; the round-trip's Core payload skips
                the questapp group.
#>
[CmdletBinding()]
param(
    [switch] $Manifest
)

$ErrorActionPreference = 'Stop'

$entries = @(
    [pscustomobject]@{ Artifact = 'build/artifacts/Release/WKOpenVR.exe';       Installed = 'app:WKOpenVR.exe';       Group = 'core' }
    [pscustomobject]@{ Artifact = 'build/artifacts/Release/openvr_api.dll';     Installed = 'app:openvr_api.dll';     Group = 'core' }
    [pscustomobject]@{ Artifact = 'build/artifacts/Release/manifest.vrmanifest'; Installed = 'app:manifest.vrmanifest'; Group = 'core' }
    [pscustomobject]@{ Artifact = 'build/artifacts/Release/dashboard_icon.png'; Installed = 'app:dashboard_icon.png'; Group = 'core' }
    [pscustomobject]@{ Artifact = 'build/artifacts/Release/resources/face-module-sync.ps1'; Installed = 'app:resources\face-module-sync.ps1'; Group = 'core' }
    [pscustomobject]@{ Artifact = 'build/artifacts/Release/resources/questapp/install-platform-tools.ps1'; Installed = 'app:resources\questapp\install-platform-tools.ps1'; Group = 'questapp' }
    [pscustomobject]@{ Artifact = 'build/artifacts/Release/resources/questapp/uninstall-questapp.ps1'; Installed = 'app:resources\questapp\uninstall-questapp.ps1'; Group = 'questapp' }
    [pscustomobject]@{ Artifact = 'build/artifacts/Release/resources/questapp/WKOpenVRQuestCompanion.apk'; Installed = 'app:resources\questapp\WKOpenVRQuestCompanion.apk'; Group = 'questapp' }
    [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/driver.vrdrivermanifest'; Installed = 'driver:driver.vrdrivermanifest'; Group = 'core' }
    [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/driver.vrresources'; Installed = 'driver:resources\driver.vrresources'; Group = 'core' }
    [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/settings/default.vrsettings'; Installed = 'driver:resources\settings\default.vrsettings'; Group = 'core' }
    # installer.nsi renames the DLL so SteamVR's alphabetic sort loads it last.
    [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/bin/win64/driver_wkopenvr.dll'; Installed = 'driver:bin\win64\driver_01wkopenvr.dll'; Group = 'core' }
)

# Per-module artifacts: only required when the module is built into this
# release (the disabled-in-release marker tells the CMake helper to skip
# add_subdirectory, so their host exes / input profiles never get produced).
# Installed = $null: the umbrella installer stages these behind install-time
# feature flags, so the round-trip does not assert their installed location.
if (-not (Test-Path -LiteralPath 'modules/facetracking/disabled-in-release.flag')) {
    $entries += [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/facetracking/host/WKOpenVR.FaceModuleHost.exe'; Installed = $null; Group = 'core' }
}
if (-not (Test-Path -LiteralPath 'modules/captions/disabled-in-release.flag')) {
    $entries += @(
        [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/captions/host/WKOpenVR.CaptionsHost.exe'; Installed = $null; Group = 'core' }
        [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/captions/host/openvr_api.dll'; Installed = $null; Group = 'core' }
        [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/captions/host/resources/captions-packs.json'; Installed = $null; Group = 'core' }
        [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/captions/host/resources/install-captions-pack.ps1'; Installed = $null; Group = 'core' }
    )
}
if (-not (Test-Path -LiteralPath 'modules/phantom/disabled-in-release.flag')) {
    $entries += @(
        [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/phantom/host/WKOpenVRPhantomSidecar.exe'; Installed = $null; Group = 'core' }
        [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/input/vive_tracker_waist_profile.json'; Installed = $null; Group = 'core' }
        [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/input/vive_tracker_left_foot_profile.json'; Installed = $null; Group = 'core' }
        [pscustomobject]@{ Artifact = 'build/driver_wkopenvr/resources/input/vive_tracker_right_foot_profile.json'; Installed = $null; Group = 'core' }
    )
}

if ($Manifest) {
    return $entries
}

$missing = @()
foreach ($entry in $entries) {
    if (-not (Test-Path -LiteralPath $entry.Artifact)) { $missing += $entry.Artifact }
}
if ($missing.Count -gt 0) {
    Write-Host "Missing artifacts required by installer.nsi:"
    $missing | ForEach-Object { Write-Host "  - $_" }
    throw "NSIS input artifacts are not fully staged. The CMake build did not produce one or more required files."
}
Write-Host "All NSIS input artifacts present."
