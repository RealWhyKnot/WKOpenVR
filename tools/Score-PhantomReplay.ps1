#Requires -Version 5.1
<#
.SYNOPSIS
  Score a phantom_replay capture offline to tune Phantom auto role-detection
  without re-entering VR.

.DESCRIPTION
  Replays a recorded phantom_replay_v1 CSV (HMD + controllers + trackers, with a
  ground-truth body_role column) through the same role inference + snap the driver
  runs, and reports per-tracker predicted-vs-truth + overall accuracy. The driver
  records these CSVs to %LocalAppDataLow%\WKOpenVR\Logs automatically whenever
  debug logging is on or WKOPENVR_PHANTOM_REPLAY_RECORD=1.

  Iteration loop: capture once in VR -> -Pin it -> tweak thresholds in the
  PassiveRoleInference/SnapCalibrate/RoleArbiter headers -> re-run with -Build ->
  read the accuracy -> repeat. No VR needed between iterations.

.PARAMETER Csv
  Capture to score. Defaults to the pinned capture (.local\phantom_capture.csv),
  then the newest phantom_replay*.csv in the WKOpenVR Logs folder.

.PARAMETER Build
  Rebuild the sidecar (Release) first so header/threshold changes take effect.

.PARAMETER Pin
  Copy the resolved capture to .local\phantom_capture.csv so later runs reuse the
  exact same data.

.EXAMPLE
./Score-PhantomReplay.ps1 -Build
#>
[CmdletBinding()]
param(
    [string]$Csv = "",
    [switch]$Build,
    [switch]$Pin
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$sidecar = Join-Path $repo "build\artifacts\Release\WKOpenVRPhantomSidecar.exe"
$pinned = Join-Path $repo ".local\phantom_capture.csv"
$logsDir = Join-Path $env:USERPROFILE "AppData\LocalLow\WKOpenVR\Logs"

if ($Build) {
    Write-Host "Building WKOpenVRPhantomSidecar (Release)..."
    & cmake --build (Join-Path $repo "build") --config Release --target WKOpenVRPhantomSidecar
    if ($LASTEXITCODE -ne 0) { throw "sidecar build failed (exit $LASTEXITCODE)" }
}

if (-not (Test-Path -LiteralPath $sidecar)) {
    throw "sidecar not built: $sidecar (run with -Build, or build the WKOpenVRPhantomSidecar target)"
}

if ([string]::IsNullOrEmpty($Csv)) {
    if (Test-Path -LiteralPath $pinned) {
        $Csv = $pinned
    }
    elseif (Test-Path -LiteralPath $logsDir) {
        $latest = Get-ChildItem -LiteralPath $logsDir -Filter "phantom_replay*.csv" -File -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if ($null -ne $latest) { $Csv = $latest.FullName }
    }
}

if ([string]::IsNullOrEmpty($Csv) -or -not (Test-Path -LiteralPath $Csv)) {
    Write-Host "No capture found. Get into VR with Phantom enabled + trackers worn, assign the"
    Write-Host "correct roles in the overlay, move around ~60s, then re-run. Logs land in:"
    Write-Host "  $logsDir\phantom_replay.<timestamp>.csv"
    exit 2
}

if ($Pin) {
    $localDir = Join-Path $repo ".local"
    if (-not (Test-Path -LiteralPath $localDir)) { New-Item -ItemType Directory -Path $localDir | Out-Null }
    Copy-Item -LiteralPath $Csv -Destination $pinned -Force
    Write-Host "Pinned capture -> $pinned"
    $Csv = $pinned
}

Write-Host "Scoring $Csv"
& $sidecar --score-replay $Csv
exit $LASTEXITCODE
