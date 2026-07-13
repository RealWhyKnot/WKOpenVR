#Requires -Version 5.1
<#
.SYNOPSIS
Measures per-device pose jitter from recorded spacecal_log sessions.
.DESCRIPTION
Estimates angular (sigma_theta) and translational (sigma_jit) noise from the
stationary stretches of recorded sessions. These are the parameters of the
lever-arm covariance sample weighting; the medians across recordings feed the
defaults in LeverArmCovariance.h and the profile knobs.
.EXAMPLE
./Measure-TrackerNoise.ps1 -SkipBuild
#>
[CmdletBinding()]
param(
	# spacecal_log recordings to measure. Defaults to every pinned
	# spacecal_log capture in Logs\corpus.
	[string[]]$RecordingPath = @(),

	# Run the current test binary without rebuilding first.
	[switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot

function Get-DefaultLogDir {
	if ($env:LOCALAPPDATALOW) {
		return (Join-Path $env:LOCALAPPDATALOW "WKOpenVR\Logs")
	}
	if ($env:LOCALAPPDATA) {
		$LocalParent = [System.IO.Directory]::GetParent($env:LOCALAPPDATA)
		if ($LocalParent) {
			return (Join-Path $LocalParent.FullName "LocalLow\WKOpenVR\Logs")
		}
	}
	throw "Unable to resolve the WKOpenVR logs directory; pass -RecordingPath."
}

if ($RecordingPath.Count -eq 0) {
	$Corpus = Join-Path (Get-DefaultLogDir) "corpus"
	$RecordingPath = @(Get-ChildItem (Join-Path $Corpus "spacecal_log.*.txt") -ErrorAction SilentlyContinue |
		Sort-Object Name | ForEach-Object { $_.FullName })
	if ($RecordingPath.Count -eq 0) { throw "No spacecal_log capture in $Corpus; pass -RecordingPath." }
}
foreach ($p in $RecordingPath) {
	if (-not (Test-Path -LiteralPath $p)) { throw "Recording not found: $p" }
}

if (-not $SkipBuild) {
	& (Join-Path $RepoRoot "build.ps1")
	if ($LASTEXITCODE -ne 0) { throw "build failed" }
}

$TestExe = Join-Path $RepoRoot "build\artifacts\Release\spacecal_tests.exe"
if (-not (Test-Path $TestExe)) { throw "missing $TestExe; run without -SkipBuild" }

$env:WKOPENVR_NOISE_ESTIMATE = "1"
$env:WKOPENVR_REPLAY_PATHS = ($RecordingPath -join ";")
try {
	$OutputLines = & $TestExe --gtest_filter=LeverArmNoiseTest.EstimateFromRecordingsWhenRequested 2>&1 |
		ForEach-Object { "$_" }
	if ($LASTEXITCODE -ne 0) { throw "spacecal_tests noise estimate failed (exit $LASTEXITCODE)" }
}
finally {
	Remove-Item Env:WKOPENVR_NOISE_ESTIMATE -ErrorAction SilentlyContinue
	Remove-Item Env:WKOPENVR_REPLAY_PATHS -ErrorAction SilentlyContinue
}

$Lines = @($OutputLines | Where-Object { $_ -match '^\[noise-estimate\] ' })
if ($Lines.Count -eq 0) { throw "No [noise-estimate] lines in test output." }
$Lines | ForEach-Object { Write-Host $_ }

# Median across recordings, per metric.
function Get-Median([double[]]$Values) {
	$Sorted = $Values | Sort-Object
	return $Sorted[[int][Math]::Floor(($Sorted.Count - 1) / 2)]
}

$Keys = @("ref_sigma_theta_rad", "ref_sigma_jit_mm", "tgt_sigma_theta_rad", "tgt_sigma_jit_mm")
$Collected = @{}
foreach ($k in $Keys) { $Collected[$k] = @() }
foreach ($line in $Lines) {
	if ($line -match 'skipped=') { continue }
	foreach ($m in [regex]::Matches($line, '([A-Za-z_][A-Za-z_0-9]*)=([-0-9.eE]+)')) {
		$name = $m.Groups[1].Value
		if ($Collected.ContainsKey($name)) {
			$Collected[$name] += [double]$m.Groups[2].Value
		}
	}
}
Write-Host ""
foreach ($k in $Keys) {
	if ($Collected[$k].Count -gt 0) {
		Write-Host ("median {0} = {1}" -f $k, (Get-Median $Collected[$k]))
	}
}
