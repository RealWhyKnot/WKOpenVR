#Requires -Version 5.1
<#
.SYNOPSIS
Estimates the time offset between the headset tracking stack and the lighthouse stack.
.DESCRIPTION
Cross-correlates the angular-speed profiles of the HMD and the head-mounted tracker
in a full-rate phantom_replay capture, then reports the relative-pose MAD paired at
lag zero vs at the recovered lag.
.EXAMPLE
./Run-TimeOffsetXcorr.ps1 -SkipBuild
.NOTES
Full-rate capture: create the flag file
%USERPROFILE%\AppData\LocalLow\WKOpenVR\phantom_replay_fullrate.enabled
before starting SteamVR (WKOPENVR_PHANTOM_REPLAY_FULLRATE=1 also works when the
environment actually reaches vrserver). Hidden devices are recorded with their
real pre-quash poses, so the head tracker may stay hidden.
#>
[CmdletBinding()]
param(
	# phantom_replay CSV to analyse. Defaults to the newest pinned
	# phantom_replay capture in Logs\corpus.
	[string]$RecordingPath = "",

	# Device serials. Defaults match the rig this analysis was built for.
	[string]$HmdSerial = "",
	[string]$WitnessSerial = "",

	[switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot

if (-not $RecordingPath) {
	$LocalLow = Join-Path ([System.IO.Directory]::GetParent($env:LOCALAPPDATA).FullName) "LocalLow"
	$Corpus = Join-Path $LocalLow "WKOpenVR\Logs\corpus"
	$Newest = Get-ChildItem (Join-Path $Corpus "phantom_replay.*.csv") -ErrorAction SilentlyContinue |
		Sort-Object LastWriteTime -Descending | Select-Object -First 1
	if (-not $Newest) { throw "No phantom_replay capture in $Corpus; pass -RecordingPath." }
	$RecordingPath = $Newest.FullName
}

if (-not $SkipBuild) {
	& (Join-Path $RepoRoot "build.ps1")
	if ($LASTEXITCODE -ne 0) { throw "build failed" }
}

$TestExe = Join-Path $RepoRoot "build\artifacts\Release\spacecal_tests.exe"
if (-not (Test-Path $TestExe)) { throw "missing $TestExe; run without -SkipBuild" }

$env:WKOPENVR_XCORR_PHANTOM = $RecordingPath
if ($HmdSerial) { $env:WKOPENVR_XCORR_HMD_SERIAL = $HmdSerial }
if ($WitnessSerial) { $env:WKOPENVR_XCORR_WITNESS_SERIAL = $WitnessSerial }
try {
	& $TestExe --gtest_filter=TimeOffsetXcorrTest.EstimateFromPhantomRecordingWhenRequested
	if ($LASTEXITCODE -ne 0) { throw "analysis run failed" }
}
finally {
	Remove-Item Env:WKOPENVR_XCORR_PHANTOM -ErrorAction SilentlyContinue
	Remove-Item Env:WKOPENVR_XCORR_HMD_SERIAL -ErrorAction SilentlyContinue
	Remove-Item Env:WKOPENVR_XCORR_WITNESS_SERIAL -ErrorAction SilentlyContinue
}
