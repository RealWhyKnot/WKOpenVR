#Requires -Version 5.1
param(
	# spacecal_log recording to session-replay. Defaults to the newest pinned
	# spacecal_log capture in Logs\corpus.
	[string]$RecordingPath = "",

	# Compare this run's session-replay counters against the recording's stored
	# baseline (tools\replay-baselines\<recording>.session.baseline.json);
	# non-zero exit on drift.
	[switch]$Baseline,

	# Write/overwrite the recording's session baseline from this run.
	[switch]$UpdateBaseline,

	# Run the current test binary without rebuilding first.
	[switch]$SkipBuild
)

# Replays a recorded session through the full session-replay layer (solver +
# auto-lock + relocalization recovery, including recorded snap corroboration
# and warm-restart away-gap eviction) and gates the counters against a stored
# baseline. Complements Run-CalibrationReplayMatrix.ps1, which gates the
# solver-scenario metrics but never exercises the recovery layer.

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

if ([string]::IsNullOrWhiteSpace($RecordingPath)) {
	$Corpus = Join-Path (Get-DefaultLogDir) "corpus"
	$Newest = Get-ChildItem (Join-Path $Corpus "spacecal_log.*.txt") -ErrorAction SilentlyContinue |
		Sort-Object LastWriteTime -Descending | Select-Object -First 1
	if (-not $Newest) { throw "No spacecal_log capture in $Corpus; pass -RecordingPath." }
	$RecordingPath = $Newest.FullName
}
if (-not (Test-Path -LiteralPath $RecordingPath)) { throw "Recording not found: $RecordingPath" }

if (-not $SkipBuild) {
	& (Join-Path $RepoRoot "build.ps1")
	if ($LASTEXITCODE -ne 0) { throw "build failed" }
}

$TestExe = Join-Path $RepoRoot "build\artifacts\Release\spacecal_tests.exe"
if (-not (Test-Path $TestExe)) { throw "missing $TestExe; run without -SkipBuild" }

$env:WKOPENVR_REPLAY_SESSION = "1"
$env:WKOPENVR_REPLAY_PATHS = $RecordingPath
try {
	$OutputLines = & $TestExe --gtest_filter=SessionReplayTest.ReplaySessionsWhenRequested 2>&1 | ForEach-Object { "$_" }
	if ($LASTEXITCODE -ne 0) { throw "spacecal_tests session replay failed (exit $LASTEXITCODE)" }
}
finally {
	Remove-Item Env:WKOPENVR_REPLAY_SESSION -ErrorAction SilentlyContinue
	Remove-Item Env:WKOPENVR_REPLAY_PATHS -ErrorAction SilentlyContinue
}

$SummaryLine = $OutputLines | Where-Object { $_ -match '^\[session-replay\] ' } | Select-Object -First 1
if (-not $SummaryLine) { throw "No [session-replay] summary line in test output." }
Write-Host $SummaryLine

$Metrics = @{}
foreach ($m in [regex]::Matches($SummaryLine, '([A-Za-z_][A-Za-z_0-9]*)=([-0-9.eE]+)')) {
	$Metrics[$m.Groups[1].Value] = [double]$m.Groups[2].Value
}

# Recovery-layer decisions must be bit-stable run to run; continuous
# trajectory metrics get a small tolerance for solver evolution.
$ExactKeys = @("relocs", "snap_suppressed", "holds", "reanchors", "destructive_clears",
	"samples_evicted", "warm_restart_snaps", "sub_threshold_relocs", "rows")
$TolerantKeys = @("accepts", "applied_path_cm", "peak_step_cm", "net_drift_mag_cm", "sub_threshold_residual_cm",
	"wander_per_10min_cm", "max_unclassified_step_cm",
	"rot_wander_per_10min_deg", "max_unclassified_rot_step_deg", "drift_steps", "drift_path_cm")
$TolerantFraction = 0.10

foreach ($k in $ExactKeys + $TolerantKeys) {
	if (-not $Metrics.ContainsKey($k)) { throw "Summary line missing metric '$k'." }
}

$RecordingName = [System.IO.Path]::GetFileNameWithoutExtension($RecordingPath)
$BaselinePath = Join-Path $RepoRoot "tools\replay-baselines\$RecordingName.session.baseline.json"

if ($UpdateBaseline) {
	$Payload = [ordered]@{}
	foreach ($k in $ExactKeys + $TolerantKeys) { $Payload[$k] = $Metrics[$k] }
	$Json = ($Payload | ConvertTo-Json)
	[System.IO.File]::WriteAllText($BaselinePath, $Json, (New-Object System.Text.UTF8Encoding($false)))
	Write-Host "Wrote session baseline: $BaselinePath"
}

if ($Baseline) {
	if (-not (Test-Path -LiteralPath $BaselinePath)) {
		throw "No session baseline for this recording; run with -UpdateBaseline first: $BaselinePath"
	}
	$Stored = Get-Content -LiteralPath $BaselinePath -Raw | ConvertFrom-Json
	$Failures = @()
	foreach ($k in $ExactKeys) {
		$expected = [double]$Stored.$k
		if ($Metrics[$k] -ne $expected) {
			$Failures += ("{0}: expected {1}, got {2}" -f $k, $expected, $Metrics[$k])
		}
	}
	foreach ($k in $TolerantKeys) {
		$expected = [double]$Stored.$k
		$allowed = [Math]::Max([Math]::Abs($expected) * $TolerantFraction, 0.5)
		if ([Math]::Abs($Metrics[$k] - $expected) -gt $allowed) {
			$Failures += ("{0}: expected {1} (+/-{2:F2}), got {3}" -f $k, $expected, $allowed, $Metrics[$k])
		}
	}
	if ($Failures.Count -gt 0) {
		Write-Host "Session baseline check FAILED: $BaselinePath"
		$Failures | ForEach-Object { Write-Host "  $_" }
		exit 1
	}
	Write-Host "Session baseline check passed: $BaselinePath"
}
