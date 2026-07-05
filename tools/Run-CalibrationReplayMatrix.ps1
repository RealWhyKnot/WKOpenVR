#Requires -Version 5.1
param(
	# spacecal_log_v2/v3/v4/v5 recording to replay. Defaults to the newest retained
	# %LocalAppDataLow%\WKOpenVR\Logs\spacecal_log.*.txt capture.
	[string]$RecordingPath = "",

	# Output CSV for parsed replay rows. Defaults under build\calibration-replay-matrix.
	[string]$OutputCsv = "",

	# Scenario names to run. Defaults to the standard drift-guard matrix.
	[string[]]$Scenario = @(),

	# Continuous sample window(s) to replay. Keep 200 for live-shaped comparisons.
	[int[]]$SampleWindow = @(200),

	# Periodic shadow-quality interval for replay summaries. Use 0 only for fast legacy comparisons.
	[int]$QualityInterval = 10,

	# Disable holdout quality scoring while still collecting periodic quality reports.
	[switch]$NoHoldout,

	# Match the old matrix behavior: quality_interval=0 and holdout=0.
	[switch]$FastLegacyMetrics,

	# Directory for per-tick trace CSVs (one per recording x window x scenario run).
	[string]$TraceDir = "",

	# Compare this run's metrics against the recording's stored golden baseline
	# (tools\replay-baselines\<recording>.baseline.json); non-zero exit on drift.
	[switch]$Baseline,

	# Write/overwrite the recording's golden baseline from this run's metrics.
	[switch]$UpdateBaseline,

	# Run the current test binary without rebuilding first.
	[switch]$SkipBuild,

	# Pass through to build.ps1 when building.
	[switch]$SkipConfigure
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

function Resolve-RepoPath {
	param([string]$Path, [string]$DefaultRelativePath)
	$Value = $Path
	if ([string]::IsNullOrWhiteSpace($Value)) {
		$Value = $DefaultRelativePath
	}
	if ([System.IO.Path]::IsPathRooted($Value)) {
		return [System.IO.Path]::GetFullPath($Value)
	}
	return [System.IO.Path]::GetFullPath((Join-Path $RepoRoot $Value))
}

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
	return ""
}

function Resolve-RecordingPath {
	param([string]$Path)
	if (-not [string]::IsNullOrWhiteSpace($Path)) {
		$Resolved = Resolve-RepoPath $Path ""
		if (-not (Test-Path -LiteralPath $Resolved)) {
			throw "Recording not found: $Resolved"
		}
		return $Resolved
	}

	$LogDir = Get-DefaultLogDir
	if ([string]::IsNullOrWhiteSpace($LogDir) -or -not (Test-Path -LiteralPath $LogDir)) {
		throw "No recording path was supplied and the WKOpenVR log directory was not found."
	}

	$Latest = Get-ChildItem -LiteralPath $LogDir -Filter "spacecal_log.*.txt" -File -ErrorAction SilentlyContinue |
		Sort-Object LastWriteTime -Descending | Select-Object -First 1
	if (-not $Latest) {
		throw "No spacecal_log.*.txt recordings found under $LogDir"
	}
	return $Latest.FullName
}

function New-Scenario {
	param([string]$Name, [hashtable]$Env)
	New-Object psobject -Property @{
		Name = $Name
		Env = $Env
	}
}

function Get-ScenarioCatalog {
	# Catalog note: the June drift-guard toggles (quarantine, drift breaker,
	# bounded solve, locked snap, reloc source) were removed from the test
	# binary; their scenarios and env names are gone with them.
	$Catalog = @{}
	$Catalog["baseline"] = New-Scenario "baseline" @{}
	# Far-from-origin fix A/B: relpose-locked head-mount solve, uniform vs
	# geometry-precision-weighted. Compare applied_mag_wander_cm across the pair.
	$Catalog["relpose_uniform"] = New-Scenario "relpose_uniform" @{
		WKOPENVR_REPLAY_LOCK_REL         = "1"
		WKOPENVR_REPLAY_PRECISION_WEIGHT = "0"
	}
	$Catalog["relpose_weighted"] = New-Scenario "relpose_weighted" @{
		WKOPENVR_REPLAY_LOCK_REL         = "1"
		WKOPENVR_REPLAY_PRECISION_WEIGHT = "1"
	}
	# Gravity-constrained 4-DoF: same weighted solve with the calibration
	# rotation projected to yaw-about-gravity. Compare net_drift_cm and
	# final_error_mm against relpose_weighted. Not run by default -- request
	# explicitly via -Scenario.
	$Catalog["relpose_weighted_gravity"] = New-Scenario "relpose_weighted_gravity" @{
		WKOPENVR_REPLAY_LOCK_REL         = "1"
		WKOPENVR_REPLAY_PRECISION_WEIGHT = "1"
		WKOPENVR_REPLAY_GRAVITY_4DOF     = "1"
	}
	# Warm-start A/B: replay from the recording's own stored-profile seed. The
	# fused variant reproduces the confidence-fusion bad-seed behavior offline;
	# compare net_drift_mag_cm and perceptible_shifts across the pair.
	$Catalog["seed_recorded_uniform"] = New-Scenario "seed_recorded_uniform" @{
		WKOPENVR_REPLAY_LOCK_REL         = "1"
		WKOPENVR_REPLAY_PRECISION_WEIGHT = "0"
		WKOPENVR_REPLAY_SEED_PROFILE     = "recorded"
	}
	$Catalog["seed_recorded_fused"] = New-Scenario "seed_recorded_fused" @{
		WKOPENVR_REPLAY_LOCK_REL         = "1"
		WKOPENVR_REPLAY_PRECISION_WEIGHT = "1"
		WKOPENVR_REPLAY_SEED_PROFILE     = "recorded"
	}
	return $Catalog
}

function Convert-ToEnvValue {
	param([object]$Value)
	if ($null -eq $Value) { return "" }
	return [string]$Value
}

function Set-ReplayEnvironment {
	param([hashtable]$BaseEnv, [hashtable]$ScenarioEnv)
	foreach ($Key in $script:ReplayEnvNames) {
		Remove-Item -Path ("Env:\" + $Key) -ErrorAction SilentlyContinue
	}
	foreach ($Item in $BaseEnv.GetEnumerator()) {
		Set-Item -Path ("Env:\" + $Item.Key) -Value (Convert-ToEnvValue $Item.Value)
	}
	foreach ($Item in $ScenarioEnv.GetEnumerator()) {
		Set-Item -Path ("Env:\" + $Item.Key) -Value (Convert-ToEnvValue $Item.Value)
	}
}

function Restore-ReplayEnvironment {
	foreach ($Key in $script:ReplayEnvNames) {
		Remove-Item -Path ("Env:\" + $Key) -ErrorAction SilentlyContinue
		if ($script:OriginalEnv.ContainsKey($Key)) {
			Set-Item -Path ("Env:\" + $Key) -Value $script:OriginalEnv[$Key]
		}
	}
}

function Parse-ReplayLine {
	param([string]$Line, [string]$ScenarioName, [int]$ExitCode)
	$Tokens = @($Line -split "\s+")
	if ($Tokens.Count -lt 3) { return $null }
	$Values = @{
		Scenario = $ScenarioName
		Recording = $Tokens[1]
		ExitCode = $ExitCode
	}
	for ($i = 2; $i -lt $Tokens.Count; ++$i) {
		$Eq = $Tokens[$i].IndexOf("=")
		if ($Eq -le 0) { continue }
		$Key = $Tokens[$i].Substring(0, $Eq)
		$Value = $Tokens[$i].Substring($Eq + 1)
		$Values[$Key] = $Value
	}

	New-Object psobject -Property @{
		Scenario = $Values["Scenario"]
		Recording = $Values["Recording"]
		ExitCode = $Values["ExitCode"]
		Window = $Values["window"]
		Accepts = $Values["accepts"]
		FinalErrorMm = $Values["final_error_mm"]
		MedianRelPoseMadMm = $Values["median_relpose_mad_mm"]
		FinalRelPoseMadMm = $Values["final_relpose_mad_mm"]
		LockRel = $Values["lock_rel"]
		PrecisionWeight = $Values["precision_weight"]
		Seed = $Values["seed"]
		SeedApplied = $Values["seed_applied"]
		SeedMagCm = $Values["seed_mag_cm"]
		PerceptibleShifts = $Values["perceptible_shifts"]
		PerceptibleMaxMm = $Values["perceptible_max_mm"]
		PerceptibleSumMm = $Values["perceptible_sum_mm"]
		NetDriftCm = $Values["net_drift_cm"]
		NetDriftMagCm = $Values["net_drift_mag_cm"]
		PeakAppliedMagCm = $Values["peak_applied_mag_cm"]
		AppliedMagWanderCm = $Values["applied_mag_wander_cm"]
		PeakAppliedStepCm = $Values["peak_applied_step_cm"]
		TotalAppliedPathCm = $Values["total_applied_path_cm"]
		HoldoutRmsMm = $Values["holdout_rms_mm"]
		HoldoutP90Mm = $Values["holdout_p90_mm"]
		HoldoutP95Mm = $Values["holdout_p95_mm"]
		HoldoutPass = $Values["holdout_pass"]
		QualityReports = $Values["quality_reports"]
		ShadowWouldAccept = $Values["shadow_accepts"]
		ShadowWouldReject = $Values["shadow_rejects"]
		ShadowRejectReasons = $Values["shadow_reject_reasons"]
		SamplesQuarantined = $Values["samples_quarantined"]
		SolverSampleRows = $Values["solver_sample_rows"]
		SolverSampleRatio = $Values["solver_sample_ratio"]
		SampleStarved = $Values["sample_starved"]
		FreezeEngagements = $Values["freeze_engagements"]
		SnapReanchors = $Values["snap_reanchors"]
		LockedSnapHmdJumps = $Values["locked_snap_hmd_jumps"]
		LockedSnapTrackerInvalid = $Values["locked_snap_tracker_invalid"]
		LockedSnapCorroborated = $Values["locked_snap_corroborated"]
		RelocEvents = $Values["reloc_events"]
		RelocSource = $Values["reloc_source"]
		FinalShadowReason = $Values["final_shadow_reason"]
		RawLine = $Line
	}
}

if ($SampleWindow.Count -eq 0) {
	throw "At least one sample window is required."
}
foreach ($Window in $SampleWindow) {
	if ($Window -lt 0) {
		throw "SampleWindow values must be zero or positive."
	}
}
if ($QualityInterval -lt 0) {
	throw "QualityInterval must be zero or positive."
}
if ($FastLegacyMetrics) {
	$QualityInterval = 0
	$NoHoldout = $true
}

$Catalog = Get-ScenarioCatalog
if ($Scenario.Count -eq 0) {
	$Scenario = @(
		"baseline",
		"relpose_uniform",
		"relpose_weighted",
		"seed_recorded_uniform",
		"seed_recorded_fused"
	)
}
else {
	$ExpandedScenario = @()
	foreach ($Name in $Scenario) {
		foreach ($Part in ([string]$Name -split ",")) {
			$Trimmed = $Part.Trim()
			if (-not [string]::IsNullOrWhiteSpace($Trimmed)) {
				$ExpandedScenario += $Trimmed
			}
		}
	}
	$Scenario = $ExpandedScenario
}

$Scenarios = @()
foreach ($Name in $Scenario) {
	$Key = $Name.Trim().ToLowerInvariant()
	if (-not $Catalog.ContainsKey($Key)) {
		$Known = (($Catalog.Keys | Sort-Object) -join ", ")
		throw "Unknown scenario '$Name'. Known scenarios: $Known"
	}
	$Scenarios += $Catalog[$Key]
}

$Recording = Resolve-RecordingPath $RecordingPath
if ([string]::IsNullOrWhiteSpace($OutputCsv)) {
	$OutputCsv = Resolve-RepoPath "" "build\calibration-replay-matrix\calibration-replay-matrix.csv"
} else {
	$OutputCsv = Resolve-RepoPath $OutputCsv ""
}
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $OutputCsv) | Out-Null

if (-not $SkipBuild) {
	$BuildArgs = @{ Target = @("spacecal_tests") }
	if ($SkipConfigure) { $BuildArgs["SkipConfigure"] = $true }
	& "$RepoRoot\build.ps1" @BuildArgs
	if ($LASTEXITCODE -ne 0) {
		throw "build.ps1 failed (exit $LASTEXITCODE)"
	}
}

$TestExe = Join-Path $RepoRoot "build\artifacts\Release\spacecal_tests.exe"
if (-not (Test-Path -LiteralPath $TestExe)) {
	throw "spacecal_tests.exe not found at $TestExe"
}

$script:ReplayEnvNames = @(
	"WKOPENVR_REPLAY_RECORDINGS",
	"WKOPENVR_REPLAY_PATHS",
	"WKOPENVR_REPLAY_SAMPLE_WINDOWS",
	"WKOPENVR_REPLAY_QUALITY_INTERVAL",
	"WKOPENVR_REPLAY_HOLDOUT",
	"WKOPENVR_REPLAY_LOCK_REL",
	"WKOPENVR_REPLAY_PRECISION_WEIGHT",
	"WKOPENVR_REPLAY_GRAVITY_4DOF",
	"WKOPENVR_REPLAY_SEED_PROFILE",
	"WKOPENVR_REPLAY_TRACE_CSV",
	"WKOPENVR_REPLAY_AUTOLOCK_SIM",
	"WKOPENVR_REPLAY_AUTOLOCK_ENTER_MM",
	"WKOPENVR_REPLAY_AUTOLOCK_LEAVE_MM",
	"WKOPENVR_REPLAY_AUTOLOCK_SCALE",
	"WKOPENVR_REPLAY_AUTOLOCK_PANIC_MM",
	"WKOPENVR_REPLAY_AUTOLOCK_SETTLED_HOLD_SEC",
	"WKOPENVR_REPLAY_AUTOLOCK_STATIONARY_MPS",
	"WKOPENVR_REPLAY_AUTOLOCK_UNLOCK_WAIT_SEC",
	"WKOPENVR_REPLAY_AUTOLOCK_FLOOR_WINDOW_SEC",
	"WKOPENVR_REPLAY_AUTOLOCK_WINDOW"
)
$script:OriginalEnv = @{}
foreach ($Key in $script:ReplayEnvNames) {
	$Value = [Environment]::GetEnvironmentVariable($Key, "Process")
	if ($null -ne $Value) {
		$script:OriginalEnv[$Key] = $Value
	}
}

$BaseEnv = @{
	WKOPENVR_REPLAY_RECORDINGS = "1"
	WKOPENVR_REPLAY_PATHS = $Recording
	WKOPENVR_REPLAY_SAMPLE_WINDOWS = (($SampleWindow | ForEach-Object { [string]$_ }) -join ";")
	WKOPENVR_REPLAY_QUALITY_INTERVAL = [string]$QualityInterval
	WKOPENVR_REPLAY_HOLDOUT = $(if ($NoHoldout) { "0" } else { "1" })
}

Write-Host "Recording: $Recording"
Write-Host "Output: $OutputCsv"
Write-Host ""

$Results = @()
try {
	foreach ($Item in $Scenarios) {
		Write-Host ("== Calibration replay: {0} ==" -f $Item.Name)
		Set-ReplayEnvironment $BaseEnv $Item.Env
		if (-not [string]::IsNullOrWhiteSpace($TraceDir)) {
			# Per-scenario subdirectory so runs don't overwrite each other's traces.
			Set-Item -Path "Env:\WKOPENVR_REPLAY_TRACE_CSV" -Value (Join-Path (Resolve-RepoPath $TraceDir "") $Item.Name)
		}
		$PreviousErrorActionPreference = $ErrorActionPreference
		$ErrorActionPreference = "Continue"
		try {
			$Output = & $TestExe --gtest_filter=*ReplayLocalRecordingsWhenRequested --gtest_brief=1 2>&1
			$ExitCode = $LASTEXITCODE
		}
		finally {
			$ErrorActionPreference = $PreviousErrorActionPreference
		}
		$ReplayLines = @($Output | Where-Object { $_ -is [string] -and $_.StartsWith("[replay] ") })
		if ($ReplayLines.Count -eq 0) {
			$Tail = (($Output | Select-Object -Last 20) -join [Environment]::NewLine)
			throw "No [replay] output for scenario '$($Item.Name)' (exit $ExitCode). Tail:`n$Tail"
		}
		foreach ($Line in $ReplayLines) {
			$Parsed = Parse-ReplayLine $Line $Item.Name $ExitCode
			if ($Parsed) {
				$Results += $Parsed
				Write-Host ("{0}: final_error_mm={1} holdout_rms_mm={2} median_mad_mm={3} final_mad_mm={4} quarantined={5} solver_ratio={6} starved={7} freeze={8} snaps={9} reloc={10}/{11} reason={12}" -f
					$Item.Name, $Parsed.FinalErrorMm, $Parsed.HoldoutRmsMm, $Parsed.MedianRelPoseMadMm,
					$Parsed.FinalRelPoseMadMm, $Parsed.SamplesQuarantined, $Parsed.SolverSampleRatio,
					$Parsed.SampleStarved, $Parsed.FreezeEngagements, $Parsed.SnapReanchors,
					$Parsed.RelocEvents, $Parsed.RelocSource, $Parsed.FinalShadowReason)
			}
		}
		if ($ExitCode -ne 0) {
			throw "spacecal_tests failed for scenario '$($Item.Name)' (exit $ExitCode)"
		}
	}
}
finally {
	Restore-ReplayEnvironment
}

$Results | Sort-Object Scenario, Window | Export-Csv -LiteralPath $OutputCsv -NoTypeInformation

Write-Host ""
Write-Host "Scenario,Window,FinalErrorMm,HoldoutRmsMm,MedianRelPoseMadMm,FinalRelPoseMadMm,Quarantined,SolverSampleRatio,SampleStarved,Freeze,Snaps,RelocEvents,RelocSource,FinalReason"
foreach ($Result in $Results) {
	Write-Host ("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13}" -f $Result.Scenario, $Result.Window,
		$Result.FinalErrorMm, $Result.HoldoutRmsMm, $Result.MedianRelPoseMadMm, $Result.FinalRelPoseMadMm,
		$Result.SamplesQuarantined, $Result.SolverSampleRatio, $Result.SampleStarved, $Result.FreezeEngagements,
		$Result.SnapReanchors, $Result.RelocEvents, $Result.RelocSource, $Result.FinalShadowReason)
}

Write-Host ""
Write-Host "Wrote $OutputCsv"

# Golden-baseline mode: per (scenario, window) metric snapshot for this
# recording, with per-metric tolerances. -UpdateBaseline writes the snapshot;
# -Baseline compares and exits non-zero on drift so CI/pre-flight runs fail
# loudly instead of quietly shifting.
if ($Baseline -or $UpdateBaseline) {
	$BaselineDir = Resolve-RepoPath "" "tools\replay-baselines"
	$RecName = [System.IO.Path]::GetFileNameWithoutExtension($Recording)
	$BaselinePath = Join-Path $BaselineDir ($RecName + ".baseline.json")

	$MetricNames = @("Accepts", "FinalErrorMm", "AppliedMagWanderCm", "TotalAppliedPathCm", "PeakAppliedStepCm",
		"PerceptibleShifts", "NetDriftMagCm")
	$Tolerances = @{
		Accepts            = @{ Abs = 5.0; Rel = 0.05 }
		FinalErrorMm       = @{ Abs = 0.5; Rel = 0.10 }
		AppliedMagWanderCm = @{ Abs = 0.5; Rel = 0.10 }
		TotalAppliedPathCm = @{ Abs = 2.0; Rel = 0.10 }
		PeakAppliedStepCm  = @{ Abs = 0.2; Rel = 0.15 }
		PerceptibleShifts  = @{ Abs = 2.0; Rel = 0.20 }
		NetDriftMagCm      = @{ Abs = 1.0; Rel = 0.20 }
	}

	$Current = @{}
	foreach ($Result in $Results) {
		$Key = "{0}|w{1}" -f $Result.Scenario, $Result.Window
		$Metrics = @{}
		foreach ($Name in $MetricNames) {
			$Raw = $Result.$Name
			if ($null -ne $Raw -and "$Raw" -ne "") {
				$Metrics[$Name] = [double]$Raw
			}
		}
		$Current[$Key] = $Metrics
	}

	if ($UpdateBaseline) {
		New-Item -ItemType Directory -Force -Path $BaselineDir | Out-Null
		$Json = $Current | ConvertTo-Json -Depth 4
		[System.IO.File]::WriteAllText($BaselinePath, $Json, (New-Object System.Text.UTF8Encoding($false)))
		Write-Host "Baseline updated: $BaselinePath"
	}
	elseif ($Baseline) {
		if (-not (Test-Path -LiteralPath $BaselinePath)) {
			throw "No baseline for this recording; run with -UpdateBaseline first. Expected: $BaselinePath"
		}
		$Stored = Get-Content -LiteralPath $BaselinePath -Raw | ConvertFrom-Json
		$Failures = @()
		foreach ($Key in $Current.Keys) {
			$StoredEntry = $Stored.$Key
			if ($null -eq $StoredEntry) {
				Write-Host ("baseline: no stored entry for {0} (new scenario?); skipping" -f $Key)
				continue
			}
			foreach ($Name in $MetricNames) {
				if (-not $Current[$Key].ContainsKey($Name)) { continue }
				$StoredValue = $StoredEntry.$Name
				if ($null -eq $StoredValue) { continue }
				$Cur = [double]$Current[$Key][$Name]
				$Base = [double]$StoredValue
				$Tol = [math]::Max($Tolerances[$Name].Abs, $Tolerances[$Name].Rel * [math]::Abs($Base))
				if ([math]::Abs($Cur - $Base) -gt $Tol) {
					$Failures += ("{0} {1}: {2} vs baseline {3} (tol {4})" -f $Key, $Name, $Cur, $Base, $Tol)
				}
			}
		}
		if ($Failures.Count -gt 0) {
			Write-Host "BASELINE DRIFT:"
			foreach ($Failure in $Failures) { Write-Host ("  " + $Failure) }
			exit 1
		}
		Write-Host "Baseline check passed: $BaselinePath"
	}
}
