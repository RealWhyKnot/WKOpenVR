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

	# Include the legacy relative-pose proxy source as a diagnostic comparison.
	[switch]$IncludeProxyComparison,

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
	$Catalog = @{}
	$Catalog["baseline"] = New-Scenario "baseline" @{}
	$Catalog["quarantine"] = New-Scenario "quarantine" @{ WKOPENVR_REPLAY_QUARANTINE = "1" }
	$Catalog["bounded_full"] = New-Scenario "bounded_full" @{
		WKOPENVR_REPLAY_BOUNDED_SOLVE = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_SLEW = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_COMMONMODE = "1"
	}
	$Catalog["bounded_prior"] = New-Scenario "bounded_prior" @{
		WKOPENVR_REPLAY_BOUNDED_SOLVE = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_PRIOR = "1"
	}
	$Catalog["quarantine_bounded"] = New-Scenario "quarantine_bounded" @{
		WKOPENVR_REPLAY_QUARANTINE = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_SLEW = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_COMMONMODE = "1"
	}
	$Catalog["drift_breaker"] = New-Scenario "drift_breaker" @{ WKOPENVR_REPLAY_DRIFT_BREAKER = "1" }
	$Catalog["quarantine_drift"] = New-Scenario "quarantine_drift" @{
		WKOPENVR_REPLAY_QUARANTINE = "1"
		WKOPENVR_REPLAY_DRIFT_BREAKER = "1"
	}
	$Catalog["bounded_drift"] = New-Scenario "bounded_drift" @{
		WKOPENVR_REPLAY_DRIFT_BREAKER = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_SLEW = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_COMMONMODE = "1"
	}
	$Catalog["quarantine_bounded_drift"] = New-Scenario "quarantine_bounded_drift" @{
		WKOPENVR_REPLAY_QUARANTINE = "1"
		WKOPENVR_REPLAY_DRIFT_BREAKER = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_SLEW = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_COMMONMODE = "1"
	}
	$Catalog["locked_snap"] = New-Scenario "locked_snap" @{
		WKOPENVR_REPLAY_LOCKED_SNAP = "1"
		WKOPENVR_REPLAY_TRACKING_STYLE = "locked"
	}
	$Catalog["all"] = New-Scenario "all" @{
		WKOPENVR_REPLAY_QUARANTINE = "1"
		WKOPENVR_REPLAY_DRIFT_BREAKER = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_SLEW = "1"
		WKOPENVR_REPLAY_BOUNDED_SOLVE_COMMONMODE = "1"
		WKOPENVR_REPLAY_LOCKED_SNAP = "1"
		WKOPENVR_REPLAY_TRACKING_STYLE = "locked"
	}
	$Catalog["quarantine_proxy"] = New-Scenario "quarantine_proxy" @{
		WKOPENVR_REPLAY_QUARANTINE = "1"
		WKOPENVR_REPLAY_RELOC_SOURCE = "proxy"
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
		FinalErrorMm = $Values["final_error_mm"]
		MedianRelPoseMadMm = $Values["median_relpose_mad_mm"]
		FinalRelPoseMadMm = $Values["final_relpose_mad_mm"]
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
		"quarantine",
		"bounded_full",
		"quarantine_bounded",
		"drift_breaker",
		"quarantine_drift",
		"bounded_drift",
		"quarantine_bounded_drift",
		"locked_snap",
		"all"
	)
	if ($IncludeProxyComparison) {
		$Scenario += "quarantine_proxy"
	}
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
	"WKOPENVR_REPLAY_QUARANTINE",
	"WKOPENVR_REPLAY_QUARANTINE_SEC",
	"WKOPENVR_REPLAY_DRIFT_BREAKER",
	"WKOPENVR_REPLAY_DRIFT_BREAKER_MULT",
	"WKOPENVR_REPLAY_DRIFT_BREAKER_CAP_MM",
	"WKOPENVR_REPLAY_BOUNDED_SOLVE",
	"WKOPENVR_REPLAY_BOUNDED_SOLVE_PRIOR",
	"WKOPENVR_REPLAY_BOUNDED_SOLVE_LAMBDA",
	"WKOPENVR_REPLAY_BOUNDED_SOLVE_SLEW",
	"WKOPENVR_REPLAY_BOUNDED_SOLVE_STEP_MM",
	"WKOPENVR_REPLAY_BOUNDED_SOLVE_COMMONMODE",
	"WKOPENVR_REPLAY_RELOC_PROXY_M",
	"WKOPENVR_REPLAY_RELOC_SOURCE",
	"WKOPENVR_REPLAY_LOCKED_SNAP",
	"WKOPENVR_REPLAY_TRACKING_STYLE"
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
