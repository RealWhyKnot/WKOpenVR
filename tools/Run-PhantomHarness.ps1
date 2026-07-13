#Requires -Version 5.1
<#
.SYNOPSIS
Runs recorded replay scenarios through WKOpenVR.exe --test-harness.
.DESCRIPTION
Feeds a replay CSV into WKOpenVR.exe --test-harness for one or more scenario
windows and writes per-scenario metric CSVs and captured logs to the output
directory. Builds OpenVRPairOverlay first unless -SkipBuild is set.
.EXAMPLE
./Run-PhantomHarness.ps1 -ReplayPath recordings\session.csv -Scenario waist:0:60000
#>
[CmdletBinding()]
param(
	# Replay CSV to feed into WKOpenVR.exe --test-harness.
	[string]$ReplayPath = "",

	# Scenario specs:
	#   role:start_ms:end_ms
	#   name:role:start_ms:end_ms
	# Use role "none" for virtual-tracker-only no-FBT replays.
	[string[]]$Scenario = @(),

	# Output directory for per-scenario metric CSVs and captured logs.
	[string]$OutputDir = "",

	# Skip the OpenVRPairOverlay build step.
	[switch]$SkipBuild,

	# Pass through to build.ps1 when building.
	[switch]$SkipConfigure,

	# Replay speed multiplier. Keep at 1.0 for timing-sensitive dropout checks.
	[double]$Speed = 1.0
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

function New-ScenarioFromSpec {
	param([string]$Spec)
	$Parts = @($Spec -split ":")
	if ($Parts.Count -eq 3) {
		$Role = $Parts[0]
		$Start = [double]::Parse($Parts[1], [System.Globalization.CultureInfo]::InvariantCulture)
		$End = [double]::Parse($Parts[2], [System.Globalization.CultureInfo]::InvariantCulture)
		$Name = "{0}-{1}-{2}" -f $Role, [int]$Start, [int]$End
	} elseif ($Parts.Count -eq 4) {
		$Name = $Parts[0]
		$Role = $Parts[1]
		$Start = [double]::Parse($Parts[2], [System.Globalization.CultureInfo]::InvariantCulture)
		$End = [double]::Parse($Parts[3], [System.Globalization.CultureInfo]::InvariantCulture)
	} else {
		throw "Invalid scenario '$Spec'. Use role:start_ms:end_ms or name:role:start_ms:end_ms."
	}
	if ($End -le $Start) {
		throw "Invalid scenario '$Spec': end_ms must be greater than start_ms."
	}
	New-Object psobject -Property @{
		Name = $Name
		Role = $Role
		StartMs = $Start
		EndMs = $End
	}
}

function Read-MetricReport {
	param([string]$Path)
	$Metrics = @{}
	if (-not (Test-Path -LiteralPath $Path)) {
		return $Metrics
	}
	foreach ($Line in Get-Content -LiteralPath $Path) {
		if ($Line -eq "metric,value") { continue }
		$Comma = $Line.IndexOf(",")
		if ($Comma -lt 0) { continue }
		$Key = $Line.Substring(0, $Comma)
		$Value = $Line.Substring($Comma + 1)
		$Metrics[$Key] = $Value
	}
	return $Metrics
}

function Invoke-HarnessScenario {
	param($ScenarioInfo, [string]$HarnessExe, [string]$Replay, [string]$OutDir)

	$SafeName = ($ScenarioInfo.Name -replace '[^A-Za-z0-9_.-]', '_')
	$ReportPath = Join-Path $OutDir "$SafeName.metrics.csv"
	$StdoutPath = Join-Path $OutDir "$SafeName.stdout.log"
	$StderrPath = Join-Path $OutDir "$SafeName.stderr.log"

	$Args = @(
		"--test-harness",
		"--filter", "phantom_replay",
		"--phantom-replay", $Replay,
		"--phantom-replay-report", $ReportPath,
		"--phantom-replay-dropout-role", $ScenarioInfo.Role,
		"--phantom-replay-dropout-start-ms", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0:F3}", $ScenarioInfo.StartMs)),
		"--phantom-replay-dropout-end-ms", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0:F3}", $ScenarioInfo.EndMs)),
		"--phantom-replay-speed", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0:F3}", $Speed))
	)

	$Process = Start-Process -FilePath $HarnessExe -ArgumentList $Args -RedirectStandardOutput $StdoutPath `
		-RedirectStandardError $StderrPath -WindowStyle Hidden -Wait -PassThru
	$Metrics = Read-MetricReport $ReportPath
	New-Object psobject -Property @{
		Name = $ScenarioInfo.Name
		Role = $ScenarioInfo.Role
		ExitCode = $Process.ExitCode
		Report = $ReportPath
		Stdout = $StdoutPath
		Stderr = $StderrPath
		Coverage = $Metrics["hidden_coverage"]
		RmsM = $Metrics["hidden_rms_error_m"]
		MaxM = $Metrics["hidden_max_error_m"]
		OrientDeg = $Metrics["hidden_orientation_rms_deg"]
		Teleports = $Metrics["hidden_continuity_teleports"]
		RecoveryMs = $Metrics["hidden_recovery_latency_ms"]
		VirtualActivated = $Metrics["virtual_activated_roles"]
		VirtualRequested = $Metrics["virtual_requested_count"]
	}
}

if ($Speed -le 0.0) {
	throw "Speed must be greater than zero."
}

$LiveSteamVr = @(Get-Process -Name "vrserver" -ErrorAction SilentlyContinue)
if ($LiveSteamVr.Count -gt 0) {
	throw "Close SteamVR before running the Phantom harness. vrserver.exe is still running."
}

$Replay = Resolve-RepoPath $ReplayPath "modules\phantom\tests\fixtures\phantom_replay_fullbody_v1.csv"
if (-not (Test-Path -LiteralPath $Replay)) {
	throw "Replay file not found: $Replay"
}

$OutDir = Resolve-RepoPath $OutputDir "build\phantom-harness"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

if ($Scenario.Count -eq 0) {
	$Scenario = @("waist-default:waist:250:650")
}
$Scenarios = @()
foreach ($Spec in $Scenario) {
	$Scenarios += New-ScenarioFromSpec $Spec
}

if (-not $SkipBuild) {
	$BuildArgs = @{ Target = @("OpenVRPairOverlay") }
	if ($SkipConfigure) { $BuildArgs["SkipConfigure"] = $true }
	& "$RepoRoot\build.ps1" @BuildArgs
	if ($LASTEXITCODE -ne 0) {
		throw "build.ps1 failed (exit $LASTEXITCODE)"
	}
}

$HarnessExe = Join-Path $RepoRoot "build\artifacts\Release\WKOpenVR.exe"
if (-not (Test-Path -LiteralPath $HarnessExe)) {
	throw "WKOpenVR.exe not found at $HarnessExe"
}

Write-Host "Replay: $Replay"
Write-Host "Output: $OutDir"
Write-Host ""

$Results = @()
foreach ($Item in $Scenarios) {
	Write-Host ("== Phantom harness: {0} ==" -f $Item.Name)
	$Results += Invoke-HarnessScenario $Item $HarnessExe $Replay $OutDir
}

Write-Host ""
Write-Host "Name,Exit,Coverage,RMS_m,Max_m,OrientRMS_deg,Teleports,Recovery_ms,Virtual"
foreach ($Result in $Results) {
	$Virtual = "{0}/{1}" -f $Result.VirtualActivated, $Result.VirtualRequested
	Write-Host ("{0},{1},{2},{3},{4},{5},{6},{7},{8}" -f $Result.Name, $Result.ExitCode, $Result.Coverage,
		$Result.RmsM, $Result.MaxM, $Result.OrientDeg, $Result.Teleports, $Result.RecoveryMs, $Virtual)
}

$Failed = @($Results | Where-Object { $_.ExitCode -ne 0 })
if ($Failed.Count -gt 0) {
	$Names = (($Failed | ForEach-Object { $_.Name }) -join ", ")
	throw "Phantom harness scenario failed: $Names"
}
