#Requires -Version 5.1
param(
	[ValidateSet(
		"show",
		"baseline",
		"quarantine",
		"bounded",
		"bounded_full",
		"quarantine_bounded",
		"drift",
		"drift_breaker",
		"quarantine_drift",
		"bounded_drift",
		"quarantine_bounded_drift",
		"locked_snap",
		"all"
	)]
	[string]$Scenario = "show",

	# Read a profile JSON blob from a file instead of the live registry.
	[string]$InputPath = "",

	# Write the modified profile JSON blob to a file instead of the live registry.
	[string]$OutputPath = "",

	# Exact raw-profile backup path. When omitted, applying to the registry writes
	# one under %LocalAppDataLow%\WKOpenVR\calibration_experiment_backups.
	[string]$BackupPath = "",

	# Restore this exact raw-profile backup to the live registry.
	[string]$RestorePath = "",

	# Required for any registry or file write. Without this switch, the script only previews.
	[switch]$Apply,

	# Create the shared debug logging opt-in flag for release builds.
	[switch]$EnableDebugLogging
)

$ErrorActionPreference = "Stop"

$RegistryPath = "Registry::HKEY_CURRENT_USER\Software\Classes\Local Settings\Software\WKOpenVR-SpaceCalibrator"
$RegistryValueName = "Config"

$ExperimentalKeys = @(
	"experimental_reloc_quarantine",
	"experimental_reloc_quarantine_sec",
	"experimental_drift_breaker",
	"experimental_drift_breaker_mad_mult",
	"experimental_drift_breaker_abs_cap_mm",
	"experimental_bounded_solve",
	"experimental_bounded_solve_prior",
	"experimental_bounded_solve_prior_lambda",
	"experimental_bounded_solve_slew",
	"experimental_bounded_solve_max_step_mm",
	"experimental_bounded_solve_max_step_deg",
	"experimental_bounded_solve_common_mode",
	"experimental_locked_snap_recovery"
)

function Get-LocalLowRoot {
	if (-not [string]::IsNullOrWhiteSpace($env:LOCALAPPDATALOW)) {
		return $env:LOCALAPPDATALOW
	}
	if (-not [string]::IsNullOrWhiteSpace($env:LOCALAPPDATA)) {
		$Parent = [System.IO.Directory]::GetParent($env:LOCALAPPDATA)
		if ($Parent) {
			return (Join-Path $Parent.FullName "LocalLow")
		}
	}
	throw "Could not resolve LocalLow."
}

function Resolve-FullPath {
	param([string]$Path)
	return [System.IO.Path]::GetFullPath($Path)
}

function Read-ExactText {
	param([string]$Path)
	return [System.IO.File]::ReadAllText((Resolve-FullPath $Path))
}

function Write-ExactText {
	param([string]$Path, [string]$Text)
	$Full = Resolve-FullPath $Path
	$Dir = Split-Path -Parent $Full
	if (-not [string]::IsNullOrWhiteSpace($Dir)) {
		New-Item -ItemType Directory -Force -Path $Dir | Out-Null
	}
	$Utf8NoBom = New-Object System.Text.UTF8Encoding($false)
	[System.IO.File]::WriteAllText($Full, $Text, $Utf8NoBom)
}

function Get-DefaultBackupPath {
	$Dir = Join-Path (Get-LocalLowRoot) "WKOpenVR\calibration_experiment_backups"
	$Stamp = Get-Date -Format "yyyyMMdd-HHmmss"
	return (Join-Path $Dir ("spacecal-config-{0}.json" -f $Stamp))
}

function Read-LiveConfig {
	if (-not (Test-Path -LiteralPath $RegistryPath)) {
		throw "Calibration registry key not found: $RegistryPath"
	}
	$Item = Get-ItemProperty -LiteralPath $RegistryPath -Name $RegistryValueName -ErrorAction Stop
	$Value = $Item.$RegistryValueName
	if ([string]::IsNullOrWhiteSpace($Value)) {
		throw "Calibration registry value '$RegistryValueName' is empty."
	}
	return [string]$Value
}

function Write-LiveConfig {
	param([string]$Text)
	New-Item -Path $RegistryPath -Force | Out-Null
	New-ItemProperty -LiteralPath $RegistryPath -Name $RegistryValueName -Value $Text -PropertyType String -Force |
		Out-Null
}

function ConvertFrom-ProfileJson {
	param([string]$Text)
	$Parsed = ConvertFrom-Json -InputObject $Text
	$Profiles = @($Parsed)
	if ($Profiles.Count -lt 1) {
		throw "Calibration profile JSON must be a non-empty array."
	}
	return $Profiles
}

function ConvertTo-ProfileJson {
	param([object[]]$Profiles)
	return (ConvertTo-Json -InputObject $Profiles -Depth 100)
}

function Get-ProfileValue {
	param([object]$Profile, [string]$Name, [object]$Fallback)
	$Prop = $Profile.PSObject.Properties[$Name]
	if ($null -eq $Prop) { return $Fallback }
	return $Prop.Value
}

function Set-ProfileValue {
	param([object]$Profile, [string]$Name, [object]$Value)
	$Prop = $Profile.PSObject.Properties[$Name]
	if ($null -eq $Prop) {
		Add-Member -InputObject $Profile -MemberType NoteProperty -Name $Name -Value $Value
	}
	else {
		$Prop.Value = $Value
	}
}

function Remove-ProfileValue {
	param([object]$Profile, [string]$Name)
	$Profile.PSObject.Properties.Remove($Name)
}

function Reset-ExperimentalKeys {
	param([object]$Profile)
	foreach ($Name in $ExperimentalKeys) {
		Remove-ProfileValue $Profile $Name
	}
}

function Set-ExperimentalDefaults {
	param([object]$Profile, [bool]$Reloc, [bool]$Drift, [bool]$Bounded, [bool]$LockedSnap)
	Reset-ExperimentalKeys $Profile

	if ($Reloc) {
		Set-ProfileValue $Profile "experimental_reloc_quarantine" $true
		Set-ProfileValue $Profile "experimental_reloc_quarantine_sec" 1.0
	}
	if ($Drift) {
		Set-ProfileValue $Profile "experimental_drift_breaker" $true
		Set-ProfileValue $Profile "experimental_drift_breaker_mad_mult" 8.0
		Set-ProfileValue $Profile "experimental_drift_breaker_abs_cap_mm" 60.0
	}
	if ($Bounded) {
		Set-ProfileValue $Profile "experimental_bounded_solve" $true
		Set-ProfileValue $Profile "experimental_bounded_solve_prior" $false
		Set-ProfileValue $Profile "experimental_bounded_solve_slew" $true
		Set-ProfileValue $Profile "experimental_bounded_solve_max_step_mm" 50.0
		Set-ProfileValue $Profile "experimental_bounded_solve_max_step_deg" 2.0
		Set-ProfileValue $Profile "experimental_bounded_solve_common_mode" $true
	}
	if ($LockedSnap) {
		Set-ProfileValue $Profile "experimental_locked_snap_recovery" $true
	}
}

function Apply-Scenario {
	param([object]$Profile, [string]$Name)
	switch ($Name) {
		"baseline" { Set-ExperimentalDefaults $Profile $false $false $false $false }
		"quarantine" { Set-ExperimentalDefaults $Profile $true $false $false $false }
		"bounded" { Set-ExperimentalDefaults $Profile $false $false $true $false }
		"bounded_full" { Set-ExperimentalDefaults $Profile $false $false $true $false }
		"bounded_prior" {
			Set-ExperimentalDefaults $Profile $false $false $false $false
			Set-ProfileValue $Profile "experimental_bounded_solve" $true
			Set-ProfileValue $Profile "experimental_bounded_solve_prior" $true
			Set-ProfileValue $Profile "experimental_bounded_solve_prior_lambda" 0.2
		}
		"quarantine_bounded" { Set-ExperimentalDefaults $Profile $true $false $true $false }
		"drift" { Set-ExperimentalDefaults $Profile $false $true $false $false }
		"drift_breaker" { Set-ExperimentalDefaults $Profile $false $true $false $false }
		"quarantine_drift" { Set-ExperimentalDefaults $Profile $true $true $false $false }
		"bounded_drift" { Set-ExperimentalDefaults $Profile $false $true $true $false }
		"quarantine_bounded_drift" { Set-ExperimentalDefaults $Profile $true $true $true $false }
		"locked_snap" { Set-ExperimentalDefaults $Profile $false $false $false $true }
		"all" { Set-ExperimentalDefaults $Profile $true $true $true $true }
		default { throw "Cannot apply scenario '$Name'." }
	}
}

function Format-Bool {
	param([object]$Value)
	if ($Value -eq $true) { return "on" }
	return "off"
}

function Show-ProfileState {
	param([object]$Profile)
	Write-Host ("reloc_quarantine={0} sec={1}" -f
		(Format-Bool (Get-ProfileValue $Profile "experimental_reloc_quarantine" $false)),
		(Get-ProfileValue $Profile "experimental_reloc_quarantine_sec" 1.0))
	Write-Host ("drift_breaker={0} mult={1} cap_mm={2}" -f
		(Format-Bool (Get-ProfileValue $Profile "experimental_drift_breaker" $false)),
		(Get-ProfileValue $Profile "experimental_drift_breaker_mad_mult" 8.0),
		(Get-ProfileValue $Profile "experimental_drift_breaker_abs_cap_mm" 60.0))
	Write-Host ("bounded_solve={0} prior={1} slew={2} common_mode={3}" -f
		(Format-Bool (Get-ProfileValue $Profile "experimental_bounded_solve" $false)),
		(Format-Bool (Get-ProfileValue $Profile "experimental_bounded_solve_prior" $false)),
		(Format-Bool (Get-ProfileValue $Profile "experimental_bounded_solve_slew" $false)),
		(Format-Bool (Get-ProfileValue $Profile "experimental_bounded_solve_common_mode" $false)))
	Write-Host ("locked_snap_recovery={0}" -f
		(Format-Bool (Get-ProfileValue $Profile "experimental_locked_snap_recovery" $false)))
}

function Set-DebugLoggingFlag {
	$Path = Join-Path (Get-LocalLowRoot) "WKOpenVR\debug_logging.enabled"
	Write-ExactText $Path "1`r`n"
	return $Path
}

if (-not [string]::IsNullOrWhiteSpace($RestorePath)) {
	if (-not [string]::IsNullOrWhiteSpace($InputPath) -or -not [string]::IsNullOrWhiteSpace($OutputPath) -or
	    $Scenario -ne "show") {
		throw "RestorePath cannot be combined with InputPath, OutputPath, or a scenario."
	}
	$RestoreText = Read-ExactText $RestorePath
	$Profiles = ConvertFrom-ProfileJson $RestoreText
	Write-Host "Restore preview:"
	Show-ProfileState $Profiles[0]
	if ($Apply) {
		Write-LiveConfig $RestoreText
		Write-Host "Restored live calibration profile from $RestorePath"
	}
	else {
		Write-Host "Preview only. Add -Apply to restore this backup."
	}
	return
}

$UsesRegistry = [string]::IsNullOrWhiteSpace($InputPath)
$Raw = if ($UsesRegistry) { Read-LiveConfig } else { Read-ExactText $InputPath }
$Profiles = ConvertFrom-ProfileJson $Raw
$Profile = $Profiles[0]

Write-Host "Current experimental state:"
Show-ProfileState $Profile

if ($Scenario -ne "show") {
	Apply-Scenario $Profile $Scenario
	Write-Host ""
	Write-Host ("Scenario preview: {0}" -f $Scenario)
	Show-ProfileState $Profile

	if ($Apply) {
		$NextJson = ConvertTo-ProfileJson $Profiles
		if ($UsesRegistry) {
			$Backup = $BackupPath
			if ([string]::IsNullOrWhiteSpace($Backup)) {
				$Backup = Get-DefaultBackupPath
			}
			Write-ExactText $Backup $Raw
			Write-LiveConfig $NextJson
			Write-Host "Wrote live calibration profile."
			Write-Host "Backup: $Backup"
		}
		else {
			if ([string]::IsNullOrWhiteSpace($OutputPath)) {
				throw "OutputPath is required when applying a scenario to InputPath."
			}
			Write-ExactText $OutputPath $NextJson
			Write-Host "Wrote $OutputPath"
		}
	}
	else {
		Write-Host "Preview only. Add -Apply to write the scenario."
	}
}

if ($EnableDebugLogging) {
	if ($Apply) {
		$Flag = Set-DebugLoggingFlag
		Write-Host "Enabled debug logging flag: $Flag"
	}
	else {
		$Flag = Join-Path (Get-LocalLowRoot) "WKOpenVR\debug_logging.enabled"
		Write-Host "Debug logging preview: would create $Flag"
	}
}
