#Requires -Version 5.1
param(
	# One or more spacecal_log.*.txt files. Defaults to the newest retained logs.
	[string[]]$LogPath = @(),

	# Number of newest retained logs to summarize when LogPath is omitted.
	[int]$Latest = 1,

	# Optional CSV path for the summary rows.
	[string]$OutputCsv = ""
)

$ErrorActionPreference = "Stop"

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
	if ([System.IO.Path]::IsPathRooted($Path)) {
		return [System.IO.Path]::GetFullPath($Path)
	}
	return [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $Path))
}

function Resolve-LogPaths {
	param([string[]]$Paths, [int]$Count)
	if ($Paths.Count -gt 0) {
		$Resolved = @()
		foreach ($Path in $Paths) {
			$Full = Resolve-FullPath $Path
			if (-not (Test-Path -LiteralPath $Full)) {
				throw "Log not found: $Full"
			}
			$Resolved += $Full
		}
		return $Resolved
	}

	if ($Count -lt 1) {
		throw "Latest must be at least 1."
	}
	$LogDir = Join-Path (Get-LocalLowRoot) "WKOpenVR\Logs"
	if (-not (Test-Path -LiteralPath $LogDir)) {
		throw "WKOpenVR log directory not found: $LogDir"
	}
	$Files = @(Get-ChildItem -LiteralPath $LogDir -Filter "spacecal_log.*.txt" -File -ErrorAction SilentlyContinue |
		Sort-Object LastWriteTime -Descending | Select-Object -First $Count)
	if ($Files.Count -eq 0) {
		throw "No spacecal_log.*.txt files found under $LogDir"
	}
	return @($Files | ForEach-Object { $_.FullName })
}

function Parse-Double {
	param([string]$Text, [double]$Fallback)
	$Value = 0.0
	if ([double]::TryParse($Text, [System.Globalization.NumberStyles]::Float,
	    [System.Globalization.CultureInfo]::InvariantCulture, [ref]$Value)) {
		return $Value
	}
	return $Fallback
}

function Parse-Int {
	param([string]$Text, [int]$Fallback)
	$Value = 0
	if ([int]::TryParse($Text, [ref]$Value)) {
		return $Value
	}
	return $Fallback
}

function Extract-Timestamp {
	param([string]$Line)
	if ($Line -match '^# \[(?<t>[0-9.]+)\]') {
		return (Parse-Double $Matches["t"] 0.0)
	}
	return 0.0
}

function Extract-Field {
	param([string]$Line, [string]$Name)
	$Pattern = "(?:^|[\s\(])" + [regex]::Escape($Name) + "=([^\s\)]+)"
	$Match = [regex]::Match($Line, $Pattern)
	if ($Match.Success) {
		return $Match.Groups[1].Value.Trim("'`"")
	}
	return $null
}

function Median-Double {
	param([System.Collections.Generic.List[double]]$Values)
	if ($Values.Count -eq 0) { return "" }
	$Sorted = @($Values | Sort-Object)
	$Mid = [int]($Sorted.Count / 2)
	if (($Sorted.Count % 2) -eq 1) {
		return $Sorted[$Mid]
	}
	return (($Sorted[$Mid - 1] + $Sorted[$Mid]) / 2.0)
}

function Format-OnOff {
	param([int]$Value)
	if ($Value -eq 1) { return "on" }
	return "off"
}

function Scenario-FromToggles {
	param([int]$Reloc, [int]$Drift, [int]$Bounded, [int]$LockedSnap)
	if ($Reloc -eq 0 -and $Drift -eq 0 -and $Bounded -eq 0 -and $LockedSnap -eq 0) { return "baseline" }
	if ($Reloc -eq 1 -and $Drift -eq 0 -and $Bounded -eq 0 -and $LockedSnap -eq 0) { return "quarantine" }
	if ($Reloc -eq 0 -and $Drift -eq 0 -and $Bounded -eq 1 -and $LockedSnap -eq 0) { return "bounded_full" }
	if ($Reloc -eq 1 -and $Drift -eq 0 -and $Bounded -eq 1 -and $LockedSnap -eq 0) { return "quarantine_bounded" }
	if ($Reloc -eq 0 -and $Drift -eq 1 -and $Bounded -eq 0 -and $LockedSnap -eq 0) { return "drift_breaker" }
	if ($Reloc -eq 1 -and $Drift -eq 1 -and $Bounded -eq 0 -and $LockedSnap -eq 0) { return "quarantine_drift" }
	if ($Reloc -eq 0 -and $Drift -eq 1 -and $Bounded -eq 1 -and $LockedSnap -eq 0) { return "bounded_drift" }
	if ($Reloc -eq 1 -and $Drift -eq 1 -and $Bounded -eq 1 -and $LockedSnap -eq 0) {
		return "quarantine_bounded_drift"
	}
	if ($Reloc -eq 0 -and $Drift -eq 0 -and $Bounded -eq 0 -and $LockedSnap -eq 1) { return "locked_snap" }
	if ($Reloc -eq 1 -and $Drift -eq 1 -and $Bounded -eq 1 -and $LockedSnap -eq 1) { return "all" }
	return "custom"
}

function Reason-Summary {
	param([hashtable]$Counts)
	if ($Counts.Count -eq 0) { return "" }
	$Pairs = @()
	foreach ($Key in ($Counts.Keys | Sort-Object)) {
		$Pairs += ("{0}:{1}" -f $Key, $Counts[$Key])
	}
	return ($Pairs -join ";")
}

function Add-ReasonCount {
	param([hashtable]$Counts, [string]$Reason)
	if ([string]::IsNullOrWhiteSpace($Reason)) { return }
	if (-not $Counts.ContainsKey($Reason)) {
		$Counts[$Reason] = 0
	}
	$Counts[$Reason] = [int]$Counts[$Reason] + 1
}

function Summarize-Log {
	param([string]$Path)
	$Info = Get-Item -LiteralPath $Path
	$TranslMad = New-Object 'System.Collections.Generic.List[double]'
	$ShadowReasons = @{}

	$RelocEnabled = 0
	$DriftEnabled = 0
	$BoundedEnabled = 0
	$LockedSnapEnabled = 0
	$DumpCount = 0

	$HeartbeatCount = 0
	$LockRelHeartbeats = 0
	$FirstLockRelSec = ""
	$LastLockRel = 0
	$LastTranslMad = ""
	$MaxTranslMad = ""
	$LastTimestamp = 0.0
	$LastShadowReason = ""

	$RelocCount = 0
	$RelocMaxHmdDelta = ""
	$AutoRecoverSkipped = 0
	$AutoRecoverFired = 0
	$QuarantineDrops = 0
	$DriftFreezeEngaged = 0
	$DriftFreezeReleased = 0
	$BoundedCommonRejects = 0
	$FastReanchors = 0
	$ProfileSnapCompletes = 0

	foreach ($Line in [System.IO.File]::ReadLines($Path)) {
		if ($Line.Length -lt 3) { continue }
		if ($Line.IndexOf("# [", [System.StringComparison]::Ordinal) -ne 0) { continue }

		if ($Line.IndexOf("session_experimental_dump:", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$DumpCount
			$RelocEnabled = Parse-Int (Extract-Field $Line "reloc_quarantine") $RelocEnabled
			$DriftEnabled = Parse-Int (Extract-Field $Line "drift_breaker") $DriftEnabled
			$BoundedEnabled = Parse-Int (Extract-Field $Line "bounded_solve") $BoundedEnabled
			$LockedSnapEnabled = Parse-Int (Extract-Field $Line "locked_snap_recovery") $LockedSnapEnabled
			continue
		}

		if ($Line.IndexOf("[cal-heartbeat]", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$HeartbeatCount
			$LockRel = Parse-Int (Extract-Field $Line "lockRel") 0
			$LastLockRel = $LockRel
			if ($LockRel -eq 1) {
				++$LockRelHeartbeats
				if ($FirstLockRelSec -eq "") {
					$FirstLockRelSec = $Ts
				}
			}
			$MadField = Extract-Field $Line "translMad_mm"
			if ($null -ne $MadField) {
				$Mad = Parse-Double $MadField 0.0
				$TranslMad.Add($Mad)
				$LastTranslMad = $Mad
				if ($MaxTranslMad -eq "" -or $Mad -gt $MaxTranslMad) {
					$MaxTranslMad = $Mad
				}
			}
			continue
		}

		if ($Line.IndexOf("hmd_relocalization_detected:", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$RelocCount
			$DeltaField = Extract-Field $Line "hmdDelta"
			if ($null -ne $DeltaField) {
				$Delta = Parse-Double $DeltaField 0.0
				if ($RelocMaxHmdDelta -eq "" -or $Delta -gt $RelocMaxHmdDelta) {
					$RelocMaxHmdDelta = $Delta
				}
			}
			continue
		}

		if ($Line.IndexOf("auto_recover_skipped:", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$AutoRecoverSkipped
			continue
		}
		if ($Line.IndexOf("quest_relocalization_recovery", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$AutoRecoverFired
			continue
		}
		if ($Line.IndexOf("reloc_quarantine_sample_dropped", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$QuarantineDrops
			continue
		}
		if ($Line.IndexOf("drift_breaker_freeze_engaged", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$DriftFreezeEngaged
			continue
		}
		if ($Line.IndexOf("drift_breaker_freeze_released", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$DriftFreezeReleased
			continue
		}
		if ($Line.IndexOf("bounded_solve_common_mode_reject", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$BoundedCommonRejects
			continue
		}
		if ($Line.IndexOf("fast_reanchor requested", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$FastReanchors
			continue
		}
		if ($Line.IndexOf("profile_apply_snap_complete", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			++$ProfileSnapCompletes
			continue
		}
		if ($Line.IndexOf("[cal-shadow]", [System.StringComparison]::Ordinal) -ge 0) {
			$Ts = Extract-Timestamp $Line
			if ($Ts -gt $LastTimestamp) { $LastTimestamp = $Ts }
			$Reason = Extract-Field $Line "first_reject"
			if (-not [string]::IsNullOrWhiteSpace($Reason)) {
				$LastShadowReason = $Reason
				Add-ReasonCount $ShadowReasons $Reason
			}
			continue
		}
	}

	$Scenario = Scenario-FromToggles $RelocEnabled $DriftEnabled $BoundedEnabled $LockedSnapEnabled
	New-Object psobject -Property ([ordered]@{
		Log = $Info.FullName
		SizeMB = [Math]::Round($Info.Length / 1MB, 1)
		LastWriteTime = $Info.LastWriteTime
		DurationSec = [Math]::Round($LastTimestamp, 1)
		Scenario = $Scenario
		ExperimentalDumpCount = $DumpCount
		RelocQuarantine = Format-OnOff $RelocEnabled
		DriftBreaker = Format-OnOff $DriftEnabled
		BoundedSolve = Format-OnOff $BoundedEnabled
		LockedSnapRecovery = Format-OnOff $LockedSnapEnabled
		Relocalizations = $RelocCount
		RelocMaxHmdDeltaM = $RelocMaxHmdDelta
		AutoRecoverSkipped = $AutoRecoverSkipped
		AutoRecoverFired = $AutoRecoverFired
		QuarantineDrops = $QuarantineDrops
		DriftFreezeEngaged = $DriftFreezeEngaged
		DriftFreezeReleased = $DriftFreezeReleased
		BoundedCommonRejects = $BoundedCommonRejects
		FastReanchors = $FastReanchors
		ProfileSnapCompletes = $ProfileSnapCompletes
		HeartbeatCount = $HeartbeatCount
		LockRelHeartbeats = $LockRelHeartbeats
		FirstLockRelSec = $FirstLockRelSec
		LastLockRel = $LastLockRel
		MaxTranslMadMm = $MaxTranslMad
		MedianTranslMadMm = Median-Double $TranslMad
		LastTranslMadMm = $LastTranslMad
		LastShadowReason = $LastShadowReason
		ShadowReasonCounts = Reason-Summary $ShadowReasons
	})
}

$ResolvedLogs = Resolve-LogPaths $LogPath $Latest
$Rows = @()
foreach ($Log in $ResolvedLogs) {
	Write-Host "Scanning $Log"
	$Rows += Summarize-Log $Log
}

Write-Host ""
$Rows | Format-Table -AutoSize Scenario,RelocQuarantine,BoundedSolve,DriftBreaker,LockedSnapRecovery,Relocalizations,QuarantineDrops,DriftFreezeEngaged,LockRelHeartbeats,MaxTranslMadMm,MedianTranslMadMm,LastTranslMadMm,LastShadowReason

Write-Host ""
$Rows | Format-List

if (-not [string]::IsNullOrWhiteSpace($OutputCsv)) {
	$Full = Resolve-FullPath $OutputCsv
	$Dir = Split-Path -Parent $Full
	if (-not [string]::IsNullOrWhiteSpace($Dir)) {
		New-Item -ItemType Directory -Force -Path $Dir | Out-Null
	}
	$Rows | Export-Csv -LiteralPath $Full -NoTypeInformation
	Write-Host "Wrote $Full"
}
