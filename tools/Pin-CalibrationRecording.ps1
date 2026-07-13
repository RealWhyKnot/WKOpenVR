#Requires -Version 5.1
<#
.SYNOPSIS
Pins a spacecal_log recording into the corpus folder so retention never deletes it.
.DESCRIPTION
Convenience wrapper around Pin-Recording.ps1 with -Prefix spacecal_log, the most
common capture type. See Pin-Recording.ps1 for the pinning mechanics.
.EXAMPLE
./Pin-CalibrationRecording.ps1 -Why "baseline walk capture"
#>
[CmdletBinding()]
param(
	# Wildcard file name(s) under the logs directory to pin (e.g. "spacecal_log.2026-07-02*").
	# Defaults to the newest retained capture.
	[string[]]$Name = @(),

	# Short note stored with the manifest entry (what makes this capture worth keeping).
	[string]$Why = "",

	# Logs directory override. Defaults to %LocalAppDataLow%\WKOpenVR\Logs.
	[string]$LogDir = ""
)

$ErrorActionPreference = "Stop"

& (Join-Path $PSScriptRoot "Pin-Recording.ps1") -Prefix spacecal_log -Name $Name -Why $Why -LogDir $LogDir
