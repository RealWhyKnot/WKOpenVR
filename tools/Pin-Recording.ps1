#Requires -Version 5.1
param(
	# Recording prefix under the logs directory (e.g. "phantom_replay", "spacecal_log").
	[Parameter(Mandatory = $true)]
	[string]$Prefix,

	# Wildcard file name(s) to pin (e.g. "phantom_replay.2026-07-03*").
	# Defaults to the newest retained capture for the prefix.
	[string[]]$Name = @(),

	# Short note stored with the manifest entry (what makes this capture worth keeping).
	[string]$Why = "",

	# Logs directory override. Defaults to %LocalAppDataLow%\WKOpenVR\Logs.
	[string]$LogDir = ""
)

# Recording retention prunes Logs\<prefix>.*.<ext> by count/size; it never looks inside
# subdirectories, so copies placed in Logs\corpus\ survive rotation indefinitely.

$ErrorActionPreference = "Stop"

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
	throw "Unable to resolve the WKOpenVR logs directory; pass -LogDir."
}

if ([string]::IsNullOrWhiteSpace($LogDir)) {
	$LogDir = Get-DefaultLogDir
}
if (-not (Test-Path $LogDir)) {
	throw "Logs directory not found: $LogDir"
}

$Candidates = @(Get-ChildItem -Path $LogDir -File -Filter "$Prefix.*")
if ($Name.Count -gt 0) {
	$Picked = @()
	foreach ($Pattern in $Name) {
		$Matched = @($Candidates | Where-Object { $_.Name -like $Pattern })
		if ($Matched.Count -eq 0) {
			throw "No recording matches '$Pattern' in $LogDir"
		}
		$Picked += $Matched
	}
	$Picked = @($Picked | Sort-Object FullName -Unique)
} else {
	$Picked = @($Candidates | Sort-Object LastWriteTime -Descending | Select-Object -First 1)
	if ($Picked.Count -eq 0) {
		throw "No $Prefix.* recordings in $LogDir"
	}
}

$CorpusDir = Join-Path $LogDir "corpus"
if (-not (Test-Path $CorpusDir)) {
	New-Item -ItemType Directory -Path $CorpusDir | Out-Null
}
$ManifestPath = Join-Path $CorpusDir "manifest.txt"

foreach ($File in $Picked) {
	$Dest = Join-Path $CorpusDir $File.Name
	if ((Test-Path $Dest) -and ((Get-Item $Dest).Length -eq $File.Length)) {
		Write-Host "Already pinned: $($File.Name)"
		continue
	}
	Copy-Item -Path $File.FullName -Destination $Dest -Force
	$Sha = (Get-FileHash -Path $Dest -Algorithm SHA256).Hash
	$Mtime = $File.LastWriteTime.ToString("yyyy-MM-ddTHH:mm:ss")
	Add-Content -Path $ManifestPath -Value "$($File.Name)`tbytes=$($File.Length)`tmtime=$Mtime`tsha256=$Sha`tnote=$Why"
	Write-Host ("Pinned: {0} ({1:N1} MB)" -f $File.Name, ($File.Length / 1MB))
}
Write-Host "Corpus: $CorpusDir"
