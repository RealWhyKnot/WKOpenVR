<#
    reload.ps1 -- developer hot-reload for iterating while in VR.

    One entry point for the four reload paths, lowest interruption first:

      .\reload.ps1 Tuning              Create/open dev_tuning.ini (edit values;
                                       the driver re-reads it in ~1s, no restart).
      .\reload.ps1 Overlay            Rebuild WKOpenVR.exe and hand it to the
                                       running overlay to swap itself in. Stays
                                       in VR (SteamVR/driver untouched).
      .\reload.ps1 Sidecar -Sidecar face|captions|phantom
                                       Rebuild a host and drop it next to the
                                       driver; the driver's supervisor respawns
                                       it. Stays in VR.
      .\reload.ps1 Driver             Rebuild the driver DLL and redeploy via
                                       quick.ps1. This restarts SteamVR (~15s VR
                                       blip) -- the driver DLL cannot be swapped
                                       while vrserver holds it.

    Overlay/Sidecar/Tuning write into Program Files / the SteamVR drivers dir, so
    run this from an ELEVATED PowerShell (Overlay and Sidecar staging need it).
    All hot-reload behavior is dev-build only; a release driver ignores it.
#>
param(
	[Parameter(Mandatory = $true, Position = 0)]
	[ValidateSet("Driver", "Overlay", "Sidecar", "Tuning")]
	[string]$What,

	# Required for -What Sidecar: which host to reload.
	[ValidateSet("face", "captions", "phantom")]
	[string]$Sidecar = "",

	# Run a full CMake configure before building (default reuses the build dir for
	# speed; use this the first time or after adding/removing source files).
	[switch]$Configure,

	# Reload whatever is already under build/ without rebuilding.
	[switch]$SkipBuild,

	# Overrides matching quick.ps1's defaults.
	[string]$InstallDir = "C:\Program Files\WKOpenVR",
	[string]$SteamVRDriversDir = ""
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$sidecarInfo = @{
	face     = @{ TargetName = "openvr_pair_feature_facetracking_host"; Exe = "WKOpenVR.FaceModuleHost.exe"; Sub = "facetracking" }
	captions = @{ TargetName = "WKOpenVRCaptionsHost"; Exe = "WKOpenVR.CaptionsHost.exe"; Sub = "captions" }
	phantom  = @{ TargetName = "WKOpenVRPhantomSidecar"; Exe = "WKOpenVRPhantomSidecar.exe"; Sub = "phantom" }
}

function Invoke-BuildTarget {
	param([Parameter(Mandatory = $true)][string]$Name)
	if ($SkipBuild) {
		Write-Host "Skipping build ($Name); using existing artifacts."
		return
	}
	$buildArgs = @("-Target", $Name)
	if (-not $Configure) {
		$buildArgs = @("-SkipConfigure") + $buildArgs
	}
	Write-Host "Building target $Name ..."
	& "$PSScriptRoot\build.ps1" @buildArgs
}

function Resolve-DriversDir {
	param([string]$Override)
	if ($Override) { return $Override }
	$candidates = @()
	foreach ($rp in @("HKCU:\Software\Valve\Steam", "HKLM:\SOFTWARE\Wow6432Node\Valve\Steam")) {
		try {
			$sp = (Get-ItemProperty -Path $rp -ErrorAction Stop).SteamPath
			if ($sp) {
				$candidates += (Join-Path ($sp -replace '/', '\') "steamapps\common\SteamVR\drivers")
			}
		}
		catch { }
	}
	$candidates += "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers"
	foreach ($c in $candidates) {
		if (Test-Path -LiteralPath (Join-Path $c "01wkopenvr")) { return $c }
	}
	throw "Deployed 01wkopenvr driver not found. Run quick.ps1 once first, or pass -SteamVRDriversDir."
}

# Replace a file that may be running: copy the new build to a temp name in the
# destination directory first (the slow part), then rename the current one aside
# (allowed while in use on Windows) and rename the temp into place. Renames are
# atomic on the same volume, so watchers never see a half-written or missing
# exe. The old copy is deletable once its process exits and is cleared on the
# next reload.
function Update-DeployedExe {
	param(
		[Parameter(Mandatory = $true)][string]$Dest,
		[Parameter(Mandatory = $true)][string]$Src
	)
	if (-not (Test-Path -LiteralPath $Src)) { throw "Built artifact missing: $Src" }
	$old = "$Dest.old"
	$tmp = "$Dest.tmp"
	if (Test-Path -LiteralPath $old) {
		Remove-Item -LiteralPath $old -Force -ErrorAction SilentlyContinue
	}
	Copy-Item -LiteralPath $Src -Destination $tmp -Force
	if (Test-Path -LiteralPath $Dest) {
		Move-Item -LiteralPath $Dest -Destination $old -Force
	}
	Move-Item -LiteralPath $tmp -Destination $Dest -Force
}

# Copy $Src into $Dir under $Name via a temp file + rename, so the watcher in
# the running overlay only ever sees a complete staged exe appear. Throws a
# clear elevation hint when the directory is not writable.
function Copy-StagedExe {
	param(
		[Parameter(Mandatory = $true)][string]$Dir,
		[Parameter(Mandatory = $true)][string]$Name,
		[Parameter(Mandatory = $true)][string]$Src
	)
	$tmp = Join-Path $Dir "$Name.tmp"
	try {
		Copy-Item -LiteralPath $Src -Destination $tmp -Force -ErrorAction Stop
	}
	catch {
		throw "Cannot write to $Dir ($($_.Exception.Message)). Run this from an elevated PowerShell."
	}
	Move-Item -LiteralPath $tmp -Destination (Join-Path $Dir $Name) -Force
}

switch ($What) {
	"Driver" {
		Invoke-BuildTarget "OpenVRPairDriver"
		Write-Host "Redeploying driver via quick.ps1 (this restarts SteamVR)..."
		& "$PSScriptRoot\quick.ps1" -SkipBuild -Yes
	}

	"Overlay" {
		Invoke-BuildTarget "OpenVRPairOverlay"
		$src = Join-Path $PSScriptRoot "build\artifacts\Release\WKOpenVR.exe"
		if (-not (Test-Path -LiteralPath $src)) { throw "Built overlay missing: $src" }
		$running = @(Get-Process -Name "WKOpenVR" -ErrorAction SilentlyContinue)
		if ($running.Count -eq 0) {
			$canonical = Join-Path $InstallDir "WKOpenVR.exe"
			Update-DeployedExe -Dest $canonical -Src $src
			Write-Host "Overlay not running; copied new build to $canonical. Launch it when ready."
		}
		else {
			# Stage next to the exe each instance is actually running from, not a
			# fixed install dir -- a staged file the running overlay never sees
			# would silently do nothing.
			$dirs = @($running | ForEach-Object { $_.Path } | Where-Object { $_ } |
					Split-Path -Parent | Sort-Object -Unique)
			if ($dirs.Count -eq 0) { $dirs = @($InstallDir) }
			foreach ($dir in $dirs) {
				if ($dir -ne $InstallDir) {
					Write-Host "Note: overlay is running from $dir; staging there."
				}
				Copy-StagedExe -Dir $dir -Name "WKOpenVR.new.exe" -Src $src
				Write-Host "Staged new overlay in $dir; waiting for the running instance to swap itself in..."
			}

			# Watch the handover: staged file consumed -> old instance exits ->
			# a new instance appears. Report each step so a stalled reload is
			# visible instead of silently doing nothing.
			$oldPids = @($running | ForEach-Object { $_.Id })
			$deadline = (Get-Date).AddSeconds(20)
			$consumed = $false
			$oldGone = $false
			$newUp = $false
			while ((Get-Date) -lt $deadline) {
				if (-not $consumed) {
					$left = @($dirs | Where-Object { Test-Path -LiteralPath (Join-Path $_ "WKOpenVR.new.exe") })
					if ($left.Count -eq 0) {
						$consumed = $true
						Write-Host "Swap picked up by the running overlay."
					}
				}
				if ($consumed -and -not $oldGone) {
					$stillOld = @(Get-Process -Name "WKOpenVR" -ErrorAction SilentlyContinue |
							Where-Object { $oldPids -contains $_.Id })
					if ($stillOld.Count -eq 0) {
						$oldGone = $true
						Write-Host "Previous overlay instance exited."
					}
				}
				if ($oldGone -and -not $newUp) {
					$newProcs = @(Get-Process -Name "WKOpenVR" -ErrorAction SilentlyContinue |
							Where-Object { $oldPids -notcontains $_.Id })
					if ($newProcs.Count -gt 0) {
						$newUp = $true
						Write-Host ("Overlay swap finished; WKOpenVR is running (pid {0})." -f $newProcs[0].Id)
					}
				}
				if ($newUp) { break }
				Start-Sleep -Milliseconds 250
			}
			if ($newUp) {
				Write-Host "If the overlay shows a driver version error, the change touched the driver: run .\reload.ps1 Driver."
			}
			else {
				# Remove any unconsumed staged exe so it cannot trigger a
				# surprise swap much later.
				foreach ($dir in $dirs) {
					$staged = Join-Path $dir "WKOpenVR.new.exe"
					if (Test-Path -LiteralPath $staged) {
						Remove-Item -LiteralPath $staged -Force -ErrorAction SilentlyContinue
					}
				}
				if (-not $consumed) {
					Write-Warning "The running overlay never picked up the staged build (staged file removed). Check that it is a dev-channel build running from the directory above."
				}
				elseif (-not $oldGone) {
					Write-Warning "The overlay picked up the staged build but did not exit; check the diagnostics log."
				}
				else {
					Write-Warning "The previous overlay exited but no new instance appeared; check the diagnostics log and launch WKOpenVR.exe manually if needed."
				}
			}
		}
	}

	"Sidecar" {
		if (-not $Sidecar) { throw "Pass -Sidecar face|captions|phantom" }
		$info = $sidecarInfo[$Sidecar]
		Invoke-BuildTarget $info.TargetName
		$src = Join-Path $PSScriptRoot ("build\driver_wkopenvr\resources\{0}\host\{1}" -f $info.Sub, $info.Exe)
		$driversDir = Resolve-DriversDir -Override $SteamVRDriversDir
		$destDir = Join-Path $driversDir ("01wkopenvr\resources\{0}\host" -f $info.Sub)
		if (-not (Test-Path -LiteralPath $destDir)) {
			New-Item -ItemType Directory -Force -Path $destDir | Out-Null
		}
		$dest = Join-Path $destDir $info.Exe
		Update-DeployedExe -Dest $dest -Src $src
		Write-Host "Reloaded $Sidecar host at $dest; the driver's supervisor respawns it (~1s). Stay in VR."
	}

	"Tuning" {
		$driversDir = Resolve-DriversDir -Override $SteamVRDriversDir
		$resDir = Join-Path $driversDir "01wkopenvr\resources"
		if (-not (Test-Path -LiteralPath $resDir)) { throw "Driver resources dir not found: $resDir" }
		$ini = Join-Path $resDir "dev_tuning.ini"
		if (-not (Test-Path -LiteralPath $ini)) {
			$template = @"
# WKOpenVR dev live-tuning (dev builds only). Edit a value and save; the driver
# re-reads this file within ~1s while SteamVR keeps running. Delete a line to
# restore its built-in default; delete this file to restore all defaults.
#
#   key = value        (# or ; begins a comment)

# Max finger-smoothing strength at smoothness=100 (built-in default 0.95).
smoothing.finger_max_strength = 0.95
"@
			[System.IO.File]::WriteAllText($ini, $template, (New-Object System.Text.UTF8Encoding($false)))
			Write-Host "Created $ini"
		}
		else {
			Write-Host "Opening existing $ini"
		}
		Start-Process notepad.exe -ArgumentList $ini
	}
}

Write-Host "Done."
