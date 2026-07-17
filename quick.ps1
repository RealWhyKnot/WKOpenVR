#Requires -Version 5.1
<#
.SYNOPSIS
Builds, validates, and deploys WKOpenVR to the local install and SteamVR driver directories.
.DESCRIPTION
Runs build.ps1 (unless skipped), validates the artifacts under build/, then copies them
to the install directory and the SteamVR drivers directory and verifies the copies.
Stops Steam/SteamVR before deploying and relaunches SteamVR afterward unless disabled.
Refuses to deploy while VRChat is running unless explicitly allowed.
.EXAMPLE
./quick.ps1 -Yes
#>
[CmdletBinding()]
param(
	# Skip the build and deploy the artifacts currently under build/.
	[switch]$SkipBuild,

	# Pass through to build.ps1 when building.
	[switch]$SkipConfigure,

	# Do not prompt before stopping Steam/SteamVR and launching the elevated copy.
	[switch]$Yes,

	# Validate artifacts and print the deploy file count without stopping or copying.
	[switch]$DryRun,

	# Copy and verify, but do not relaunch SteamVR afterward.
	[switch]$NoRestart,

	# Deploy even if VRChat is running. By default the script refuses to stop
	# SteamVR/Steam while VRChat is open so a deploy never kills a game session.
	[switch]$AllowGameRunning,

	# Override the deployed install dir.
	[string]$InstallDir = "C:\Program Files\WKOpenVR",

	# Override the SteamVR drivers dir. When omitted, the script derives it from
	# the Steam install path and falls back to the standard Steam location.
	[string]$SteamVRDriversDir = "",

	# Override steam.exe. When omitted, the script reads the Steam registry keys
	# and falls back to the standard Steam location.
	[string]$SteamExe = "",

	# Build a CPU-only captions host (not recommended). By default the host is
	# built with the Vulkan GPU backend; build.ps1 offers to install the Vulkan
	# SDK when it is missing. Passed through to build.ps1.
	[switch]$CaptionsCpuOnly,

	# Install the Vulkan SDK without prompting (for non-interactive builds).
	[switch]$InstallVulkanSdk,

	# Install the Vulkan SDK into this directory instead of the default
	# C:\VulkanSDK\<version>. The install runs elevated (a UAC prompt appears).
	[string]$VulkanSdkRoot = ""
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

function Write-Step {
	param([Parameter(Mandatory=$true)][string]$Message)
	Write-Host ""
	Write-Host ("== {0} ==" -f $Message)
}

function Get-NormalizedPath {
	param([string]$Path)
	if (-not $Path) { return "" }
	return $Path.Replace('/', '\').TrimEnd('\')
}

function Get-Sha {
	param([Parameter(Mandatory=$true)][string]$Path)
	if (-not (Test-Path -LiteralPath $Path)) { return $null }
	return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash
}

function Assert-File {
	param(
		[Parameter(Mandatory=$true)][string]$Path,
		[Parameter(Mandatory=$true)][string]$Label
	)
	if (-not (Test-Path -LiteralPath $Path)) {
		throw "$Label missing: $Path"
	}
}

function Resolve-Version {
	param([Parameter(Mandatory=$true)][string]$ExePath)
	if (-not (Test-Path -LiteralPath $ExePath)) { return "(missing)" }
	try {
		$info = (Get-Item -LiteralPath $ExePath).VersionInfo
		if ($info.FileVersion) { return $info.FileVersion }
	} catch { }
	$versionFile = Join-Path $PSScriptRoot "version.txt"
	if (Test-Path -LiteralPath $versionFile) {
		$stamp = (Get-Content -LiteralPath $versionFile -Raw).Trim()
		if ($stamp) { return $stamp }
	}
	return "(unknown)"
}

function Resolve-SteamExe {
	param([string]$Override)

	$candidates = @()
	if ($Override) { $candidates += $Override }

	$regPaths = @(
		"HKCU:\Software\Valve\Steam",
		"HKLM:\SOFTWARE\Wow6432Node\Valve\Steam",
		"HKLM:\SOFTWARE\Valve\Steam"
	)
	foreach ($regPath in $regPaths) {
		try {
			$props = Get-ItemProperty -Path $regPath -ErrorAction Stop
			foreach ($name in @("SteamExe", "InstallPath", "SteamPath")) {
				$value = $props.$name
				if (-not $value) { continue }
				$value = Get-NormalizedPath $value
				if ([IO.Path]::GetFileName($value) -ieq "steam.exe") {
					$candidates += $value
				} else {
					$candidates += (Join-Path $value "steam.exe")
				}
			}
		} catch { }
	}

	$candidates += "C:\Program Files (x86)\Steam\steam.exe"
	$candidates += "C:\Program Files\Steam\steam.exe"

	foreach ($candidate in $candidates) {
		if ($candidate -and (Test-Path -LiteralPath $candidate)) {
			return (Resolve-Path -LiteralPath $candidate).ProviderPath
		}
	}

	return ""
}

function Resolve-SteamVRDriversDir {
	param(
		[string]$Override,
		[string]$ResolvedSteamExe
	)

	if ($Override) { return (Get-NormalizedPath $Override) }

	if ($ResolvedSteamExe) {
		$steamRoot = Split-Path -Parent $ResolvedSteamExe
		$candidate = Join-Path $steamRoot "steamapps\common\SteamVR\drivers"
		if (Test-Path -LiteralPath $candidate) { return $candidate }
	}

	return "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers"
}

function Wait-ForProcessesToExit {
	param(
		[Parameter(Mandatory=$true)][int[]]$Ids,
		[int]$Seconds = 8
	)

	$deadline = (Get-Date).AddSeconds($Seconds)
	while ((Get-Date) -lt $deadline) {
		$alive = @()
		foreach ($id in $Ids) {
			$p = Get-Process -Id $id -ErrorAction SilentlyContinue
			if ($p) { $alive += $p }
		}
		if ($alive.Count -eq 0) { return $true }
		Start-Sleep -Milliseconds 250
	}
	return $false
}

function Test-NamedProcess {
	param([Parameter(Mandatory=$true)][string[]]$Names)

	$nameSet = @{}
	foreach ($name in $Names) { $nameSet[$name.ToLowerInvariant()] = $true }

	$process = Get-Process -ErrorAction SilentlyContinue | Where-Object {
		$nameSet.ContainsKey($_.ProcessName.ToLowerInvariant())
	} | Select-Object -First 1

	return $null -ne $process
}

function Stop-NamedProcesses {
	param(
		[Parameter(Mandatory=$true)][string]$Label,
		[Parameter(Mandatory=$true)][string[]]$Names
	)

	$nameSet = @{}
	foreach ($name in $Names) { $nameSet[$name.ToLowerInvariant()] = $true }

	$processes = @(Get-Process -ErrorAction SilentlyContinue | Where-Object {
		$nameSet.ContainsKey($_.ProcessName.ToLowerInvariant())
	})

	if ($processes.Count -eq 0) {
		Write-Host "$Label not running."
		return
	}

	Write-Host ("Stopping {0}: {1}" -f $Label, (($processes | Select-Object -ExpandProperty ProcessName -Unique) -join ", "))

	$windowed = @($processes | Where-Object { $_.MainWindowHandle -ne 0 })
	foreach ($p in $windowed) {
		try { [void]$p.CloseMainWindow() } catch { }
	}

	$ids = @($processes | Select-Object -ExpandProperty Id)
	if (-not (Wait-ForProcessesToExit -Ids $ids -Seconds 8)) {
		foreach ($p in $processes) {
			$live = Get-Process -Id $p.Id -ErrorAction SilentlyContinue
			if ($live) {
				try { Stop-Process -Id $p.Id -Force -ErrorAction Stop } catch { }
			}
		}
		[void](Wait-ForProcessesToExit -Ids $ids -Seconds 4)
	}
}

function Stop-ProcessList {
	param(
		[Parameter(Mandatory=$true)][string]$Label,
		[Parameter(Mandatory=$true)]$Processes
	)

	$processes = @($Processes | Where-Object { $_ } | Sort-Object Id -Unique)
	if ($processes.Count -eq 0) { return }

	Write-Host ("Stopping {0}: {1}" -f $Label, (($processes | Select-Object -ExpandProperty ProcessName -Unique) -join ", "))
	foreach ($p in @($processes | Where-Object { $_.MainWindowHandle -ne 0 })) {
		try { [void]$p.CloseMainWindow() } catch { }
	}

	$ids = @($processes | Select-Object -ExpandProperty Id)
	if (-not (Wait-ForProcessesToExit -Ids $ids -Seconds 8)) {
		foreach ($p in $processes) {
			$live = Get-Process -Id $p.Id -ErrorAction SilentlyContinue
			if ($live) {
				try { Stop-Process -Id $p.Id -Force -ErrorAction Stop } catch { }
			}
		}
		[void](Wait-ForProcessesToExit -Ids $ids -Seconds 4)
	}
}

function Get-ProcessesUsingModulePath {
	param([Parameter(Mandatory=$true)][string]$Path)

	$normalized = (Get-NormalizedPath $Path).ToLowerInvariant()
	$hits = @()
	foreach ($process in Get-Process -ErrorAction SilentlyContinue) {
		try {
			foreach ($module in $process.Modules) {
				if ((Get-NormalizedPath $module.FileName).ToLowerInvariant() -eq $normalized) {
					$hits += $process
					break
				}
			}
		} catch { }
	}

	return $hits
}

function Wait-ForNoNamedProcesses {
	param(
		[Parameter(Mandatory=$true)][string]$Label,
		[Parameter(Mandatory=$true)][string[]]$Names,
		[int]$Seconds = 20
	)

	$nameSet = @{}
	foreach ($name in $Names) { $nameSet[$name.ToLowerInvariant()] = $true }

	$deadline = (Get-Date).AddSeconds($Seconds)
	do {
		$alive = @(Get-Process -ErrorAction SilentlyContinue | Where-Object {
			$nameSet.ContainsKey($_.ProcessName.ToLowerInvariant())
		})
		if ($alive.Count -eq 0) { return }
		Start-Sleep -Milliseconds 500
	} while ((Get-Date) -lt $deadline)

	$namesAlive = (($alive | Select-Object -ExpandProperty ProcessName -Unique) -join ", ")
	throw "$Label did not exit after $Seconds seconds: $namesAlive"
}

function Get-RelativePathFromRoot {
	param(
		[Parameter(Mandatory=$true)][string]$Root,
		[Parameter(Mandatory=$true)][string]$FullName
	)
	$rootPath = (Resolve-Path -LiteralPath $Root).ProviderPath.TrimEnd('\')
	return $FullName.Substring($rootPath.Length).TrimStart('\')
}

function Get-DeployEntries {
	param(
		[Parameter(Mandatory=$true)]$Plan
	)

	$entries = @()
	foreach ($file in $Plan.OverlayFiles) {
		$entries += [pscustomobject]@{
			Label       = $file.Label
			Source      = $file.Source
			Destination = $file.Destination
		}
	}

	$resourceRoot = (Resolve-Path -LiteralPath $Plan.OverlayResourceSource).ProviderPath
	foreach ($file in Get-ChildItem -LiteralPath $resourceRoot -Recurse -File) {
		$rel = Get-RelativePathFromRoot -Root $resourceRoot -FullName $file.FullName
		$entries += [pscustomobject]@{
			Label       = "resources\$rel"
			Source      = $file.FullName
			Destination = Join-Path $Plan.OverlayResourceDest $rel
		}
	}

	$driverRoot = (Resolve-Path -LiteralPath $Plan.DriverSourceDir).ProviderPath
	foreach ($file in Get-ChildItem -LiteralPath $driverRoot -Recurse -File) {
		$rel = Get-RelativePathFromRoot -Root $driverRoot -FullName $file.FullName
		$dstRel = $rel
		if ($rel -ieq $Plan.DriverSourceDllRel) {
			$dstRel = $Plan.DriverDestDllRel
		}
		$entries += [pscustomobject]@{
			Label       = "driver_wkopenvr\$dstRel"
			Source      = $file.FullName
			Destination = Join-Path $Plan.DriverDestDir $dstRel
		}
	}

	return $entries
}

function Write-JsonNoBom {
	param(
		[Parameter(Mandatory=$true)][string]$Path,
		[Parameter(Mandatory=$true)][string]$Text
	)
	$enc = New-Object System.Text.UTF8Encoding($false)
	[System.IO.File]::WriteAllText($Path, $Text, $enc)
}

function Invoke-ElevatedDeployCopy {
	param(
		[Parameter(Mandatory=$true)]$Plan
	)

	$guid = [guid]::NewGuid().ToString("N")
	$planPath = Join-Path $env:TEMP "WKOpenVR-deploy-plan-$guid.json"
	$helperPath = Join-Path $env:TEMP "WKOpenVR-deploy-helper-$guid.ps1"
	$resultPath = Join-Path $env:TEMP "WKOpenVR-deploy-result-$guid.txt"

	Write-JsonNoBom -Path $planPath -Text ($Plan | ConvertTo-Json -Depth 8)

	$helperScript = @'
param(
	[Parameter(Mandatory=$true)][string]$PlanPath,
	[Parameter(Mandatory=$true)][string]$ResultPath
)

$ErrorActionPreference = "Stop"

function Copy-DirectoryFresh {
	param(
		[Parameter(Mandatory=$true)][string]$SourceDir,
		[Parameter(Mandatory=$true)][string]$DestinationDir
	)

	if (Test-Path -LiteralPath $DestinationDir) {
		Remove-Item -LiteralPath $DestinationDir -Recurse -Force
	}
	New-Item -ItemType Directory -Force -Path $DestinationDir | Out-Null
	Copy-Item -Path (Join-Path $SourceDir "*") -Destination $DestinationDir -Recurse -Force
}

function Wait-ForWritableFile {
	param(
		[Parameter(Mandatory=$true)][string]$Path,
		[int]$Seconds = 25
	)

	if (-not (Test-Path -LiteralPath $Path)) { return }

	$deadline = (Get-Date).AddSeconds($Seconds)
	$lastError = $null
	do {
		try {
			$stream = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::ReadWrite, [System.IO.FileShare]::None)
			$stream.Dispose()
			return
		} catch {
			$lastError = $_.Exception.Message
			Start-Sleep -Milliseconds 500
		}
	} while ((Get-Date) -lt $deadline)

	throw "Timed out waiting for writable file: $Path ($lastError)"
}

function Copy-DriverTree {
	param(
		[Parameter(Mandatory=$true)]$Plan
	)

	$preserveRoot = Join-Path $env:TEMP ("WKOpenVR-preserve-" + [guid]::NewGuid().ToString("N"))
	$preserved = @()
	try {
		$existingDriverDll = Join-Path $Plan.DriverDestDir $Plan.DriverDestDllRel
		Wait-ForWritableFile -Path $existingDriverDll

		New-Item -ItemType Directory -Force -Path $preserveRoot | Out-Null

		$resourcesDir = Join-Path $Plan.DriverDestDir "resources"
		$resourcesPreExisted = Test-Path -LiteralPath $resourcesDir
		if ($resourcesPreExisted) {
			foreach ($flag in Get-ChildItem -LiteralPath $resourcesDir -Filter "*.flag" -File -ErrorAction SilentlyContinue) {
				$rel = Join-Path "resources" $flag.Name
				$tempPath = Join-Path $preserveRoot $rel
				New-Item -ItemType Directory -Force -Path (Split-Path -Parent $tempPath) | Out-Null
				Copy-Item -LiteralPath $flag.FullName -Destination $tempPath -Force
				$preserved += [pscustomobject]@{ Rel = $rel; Temp = $tempPath }
			}
		}

		Copy-DirectoryFresh -SourceDir $Plan.DriverSourceDir -DestinationDir $Plan.DriverDestDir

		$bareDll = Join-Path $Plan.DriverDestDir $Plan.DriverSourceDllRel
		$loaderDll = Join-Path $Plan.DriverDestDir $Plan.DriverDestDllRel
		if (-not (Test-Path -LiteralPath $bareDll)) {
			throw "Copied driver tree is missing $($Plan.DriverSourceDllRel)"
		}
		if (Test-Path -LiteralPath $loaderDll) {
			Remove-Item -LiteralPath $loaderDll -Force
		}
		Move-Item -LiteralPath $bareDll -Destination $loaderDll -Force

		foreach ($item in $preserved) {
			$dst = Join-Path $Plan.DriverDestDir $item.Rel
			New-Item -ItemType Directory -Force -Path (Split-Path -Parent $dst) | Out-Null
			Copy-Item -LiteralPath $item.Temp -Destination $dst -Force
		}

		# Module enable-flags are user state. On a REDEPLOY (the resources dir
		# already existed) the deployed set is authoritative: remove any
		# enable_*.flag the fresh build re-added that the user did not have, so a
		# module the user disabled stays disabled. A first install (no prior
		# resources dir) keeps the build's defaults.
		if ($resourcesPreExisted) {
			$keptFlagNames = @($preserved | ForEach-Object { Split-Path -Leaf $_.Rel })
			$destResources = Join-Path $Plan.DriverDestDir "resources"
			foreach ($flag in Get-ChildItem -LiteralPath $destResources -Filter "enable_*.flag" -File -ErrorAction SilentlyContinue) {
				if ($keptFlagNames -notcontains $flag.Name) {
					Remove-Item -LiteralPath $flag.FullName -Force -ErrorAction SilentlyContinue
				}
			}
		}
	} finally {
		if (Test-Path -LiteralPath $preserveRoot) {
			Remove-Item -LiteralPath $preserveRoot -Recurse -Force -ErrorAction SilentlyContinue
		}
	}
}

function Refresh-Shortcuts {
	param([Parameter(Mandatory=$true)]$Plan)

	$startMenuDir = Join-Path $env:ProgramData "Microsoft\Windows\Start Menu\Programs\WKOpenVR"
	New-Item -ItemType Directory -Force -Path $startMenuDir | Out-Null

	$argMap = @{}
	foreach ($shortcut in $Plan.Shortcuts) {
		$argMap[$shortcut.Name] = $shortcut.Arguments
	}

	$shortcutNames = @()
	if ($argMap.ContainsKey("WKOpenVR.lnk")) {
		$shortcutNames += "WKOpenVR.lnk"
	}
	foreach ($lnk in Get-ChildItem -LiteralPath $startMenuDir -Filter "*.lnk" -File -ErrorAction SilentlyContinue) {
		if (-not $argMap.ContainsKey($lnk.Name)) { continue }
		if ($shortcutNames -notcontains $lnk.Name) {
			$shortcutNames += $lnk.Name
		}
	}

	$wsh = New-Object -ComObject WScript.Shell
	foreach ($shortcutName in $shortcutNames) {
		$shortcutPath = Join-Path $startMenuDir $shortcutName
		$sc = $wsh.CreateShortcut($shortcutPath)
		$sc.TargetPath = $Plan.OverlayExeDest
		$sc.Arguments = $argMap[$shortcutName]
		$sc.IconLocation = $Plan.OverlayExeDest + ",0"
		$sc.Description = "Open WKOpenVR"
		$sc.Save()
	}
}

try {
	$plan = Get-Content -LiteralPath $PlanPath -Raw | ConvertFrom-Json

	New-Item -ItemType Directory -Force -Path $plan.InstallDir | Out-Null
	foreach ($file in $plan.OverlayFiles) {
		Wait-ForWritableFile -Path $file.Destination
		New-Item -ItemType Directory -Force -Path (Split-Path -Parent $file.Destination) | Out-Null
		Copy-Item -LiteralPath $file.Source -Destination $file.Destination -Force
	}

	Copy-DirectoryFresh -SourceDir $plan.OverlayResourceSource -DestinationDir $plan.OverlayResourceDest
	Copy-DriverTree -Plan $plan
	Refresh-Shortcuts -Plan $plan

	Set-Content -LiteralPath $ResultPath -Value "OK" -Encoding ASCII
	exit 0
} catch {
	Set-Content -LiteralPath $ResultPath -Value ("ERR`n" + $_.Exception.Message) -Encoding ASCII
	exit 1
}
'@

	Set-Content -LiteralPath $helperPath -Value $helperScript -Encoding UTF8

	$proc = Start-Process -FilePath "powershell.exe" `
		-ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $helperPath, "-PlanPath", $planPath, "-ResultPath", $resultPath) `
		-Verb RunAs `
		-PassThru `
		-Wait

	if (-not $proc) {
		throw "Elevated helper did not launch. UAC may have been denied."
	}
	if (-not (Test-Path -LiteralPath $resultPath)) {
		throw "Elevated helper produced no result file at $resultPath."
	}

	$resultLines = @()
	$resultDeadline = (Get-Date).AddSeconds(60)
	do {
		$resultLines = @(Get-Content -LiteralPath $resultPath -ErrorAction SilentlyContinue)
		if ($resultLines.Count -ge 1 -and ($resultLines[0] -eq "OK" -or $resultLines[0] -eq "ERR")) {
			break
		}
		Start-Sleep -Milliseconds 250
	} while ((Get-Date) -lt $resultDeadline)

	if ($resultLines.Count -lt 1 -or $resultLines[0] -ne "OK") {
		$msg = if ($resultLines.Count -ge 2) { $resultLines[1] } else { "(empty)" }
		throw "Elevated helper failed: $msg"
	}

	foreach ($path in @($planPath, $helperPath, $resultPath)) {
		if (Test-Path -LiteralPath $path) {
			Remove-Item -LiteralPath $path -Force -ErrorAction SilentlyContinue
		}
	}
}

function Restart-SteamVR {
	param(
		[string]$ResolvedSteamExe,
		[bool]$ShouldRestart
	)

	if ($NoRestart) {
		Write-Host "Restart skipped."
		return
	}

	if (-not $ShouldRestart) {
		Write-Host "SteamVR was not running with Steam at script start; SteamVR restart skipped."
		return
	}

	Write-Step "Restarting SteamVR"
	if ($ResolvedSteamExe -and (Test-Path -LiteralPath $ResolvedSteamExe)) {
		Write-Host "Starting SteamVR..."
		Start-Process -FilePath $ResolvedSteamExe -ArgumentList @("-applaunch", "250820") | Out-Null
		return
	}

	Write-Host "steam.exe was not found. Starting SteamVR through the registered steam:// URL."
	Start-Process "steam://rungameid/250820" | Out-Null
}

function Ensure-SteamRunning {
	param(
		[string]$ResolvedSteamExe,
		[string[]]$ProcessNames
	)

	if (Test-NamedProcess -Names $ProcessNames) {
		Write-Host "Steam is running."
		return
	}

	Write-Step "Starting Steam"
	if ($ResolvedSteamExe -and (Test-Path -LiteralPath $ResolvedSteamExe)) {
		Write-Host "Starting Steam..."
		Start-Process -FilePath $ResolvedSteamExe | Out-Null
		return
	}

	Write-Host "steam.exe was not found. Starting Steam through the registered steam:// URL."
	Start-Process "steam://open/main" | Out-Null
}

$steamExeResolved = Resolve-SteamExe -Override $SteamExe
$driversDirResolved = Resolve-SteamVRDriversDir -Override $SteamVRDriversDir -ResolvedSteamExe $steamExeResolved
$driverDestDir = Join-Path $driversDirResolved "01wkopenvr"

$steamProcessNames = @("steam", "steamwebhelper", "GameOverlayUI")
$steamVrProcessNames = @(
	"vrmonitor",
	"vrserver",
	"vrcompositor",
	"vrdashboard",
	"vrwebhelper",
	"vrstartup",
	"vrpathreg",
	"WKOpenVR",
	"WKOpenVR.FaceModuleHost",
	"WKOpenVR.FaceModuleProcess",
	"WKOpenVR.CaptionsHost",
	"WKOpenVRPhantomSidecar"
)
$steamWasRunning = Test-NamedProcess -Names $steamProcessNames
$steamVrWasRunning = Test-NamedProcess -Names $steamVrProcessNames
$restartSteamVrAfterDeploy = $steamWasRunning -and $steamVrWasRunning
$deployedDriverDll = Join-Path $driverDestDir "bin\win64\driver_01wkopenvr.dll"

if (-not $SkipBuild) {
	Write-Step "Building"
	$buildArgs = @{}
	if ($SkipConfigure) { $buildArgs["SkipConfigure"] = $true }
	if ($CaptionsCpuOnly) { $buildArgs["CaptionsCpuOnly"] = $true }
	if ($InstallVulkanSdk) { $buildArgs["InstallVulkanSdk"] = $true }
	if ($VulkanSdkRoot) { $buildArgs["VulkanSdkRoot"] = $VulkanSdkRoot }
	& "$PSScriptRoot\build.ps1" @buildArgs
	if ($LASTEXITCODE -ne 0) { throw "build.ps1 failed (exit $LASTEXITCODE)" }
} else {
	Write-Step "Using existing build artifacts"
}

$artifactsDir = Join-Path $PSScriptRoot "build\artifacts\Release"
$driverSourceDir = Join-Path $PSScriptRoot "build\driver_wkopenvr"
$overlayResourcesSource = Join-Path $artifactsDir "resources"

$srcExe = Join-Path $artifactsDir "WKOpenVR.exe"
$srcOpenVR = Join-Path $artifactsDir "openvr_api.dll"
$srcManifest = Join-Path $artifactsDir "manifest.vrmanifest"
$srcIcon = Join-Path $artifactsDir "dashboard_icon.png"

$required = @(
	@("WKOpenVR.exe", $srcExe),
	@("openvr_api.dll", $srcOpenVR),
	@("manifest.vrmanifest", $srcManifest),
	@("dashboard_icon.png", $srcIcon),
	@("overlay resources", $overlayResourcesSource),
	@("face-module-sync.ps1", (Join-Path $overlayResourcesSource "face-module-sync.ps1")),
	@("driver tree", $driverSourceDir),
	@("driver.vrdrivermanifest", (Join-Path $driverSourceDir "driver.vrdrivermanifest")),
	@("driver.vrresources", (Join-Path $driverSourceDir "resources\driver.vrresources")),
	@("default.vrsettings", (Join-Path $driverSourceDir "resources\settings\default.vrsettings")),
	@("driver_wkopenvr.dll", (Join-Path $driverSourceDir "bin\win64\driver_wkopenvr.dll")),
	@("WKOpenVR.FaceModuleHost.exe", (Join-Path $driverSourceDir "resources\facetracking\host\WKOpenVR.FaceModuleHost.exe")),
	@("WKOpenVR.CaptionsHost.exe", (Join-Path $driverSourceDir "resources\captions\host\WKOpenVR.CaptionsHost.exe")),
	@("captions host openvr_api.dll", (Join-Path $driverSourceDir "resources\captions\host\openvr_api.dll")),
	@("captions-packs.json", (Join-Path $driverSourceDir "resources\captions\host\resources\captions-packs.json")),
	@("install-captions-pack.ps1", (Join-Path $driverSourceDir "resources\captions\host\resources\install-captions-pack.ps1")),
	@("WKOpenVRPhantomSidecar.exe", (Join-Path $driverSourceDir "resources\phantom\host\WKOpenVRPhantomSidecar.exe")),
	@("questapp platform-tools installer", (Join-Path $overlayResourcesSource "questapp\install-platform-tools.ps1")),
	@("questapp uninstaller", (Join-Path $overlayResourcesSource "questapp\uninstall-questapp.ps1")),
	@("questapp companion apk", (Join-Path $overlayResourcesSource "questapp\WKOpenVRQuestCompanion.apk"))
)

Write-Step "Checking artifacts"
foreach ($item in $required) {
	Assert-File -Label $item[0] -Path $item[1]
}

$overlayFiles = @(
	[ordered]@{ Label = "WKOpenVR.exe";          Source = $srcExe;          Destination = (Join-Path $InstallDir "WKOpenVR.exe") },
	[ordered]@{ Label = "openvr_api.dll";       Source = $srcOpenVR;       Destination = (Join-Path $InstallDir "openvr_api.dll") },
	[ordered]@{ Label = "manifest.vrmanifest";  Source = $srcManifest;     Destination = (Join-Path $InstallDir "manifest.vrmanifest") },
	[ordered]@{ Label = "dashboard_icon.png";   Source = $srcIcon;         Destination = (Join-Path $InstallDir "dashboard_icon.png") }
)

$plan = [ordered]@{
	InstallDir            = $InstallDir
	DriverSourceDir       = $driverSourceDir
	DriverDestDir         = $driverDestDir
	DriverSourceDllRel    = "bin\win64\driver_wkopenvr.dll"
	DriverDestDllRel      = "bin\win64\driver_01wkopenvr.dll"
	OverlayResourceSource = $overlayResourcesSource
	OverlayResourceDest   = (Join-Path $InstallDir "resources")
	OverlayExeDest        = (Join-Path $InstallDir "WKOpenVR.exe")
	OverlayFiles          = $overlayFiles
	Shortcuts = @(
		[ordered]@{ Name = "WKOpenVR.lnk";                    Arguments = "--launch=umbrella" },
		[ordered]@{ Name = "WKOpenVR - Space Calibrator.lnk"; Arguments = "--launch=calibration" },
		[ordered]@{ Name = "WKOpenVR - Smoothing.lnk";        Arguments = "--launch=smoothing" },
		[ordered]@{ Name = "WKOpenVR - Input Health.lnk";     Arguments = "--launch=inputhealth" },
		[ordered]@{ Name = "WKOpenVR - Face Tracking.lnk";    Arguments = "--launch=facetracking" },
		[ordered]@{ Name = "WKOpenVR - Quest App.lnk";        Arguments = "--launch=questapp" }
	)
}

Write-Host ("Install dir:        {0}" -f $InstallDir)
Write-Host ("Steam exe:          {0}" -f ($(if ($steamExeResolved) { $steamExeResolved } else { "(not found)" })))
Write-Host ("SteamVR drivers:    {0}" -f $driversDirResolved)
Write-Host ("Driver destination: {0}" -f $driverDestDir)
Write-Host ("Build version:      {0}" -f (Resolve-Version $srcExe))
Write-Host ("Steam was running:  {0}" -f $steamWasRunning)
Write-Host ("SteamVR was running:{0}" -f $steamVrWasRunning)
Write-Host ("Restart SteamVR:    {0}" -f ($restartSteamVrAfterDeploy -and -not $NoRestart))

if ($DryRun) {
	$entries = @(Get-DeployEntries -Plan ([pscustomobject]$plan))
	Write-Host ("Dry run: {0} files would be deployed." -f $entries.Count)
	exit 0
}

# Never kill an active game. VRChat running means SteamVR is up, so a deploy
# would stop it out from under the session. Refuse unless explicitly overridden.
# (DryRun already exited above, so this never blocks a no-op preview.)
$runningGame = @(Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.ProcessName -ieq 'VRChat' })
if ($runningGame.Count -gt 0 -and -not $AllowGameRunning) {
	Write-Host ""
	Write-Host "==================== DEPLOY BLOCKED ====================" -ForegroundColor Yellow
	Write-Host " VRChat is running -- this deploy would stop SteamVR and" -ForegroundColor Yellow
	Write-Host " kill your game session. Close VRChat first, then re-run." -ForegroundColor Yellow
	Write-Host " (Override with -AllowGameRunning if you really mean it.)" -ForegroundColor Yellow
	Write-Host "=======================================================" -ForegroundColor Yellow
	exit 3
}

if (-not $Yes) {
	Write-Host ""
	if ($restartSteamVrAfterDeploy -and -not $NoRestart) {
		$prompt = "This will stop SteamVR, copy into Program Files, and restart SteamVR through Steam. Continue? [y/N]"
	} elseif ($steamVrWasRunning) {
		$prompt = "This will stop SteamVR and copy into Program Files. SteamVR will not restart because Steam was not running at script start. Continue? [y/N]"
	} else {
		$prompt = "This will copy into Program Files without stopping or starting SteamVR. Continue? [y/N]"
	}
	$reply = Read-Host $prompt
	if ($reply -notmatch "^(y|yes)$") {
		Write-Host "Aborted."
		exit 1
	}
}

if ($steamVrWasRunning) {
	Write-Step "Stopping SteamVR"
	Stop-NamedProcesses -Label "SteamVR/WKOpenVR processes" -Names $steamVrProcessNames
	Wait-ForNoNamedProcesses -Label "SteamVR/WKOpenVR processes" -Names $steamVrProcessNames
} else {
	Write-Step "SteamVR not running"
	Write-Host "Skipping SteamVR stop."
}

$overlayLockers = @(Get-ProcessesUsingModulePath -Path $plan.OverlayExeDest)
if ($overlayLockers.Count -gt 0) {
	Write-Step "Stopping WKOpenVR"
	Stop-ProcessList -Label "WKOpenVR processes holding the installed executable" -Processes $overlayLockers
}

$driverLockers = @(Get-ProcessesUsingModulePath -Path $deployedDriverDll)
$steamWasStopped = $false
if ($driverLockers.Count -gt 0) {
	$lockNames = (($driverLockers | Select-Object -ExpandProperty ProcessName -Unique) -join ", ")
	Write-Host "Driver DLL is still loaded by: $lockNames"
	$steamLockers = @($driverLockers | Where-Object { $_.ProcessName -in $steamProcessNames })
	if ($steamLockers.Count -gt 0) {
		Write-Step "Stopping Steam"
		Stop-NamedProcesses -Label "Steam processes holding the driver DLL" -Names $steamProcessNames
		Wait-ForNoNamedProcesses -Label "Steam processes holding the driver DLL" -Names $steamProcessNames
		$steamWasStopped = $true
	} else {
		throw "Driver DLL is still loaded by non-Steam process(es): $lockNames"
	}
}

Write-Step "Deploying"
$driverResourcesDir = Join-Path $plan.DriverDestDir "resources"
$driverResourcesPreExisted = Test-Path -LiteralPath $driverResourcesDir
$preDeployEnableFlags = @{}
if ($driverResourcesPreExisted) {
	foreach ($flag in Get-ChildItem -LiteralPath $driverResourcesDir -Filter "enable_*.flag" -File -ErrorAction SilentlyContinue) {
		$preDeployEnableFlags[$flag.Name.ToLowerInvariant()] = $true
	}
}
Invoke-ElevatedDeployCopy -Plan $plan

Write-Step "Verifying"
$entries = @(Get-DeployEntries -Plan ([pscustomobject]$plan))
$mismatches = @()
foreach ($entry in $entries) {
	if ($driverResourcesPreExisted -and $entry.Label -like "driver_wkopenvr\resources\enable_*.flag") {
		$flagName = (Split-Path -Leaf $entry.Label).ToLowerInvariant()
		$wasEnabled = $preDeployEnableFlags.ContainsKey($flagName)
		$isEnabled = Test-Path -LiteralPath $entry.Destination
		if ($wasEnabled -and -not $isEnabled) {
			$mismatches += ("{0}: enabled before deploy but missing after deploy" -f $entry.Label)
		}
		elseif (-not $wasEnabled -and $isEnabled) {
			$mismatches += ("{0}: disabled before deploy but present after deploy" -f $entry.Label)
		}
		continue
	}

	$srcSha = Get-Sha $entry.Source
	$dstSha = Get-Sha $entry.Destination
	if ($srcSha -ne $dstSha) {
		$mismatches += ("{0}: source={1} deployed={2}" -f $entry.Label, $srcSha, $dstSha)
	}
}

if ($mismatches.Count -gt 0) {
	Write-Host "Deploy verification mismatches:"
	$mismatches | ForEach-Object { Write-Host ("  - {0}" -f $_) }
	throw "Deploy did not converge."
}

Write-Host ("Verified {0} deployed files." -f $entries.Count)

Restart-SteamVR -ResolvedSteamExe $steamExeResolved -ShouldRestart $restartSteamVrAfterDeploy

Ensure-SteamRunning -ResolvedSteamExe $steamExeResolved -ProcessNames $steamProcessNames

Write-Step "Done"
Write-Host "Deploy verified. Installed build: $(Resolve-Version (Join-Path $InstallDir "WKOpenVR.exe"))"
