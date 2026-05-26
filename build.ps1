param(
	# When set, overrides the auto-derived YYYY.M.D.N-XXXX stamp. Release CI
	# passes the git tag (with leading "v" stripped) so the published
	# release's tag, zip filename, and embedded version are all the same string.
	[string]$Version = "",

	# Skip the CMake configure step (rerun MSBuild only). Useful when iterating
	# on a single source file.
	[switch]$SkipConfigure,

	# Produce a release zip + per-file manifest TSV under release/. Required
	# by .github/workflows/release.yml; set automatically by it. A local dev
	# build can pass this too if you want to test the packaging step.
	[switch]$Release,

	# Developer opt-in for the CUDA whisper backend. Public/local builds force
	# CPU-only unless this switch or WKOPENVR_CAPTIONS_CUDA=ON is explicit.
	[switch]$CaptionsCuda
)

$ErrorActionPreference = "Stop"

# Pin the working directory to the script's own root so relative paths resolve
# consistently regardless of how the script is invoked.
Set-Location $PSScriptRoot

function Clear-StaleCMakeGeneratorInstance {
	param([Parameter(Mandatory=$true)][string]$BuildDir)

	$cachePath = Join-Path $BuildDir "CMakeCache.txt"
	if (-not (Test-Path -LiteralPath $cachePath)) { return }

	$instancePath = $null
	foreach ($line in Get-Content -LiteralPath $cachePath) {
		if ($line -match '^CMAKE_GENERATOR_INSTANCE:[^=]*=(.*)$') {
			$instancePath = $Matches[1]
			break
		}
	}
	if (-not $instancePath) { return }

	if (-not (Test-Path -LiteralPath $instancePath)) {
		Write-Host "CMake cached Visual Studio instance no longer exists: $instancePath" -ForegroundColor Yellow
		Write-Host "Clearing generated CMake configure cache under $BuildDir" -ForegroundColor Yellow
		Remove-Item -LiteralPath $cachePath -Force
		$cmakeFiles = Join-Path $BuildDir "CMakeFiles"
		if (Test-Path -LiteralPath $cmakeFiles) {
			Remove-Item -LiteralPath $cmakeFiles -Recurse -Force
		}
	}
}

# Activate the repo's tracked git hooks the first time the build runs in a
# clone. Idempotent: only writes when the value would change.
$currentHooksPath = & git config --get core.hooksPath 2>$null
if ($currentHooksPath -ne ".githooks") {
	& git config core.hooksPath ".githooks"
	Write-Host "Activated .githooks/ via core.hooksPath"
}

# Stamp the build version. Release CI passes -Version; local builds derive the
# stamp from today's date + a per-day counter + a 4-hex GUID prefix.
if ($Version -eq "") {
	$today = Get-Date -Format "yyyy.M.d"
	$counterFile = "build/local_build_state.json"
	$counter = 0
	if (Test-Path $counterFile) {
		$state = Get-Content $counterFile -Raw | ConvertFrom-Json
		if ($state.date -eq $today) {
			$counter = [int]$state.counter + 1
		}
	}
	$uid = ([guid]::NewGuid().ToString("N").Substring(0, 4)).ToUpper()
	$Version = "$today.$counter-$uid"
	New-Item -ItemType Directory -Force -Path "build" | Out-Null
	@{ date = $today; counter = $counter } | ConvertTo-Json | Set-Content $counterFile
}
Set-Content -Path "version.txt" -Value $Version -NoNewline
Write-Host "Build version: $Version"

# Channel string baked into BuildChannel.h / BuildStamp.h. -Release flips
# this to "release" so DebugLogging stops forcing verbose output and the
# CMake module gate (-DWKOPENVR_RELEASE_BUILD=ON, set below) honours the
# disabled-in-release.flag markers on the four unstable feature modules.
$ChannelName = if ($Release) { "release" } else { "dev" }
Write-Host "Build channel: $ChannelName"

# Stamp the common build channel header consumed by shared diagnostics code.
$CommonBuildChannel = Join-Path $PSScriptRoot "core/src/common/BuildChannel.h"
$IsDevValue = if ($ChannelName -eq "dev") { 1 } else { 0 }
if (Test-Path (Split-Path -Parent $CommonBuildChannel)) {
	Set-Content -Path $CommonBuildChannel -Value @"
// Overwritten by WKOpenVR/build.ps1 with the umbrella binary's
// per-build stamp so every process uses the same debug-logging policy.
#pragma once

#define WKOPENVR_BUILD_STAMP "$Version"
#define WKOPENVR_BUILD_CHANNEL "$ChannelName"
#define WKOPENVR_BUILD_IS_DEV $IsDevValue
"@
}

# Stamp the SpaceCalibrator feature plugin's BuildStamp.h with the same
# version. Without this, the standalone SC fallback ("0.0.0.0-DEV") shows up
# in the calibration UI's version line even though the umbrella binary
# itself reports the real stamp. The file is generated; the SC repo tracks a
# fallback that this overwrite replaces only for the umbrella build.
$ScBuildStamp = Join-Path $PSScriptRoot "modules/calibration/src/overlay/BuildStamp.h"
if (Test-Path (Split-Path -Parent $ScBuildStamp)) {
	Set-Content -Path $ScBuildStamp -Value @"
// Overwritten by WKOpenVR/build.ps1 with the umbrella binary's
// per-build stamp so SC's version footer reads the same string the
// umbrella top header reports.
#pragma once

#define SPACECAL_BUILD_STAMP "$Version"
#define SPACECAL_BUILD_CHANNEL "$ChannelName"
"@
}

# Stamp the FaceTracking feature plugin's BuildStamp.h with the same
# version. Same reason as the calibration block above: the face-tracking
# footer reads FACETRACKING_BUILD_STAMP which otherwise stays at its
# in-tree fallback ("0.0.0.0-dev") and renders that in the UI for every
# user. The umbrella binary's main.cpp is stamped via target_compile_
# definitions in core/src/overlay/CMakeLists.txt; this file is what
# feeds the FaceTracking module's own version line.
$FtBuildStamp = Join-Path $PSScriptRoot "modules/facetracking/src/overlay/BuildStamp.h"
if (Test-Path (Split-Path -Parent $FtBuildStamp)) {
	Set-Content -Path $FtBuildStamp -Value @"
// Overwritten by WKOpenVR/build.ps1 with the umbrella binary's
// per-build stamp so the FaceTracking footer reads the same string
// the umbrella top header reports.
#pragma once

#define FACETRACKING_BUILD_STAMP "$Version"
#define FACETRACKING_BUILD_CHANNEL "$ChannelName"
"@
}

# Configure (skippable for incremental edits). The CMAKE_POLICY_VERSION_MINIMUM
# bump is needed because the minhook submodule pins cmake_minimum_required at
# 2.8 and current CMake versions reject anything below 3.5. -Wno-dev silences
# developer-mode warnings from the minhook submodule (unrelated upstream).
#
# PowerShell 5.1 wraps every stderr line from a native command as a
# NativeCommandError ErrorRecord. Under the script-wide 'Stop' default that
# wrap kills the script on the first CMake message() line; under 'Continue'
# the script survives but the lines still render with the red "+ CategoryInfo
# ... NativeCommandError" preamble that buries real diagnostics in noise.
# Pipe through a ForEach-Object that coerces ErrorRecord -> plain string so
# stderr lines print clean alongside stdout. $LASTEXITCODE is preserved
# across the pipe so the exit-code check still works.
function Invoke-NativeQuiet {
	param([scriptblock]$Cmd)
	$PrevEap = $ErrorActionPreference
	$ErrorActionPreference = "Continue"
	try {
		& $Cmd 2>&1 | ForEach-Object {
			if ($_ -is [System.Management.Automation.ErrorRecord]) {
				Write-Host $_.Exception.Message
			} else {
				Write-Host $_
			}
		}
	} finally {
		$ErrorActionPreference = $PrevEap
	}
}

if (-not $SkipConfigure) {
	Clear-StaleCMakeGeneratorInstance -BuildDir "build"
	$captionsCudaValue = "OFF"
	if ($CaptionsCuda -or $env:WKOPENVR_CAPTIONS_CUDA -eq "ON") {
		$captionsCudaValue = "ON"
	}
	$releaseBuildValue = if ($Release) { "ON" } else { "OFF" }
	$configureArgs = @(
		"-S", ".",
		"-B", "build",
		"-A", "x64",
		"-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
		"-DWKOPENVR_CAPTIONS_CUDA=$captionsCudaValue",
		"-DWKOPENVR_RELEASE_BUILD=$releaseBuildValue",
		"-Wno-dev"
	)
	Invoke-NativeQuiet { cmake @configureArgs }
	if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (exit $LASTEXITCODE)" }
}

# Build Release. Pass the job count explicitly so Visual Studio/MSBuild gets
# the full logical CPU count instead of relying on generator defaults.
$ParallelJobs = [Environment]::ProcessorCount
if ($ParallelJobs -lt 1) { $ParallelJobs = 1 }
Write-Host ("Build parallelism: {0} jobs" -f $ParallelJobs)
Invoke-NativeQuiet { cmake --build build --config Release --parallel $ParallelJobs }
if ($LASTEXITCODE -ne 0) { throw "Build failed (exit $LASTEXITCODE)" }

# Verify the artifact lands where we expect.
$dllPath = "build/driver_wkopenvr/bin/win64/driver_wkopenvr.dll"
if (-not (Test-Path $dllPath)) {
	throw "Expected driver DLL not found at $dllPath"
}
$dll = Get-Item $dllPath
Write-Host ""
Write-Host ("Built {0} ({1:N0} bytes, {2})" -f $dll.Name, $dll.Length, $dll.LastWriteTime)
Write-Host ("  -> {0}" -f $dll.FullName)

# Stage the C# face-tracking host into the driver tree so SteamVR can launch
# it via the driver's HostSupervisor at runtime. Build output lives at
# build/artifacts/FaceModuleHost/ (the host CMakeLists.txt publishes there).
# The driver's HostSupervisor resolves the host exe relative to its own
# resources directory, so the deployable tree must contain:
#   driver_wkopenvr/resources/facetracking/host/WKOpenVR.FaceModuleHost.exe
# If OPENVR_PAIR_BUILD_FACE_HOST=OFF (no .NET SDK on the build host), the
# artifacts dir is absent and the staging step no-ops. The driver detects
# the missing exe at runtime, logs once, and keeps the feature inert.
$hostBuildDir = "build/artifacts/FaceModuleHost"
$hostStageDir = "build/driver_wkopenvr/resources/facetracking/host"
if (Test-Path $hostBuildDir) {
	New-Item -ItemType Directory -Force -Path $hostStageDir | Out-Null
	Copy-Item -Recurse -Force -Path "$hostBuildDir\*" -Destination $hostStageDir
	$hostExe = Join-Path $hostStageDir "WKOpenVR.FaceModuleHost.exe"
	if (Test-Path $hostExe) {
		$hostExeItem = Get-Item $hostExe
		Write-Host ("Staged FaceModuleHost: {0} ({1:N0} bytes)" -f $hostExeItem.Name, $hostExeItem.Length)
	} else {
		Write-Host "FaceModuleHost staging directory present but WKOpenVR.FaceModuleHost.exe missing -- did the publish step fail silently?" -ForegroundColor Yellow
	}
} else {
	Write-Host "FaceModuleHost build artifacts not found at $hostBuildDir (OPENVR_PAIR_BUILD_FACE_HOST=OFF or .NET SDK missing). Driver will load without the host sidecar." -ForegroundColor Yellow
}

if ($Release) {
	# Pack the deployable driver tree (manifest, resources, bin/win64/DLL) into
	# a zip plus a sibling manifest TSV. The release workflow consumes both --
	# the zip is the asset, the manifest feeds the File integrity section of
	# the release body.
	$driverTree = "build/driver_wkopenvr"
	if (-not (Test-Path $driverTree)) {
		throw "Driver tree not found at $driverTree -- expected the CMake post-build copy step to populate it."
	}
	New-Item -ItemType Directory -Force -Path "release" | Out-Null
	$stageDir = Join-Path "release" "_stage_$Version"
	if (Test-Path $stageDir) { Remove-Item -Recurse -Force $stageDir }
	New-Item -ItemType Directory -Force -Path $stageDir | Out-Null
	Copy-Item -Recurse -Path "$driverTree/*" -Destination $stageDir

	$stagedDriverBin = Join-Path $stageDir "bin/win64"
	$bareDriverDll = Join-Path $stagedDriverBin "driver_wkopenvr.dll"
	$loaderDriverDll = Join-Path $stagedDriverBin "driver_01wkopenvr.dll"
	if (Test-Path $bareDriverDll) {
		Move-Item -Force -Path $bareDriverDll -Destination $loaderDriverDll
	}
	if (-not (Test-Path $loaderDriverDll)) {
		throw "Staged shared driver DLL not found at $loaderDriverDll"
	}

	$zipName = "WKOpenVR-v$Version.zip"
	$zipPath = Join-Path "release" $zipName
	if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
	Compress-Archive -Path "$stageDir/*" -DestinationPath $zipPath -CompressionLevel Optimal
	$zipItem = Get-Item $zipPath

	$manifestName = "WKOpenVR-v$Version.manifest.tsv"
	$manifestPath = Join-Path "release" $manifestName
	$rootLength = (Resolve-Path $stageDir).Path.Length + 1
	$rows = Get-ChildItem $stageDir -Recurse -File | ForEach-Object {
		$rel = $_.FullName.Substring($rootLength).Replace('\', '/')
		$h = (Get-FileHash $_.FullName -Algorithm SHA256).Hash
		"{0}`t{1}`t{2}" -f $h, $_.Length, $rel
	}
	# WriteAllLines via System.IO.File with a no-BOM UTF8Encoding so the
	# downstream Generate-ReleaseNotes.ps1 manifest parser sees clean column-1
	# bytes rather than the UTF-8 BOM that Out-File -Encoding utf8 prepends on
	# Windows PowerShell 5.1.
	$enc = [System.Text.UTF8Encoding]::new($false)
	[System.IO.File]::WriteAllLines((Resolve-Path -LiteralPath (Split-Path $manifestPath)).Path + "\" + (Split-Path -Leaf $manifestPath), $rows, $enc)
	Remove-Item -Recurse -Force $stageDir

	Write-Host ""
	Write-Host ("Packaged release zip:      {0} ({1:N0} bytes)" -f $zipItem.Name, $zipItem.Length)
	Write-Host ("Packaged release manifest: {0}" -f $manifestName)
}
