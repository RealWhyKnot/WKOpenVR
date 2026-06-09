param(
	# Run the current test binaries without rebuilding first.
	[switch]$SkipBuild,

	# Pass through to build.ps1 when building.
	[switch]$SkipConfigure,

	# Optional GoogleTest filter, for example "Captions*".
	[string]$Filter = "",

	# Optional logical suites to build and run, for example calibration.
	# Known suites map to one CMake target and one *_tests.exe binary.
	[string[]]$Suite = @(),

	# Optional exact test binaries to run, for example spacecal_tests.exe.
	# The CMake build target is inferred from the binary base name.
	[string[]]$TestBinary = @(),

	# Skip the WKOpenVR.exe --test-harness in-process driver harness step.
	# Useful when iterating on a single per-module gtest exe without paying
	# the harness's startup cost.
	[switch]$SkipHarness,

	# Skip the Phase 4 download + load tests. Useful when iterating on the
	# in-process harness alone.
	[switch]$SkipDownload,

	# Optional filter passed to WKOpenVR.exe --test-harness --filter <list>
	# (comma-separated slug list: calibration,smoothing,inputhealth, ...).
	[string]$HarnessFilter = ""
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

function Invoke-NativeQuiet {
	param([scriptblock]$Cmd)
	$prevEap = $ErrorActionPreference
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
		$ErrorActionPreference = $prevEap
	}
}

$KnownTestSpecs = @(
	@{ Suite = "calibration"; Binary = "spacecal_tests.exe"; Target = "spacecal_tests" },
	@{ Suite = "captions"; Binary = "captions_tests.exe"; Target = "captions_tests" },
	@{ Suite = "driver"; Binary = "driver_safety_tests.exe"; Target = "driver_safety_tests" },
	@{ Suite = "e2e"; Binary = "e2e_tests.exe"; Target = "e2e_tests" },
	@{ Suite = "facetracking"; Binary = "facetracking_tests.exe"; Target = "facetracking_tests" },
	@{ Suite = "inputhealth"; Binary = "inputhealth_tests.exe"; Target = "inputhealth_tests" },
	@{ Suite = "oscrouter"; Binary = "oscrouter_tests.exe"; Target = "oscrouter_tests" },
	@{ Suite = "phantom"; Binary = "phantom_tests.exe"; Target = "phantom_tests" },
	@{ Suite = "questapp"; Binary = "questapp_tests.exe"; Target = "questapp_tests" },
	@{ Suite = "smoothing"; Binary = "smoothing_tests.exe"; Target = "smoothing_tests" },
	@{ Suite = "ui"; Binary = "ui_core_tests.exe"; Target = "ui_core_tests" }
)

function Normalize-TestBinaryName {
	param([string]$Name)
	if ([string]::IsNullOrWhiteSpace($Name)) { return "" }
	$Value = $Name.Trim()
	if (-not $Value.EndsWith(".exe", [System.StringComparison]::OrdinalIgnoreCase)) {
		$Value = "$Value.exe"
	}
	return $Value.ToLowerInvariant()
}

function Normalize-TestName {
	param([string]$Name)
	if ([string]::IsNullOrWhiteSpace($Name)) { return "" }
	return $Name.Trim().ToLowerInvariant()
}

$SuiteNames = @()
foreach ($Name in $Suite) {
	$Normalized = Normalize-TestName $Name
	if ($Normalized) { $SuiteNames += $Normalized }
}
$SuiteNames = @($SuiteNames | Select-Object -Unique)

$SelectedSpecs = @()
foreach ($SuiteName in $SuiteNames) {
	$Matches = @($KnownTestSpecs | Where-Object {
		$SpecSuite = Normalize-TestName $_["Suite"]
		$SpecTarget = Normalize-TestName $_["Target"]
		$SpecBinary = Normalize-TestBinaryName $_["Binary"]
		$RequestedBinary = Normalize-TestBinaryName $SuiteName
		$SpecSuite -eq $SuiteName -or $SpecTarget -eq $SuiteName -or $SpecBinary -eq $RequestedBinary
	})
	if ($Matches.Count -eq 0) {
		$KnownSuites = (($KnownTestSpecs | ForEach-Object { $_["Suite"] }) -join ", ")
		throw "Unknown test suite '$SuiteName'. Known suites: $KnownSuites"
	}
	$SelectedSpecs += $Matches
}

$SelectedBinaryNames = @{}
$BuildTargets = @()
foreach ($Spec in $SelectedSpecs) {
	$BinaryName = Normalize-TestBinaryName $Spec["Binary"]
	if ($BinaryName) { $SelectedBinaryNames[$BinaryName] = $true }
	if ($Spec["Target"]) { $BuildTargets += $Spec["Target"] }
}

foreach ($Name in $TestBinary) {
	$BinaryName = Normalize-TestBinaryName $Name
	if (-not $BinaryName) { continue }
	$SelectedBinaryNames[$BinaryName] = $true
	$BuildTargets += [System.IO.Path]::GetFileNameWithoutExtension($BinaryName)
}

$BuildTargets = @($BuildTargets | Where-Object { -not [string]::IsNullOrWhiteSpace($_) } | Select-Object -Unique)
$FocusedRun = ($SelectedBinaryNames.Count -gt 0)

if (-not $SkipBuild) {
	$buildArgs = @{}
	if ($SkipConfigure) { $buildArgs["SkipConfigure"] = $true }
	if ($BuildTargets.Count -gt 0) { $buildArgs["Target"] = $BuildTargets }
	& "$PSScriptRoot\build.ps1" @buildArgs
	if ($LASTEXITCODE -ne 0) { throw "build.ps1 failed (exit $LASTEXITCODE)" }
}

$testDir = Join-Path $PSScriptRoot "build\artifacts\Release"
$allTests = @(Get-ChildItem -LiteralPath $testDir -Filter "*_tests.exe" -File -ErrorAction SilentlyContinue | Sort-Object Name)
if ($allTests.Count -eq 0) {
	throw "No test binaries found under $testDir. Run build.ps1 first."
}

$tests = $allTests
if ($FocusedRun) {
	$tests = @($allTests | Where-Object { $SelectedBinaryNames.ContainsKey($_.Name.ToLowerInvariant()) } | Sort-Object Name)
	$FoundBinaryNames = @{}
	foreach ($Test in $tests) {
		$FoundBinaryNames[$Test.Name.ToLowerInvariant()] = $true
	}
	$MissingBinaryNames = @()
	foreach ($BinaryName in $SelectedBinaryNames.Keys) {
		if (-not $FoundBinaryNames.ContainsKey($BinaryName)) {
			$MissingBinaryNames += $BinaryName
		}
	}
	if ($MissingBinaryNames.Count -gt 0) {
		throw "Selected test binary not found under ${testDir}: $($MissingBinaryNames -join ', ')"
	}
}

$testArgs = @("--gtest_brief=1")
if ($Filter) {
	$testArgs += "--gtest_filter=$Filter"
}

foreach ($test in $tests) {
	Write-Host ""
	Write-Host ("== Running {0} ==" -f $test.Name)
	Invoke-NativeQuiet { & $test.FullName @testArgs }
	if ($LASTEXITCODE -ne 0) {
		throw "$($test.Name) failed (exit $LASTEXITCODE)"
	}
}

$runCaptionsHost = -not $FocusedRun
if ($runCaptionsHost) {
	$captionsHost = Join-Path $PSScriptRoot "build\driver_wkopenvr\resources\captions\host\WKOpenVR.CaptionsHost.exe"
	if (-not (Test-Path -LiteralPath $captionsHost)) {
		throw "Captions host missing at $captionsHost"
	}
	Write-Host ""
	Write-Host "== Running WKOpenVR.CaptionsHost.exe --self-test =="
	Invoke-NativeQuiet { & $captionsHost --self-test }
	if ($LASTEXITCODE -ne 0) {
		throw "Captions host self-test failed (exit $LASTEXITCODE)"
	}
}

$runPhantomSidecar = -not $FocusedRun
if ($runPhantomSidecar) {
	$phantomSidecar = Join-Path $PSScriptRoot "build\driver_wkopenvr\resources\phantom\host\WKOpenVRPhantomSidecar.exe"
	if (-not (Test-Path -LiteralPath $phantomSidecar)) {
		throw "Phantom sidecar missing at $phantomSidecar"
	}
	Write-Host ""
	Write-Host "== Running WKOpenVRPhantomSidecar.exe --self-test =="
	Invoke-NativeQuiet { & $phantomSidecar --self-test }
	if ($LASTEXITCODE -ne 0) {
		throw "Phantom sidecar self-test failed (exit $LASTEXITCODE)"
	}
}

# --------------------------------------------------------------------------
# Phase 3: in-process driver harness (--test-harness)
# --------------------------------------------------------------------------
$runHarness = -not $SkipHarness
if ($FocusedRun -and -not $HarnessFilter) {
	$runHarness = $false
}

if ($runHarness) {
	$harnessExe = Join-Path $PSScriptRoot "build\artifacts\Release\WKOpenVR.exe"
	if (-not (Test-Path -LiteralPath $harnessExe)) {
		throw "WKOpenVR.exe missing at $harnessExe -- did the dev build complete?"
	}
	Write-Host ""
	Write-Host "== Phase 3: WKOpenVR.exe --test-harness =="
	$harnessArgs = @("--test-harness")
	if ($HarnessFilter) {
		$harnessArgs += @("--filter", $HarnessFilter)
	}
	# WKOpenVR.exe is a WIN32-subsystem binary. PowerShell does not wait for
	# it through `& exe`, so use Start-Process and relay captured output.
	$harnessStdout = [System.IO.Path]::GetTempFileName()
	$harnessStderr = [System.IO.Path]::GetTempFileName()
	try {
		$p = Start-Process -FilePath $harnessExe -ArgumentList $harnessArgs `
			-RedirectStandardOutput $harnessStdout `
			-RedirectStandardError $harnessStderr `
			-WindowStyle Hidden -Wait -PassThru
		if (Test-Path -LiteralPath $harnessStdout) {
			Get-Content -LiteralPath $harnessStdout | ForEach-Object { Write-Host $_ }
		}
		if (Test-Path -LiteralPath $harnessStderr) {
			Get-Content -LiteralPath $harnessStderr | ForEach-Object { Write-Host $_ }
		}
		if ($p.ExitCode -ne 0) {
			throw "WKOpenVR.exe --test-harness failed (exit $($p.ExitCode))"
		}
	} finally {
		Remove-Item -LiteralPath $harnessStdout, $harnessStderr -Force -ErrorAction SilentlyContinue
	}
}

# --------------------------------------------------------------------------
# Phase 4: download + load (captions models, VRCFT modules)
# --------------------------------------------------------------------------
$runDownload = -not $SkipDownload
if ($FocusedRun) {
	$runDownload = $false
}

if ($runDownload) {
	$downloadRunner = Join-Path $PSScriptRoot "tests\download\Run-DownloadTests.ps1"
	if (-not (Test-Path -LiteralPath $downloadRunner)) {
		Write-Host "[skip] Phase 4: $downloadRunner not present" -ForegroundColor Yellow
	} else {
		Write-Host ""
		Write-Host "== Phase 4: download + load =="
		Invoke-NativeQuiet { & $downloadRunner }
		if ($LASTEXITCODE -ne 0) {
			throw "Download tests failed (exit $LASTEXITCODE)"
		}
	}
}

Write-Host ""
Write-Host ("All {0} selected test binaries passed." -f $tests.Count)
