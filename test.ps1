param(
	# Run the current test binaries without rebuilding first.
	[switch]$SkipBuild,

	# Pass through to build.ps1 when building.
	[switch]$SkipConfigure,

	# Optional GoogleTest filter, for example "Captions*".
	[string]$Filter = ""
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

if (-not $SkipBuild) {
	$buildArgs = @{}
	if ($SkipConfigure) { $buildArgs["SkipConfigure"] = $true }
	& "$PSScriptRoot\build.ps1" @buildArgs
	if ($LASTEXITCODE -ne 0) { throw "build.ps1 failed (exit $LASTEXITCODE)" }
}

$testDir = Join-Path $PSScriptRoot "build\artifacts\Release"
$tests = @(Get-ChildItem -LiteralPath $testDir -Filter "*_tests.exe" -File -ErrorAction SilentlyContinue | Sort-Object Name)
if ($tests.Count -eq 0) {
	throw "No test binaries found under $testDir. Run build.ps1 first."
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

Write-Host ""
Write-Host ("All {0} test binaries passed." -f $tests.Count)
