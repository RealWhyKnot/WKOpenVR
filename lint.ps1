param(
	# Verify formatting without changing files.
	[switch]$Check
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

function Resolve-RequiredCommand {
	param(
		[Parameter(Mandatory=$true)][string]$Name,
		[Parameter(Mandatory=$true)][string]$InstallHint
	)

	$command = Get-Command $Name -ErrorAction SilentlyContinue
	if (-not $command) {
		throw "$Name was not found. $InstallHint"
	}
	return $command.Source
}

function Invoke-NativeQuiet {
	param(
		[Parameter(Mandatory=$true)][scriptblock]$Command,
		[Parameter(Mandatory=$true)][string]$FailureMessage
	)

	$prevEap = $ErrorActionPreference
	$ErrorActionPreference = "Continue"
	try {
		& $Command 2>&1 | ForEach-Object {
			if ($_ -is [System.Management.Automation.ErrorRecord]) {
				Write-Host $_.Exception.Message
			} else {
				Write-Host $_
			}
		}
	} finally {
		$ErrorActionPreference = $prevEap
	}

	if ($LASTEXITCODE -ne 0) {
		throw "$FailureMessage (exit $LASTEXITCODE)"
	}
}

function Get-RepoFiles {
	param([Parameter(Mandatory=$true)][string[]]$Patterns)

	$gitArgs = @("ls-files", "-co", "--exclude-standard", "--") + $Patterns
	$files = @(& git @gitArgs)
	if ($LASTEXITCODE -ne 0) {
		throw "Unable to list repository files."
	}
	return @($files | ForEach-Object { $_ -replace "\\", "/" })
}

function Test-IsProjectCppFile {
	param([Parameter(Mandatory=$true)][string]$Path)

	if ($Path -match "^(lib|build|release|drivers)/") { return $false }
	if ($Path -match "(^|/)BuildStamp\.h$") { return $false }
	if ($Path -eq "core/src/common/BuildChannel.h") { return $false }
	if (-not (Test-Path -LiteralPath (Join-Path $PSScriptRoot $Path))) { return $false }
	return $true
}

function Invoke-ClangFormat {
	param([Parameter(Mandatory=$true)][string]$ClangFormat)

	$patterns = @("*.c", "*.cc", "*.cpp", "*.cxx", "*.h", "*.hpp")
	$files = @(Get-RepoFiles -Patterns $patterns | Where-Object { Test-IsProjectCppFile $_ } | Sort-Object -Unique)
	if ($files.Count -eq 0) {
		Write-Host "clang-format: no project C/C++ files found."
		return
	}

	$listDir = Join-Path $PSScriptRoot "build\lint"
	New-Item -ItemType Directory -Force -Path $listDir | Out-Null
	$listPath = Join-Path $listDir "clang-format-files.txt"
	$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
	[System.IO.File]::WriteAllLines($listPath, $files, $utf8NoBom)

	if ($Check) {
		Write-Host "clang-format: checking $($files.Count) file(s)."
		Invoke-NativeQuiet `
			-Command { & $ClangFormat "--dry-run" "--Werror" "--files=$listPath" } `
			-FailureMessage "clang-format found files that need cleanup"
	} else {
		Write-Host "clang-format: formatting $($files.Count) file(s)."
		Invoke-NativeQuiet `
			-Command { & $ClangFormat "-i" "--files=$listPath" } `
			-FailureMessage "clang-format failed"
	}
}

function Invoke-DotNetFormatCommand {
	param(
		[Parameter(Mandatory=$true)][string]$DotNet,
		[Parameter(Mandatory=$true)][string]$Workspace,
		[Parameter(Mandatory=$true)][string]$CommandName,
		[string[]]$Exclude = @(),
		[string[]]$ExcludeDiagnostics = @(),
		[string[]]$ExtraArgs = @()
	)

	if (-not (Test-Path -LiteralPath $Workspace)) {
		Write-Host "dotnet format: skipping missing workspace $Workspace"
		return
	}

	$formatArgs = @("format", $Workspace, $CommandName)
	if ($Check) { $formatArgs += "--verify-no-changes" }
	foreach ($path in $Exclude) {
		$formatArgs += @("--exclude", $path)
	}
	foreach ($diagnostic in $ExcludeDiagnostics) {
		$formatArgs += @("--exclude-diagnostics", $diagnostic)
	}
	$formatArgs += $ExtraArgs

	$label = if ($Check) { "checking" } else { "formatting" }
	Write-Host "dotnet format: $label $CommandName in $Workspace"
	Invoke-NativeQuiet `
		-Command { & $DotNet @formatArgs } `
		-FailureMessage "dotnet format $CommandName failed for $Workspace"
}

function Invoke-DotNetFormat {
	param([Parameter(Mandatory=$true)][string]$DotNet)

	$hostSolution = "modules/facetracking/src/host/WKOpenVR.FaceTracking.sln"
	$hostExcludes = @(
		"modules/facetracking/src/host/WKOpenVR.FaceTracking.UpstreamSdk",
		"modules/facetracking/src/host/WKOpenVR.FaceTracking.UpstreamRuntime"
	)
	$styleExcludes = @("IDE0060")
	$testProject = "tests/e2e/facetracking_test_module/WKOpenVR.FaceTracking.TestModule.csproj"

	Invoke-DotNetFormatCommand -DotNet $DotNet -Workspace $hostSolution -CommandName "whitespace" -Exclude $hostExcludes -ExtraArgs @("-v", "minimal")
	Invoke-DotNetFormatCommand -DotNet $DotNet -Workspace $hostSolution -CommandName "style" -Exclude $hostExcludes -ExcludeDiagnostics $styleExcludes -ExtraArgs @("--severity", "info", "-v", "minimal")

	Invoke-DotNetFormatCommand -DotNet $DotNet -Workspace $testProject -CommandName "whitespace" -ExtraArgs @("-v", "minimal")
	Invoke-DotNetFormatCommand -DotNet $DotNet -Workspace $testProject -CommandName "style" -ExcludeDiagnostics $styleExcludes -ExtraArgs @("--severity", "info", "-v", "minimal")
}

$clangFormat = Resolve-RequiredCommand "clang-format" "Install LLVM and make clang-format available on PATH."
$dotnet = Resolve-RequiredCommand "dotnet" "Install the .NET SDK required by modules/facetracking/src/host/global.json."

Invoke-ClangFormat -ClangFormat $clangFormat
Invoke-DotNetFormat -DotNet $dotnet

if ($Check) {
	Write-Host "Lint check passed."
} else {
	Write-Host "Lint cleanup complete."
}
