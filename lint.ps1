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

function Resolve-RunClangTidy {
	param([Parameter(Mandatory=$true)][string]$ClangTidy)

	$command = Get-Command "run-clang-tidy" -ErrorAction SilentlyContinue
	if ($command) {
		return $command.Source
	}

	$clangTidyDir = Split-Path -Parent $ClangTidy
	$candidates = @(
		(Join-Path $clangTidyDir "run-clang-tidy"),
		(Join-Path $clangTidyDir "run-clang-tidy.py")
	)
	foreach ($candidate in $candidates) {
		if (Test-Path -LiteralPath $candidate) {
			return $candidate
		}
	}

	throw "run-clang-tidy was not found. Install LLVM Python tools and make run-clang-tidy available next to clang-tidy or on PATH."
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

function Invoke-ClangTidy {
	param(
		[Parameter(Mandatory=$true)][string]$CMake,
		[Parameter(Mandatory=$true)][string]$Ninja,
		[Parameter(Mandatory=$true)][string]$Python,
		[Parameter(Mandatory=$true)][string]$RunClangTidy,
		[Parameter(Mandatory=$true)][string]$ClangTidy,
		[Parameter(Mandatory=$true)][string]$ClangApplyReplacements
	)

	$buildDir = Join-Path $PSScriptRoot "build\lint-tidy"
	$cmakeArgs = @(
		"-G", "Ninja",
		"-B", $buildDir,
		"-S", $PSScriptRoot,
		"-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
		"-DWKOPENVR_CAPTIONS_CUDA=OFF",
		"-DWKOPENVR_RELEASE_BUILD=OFF",
		"-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
		"-DCMAKE_MAKE_PROGRAM=$Ninja",
		"-Wno-dev"
	)

	Write-Host "clang-tidy: configuring compile database in $buildDir"
	Invoke-NativeQuiet `
		-Command { & $CMake @cmakeArgs } `
		-FailureMessage "clang-tidy compile database configure failed"

	$compileCommands = Join-Path $buildDir "compile_commands.json"
	if (-not (Test-Path -LiteralPath $compileCommands)) {
		throw "clang-tidy compile database was not created at $compileCommands"
	}

	$parallelJobs = [Math]::Max(1, [Math]::Min([Environment]::ProcessorCount, 8))
	$sourceFilter = ".*WKOpenVR[/\\](core|modules|tests)[/\\].*\.(c|cc|cpp|cxx)$"
	$headerFilter = ".*WKOpenVR[/\\](core|modules|tests)[/\\].*"
	$excludeHeaderFilter = ".*WKOpenVR[/\\](lib|build)[/\\].*"
	$tidyArgs = @(
		$RunClangTidy,
		"-p", $buildDir,
		"-config-file", (Join-Path $PSScriptRoot ".clang-tidy"),
		"-source-filter", $sourceFilter,
		"-header-filter", $headerFilter,
		"-exclude-header-filter", $excludeHeaderFilter,
		"-clang-tidy-binary", $ClangTidy,
		"-clang-apply-replacements-binary", $ClangApplyReplacements,
		"-quiet",
		"-hide-progress",
		"-j", [string]$parallelJobs
	)

	if (-not $Check) {
		$tidyArgs += @("-fix", "-format", "-style=file")
	}

	$label = if ($Check) { "checking" } else { "checking and applying available fixes to" }
	Write-Host "clang-tidy: $label project C/C++ translation units."
	Invoke-NativeQuiet `
		-Command { & $Python @tidyArgs } `
		-FailureMessage "clang-tidy found diagnostics"
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
$clangTidy = Resolve-RequiredCommand "clang-tidy" "Install LLVM and make clang-tidy available on PATH."
$clangApplyReplacements = Resolve-RequiredCommand "clang-apply-replacements" "Install LLVM and make clang-apply-replacements available on PATH."
$cmake = Resolve-RequiredCommand "cmake" "Install CMake and make cmake available on PATH."
$ninja = Resolve-RequiredCommand "ninja" "Install Ninja and make ninja available on PATH."
$python = Resolve-RequiredCommand "python" "Install Python and make python available on PATH."
$runClangTidy = Resolve-RunClangTidy -ClangTidy $clangTidy
$dotnet = Resolve-RequiredCommand "dotnet" "Install the .NET SDK required by modules/facetracking/src/host/global.json."

Invoke-ClangFormat -ClangFormat $clangFormat
Invoke-DotNetFormat -DotNet $dotnet
Invoke-ClangTidy `
	-CMake $cmake `
	-Ninja $ninja `
	-Python $python `
	-RunClangTidy $runClangTidy `
	-ClangTidy $clangTidy `
	-ClangApplyReplacements $clangApplyReplacements
if (-not $Check) {
	Invoke-ClangFormat -ClangFormat $clangFormat
}

if ($Check) {
	Write-Host "Lint check passed."
} else {
	Write-Host "Lint cleanup complete."
}
