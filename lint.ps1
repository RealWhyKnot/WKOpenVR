#Requires -Version 5.1
<#
.SYNOPSIS
Formats and lints the project's C/C++ and C# sources.
.DESCRIPTION
Runs clang-format over project C/C++ files, dotnet format over the
facetracking host solution and test module, and clang-tidy against a
Ninja-generated compile database. Supports check-only, format-only, and
changed-files-only runs.
.EXAMPLE
./lint.ps1 -Check -ChangedOnly
#>
[CmdletBinding()]
param(
	# Verify formatting without changing files.
	[switch]$Check,

	# Fast path for format-only failures. Skips clang-tidy entirely; use this
	# when CI reports clang-format or dotnet format drift and no C++ analysis is
	# needed.
	[Alias("SkipTidy")]
	[switch]$FormatOnly,

	# Limit format operations and clang-tidy source analysis to files changed
	# from ChangedBase plus staged, unstaged, and untracked files. Header-only
	# C/C++ changes still run full clang-tidy because headers are not compile
	# database translation units.
	[switch]$ChangedOnly,

	[string]$ChangedBase = "origin/main"
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

function Get-ChangedRepoFiles {
	param([Parameter(Mandatory=$true)][string]$BaseRef)

	$paths = New-Object System.Collections.Generic.List[string]
	$diffSources = @()

	$mergeBase = & git merge-base HEAD $BaseRef 2>$null
	if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($mergeBase)) {
		$diffSources += @($mergeBase.Trim() + "...HEAD")
	} else {
		Write-Host "changed-only: could not resolve merge-base with $BaseRef; using HEAD only."
	}

	$diffSources += @("--cached", "")
	foreach ($source in $diffSources) {
		$gitArgs = @("diff", "--name-only", "--diff-filter=ACMRTUXB")
		if (-not [string]::IsNullOrWhiteSpace($source)) {
			$gitArgs += $source
		}
		$diffPaths = @(& git @gitArgs)
		if ($LASTEXITCODE -ne 0) {
			throw "Unable to list changed files."
		}
		foreach ($path in $diffPaths) {
			if (-not [string]::IsNullOrWhiteSpace($path)) {
				$paths.Add(($path -replace "\\", "/"))
			}
		}
	}

	$untracked = @(& git ls-files --others --exclude-standard)
	if ($LASTEXITCODE -ne 0) {
		throw "Unable to list untracked files."
	}
	foreach ($path in $untracked) {
		if (-not [string]::IsNullOrWhiteSpace($path)) {
			$paths.Add(($path -replace "\\", "/"))
		}
	}

	return @($paths | Sort-Object -Unique)
}

function Test-IsChangedFile {
	param([Parameter(Mandatory=$true)][string]$Path)

	if (-not $ChangedOnly) { return $true }
	if ($null -eq $script:ChangedFileLookup) { return $false }
	return $script:ChangedFileLookup.ContainsKey($Path)
}

function Test-IsProjectCppFile {
	param([Parameter(Mandatory=$true)][string]$Path)

	if ($Path -match "^(lib|build|release|drivers)/") { return $false }
	if ($Path -match "(^|/)BuildStamp\.h$") { return $false }
	if ($Path -eq "core/src/common/BuildChannel.h") { return $false }
	if (-not (Test-Path -LiteralPath (Join-Path $PSScriptRoot $Path))) { return $false }
	return $true
}

function Test-IsCppSourceFile {
	param([Parameter(Mandatory=$true)][string]$Path)

	return $Path -match "\.(c|cc|cpp|cxx)$"
}

function Test-IsCppHeaderFile {
	param([Parameter(Mandatory=$true)][string]$Path)

	return $Path -match "\.(h|hpp)$"
}

function Convert-RepoPathToRegex {
	param([Parameter(Mandatory=$true)][string]$Path)

	$escaped = [regex]::Escape(($Path -replace "\\", "/"))
	return ".*" + ($escaped -replace "/", "[/\\]") + "$"
}

function Get-ClangTidyFileFilters {
	if (-not $ChangedOnly) {
		return [pscustomobject]@{ Skip = $false; Filters = @() }
	}

	$changedProjectCpp = @($script:ChangedFiles | Where-Object { Test-IsProjectCppFile $_ })
	$changedSources = @($changedProjectCpp | Where-Object { Test-IsCppSourceFile $_ })
	if ($changedSources.Count -gt 0) {
		Write-Host "clang-tidy: changed-only includes $($changedSources.Count) C/C++ source file(s)."
		return [pscustomobject]@{ Skip = $false; Filters = @($changedSources | ForEach-Object { Convert-RepoPathToRegex $_ }) }
	}

	$changedHeaders = @($changedProjectCpp | Where-Object { Test-IsCppHeaderFile $_ })
	if ($changedHeaders.Count -gt 0) {
		Write-Host "clang-tidy: changed-only saw $($changedHeaders.Count) C/C++ header file(s); running full project analysis."
		return [pscustomobject]@{ Skip = $false; Filters = @() }
	}

	Write-Host "clang-tidy: no changed project C/C++ source/header files found."
	return [pscustomobject]@{ Skip = $true; Filters = @() }
}

function Invoke-ClangFormat {
	param([Parameter(Mandatory=$true)][string]$ClangFormat)

	$patterns = @("*.c", "*.cc", "*.cpp", "*.cxx", "*.h", "*.hpp")
	$files = @(Get-RepoFiles -Patterns $patterns | Where-Object { (Test-IsProjectCppFile $_) -and (Test-IsChangedFile $_) } | Sort-Object -Unique)
	if ($files.Count -eq 0) {
		$message = if ($ChangedOnly) { "clang-format: no changed project C/C++ files found." } else { "clang-format: no project C/C++ files found." }
		Write-Host $message
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

	$fileFilterPlan = Get-ClangTidyFileFilters
	if ($fileFilterPlan.Skip) {
		return
	}

	$buildDir = Join-Path $PSScriptRoot "build\lint-tidy"
	$cmakeArgs = @(
		"-G", "Ninja",
		"-B", $buildDir,
		"-S", $PSScriptRoot,
		"-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
		"-DWKOPENVR_CAPTIONS_CUDA=OFF",
		"-DWKOPENVR_CAPTIONS_VULKAN=OFF",
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
	foreach ($filter in $fileFilterPlan.Filters) {
		$tidyArgs += $filter
	}

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
		[string[]]$Include = @(),
		[string[]]$ExtraArgs = @()
	)

	if (-not (Test-Path -LiteralPath $Workspace)) {
		Write-Host "dotnet format: skipping missing workspace $Workspace"
		return
	}

	$formatArgs = @("format", $Workspace, $CommandName)
	if ($Check) { $formatArgs += "--verify-no-changes" }
	foreach ($path in $Include) {
		$formatArgs += @("--include", $path)
	}
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
	$changedCsFiles = @()
	if ($ChangedOnly -and $null -ne $script:ChangedFiles) {
		$changedCsFiles = @($script:ChangedFiles | Where-Object { $_ -match "\.cs$" })
		if ($changedCsFiles.Count -eq 0) {
			Write-Host "dotnet format: no changed C# files found."
			return
		}
		Write-Host "dotnet format: changed-only includes $($changedCsFiles.Count) C# file(s)."
	}

	Invoke-DotNetFormatCommand -DotNet $DotNet -Workspace $hostSolution -CommandName "whitespace" -Exclude $hostExcludes -Include $changedCsFiles -ExtraArgs @("-v", "minimal")
	Invoke-DotNetFormatCommand -DotNet $DotNet -Workspace $hostSolution -CommandName "style" -Exclude $hostExcludes -ExcludeDiagnostics $styleExcludes -Include $changedCsFiles -ExtraArgs @("--severity", "info", "-v", "minimal")

	Invoke-DotNetFormatCommand -DotNet $DotNet -Workspace $testProject -CommandName "whitespace" -Include $changedCsFiles -ExtraArgs @("-v", "minimal")
	Invoke-DotNetFormatCommand -DotNet $DotNet -Workspace $testProject -CommandName "style" -ExcludeDiagnostics $styleExcludes -Include $changedCsFiles -ExtraArgs @("--severity", "info", "-v", "minimal")
}

if ($ChangedOnly) {
	$script:ChangedFiles = @(Get-ChangedRepoFiles -BaseRef $ChangedBase)
	$script:ChangedFileLookup = @{}
	foreach ($path in $script:ChangedFiles) {
		$script:ChangedFileLookup[$path] = $true
	}
	Write-Host "changed-only: considering $($script:ChangedFiles.Count) file(s) from $ChangedBase, index, working tree, and untracked files."
} else {
	$script:ChangedFiles = $null
	$script:ChangedFileLookup = $null
}

$clangFormat = Resolve-RequiredCommand "clang-format" "Install LLVM and make clang-format available on PATH."
$dotnet = Resolve-RequiredCommand "dotnet" "Install the .NET SDK required by modules/facetracking/src/host/global.json."

Invoke-ClangFormat -ClangFormat $clangFormat
Invoke-DotNetFormat -DotNet $dotnet

if (-not $FormatOnly) {
	$clangTidy = Resolve-RequiredCommand "clang-tidy" "Install LLVM and make clang-tidy available on PATH."
	$clangApplyReplacements = Resolve-RequiredCommand "clang-apply-replacements" "Install LLVM and make clang-apply-replacements available on PATH."
	$cmake = Resolve-RequiredCommand "cmake" "Install CMake and make cmake available on PATH."
	$ninja = Resolve-RequiredCommand "ninja" "Install Ninja and make ninja available on PATH."
	$python = Resolve-RequiredCommand "python" "Install Python and make python available on PATH."
	$runClangTidy = Resolve-RunClangTidy -ClangTidy $clangTidy

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
} else {
	Write-Host "format-only: skipped clang-tidy."
}

if ($Check) {
	Write-Host "Lint check passed."
} else {
	Write-Host "Lint cleanup complete."
}
