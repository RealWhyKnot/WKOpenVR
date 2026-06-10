[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$Planner = Join-Path $ScriptRoot "Get-NightlyBetaPlan.ps1"

function Invoke-TestGit {
  param(
    [string] $RepoRoot,
    [string[]] $Arguments
  )

  Push-Location $RepoRoot
  try {
    $output = & git @Arguments
    if ($LASTEXITCODE -ne 0) {
      throw "git $($Arguments -join ' ') failed with exit code $LASTEXITCODE"
    }
    return @($output)
  }
  finally {
    Pop-Location
  }
}

function Write-TestFile {
  param(
    [string] $Path,
    [string] $Content
  )

  $parent = Split-Path -Parent $Path
  if (-not (Test-Path -LiteralPath $parent)) {
    New-Item -ItemType Directory -Path $parent | Out-Null
  }

  [System.IO.File]::WriteAllText($Path, $Content, (New-Object System.Text.UTF8Encoding($false)))
}

function New-TestRepo {
  $root = Join-Path ([System.IO.Path]::GetTempPath()) ("wkopenvr-nightly-beta-" + [System.Guid]::NewGuid().ToString("N"))
  New-Item -ItemType Directory -Path $root | Out-Null

  Invoke-TestGit -RepoRoot $root -Arguments @("init", "-q", ".") | Out-Null
  Invoke-TestGit -RepoRoot $root -Arguments @("config", "user.name", "WKOpenVR Tests") | Out-Null
  Invoke-TestGit -RepoRoot $root -Arguments @("config", "user.email", "wkopenvr-tests@example.invalid") | Out-Null
  Invoke-TestGit -RepoRoot $root -Arguments @("config", "core.autocrlf", "false") | Out-Null

  Write-TestFile -Path (Join-Path $root "CMakeLists.txt") -Content "cmake_minimum_required(VERSION 3.25)`n"
  Write-TestFile -Path (Join-Path $root "core/src/main.cpp") -Content "int main() { return 0; }`n"
  Write-TestFile -Path (Join-Path $root "modules/calibration/source.txt") -Content "calibration`n"
  Write-TestFile -Path (Join-Path $root "modules/smoothing/source.txt") -Content "smoothing`n"
  Write-TestFile -Path (Join-Path $root "modules/inputhealth/source.txt") -Content "inputhealth`n"
  Write-TestFile -Path (Join-Path $root "modules/facetracking/source.txt") -Content "facetracking`n"
  Write-TestFile -Path (Join-Path $root "modules/oscrouter/source.txt") -Content "oscrouter`n"
  Write-TestFile -Path (Join-Path $root "modules/questapp/source.txt") -Content "questapp`n"
  Write-TestFile -Path (Join-Path $root "modules/captions/source.txt") -Content "captions`n"

  Invoke-TestGit -RepoRoot $root -Arguments @("add", ".") | Out-Null
  Invoke-TestGit -RepoRoot $root -Arguments @("commit", "-q", "-m", "initial") | Out-Null
  Invoke-TestGit -RepoRoot $root -Arguments @("tag", "v2026.6.1.0") | Out-Null

  return $root
}

function Write-ReleaseState {
  param(
    [string] $RepoRoot,
    [string] $Tag
  )

  $state = [ordered]@{
    "RealWhyKnot/WKOpenVR-SpaceCalibrator" = $Tag
    "RealWhyKnot/WKOpenVR-Smoothing" = $Tag
    "RealWhyKnot/WKOpenVR-InputHealth" = $Tag
    "RealWhyKnot/WKOpenVR-FaceTracking" = $Tag
    "RealWhyKnot/WKOpenVR-OSCRouter" = $Tag
    "RealWhyKnot/WKOpenVR-QuestApp" = $Tag
    "RealWhyKnot/WKOpenVR-Captions" = $Tag
  }

  $path = Join-Path $RepoRoot "release-state.json"
  [System.IO.File]::WriteAllText($path, ($state | ConvertTo-Json -Depth 4), (New-Object System.Text.UTF8Encoding($false)))
  return $path
}

function Invoke-Plan {
  param(
    [string] $RepoRoot,
    [string] $ReleaseStatePath,
    [string] $Tag = "",
    [string] $Today = ""
  )

  $outputPath = Join-Path $RepoRoot "plan.json"
  $arguments = @{
    RepoRoot = $RepoRoot
    ReleaseStatePath = $ReleaseStatePath
    OutputJsonPath = $outputPath
  }
  if (-not [string]::IsNullOrWhiteSpace($Tag)) {
    $arguments["Tag"] = $Tag
  }
  if (-not [string]::IsNullOrWhiteSpace($Today)) {
    $arguments["Today"] = $Today
  }

  & $Planner @arguments | Out-Host
  if ($LASTEXITCODE -ne 0) {
    throw "Planner failed with exit code $LASTEXITCODE"
  }

  return (Get-Content -LiteralPath $outputPath -Raw | ConvertFrom-Json)
}

function Assert-Equal {
  param(
    [object] $Actual,
    [object] $Expected,
    [string] $Message
  )

  if ($Actual -ne $Expected) {
    throw "$Message. Expected '$Expected', got '$Actual'."
  }
}

function Assert-SequenceEqual {
  param(
    [object[]] $Actual,
    [object[]] $Expected,
    [string] $Message
  )

  $actualText = ($Actual -join ",")
  $expectedText = ($Expected -join ",")
  if ($actualText -ne $expectedText) {
    throw "$Message. Expected '$expectedText', got '$actualText'."
  }
}

$tempRoots = New-Object System.Collections.Generic.List[string]
try {
  $repo = New-TestRepo
  [void] $tempRoots.Add($repo)
  $state = Write-ReleaseState -RepoRoot $repo -Tag "v2026.6.1.0"
  Write-TestFile -Path (Join-Path $repo "modules/smoothing/source.txt") -Content "smoothing change`n"
  Invoke-TestGit -RepoRoot $repo -Arguments @("add", ".") | Out-Null
  Invoke-TestGit -RepoRoot $repo -Arguments @("commit", "-q", "-m", "change smoothing") | Out-Null
  $plan = Invoke-Plan -RepoRoot $repo -ReleaseStatePath $state -Tag "v2026.6.2.0-beta"
  Assert-Equal -Actual $plan.has_changes -Expected $true -Message "Module change should produce a beta plan"
  Assert-SequenceEqual -Actual @($plan.features) -Expected @("Smoothing") -Message "Module change should select only the changed module"

  $repo = New-TestRepo
  [void] $tempRoots.Add($repo)
  $state = Write-ReleaseState -RepoRoot $repo -Tag "v2026.6.1.0"
  Write-TestFile -Path (Join-Path $repo "core/src/main.cpp") -Content "int main() { return 1; }`n"
  Invoke-TestGit -RepoRoot $repo -Arguments @("add", ".") | Out-Null
  Invoke-TestGit -RepoRoot $repo -Arguments @("commit", "-q", "-m", "change shared core") | Out-Null
  $plan = Invoke-Plan -RepoRoot $repo -ReleaseStatePath $state -Tag "v2026.6.2.0-beta"
  Assert-SequenceEqual -Actual @($plan.features) -Expected @("Calibration", "Smoothing", "InputHealth", "FaceTracking", "OSCRouter", "QuestApp", "Captions") -Message "Shared changes should select every public module"

  $repo = New-TestRepo
  [void] $tempRoots.Add($repo)
  $state = Write-ReleaseState -RepoRoot $repo -Tag "v2026.6.1.0"
  $plan = Invoke-Plan -RepoRoot $repo -ReleaseStatePath $state -Tag "v2026.6.2.0-beta"
  Assert-Equal -Actual $plan.has_changes -Expected $false -Message "No changes should not produce a beta plan"
  Assert-SequenceEqual -Actual @($plan.features) -Expected @() -Message "No changes should select no modules"

  $repo = New-TestRepo
  [void] $tempRoots.Add($repo)
  $state = Write-ReleaseState -RepoRoot $repo -Tag "v2026.6.1.0"
  Invoke-TestGit -RepoRoot $repo -Arguments @("tag", "v2026.6.9.0-beta") | Out-Null
  Invoke-TestGit -RepoRoot $repo -Arguments @("tag", "v2026.6.9.1-beta") | Out-Null
  Write-TestFile -Path (Join-Path $repo "modules/captions/source.txt") -Content "captions change`n"
  Invoke-TestGit -RepoRoot $repo -Arguments @("add", ".") | Out-Null
  Invoke-TestGit -RepoRoot $repo -Arguments @("commit", "-q", "-m", "change captions") | Out-Null
  $plan = Invoke-Plan -RepoRoot $repo -ReleaseStatePath $state -Today "2026.6.9"
  Assert-Equal -Actual $plan.next_tag -Expected "v2026.6.9.2-beta" -Message "Next beta tag should increment the same-day sequence"

  $repo = New-TestRepo
  [void] $tempRoots.Add($repo)
  $state = Write-ReleaseState -RepoRoot $repo -Tag "v2026.6.1.0"
  $failed = $false
  try {
    Invoke-Plan -RepoRoot $repo -ReleaseStatePath $state -Tag "v2026.6.2.0-beta.1" | Out-Null
  }
  catch {
    $failed = $true
  }
  Assert-Equal -Actual $failed -Expected $true -Message "Planner should reject numbered beta suffixes"

  Write-Host "Nightly beta planner tests passed."
}
finally {
  foreach ($tempRoot in $tempRoots) {
    if (Test-Path -LiteralPath $tempRoot) {
      Remove-Item -LiteralPath $tempRoot -Recurse -Force
    }
  }
}
