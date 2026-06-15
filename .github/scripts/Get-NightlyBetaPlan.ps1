[CmdletBinding()]
param(
  [string] $RepoRoot,
  [string] $Tag = "",
  [string] $ReleaseStatePath = "",
  [string] $Today = "",
  [string] $OutputJsonPath = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Resolve-RepoRoot {
  param([string] $Value)

  if (-not [string]::IsNullOrWhiteSpace($Value)) {
    return (Resolve-Path -LiteralPath $Value).Path
  }

  $scriptDir = $PSScriptRoot
  return (Resolve-Path -LiteralPath (Join-Path $scriptDir "..\..")).Path
}

function Invoke-Git {
  param([string[]] $Arguments)

  $output = & git @Arguments
  if ($LASTEXITCODE -ne 0) {
    throw "git $($Arguments -join ' ') failed with exit code $LASTEXITCODE"
  }

  return @($output)
}

function Test-GitTagExists {
  param([string] $Name)

  if ([string]::IsNullOrWhiteSpace($Name)) {
    return $false
  }

  & git rev-parse --verify --quiet "refs/tags/$Name^{commit}" *> $null
  return ($LASTEXITCODE -eq 0)
}

function Normalize-RepoPath {
  param([string] $Path)

  $normalized = $Path -replace "\\", "/"
  while ($normalized.StartsWith("./", [System.StringComparison]::Ordinal)) {
    $normalized = $normalized.Substring(2)
  }
  return $normalized
}

function Test-PathMatchesPattern {
  param(
    [string] $Path,
    [string] $Pattern
  )

  $repoPath = Normalize-RepoPath $Path
  $repoPattern = Normalize-RepoPath $Pattern

  if ($repoPattern.EndsWith("/", [System.StringComparison]::Ordinal)) {
    return $repoPath.StartsWith($repoPattern, [System.StringComparison]::OrdinalIgnoreCase)
  }

  return [string]::Equals($repoPath, $repoPattern, [System.StringComparison]::OrdinalIgnoreCase)
}

function Test-AnyPathMatches {
  param(
    [string[]] $Files,
    [string[]] $Patterns
  )

  foreach ($file in $Files) {
    foreach ($pattern in $Patterns) {
      if (Test-PathMatchesPattern -Path $file -Pattern $pattern) {
        return $true
      }
    }
  }

  return $false
}

function New-ModuleDefinition {
  param(
    [string] $Feature,
    [string] $Slug,
    [string] $DisplayName,
    [string] $Repo,
    [string[]] $Paths
  )

  return [pscustomobject]@{
    Feature = $Feature
    Slug = $Slug
    DisplayName = $DisplayName
    Repo = $Repo
    Paths = $Paths
  }
}

function Get-ModuleDefinitions {
  return @(
    (New-ModuleDefinition -Feature "Calibration" -Slug "calibration" -DisplayName "Space Calibrator" -Repo "RealWhyKnot/WKOpenVR-SpaceCalibrator" -Paths @("modules/calibration/")),
    (New-ModuleDefinition -Feature "Smoothing" -Slug "smoothing" -DisplayName "Smoothing" -Repo "RealWhyKnot/WKOpenVR-Smoothing" -Paths @("modules/smoothing/")),
    (New-ModuleDefinition -Feature "InputHealth" -Slug "inputhealth" -DisplayName "Input Health" -Repo "RealWhyKnot/WKOpenVR-InputHealth" -Paths @("modules/inputhealth/")),
    (New-ModuleDefinition -Feature "FaceTracking" -Slug "facetracking" -DisplayName "Face Tracking" -Repo "RealWhyKnot/WKOpenVR-FaceTracking" -Paths @("modules/facetracking/")),
    (New-ModuleDefinition -Feature "OSCRouter" -Slug "oscrouter" -DisplayName "OSC Router" -Repo "RealWhyKnot/WKOpenVR-OSCRouter" -Paths @("modules/oscrouter/")),
    (New-ModuleDefinition -Feature "QuestApp" -Slug "questapp" -DisplayName "Quest App" -Repo "RealWhyKnot/WKOpenVR-QuestApp" -Paths @("modules/questapp/")),
    (New-ModuleDefinition -Feature "Captions" -Slug "captions" -DisplayName "Captions" -Repo "RealWhyKnot/WKOpenVR-Captions" -Paths @("modules/captions/"))
  )
}

function Get-SharedPatterns {
  return @(
    ".github/scripts/",
    ".github/workflows/release.yml",
    ".github/workflows/nightly-beta.yml",
    ".github/release-template/",
    "build.ps1",
    "cmake/",
    "CMakeLists.txt",
    "core/",
    "install/",
    "lib/",
    "LICENSE",
    "README.md",
    "version.txt"
  )
}

function Read-ReleaseState {
  param([string] $Path)

  if ([string]::IsNullOrWhiteSpace($Path)) {
    return $null
  }

  $resolved = Resolve-Path -LiteralPath $Path
  $json = Get-Content -LiteralPath $resolved.Path -Raw
  if ([string]::IsNullOrWhiteSpace($json)) {
    return $null
  }

  return ($json | ConvertFrom-Json)
}

function Get-ReleaseStateTag {
  param(
    [object] $ReleaseState,
    [string] $Repo
  )

  if ($null -eq $ReleaseState) {
    return ""
  }

  $property = $ReleaseState.PSObject.Properties[$Repo]
  if ($null -eq $property -or $null -eq $property.Value) {
    return ""
  }

  return [string] $property.Value
}

function Get-LatestModuleReleaseTag {
  param(
    [object] $ReleaseState,
    [string] $Repo,
    [string] $ExcludeTag
  )

  $stateTag = Get-ReleaseStateTag -ReleaseState $ReleaseState -Repo $Repo
  if (-not [string]::IsNullOrWhiteSpace($stateTag)) {
    if ($stateTag -ne $ExcludeTag) {
      return $stateTag
    }

    return ""
  }

  $json = & gh api "repos/$Repo/releases?per_page=50"
  if ($LASTEXITCODE -ne 0) {
    throw "Failed to read releases for $Repo"
  }

  $releases = @($json | ConvertFrom-Json)
  foreach ($release in $releases) {
    $draftProperty = $release.PSObject.Properties["draft"]
    $draft = $false
    if ($null -ne $draftProperty) {
      $draft = [bool] $draftProperty.Value
    }

    $tagProperty = $release.PSObject.Properties["tag_name"]
    $releaseTag = ""
    if ($null -ne $tagProperty -and $null -ne $tagProperty.Value) {
      $releaseTag = [string] $tagProperty.Value
    }

    if (-not $draft -and -not [string]::IsNullOrWhiteSpace($releaseTag) -and $releaseTag -ne $ExcludeTag) {
      return $releaseTag
    }
  }

  return ""
}

function Get-ChangedFilesSinceTag {
  param([string] $BaseTag)

  if (-not [string]::IsNullOrWhiteSpace($BaseTag) -and (Test-GitTagExists -Name $BaseTag)) {
    return @(Invoke-Git -Arguments @("diff", "--name-only", "$BaseTag..HEAD", "--"))
  }

  return @(Invoke-Git -Arguments @("ls-files"))
}

function Get-NextBetaTag {
  param([string] $DateStamp)

  if ([string]::IsNullOrWhiteSpace($DateStamp)) {
    $DateStamp = (Get-Date).ToUniversalTime().ToString("yyyy.M.d", [System.Globalization.CultureInfo]::InvariantCulture)
  }

  $escapedDate = [regex]::Escape($DateStamp)
  $pattern = "^v$escapedDate\.(\d+)(-[A-Za-z0-9]{4})?$"
  $existingTags = @(Invoke-Git -Arguments @("tag", "--list", "v$DateStamp.*"))
  $highest = -1

  foreach ($existingTag in $existingTags) {
    if ($existingTag -match $pattern) {
      $value = [int] $Matches[1]
      if ($value -gt $highest) {
        $highest = $value
      }
    }
  }

  $next = $highest + 1
  return "v$DateStamp.$next-beta"
}

function Write-GitHubOutput {
  param(
    [string] $Name,
    [string] $Value
  )

  if ([string]::IsNullOrWhiteSpace($env:GITHUB_OUTPUT)) {
    return
  }

  Add-Content -LiteralPath $env:GITHUB_OUTPUT -Value "$Name=$Value"
}

function ConvertTo-CompactJson {
  param([object] $Value)

  return ($Value | ConvertTo-Json -Depth 8 -Compress)
}

function Write-PlanSummary {
  param(
    [object] $Plan,
    [object[]] $Modules
  )

  Write-Host "Nightly beta release plan:"
  Write-Host "  Next tag: $($Plan.next_tag)"
  Write-Host "  Changed modules: $($Plan.features_csv)"

  foreach ($module in $Modules) {
    $reason = ""
    $detail = $Plan.modules | Where-Object { $_.feature -eq $module.Feature } | Select-Object -First 1
    if ($null -ne $detail) {
      $reason = $detail.reason
    }

    if ([string]::IsNullOrWhiteSpace($reason)) {
      $reason = "unchanged"
    }

    Write-Host ("  {0}: {1}" -f $module.Feature, $reason)
  }
}

$repoRootPath = Resolve-RepoRoot -Value $RepoRoot
Push-Location $repoRootPath
try {
  if (-not [string]::IsNullOrWhiteSpace($Tag) -and $Tag -notmatch "^v\d{4}\.\d+\.\d+\.\d+-beta$") {
    throw "Nightly beta tags must match vYYYY.M.D.N-beta."
  }

  $modules = @(Get-ModuleDefinitions)
  $sharedPatterns = @(Get-SharedPatterns)
  $releaseState = Read-ReleaseState -Path $ReleaseStatePath
  $excludeTag = $Tag
  $changedFeatures = New-Object System.Collections.Generic.List[string]
  $modulePlans = New-Object System.Collections.Generic.List[object]
  $filesByBaseTag = @{}

  foreach ($module in $modules) {
    $baseTag = Get-LatestModuleReleaseTag -ReleaseState $releaseState -Repo $module.Repo -ExcludeTag $excludeTag
    if (-not $filesByBaseTag.ContainsKey($baseTag)) {
      $filesByBaseTag[$baseTag] = @(Get-ChangedFilesSinceTag -BaseTag $baseTag)
    }

    $changedFiles = @($filesByBaseTag[$baseTag])
    $baseMissing = [string]::IsNullOrWhiteSpace($baseTag) -or -not (Test-GitTagExists -Name $baseTag)
    $sharedChanged = Test-AnyPathMatches -Files $changedFiles -Patterns $sharedPatterns
    $moduleChanged = Test-AnyPathMatches -Files $changedFiles -Patterns $module.Paths
    $changed = $baseMissing -or $sharedChanged -or $moduleChanged

    $reasonParts = New-Object System.Collections.Generic.List[string]
    if ($baseMissing) {
      if ([string]::IsNullOrWhiteSpace($baseTag)) {
        [void] $reasonParts.Add("no previous module release")
      } else {
        [void] $reasonParts.Add("previous tag $baseTag is not present locally")
      }
    }
    if ($sharedChanged) {
      [void] $reasonParts.Add("shared release inputs changed")
    }
    if ($moduleChanged) {
      [void] $reasonParts.Add("module files changed")
    }
    if ($reasonParts.Count -eq 0) {
      [void] $reasonParts.Add("unchanged since $baseTag")
    }

    if ($changed) {
      [void] $changedFeatures.Add($module.Feature)
    }

    [void] $modulePlans.Add([pscustomobject]@{
      feature = $module.Feature
      slug = $module.Slug
      repo = $module.Repo
      base_tag = $baseTag
      changed = $changed
      reason = ($reasonParts -join "; ")
    })
  }

  if ([string]::IsNullOrWhiteSpace($Tag)) {
    $nextTag = Get-NextBetaTag -DateStamp $Today
  } else {
    $nextTag = $Tag
  }

  $features = @($changedFeatures.ToArray())
  $featuresCsv = ($features -join ",")
  $plan = [pscustomobject]@{
    has_changes = ($features.Count -gt 0)
    next_tag = $nextTag
    features = $features
    features_csv = $featuresCsv
    modules = @($modulePlans.ToArray())
  }

  $planJson = ConvertTo-CompactJson -Value $plan
  if (-not [string]::IsNullOrWhiteSpace($OutputJsonPath)) {
    $resolvedOutputPath = [System.IO.Path]::GetFullPath($OutputJsonPath)
    $outputParent = Split-Path -Parent $resolvedOutputPath
    if (-not (Test-Path -LiteralPath $outputParent)) {
      New-Item -ItemType Directory -Path $outputParent | Out-Null
    }
    [System.IO.File]::WriteAllText($resolvedOutputPath, $planJson, (New-Object System.Text.UTF8Encoding($false)))
  }

  Write-GitHubOutput -Name "has_changes" -Value ([string] $plan.has_changes).ToLowerInvariant()
  Write-GitHubOutput -Name "next_tag" -Value $plan.next_tag
  Write-GitHubOutput -Name "features_csv" -Value $plan.features_csv
  Write-GitHubOutput -Name "features_json" -Value (ConvertTo-CompactJson -Value $plan.features)

  if (-not [string]::IsNullOrWhiteSpace($env:GITHUB_STEP_SUMMARY)) {
    Add-Content -LiteralPath $env:GITHUB_STEP_SUMMARY -Value "### Nightly beta release plan"
    Add-Content -LiteralPath $env:GITHUB_STEP_SUMMARY -Value ""
    Add-Content -LiteralPath $env:GITHUB_STEP_SUMMARY -Value "- Tag: $($plan.next_tag)"
    Add-Content -LiteralPath $env:GITHUB_STEP_SUMMARY -Value "- Modules: $featuresCsv"
  }

  Write-PlanSummary -Plan $plan -Modules $modules
}
finally {
  Pop-Location
}
