#!/usr/bin/env pwsh
<#
.SYNOPSIS
  Installs and uninstalls the built WKOpenVR installer twice.

.DESCRIPTION
  Creates a temporary SteamVR runtime, points openvrpaths.vrpath at it,
  runs the installer silently into a temporary install directory, verifies
  the installed files, runs the uninstaller, and verifies cleanup. The pass
  is repeated so release builds exercise both fresh install and reinstall.

  This test intentionally lets the uninstaller remove the current user's
  WKOpenVR LocalLow data, matching normal product uninstall behavior. Pass
  -AllowUserDataRemoval only on disposable CI runners or after making a
  deliberate local cleanup decision.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [string] $InstallerPath,

    [ValidateRange(1, 5)]
    [int] $Passes = 2,

    [ValidateSet('Current', 'Core')]
    [string] $Payload = 'Current',

    [string] $WorkRoot,

    [switch] $AllowUserDataRemoval
)

$ErrorActionPreference = 'Stop'

function Assert-Administrator {
    $identity = [System.Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object System.Security.Principal.WindowsPrincipal($identity)
    if (-not $principal.IsInRole([System.Security.Principal.WindowsBuiltinRole]::Administrator)) {
        throw "Installer round-trip test must run from an elevated PowerShell process."
    }
}

function Assert-PathExists {
    param([Parameter(Mandatory=$true)][string] $Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Expected path missing: $Path"
    }
}

function Assert-PathMissing {
    param([Parameter(Mandatory=$true)][string] $Path)
    if (Test-Path -LiteralPath $Path) {
        throw "Expected path to be removed: $Path"
    }
}

function Test-AnyRegistryPath {
    param([Parameter(Mandatory=$true)][string[]] $Paths)
    foreach ($path in $Paths) {
        if (Test-Path -LiteralPath $path) { return $true }
    }
    return $false
}

function Assert-NoBlockerProcesses {
    $names = @('steam', 'steamwebhelper', 'vrserver', 'vrmonitor', 'vrcompositor', 'vrdashboard', 'WKOpenVR')
    $running = @()
    foreach ($name in $names) {
        $proc = Get-Process -Name $name -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($proc) { $running += $name }
    }
    if ($running.Count -gt 0) {
        throw "Close these processes before running the installer round-trip test: $($running -join ', ')"
    }
}

function Invoke-CheckedProcess {
    param(
        [Parameter(Mandatory=$true)][string] $FilePath,
        [Parameter(Mandatory=$true)][string] $ArgumentList,
        [Parameter(Mandatory=$true)][string] $Label
    )

    Write-Host "$Label"
    $process = Start-Process -FilePath $FilePath -ArgumentList $ArgumentList -Wait -PassThru -NoNewWindow
    if ($process.ExitCode -ne 0) {
        throw "$Label failed with exit code $($process.ExitCode)."
    }
}

function Write-Utf8NoBom {
    param(
        [Parameter(Mandatory=$true)][string] $Path,
        [Parameter(Mandatory=$true)][string] $Content
    )
    $encoding = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($Path, $Content, $encoding)
}

function Get-LocalLowRoot {
    $roaming = [System.Environment]::GetFolderPath('ApplicationData')
    if ($roaming -and $roaming.EndsWith('Roaming')) {
        return ($roaming.Substring(0, $roaming.Length - 'Roaming'.Length) + 'LocalLow')
    }
    if ($env:USERPROFILE) {
        return (Join-Path $env:USERPROFILE 'AppData\LocalLow')
    }
    throw "Could not resolve LocalLow."
}

function Get-RegistryPaths {
    return @{
        App = @(
            'HKLM:\Software\WKOpenVR',
            'HKLM:\Software\WOW6432Node\WKOpenVR'
        )
        Uninstall = @(
            'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\WKOpenVR',
            'HKLM:\Software\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\WKOpenVR'
        )
        User = @(
            'HKCU:\Software\WKOpenVR-SpaceCalibrator',
            'HKCU:\Software\OpenVR-WKSpaceCalibrator'
        )
    }
}

function Assert-UnderRoot {
    param(
        [Parameter(Mandatory=$true)][string] $Path,
        [Parameter(Mandatory=$true)][string] $Root
    )
    $fullPath = [System.IO.Path]::GetFullPath($Path).TrimEnd('\')
    $fullRoot = [System.IO.Path]::GetFullPath($Root).TrimEnd('\')
    if (-not $fullPath.StartsWith($fullRoot + '\', [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Path is outside the allowed temporary root: $fullPath"
    }
}

function New-OpenVrPathFile {
    param(
        [Parameter(Mandatory=$true)][string] $Path,
        [Parameter(Mandatory=$true)][string] $RuntimePath
    )

    $content = [ordered]@{
        config = @()
        external_drivers = @()
        jsonid = 'vrpathreg'
        log = @()
        runtime = @($RuntimePath)
        version = 1
    } | ConvertTo-Json -Depth 4

    $parent = Split-Path -Parent $Path
    New-Item -ItemType Directory -Force -Path $parent | Out-Null
    Write-Utf8NoBom -Path $Path -Content $content
}

function Add-LocalLowMarkers {
    param([Parameter(Mandatory=$true)][string] $Root)

    New-Item -ItemType Directory -Force -Path (Join-Path $Root 'questapp\platform-tools') | Out-Null
    New-Item -ItemType Directory -Force -Path (Join-Path $Root 'profiles') | Out-Null
    New-Item -ItemType Directory -Force -Path (Join-Path $Root 'logs') | Out-Null
    Write-Utf8NoBom -Path (Join-Path $Root 'questapp\platform-tools\roundtrip.marker') -Content 'roundtrip'
    Write-Utf8NoBom -Path (Join-Path $Root 'profiles\questapp.txt') -Content 'roundtrip'
    Write-Utf8NoBom -Path (Join-Path $Root 'logs\roundtrip.log') -Content 'roundtrip'
}

function Assert-InstalledState {
    param(
        [Parameter(Mandatory=$true)][string] $InstallDir,
        [Parameter(Mandatory=$true)][string] $RuntimePath,
        [Parameter(Mandatory=$true)][string] $Payload
    )

    $driverDir = Join-Path $RuntimePath 'drivers\01wkopenvr'
    $expected = @(
        (Join-Path $InstallDir 'WKOpenVR.exe'),
        (Join-Path $InstallDir 'openvr_api.dll'),
        (Join-Path $InstallDir 'manifest.vrmanifest'),
        (Join-Path $InstallDir 'dashboard_icon.png'),
        (Join-Path $InstallDir 'resources\face-module-sync.ps1'),
        (Join-Path $InstallDir 'Uninstall.exe'),
        (Join-Path $driverDir 'driver.vrdrivermanifest'),
        (Join-Path $driverDir 'resources\driver.vrresources'),
        (Join-Path $driverDir 'resources\settings\default.vrsettings'),
        (Join-Path $driverDir 'bin\win64\driver_01wkopenvr.dll')
    )

    if ($Payload -eq 'Current') {
        $expected += @(
            (Join-Path $InstallDir 'resources\questapp\install-platform-tools.ps1'),
            (Join-Path $InstallDir 'resources\questapp\uninstall-questapp.ps1'),
            (Join-Path $InstallDir 'resources\questapp\WKOpenVRQuestCompanion.apk')
        )
    }

    foreach ($path in $expected) { Assert-PathExists -Path $path }
    if ($Payload -eq 'Current') {
        Assert-PathMissing -Path (Join-Path $InstallDir 'bin\adb\adb.exe')
    }

    $shortcutPath = Join-Path ([System.Environment]::GetFolderPath('CommonPrograms')) 'WKOpenVR\WKOpenVR.lnk'
    Assert-PathExists -Path $shortcutPath
    $shortcut = (New-Object -ComObject WScript.Shell).CreateShortcut($shortcutPath)
    $expectedShortcutTarget = Join-Path $InstallDir 'WKOpenVR.exe'
    if (-not [System.String]::Equals($shortcut.TargetPath, $expectedShortcutTarget, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Installed shortcut target mismatch. Expected '$expectedShortcutTarget', got '$($shortcut.TargetPath)'."
    }
    if ($shortcut.Arguments -ne '--launch=umbrella') {
        throw "Installed shortcut arguments mismatch. Expected '--launch=umbrella', got '$($shortcut.Arguments)'."
    }

    $registry = Get-RegistryPaths
    if (-not (Test-AnyRegistryPath -Paths $registry.App)) {
        throw "Installed app registry key was not written."
    }
    if (-not (Test-AnyRegistryPath -Paths $registry.Uninstall)) {
        throw "Installed uninstall registry key was not written."
    }
}

function Assert-UninstalledState {
    param(
        [Parameter(Mandatory=$true)][string] $InstallDir,
        [Parameter(Mandatory=$true)][string] $RuntimePath,
        [Parameter(Mandatory=$true)][string] $LocalLowWkRoot
    )

    Assert-PathMissing -Path $InstallDir
    Assert-PathMissing -Path (Join-Path $RuntimePath 'drivers\01wkopenvr')
    Assert-PathMissing -Path $LocalLowWkRoot
    Assert-PathMissing -Path (Join-Path ([System.Environment]::GetFolderPath('CommonPrograms')) 'WKOpenVR')

    $registry = Get-RegistryPaths
    foreach ($path in ($registry.App + $registry.Uninstall + $registry.User)) {
        Assert-PathMissing -Path $path
    }
}

if (-not $AllowUserDataRemoval) {
    throw "Pass -AllowUserDataRemoval to acknowledge that this test exercises product uninstall cleanup under LocalLow."
}

Assert-Administrator
Assert-NoBlockerProcesses

$installerItem = Get-Item -LiteralPath $InstallerPath
$installerFullPath = $installerItem.FullName

$baseTemp = $env:RUNNER_TEMP
if (-not $baseTemp) { $baseTemp = [System.IO.Path]::GetTempPath() }
if (-not $WorkRoot) {
    $WorkRoot = Join-Path $baseTemp ('WKOpenVRInstallerRoundTrip-' + [Guid]::NewGuid().ToString('N'))
}
$WorkRoot = [System.IO.Path]::GetFullPath($WorkRoot)
Assert-UnderRoot -Path $WorkRoot -Root $baseTemp

$installDir = Join-Path $WorkRoot 'InstallRoot\WKOpenVR'
$runtimeDir = Join-Path $WorkRoot 'SteamVR'
$localLowWkRoot = Join-Path (Get-LocalLowRoot) 'WKOpenVR'
$localAppData = [System.Environment]::GetFolderPath('LocalApplicationData')
$openVrDir = Join-Path $localAppData 'openvr'
$openVrPathFile = Join-Path $openVrDir 'openvrpaths.vrpath'
$openVrBackup = Join-Path $WorkRoot 'openvrpaths.vrpath.backup'
$hadOpenVrPathFile = Test-Path -LiteralPath $openVrPathFile

New-Item -ItemType Directory -Force -Path $WorkRoot | Out-Null
New-Item -ItemType Directory -Force -Path $runtimeDir | Out-Null
if ($hadOpenVrPathFile) {
    Copy-Item -LiteralPath $openVrPathFile -Destination $openVrBackup -Force
}

try {
    New-OpenVrPathFile -Path $openVrPathFile -RuntimePath $runtimeDir

    for ($pass = 1; $pass -le $Passes; $pass++) {
        Write-Host "Installer round-trip pass $pass of $Passes"

        if (Test-Path -LiteralPath $installDir) {
            Remove-Item -LiteralPath $installDir -Recurse -Force
        }
        $driverDir = Join-Path $runtimeDir 'drivers\01wkopenvr'
        if (Test-Path -LiteralPath $driverDir) {
            Remove-Item -LiteralPath $driverDir -Recurse -Force
        }

        Invoke-CheckedProcess -FilePath $installerFullPath -ArgumentList "/S /D=$installDir" -Label "Installing WKOpenVR"
        Assert-InstalledState -InstallDir $installDir -RuntimePath $runtimeDir -Payload $Payload

        Add-LocalLowMarkers -Root $localLowWkRoot

        $uninstaller = Join-Path $installDir 'Uninstall.exe'
        Invoke-CheckedProcess -FilePath $uninstaller -ArgumentList '/S' -Label "Uninstalling WKOpenVR"
        Assert-UninstalledState -InstallDir $installDir -RuntimePath $runtimeDir -LocalLowWkRoot $localLowWkRoot
    }
}
finally {
    if ($hadOpenVrPathFile) {
        New-Item -ItemType Directory -Force -Path $openVrDir | Out-Null
        Copy-Item -LiteralPath $openVrBackup -Destination $openVrPathFile -Force
    } else {
        if (Test-Path -LiteralPath $openVrPathFile) {
            Remove-Item -LiteralPath $openVrPathFile -Force
        }
        if ((Test-Path -LiteralPath $openVrDir) -and -not (Get-ChildItem -LiteralPath $openVrDir -Force -ErrorAction SilentlyContinue)) {
            Remove-Item -LiteralPath $openVrDir -Force
        }
    }

    if (Test-Path -LiteralPath $WorkRoot) {
        Remove-Item -LiteralPath $WorkRoot -Recurse -Force
    }
}

Write-Host "Installer round-trip test passed."
