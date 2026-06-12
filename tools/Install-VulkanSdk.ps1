#Requires -Version 5.1
<#
.SYNOPSIS
    Ensure the Vulkan SDK (headers + glslc) is available for building the captions
    speech host with the Vulkan GPU backend (-DWKOPENVR_CAPTIONS_VULKAN=ON).

.DESCRIPTION
    Detects an existing Vulkan SDK. If none is found it prompts (or, with
    -AutoInstall, proceeds without asking) to download and silently install the
    pinned LunarG SDK. On success it sets VULKAN_SDK and prepends the SDK's Bin
    directory to PATH for the current process, so a subsequent CMake configure
    resolves find_package(Vulkan COMPONENTS glslc).

    The SDK is a build-time dependency only -- the captions host links the Vulkan
    loader statically and uses the system vulkan-1.dll at runtime.

    Designed to be invoked in-process (via the call operator) by build.ps1 and the
    release workflow so the VULKAN_SDK environment change is visible to the caller.
    It never calls 'exit'; it returns normally on success and throws on failure.

.PARAMETER Version
    Pinned LunarG SDK version. Kept here as the single source of truth for both
    local and CI builds.

.PARAMETER Root
    Install directory. Defaults to C:\VulkanSDK\<Version>. The LunarG installer
    requires administrator rights, so the install is launched elevated (a UAC
    prompt appears). On an already-elevated shell (for example CI) no prompt is
    shown.

.PARAMETER AutoInstall
    Install without prompting for confirmation. The UAC elevation prompt may
    still appear unless the shell is already elevated. Used by CI and other
    non-interactive callers.
#>
param(
    [string]$Version = "1.3.296.0",
    [string]$Root = "",
    [switch]$AutoInstall
)

$ErrorActionPreference = "Stop"

function Resolve-VulkanSdkRoot {
    param([string]$PreferredRoot)

    # 1) Explicit env var with a usable glslc.
    if ($env:VULKAN_SDK -and (Test-Path (Join-Path $env:VULKAN_SDK "Bin\glslc.exe"))) {
        return $env:VULKAN_SDK
    }
    # 2) A specific root we just installed into.
    if ($PreferredRoot -and (Test-Path (Join-Path $PreferredRoot "Bin\glslc.exe"))) {
        return $PreferredRoot
    }
    # 3) glslc already on PATH -> the SDK root is Bin's parent.
    $glslc = Get-Command glslc.exe -ErrorAction SilentlyContinue
    if ($glslc) {
        return (Split-Path -Parent (Split-Path -Parent $glslc.Source))
    }
    # 4) Newest C:\VulkanSDK\* that has a glslc.
    $standardRoot = Join-Path $env:SystemDrive "VulkanSDK"
    if (Test-Path $standardRoot) {
        $candidate = Get-ChildItem -Path $standardRoot -Directory -ErrorAction SilentlyContinue |
            Where-Object { Test-Path (Join-Path $_.FullName "Bin\glslc.exe") } |
            Sort-Object Name -Descending | Select-Object -First 1
        if ($candidate) { return $candidate.FullName }
    }
    return $null
}

function Set-VulkanSdkEnv {
    param([Parameter(Mandatory = $true)][string]$SdkRoot)

    $env:VULKAN_SDK = $SdkRoot
    $bin = Join-Path $SdkRoot "Bin"
    $parts = $env:PATH -split ";"
    if ($parts -notcontains $bin) {
        $env:PATH = "$bin;$env:PATH"
    }
}

$existing = Resolve-VulkanSdkRoot -PreferredRoot $Root
if ($existing) {
    Set-VulkanSdkEnv -SdkRoot $existing
    Write-Host "Vulkan SDK found: $existing" -ForegroundColor Green
    return
}

# Not installed -- decide whether to install.
$proceed = $false
if ($AutoInstall) {
    $proceed = $true
}
else {
    Write-Host ""
    Write-Host "The Vulkan SDK (headers + glslc shader compiler) is required to build the" -ForegroundColor Yellow
    Write-Host "captions speech host with GPU acceleration (-DWKOPENVR_CAPTIONS_VULKAN=ON)." -ForegroundColor Yellow
    Write-Host "It is a build-time dependency only; the runtime uses the system vulkan-1.dll." -ForegroundColor Yellow
    try {
        $reply = Read-Host "Download and install Vulkan SDK $Version now? [y/N]"
    }
    catch {
        throw "Vulkan SDK is required but not installed, and this session is non-interactive. Re-run with -InstallVulkanSdk (build.ps1/quick.ps1) or call tools\Install-VulkanSdk.ps1 -AutoInstall, or install it from https://vulkan.lunarg.com and re-run."
    }
    if ($reply -match "^(y|yes)$") { $proceed = $true }
}

if (-not $proceed) {
    throw "Vulkan SDK install declined. Install it from https://vulkan.lunarg.com, or build without -CaptionsVulkan for a CPU-only captions host."
}

$effectiveRoot = $Root
if (-not $effectiveRoot) {
    $effectiveRoot = Join-Path (Join-Path $env:SystemDrive "VulkanSDK") $Version
}

$installer = Join-Path $env:TEMP "VulkanSDK-$Version-Installer.exe"
$url = "https://sdk.lunarg.com/sdk/download/$Version/windows/VulkanSDK-$Version-Installer.exe"

if ((Test-Path $installer) -and ((Get-Item $installer).Length -gt 50MB)) {
    Write-Host "Using cached Vulkan SDK installer: $installer" -ForegroundColor Cyan
}
else {
    Write-Host "Downloading Vulkan SDK $Version ..." -ForegroundColor Cyan
    $oldProgress = $ProgressPreference
    $ProgressPreference = "SilentlyContinue"  # large file: skip the per-byte progress UI
    try {
        Invoke-WebRequest -Uri $url -OutFile $installer -UseBasicParsing
    }
    finally {
        $ProgressPreference = $oldProgress
    }
}

# The LunarG installer needs administrator rights. When this shell is already
# elevated (for example a CI runner) install directly; otherwise relaunch the
# installer elevated so a UAC prompt appears.
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)
$isElevated = $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)

$installArgs = @("--root", $effectiveRoot, "--accept-licenses", "--default-answer", "--confirm-command", "install")
try {
    if ($isElevated) {
        Write-Host "Installing Vulkan SDK to $effectiveRoot ..." -ForegroundColor Cyan
        $proc = Start-Process -FilePath $installer -ArgumentList $installArgs -Wait -PassThru
    }
    else {
        Write-Host "Installing Vulkan SDK to $effectiveRoot (accept the UAC prompt) ..." -ForegroundColor Cyan
        $proc = Start-Process -FilePath $installer -ArgumentList $installArgs -Verb RunAs -Wait -PassThru
    }
}
catch {
    throw "Could not start the Vulkan SDK installer: $($_.Exception.Message). Accept the UAC prompt if shown, or install manually from https://vulkan.lunarg.com."
}
if ($proc.ExitCode -ne 0) {
    throw "Vulkan SDK installer exited with code $($proc.ExitCode)."
}

$resolved = Resolve-VulkanSdkRoot -PreferredRoot $effectiveRoot
if (-not $resolved) {
    throw "Vulkan SDK install completed but glslc was not found under $effectiveRoot. The install may have failed or used a different path."
}

Set-VulkanSdkEnv -SdkRoot $resolved
Write-Host "Vulkan SDK installed: $resolved" -ForegroundColor Green
