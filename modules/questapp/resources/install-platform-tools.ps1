[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string] $QuestAppRoot
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'

$platformToolsUrl = 'https://dl.google.com/android/repository/platform-tools-latest-windows.zip'
$zipPath = Join-Path $QuestAppRoot 'platform-tools.zip'
$extractRoot = Join-Path $QuestAppRoot 'platform-tools.extract'
$finalRoot = Join-Path $QuestAppRoot 'platform-tools'

New-Item -ItemType Directory -Force -Path $QuestAppRoot | Out-Null
Remove-Item -LiteralPath $zipPath -Force -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $extractRoot -Recurse -Force -ErrorAction SilentlyContinue

Invoke-WebRequest -UseBasicParsing -Uri $platformToolsUrl -OutFile $zipPath
Expand-Archive -LiteralPath $zipPath -DestinationPath $extractRoot -Force

$inner = Join-Path $extractRoot 'platform-tools'
$adb = Join-Path $inner 'adb.exe'
if (-not (Test-Path -LiteralPath $adb)) {
    throw 'Downloaded platform-tools archive did not contain adb.exe.'
}

$backup = $finalRoot + '.old'
Remove-Item -LiteralPath $backup -Recurse -Force -ErrorAction SilentlyContinue
if (Test-Path -LiteralPath $finalRoot) {
    Move-Item -LiteralPath $finalRoot -Destination $backup -Force
}
Move-Item -LiteralPath $inner -Destination $finalRoot -Force

Remove-Item -LiteralPath $zipPath -Force -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $extractRoot -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $backup -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "Installed platform-tools to $finalRoot"
