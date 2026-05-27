$ErrorActionPreference = 'SilentlyContinue'

function Get-LocalLowPath {
    $roaming = [System.Environment]::GetFolderPath('ApplicationData')
    if ($roaming -and $roaming.EndsWith('Roaming')) {
        return ($roaming.Substring(0, $roaming.Length - 'Roaming'.Length) + 'LocalLow')
    }
    if ($env:USERPROFILE) {
        return (Join-Path $env:USERPROFILE 'AppData\LocalLow')
    }
    return $null
}

function Invoke-WithTimeout {
    param(
        [Parameter(Mandatory=$true)][string]$FilePath,
        [Parameter(Mandatory=$true)][string[]]$ArgumentList,
        [int]$TimeoutMs = 10000
    )

    function ConvertTo-QuotedArgument {
        param([string]$Value)
        return '"' + ($Value -replace '"', '\"') + '"'
    }

    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $FilePath
    $psi.Arguments = (($ArgumentList | ForEach-Object { ConvertTo-QuotedArgument $_ }) -join ' ')
    $psi.UseShellExecute = $false
    $psi.CreateNoWindow = $true

    $proc = [System.Diagnostics.Process]::Start($psi)
    if (-not $proc) { return 1 }
    if (-not $proc.WaitForExit($TimeoutMs)) {
        try { $proc.Kill() } catch { }
        return 124
    }
    return $proc.ExitCode
}

$localLow = Get-LocalLowPath
if ($localLow) {
    $wkRoot = Join-Path $localLow 'WKOpenVR'
    $adb = Join-Path $wkRoot 'questapp\platform-tools\adb.exe'
    if (Test-Path -LiteralPath $adb) {
        $devicesOutput = & $adb devices 2>$null
        foreach ($line in $devicesOutput) {
            if ($line -match '^\s*(\S+)\s+device\s*$') {
                $serial = $Matches[1]
                [void](Invoke-WithTimeout -FilePath $adb -ArgumentList @('-s', $serial, 'shell', 'am', 'force-stop', 'org.wkopenvr.quest') -TimeoutMs 5000)
                [void](Invoke-WithTimeout -FilePath $adb -ArgumentList @('-s', $serial, 'uninstall', 'org.wkopenvr.quest') -TimeoutMs 15000)
            }
        }
    }

    $questDir = Join-Path $wkRoot 'questapp'
    if (Test-Path -LiteralPath $questDir) {
        Remove-Item -LiteralPath $questDir -Recurse -Force
    }

    $questProfile = Join-Path $wkRoot 'profiles\questapp.txt'
    if (Test-Path -LiteralPath $questProfile) {
        Remove-Item -LiteralPath $questProfile -Force
    }
}
