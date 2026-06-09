param(
    [Parameter(Mandatory = $true)]
    [string]$PackId,

    [string]$Manifest = "",

    [switch]$Uninstall
)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

function Get-LocalAppDataLow {
    if ($env:WKOPENVR_LOCALAPPDATA_OVERRIDE) {
        return $env:WKOPENVR_LOCALAPPDATA_OVERRIDE
    }
    $shell = New-Object -ComObject Shell.Application
    $folder = $shell.Namespace("shell:Local AppDataLow")
    if ($folder -and $folder.Self -and $folder.Self.Path) {
        return $folder.Self.Path
    }
    return (Join-Path $env:USERPROFILE "AppData\LocalLow")
}

function Write-Log {
    param([string]$Message)
    $stamp = Get-Date -Format "yyyy-MM-ddTHH:mm:ss"
    $line = "[$stamp] $Message"
    Write-Host $line
    Add-Content -LiteralPath $script:LogPath -Value $line
}

function Invoke-WithRetry {
    param(
        [Parameter(Mandatory = $true)][scriptblock]$Script,
        [Parameter(Mandatory = $true)][string]$Operation,
        [int]$MaxAttempts = 4
    )

    $delay = 1
    for ($attempt = 1; $attempt -le $MaxAttempts; $attempt++) {
        try {
            return & $Script
        } catch {
            $message = $_.Exception.Message
            if ($attempt -ge $MaxAttempts) {
                throw "$Operation failed after $MaxAttempts attempts: $message"
            }
            Write-Log "$Operation failed on attempt $attempt/$MaxAttempts`: $message; retrying in ${delay}s"
            Start-Sleep -Seconds $delay
            $delay = [Math]::Min($delay * 2, 16)
        }
    }
}

function Ensure-Parent {
    param([string]$Path)
    $parent = Split-Path -Parent $Path
    if ($parent -and -not (Test-Path -LiteralPath $parent)) {
        New-Item -ItemType Directory -Force -Path $parent | Out-Null
    }
}

function Get-Sha256 {
    param([string]$Path)
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function Join-InstallPath {
    param([Parameter(Mandatory = $true)][string]$RelativePath)
    if ([System.IO.Path]::IsPathRooted($RelativePath)) {
        throw "Install path must be relative: $RelativePath"
    }

    $root = [System.IO.Path]::GetFullPath($script:InstallRoot)
    $candidate = [System.IO.Path]::GetFullPath((Join-Path $root $RelativePath))
    $trimChars = [char[]]@([System.IO.Path]::DirectorySeparatorChar, [System.IO.Path]::AltDirectorySeparatorChar)
    $rootWithSep = $root.TrimEnd($trimChars) + [System.IO.Path]::DirectorySeparatorChar
    if ($candidate -ne $root -and -not $candidate.StartsWith($rootWithSep, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Install path escapes captions root: $RelativePath"
    }
    return $candidate
}

function Write-Utf8NoBom {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Content
    )
    Ensure-Parent -Path $Path
    [System.IO.File]::WriteAllText($Path, $Content, (New-Object System.Text.UTF8Encoding($false)))
}

function Download-VerifiedFile {
    param(
        [Parameter(Mandatory = $true)][string]$Url,
        [Parameter(Mandatory = $true)][string]$Destination,
        [string]$Sha256 = ""
    )

    Ensure-Parent -Path $Destination
    if ((Test-Path -LiteralPath $Destination) -and $Sha256) {
        $existing = Get-Sha256 -Path $Destination
        if ($existing -eq $Sha256.ToLowerInvariant()) {
            Write-Log "Using cached $Destination"
            return
        }
    }

    $parent = Split-Path -Parent $Destination
    $leaf = Split-Path -Leaf $Destination
    if ($parent) {
        Get-ChildItem -LiteralPath $parent -Filter "$leaf.download*" -File -ErrorAction SilentlyContinue |
            Remove-Item -Force -ErrorAction SilentlyContinue
    }
    $tmp = Join-Path $parent ("$leaf.download.$PID.$([guid]::NewGuid().ToString('N'))")

    Write-Log "Downloading $Url"
    try {
        Invoke-WithRetry -Operation "Download $Url" -Script {
            if (Test-Path -LiteralPath $tmp) { Remove-Item -LiteralPath $tmp -Force -ErrorAction SilentlyContinue }
            Invoke-WebRequest -UseBasicParsing -Uri $Url -OutFile $tmp -TimeoutSec 120 -MaximumRedirection 10
            if (-not (Test-Path -LiteralPath $tmp)) {
                throw "download did not create $tmp"
            }
        } | Out-Null

        if ($Sha256) {
            $actual = Get-Sha256 -Path $tmp
            if ($actual -ne $Sha256.ToLowerInvariant()) {
                Remove-Item -LiteralPath $tmp -Force -ErrorAction SilentlyContinue
                throw "Hash mismatch for $Url. Expected $Sha256, got $actual"
            }
            Write-Log "Verified SHA256 $actual"
        }

        Move-Item -LiteralPath $tmp -Destination $Destination -Force
    } catch {
        if (Test-Path -LiteralPath $tmp) {
            Remove-Item -LiteralPath $tmp -Force -ErrorAction SilentlyContinue
        }
        throw
    }
}

function Copy-IfDifferent {
    param(
        [Parameter(Mandatory = $true)][string]$Source,
        [Parameter(Mandatory = $true)][string]$Destination,
        [string]$DisplayPath = ""
    )

    if (-not $DisplayPath) {
        $DisplayPath = $Destination
    }

    if (Test-Path -LiteralPath $Destination) {
        $sourceHash = Get-Sha256 -Path $Source
        $destHash = Get-Sha256 -Path $Destination
        if ($sourceHash -eq $destHash) {
            Write-Log "Already current $DisplayPath"
            return $false
        }
    }

    try {
        Copy-Item -LiteralPath $Source -Destination $Destination -Force
        return $true
    } catch {
        if (Test-Path -LiteralPath $Destination) {
            $sourceHash = Get-Sha256 -Path $Source
            $destHash = Get-Sha256 -Path $Destination
            if ($sourceHash -eq $destHash) {
                Write-Log "Already current $DisplayPath"
                return $false
            }
        }
        throw
    }
}

function Extract-ZipEntries {
    param(
        [Parameter(Mandatory = $true)][string]$ZipPath,
        [Parameter(Mandatory = $true)]$Entries
    )

    $tmpRoot = Join-Path ([IO.Path]::GetTempPath()) ("WKOpenVR-captions-" + [guid]::NewGuid().ToString("N"))
    New-Item -ItemType Directory -Force -Path $tmpRoot | Out-Null
    try {
        $archivePath = $ZipPath
        if ([IO.Path]::GetExtension($ZipPath) -ne ".zip") {
            $archivePath = Join-Path $tmpRoot "archive.zip"
            Copy-Item -LiteralPath $ZipPath -Destination $archivePath -Force
        }
        Expand-Archive -LiteralPath $archivePath -DestinationPath $tmpRoot -Force
        foreach ($entry in $Entries) {
            $from = Join-Path $tmpRoot ([string]$entry.from)
            $to = Join-InstallPath -RelativePath ([string]$entry.to)
            if (-not (Test-Path -LiteralPath $from)) {
                throw "Zip entry missing: $($entry.from)"
            }
            Ensure-Parent -Path $to
            $copied = Copy-IfDifferent -Source $from -Destination $to -DisplayPath ([string]$entry.to)
            if ($copied) {
                Write-Log "Extracted $($entry.from) -> $($entry.to)"
            }
        }
    } finally {
        Remove-Item -LiteralPath $tmpRoot -Recurse -Force -ErrorAction SilentlyContinue
    }
}

function Remove-PathIfPresent {
    param([string]$Path)
    if (Test-Path -LiteralPath $Path) {
        Remove-Item -LiteralPath $Path -Recurse -Force
        Write-Log "Removed $Path"
    }
}

function Remove-EmptyParents {
    param([string]$Path)
    $root = (Resolve-Path -LiteralPath $script:InstallRoot).Path
    $cur = Split-Path -Parent $Path
    while ($cur -and $cur.StartsWith($root, [System.StringComparison]::OrdinalIgnoreCase)) {
        if ((Test-Path -LiteralPath $cur) -and -not (Get-ChildItem -LiteralPath $cur -Force | Select-Object -First 1)) {
            Remove-Item -LiteralPath $cur -Force
            Write-Log "Removed empty folder $cur"
            $cur = Split-Path -Parent $cur
        } else {
            break
        }
    }
}

function Install-HfSnapshot {
    param($Snapshot)

    $repo = [string]$Snapshot.repo
    $revision = [string]$Snapshot.revision
    if (-not $revision) { $revision = "main" }
    $destRelRoot = ([string]$Snapshot.destination).TrimEnd([char[]]@('\','/'))
    $destRoot = Join-InstallPath -RelativePath $destRelRoot

    if (-not $repo) { throw "hf_snapshot.repo is missing" }
    if (-not $Snapshot.destination) { throw "hf_snapshot.destination is missing" }

    $apiUrl = "https://huggingface.co/api/models/$repo/revision/$revision`?blobs=true"
    Write-Log "Reading Hugging Face snapshot $repo@$revision"
    $meta = Invoke-WithRetry -Operation "Read Hugging Face snapshot $repo@$revision" -Script {
        Invoke-RestMethod -UseBasicParsing -Uri $apiUrl -TimeoutSec 120
    }
    if (-not $meta.siblings) { throw "No files found in Hugging Face snapshot $repo@$revision" }

    $tmpRelRoot = "$destRelRoot.install.$PID.$([guid]::NewGuid().ToString('N'))"
    $tmpDestRoot = Join-InstallPath -RelativePath $tmpRelRoot
    New-Item -ItemType Directory -Force -Path $tmpDestRoot | Out-Null
    try {
        foreach ($file in $meta.siblings) {
            $rel = [string]$file.rfilename
            if (-not $rel -or $rel.EndsWith("/")) { continue }
            if ($rel -eq ".gitattributes" -or $rel -like "*.md") { continue }

            $target = Join-InstallPath -RelativePath (Join-Path $tmpRelRoot $rel)
            $urlRel = [System.Uri]::EscapeDataString($rel).Replace("%2F", "/")
            $url = "https://huggingface.co/$repo/resolve/$revision/$urlRel"
            $sha = ""
            if ($file.lfs -and $file.lfs.sha256) {
                $sha = [string]$file.lfs.sha256
            }
            Download-VerifiedFile -Url $url -Destination $target -Sha256 $sha
        }

        if ($Snapshot.required_files) {
            foreach ($required in @($Snapshot.required_files)) {
                $rel = [string]$required
                if (-not $rel) { continue }
                $candidate = Join-InstallPath -RelativePath (Join-Path $tmpRelRoot $rel)
                if (-not (Test-Path -LiteralPath $candidate)) {
                    throw "Hugging Face snapshot $repo@$revision is missing required file: $rel"
                }
            }
        }

        Remove-PathIfPresent -Path $destRoot
        Move-Item -LiteralPath $tmpDestRoot -Destination $destRoot -Force
        Write-Log "Installed Hugging Face snapshot $repo@$revision -> $destRelRoot"
    } catch {
        Remove-PathIfPresent -Path $tmpDestRoot
        throw
    }
}

function Get-Pack {
    param(
        [Parameter(Mandatory = $true)][string]$Id,
        [bool]$Required = $true
    )

    $pack = @($script:ManifestObject.packs | Where-Object { $_.id -eq $Id }) | Select-Object -First 1
    if (-not $pack -and $Required) {
        throw "Pack '$Id' was not found in $Manifest"
    }
    return $pack
}

function Write-PackStamp {
    param($Pack)

    $deps = @()
    if ($Pack.dependencies) {
        $deps = @($Pack.dependencies | ForEach-Object { [string]$_ })
    }

    $stamp = [pscustomobject]@{
        id = $Pack.id
        label = $Pack.label
        dependencies = $deps
        installed_at = (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ")
    }
    $stampName = ("installed-" + ([string]$Pack.id -replace '[\\/:*?"<>|]', '_') + ".json")
    $stampPath = Join-InstallPath -RelativePath $stampName
    Write-Utf8NoBom -Path $stampPath -Content ($stamp | ConvertTo-Json -Depth 4)
}

function Install-Pack {
    param(
        [Parameter(Mandatory = $true)][string]$Id,
        [hashtable]$Visiting
    )

    if (-not $Visiting) { $Visiting = @{} }
    if ($Visiting.ContainsKey($Id)) {
        throw "Cyclic captions pack dependency at '$Id'"
    }
    $Visiting[$Id] = $true

    $pack = Get-Pack -Id $Id
    if ($pack.dependencies) {
        foreach ($dep in $pack.dependencies) {
            Install-Pack -Id ([string]$dep) -Visiting $Visiting
        }
    }

    if ($pack.files) {
        foreach ($file in $pack.files) {
            $dest = Join-InstallPath -RelativePath ([string]$file.destination)
            Download-VerifiedFile -Url ([string]$file.url) -Destination $dest -Sha256 ([string]$file.sha256)
            if ($file.extract) {
                Extract-ZipEntries -ZipPath $dest -Entries $file.extract
            }
        }
    }

    if ($pack.hf_snapshot) {
        Install-HfSnapshot -Snapshot $pack.hf_snapshot
    }

    Write-PackStamp -Pack $pack
    $Visiting.Remove($Id)
}

function Get-InstalledPackIds {
    $stamps = @(Get-ChildItem -LiteralPath $script:InstallRoot -Filter "installed-*.json" -File -ErrorAction SilentlyContinue)
    foreach ($stamp in $stamps) {
        try {
            $obj = Get-Content -LiteralPath $stamp.FullName -Raw | ConvertFrom-Json
            if ($obj.id) { [string]$obj.id }
        } catch {
            Write-Log "Ignoring unreadable stamp $($stamp.FullName): $($_.Exception.Message)"
        }
    }
}

function Test-InstalledPackDependency {
    param(
        [Parameter(Mandatory = $true)][string]$DependencyId,
        [string]$IgnorePackId = ""
    )

    foreach ($id in Get-InstalledPackIds) {
        if ($IgnorePackId -and $id -eq $IgnorePackId) { continue }
        $pack = Get-Pack -Id $id -Required $false
        if (-not $pack -or -not $pack.dependencies) { continue }
        foreach ($dep in $pack.dependencies) {
            if ([string]$dep -eq $DependencyId) { return $true }
        }
    }
    return $false
}

function Uninstall-Pack {
    param(
        [Parameter(Mandatory = $true)][string]$Id,
        [bool]$RemoveDependencies = $true
    )

    $Pack = Get-Pack -Id $Id

    if ($Pack.files) {
        foreach ($file in $Pack.files) {
            $dest = Join-InstallPath -RelativePath ([string]$file.destination)
            if ($file.extract) {
                foreach ($entry in $file.extract) {
                    $to = Join-InstallPath -RelativePath ([string]$entry.to)
                    Remove-PathIfPresent -Path $to
                    Remove-EmptyParents -Path $to
                }
            }
            Remove-PathIfPresent -Path $dest
            Remove-EmptyParents -Path $dest
        }
    }

    if ($Pack.hf_snapshot -and $Pack.hf_snapshot.destination) {
        $destRoot = Join-InstallPath -RelativePath ([string]$Pack.hf_snapshot.destination)
        Remove-PathIfPresent -Path $destRoot
        Remove-EmptyParents -Path $destRoot
    }

    $stampName = ("installed-" + ([string]$Pack.id -replace '[\\/:*?"<>|]', '_') + ".json")
    $stampPath = Join-InstallPath -RelativePath $stampName
    Remove-PathIfPresent -Path $stampPath

    if ($RemoveDependencies -and $Pack.dependencies) {
        foreach ($dep in $Pack.dependencies) {
            $depId = [string]$dep
            if (Test-InstalledPackDependency -DependencyId $depId -IgnorePackId $Id) {
                Write-Log "Keeping shared dependency '$depId'"
            } else {
                Write-Log "Removing unused dependency '$depId'"
                Uninstall-Pack -Id $depId -RemoveDependencies $true
            }
        }
    }
}

if (-not $Manifest) {
    $Manifest = Join-Path $PSScriptRoot "captions-packs.json"
}

$localLow = Get-LocalAppDataLow
$script:InstallRoot = Join-Path $localLow "WKOpenVR\captions"
$logDir = Join-Path $localLow "WKOpenVR\Logs"
New-Item -ItemType Directory -Force -Path $script:InstallRoot | Out-Null
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$script:LogPath = Join-Path $logDir "captions_pack_install.log"

$verb = "Installing"
if ($Uninstall) { $verb = "Uninstalling" }
Write-Log "$verb captions pack '$PackId'"

try {
    if (-not (Test-Path -LiteralPath $Manifest)) {
        throw "Pack manifest not found: $Manifest"
    }

    $script:ManifestObject = Get-Content -LiteralPath $Manifest -Raw | ConvertFrom-Json
    $pack = Get-Pack -Id $PackId

    if ($Uninstall) {
        Uninstall-Pack -Id $PackId
        Write-Log "Uninstalled captions pack '$PackId'"
        exit 0
    }

    Install-Pack -Id $PackId -Visiting @{}

    Write-Log "Installed captions pack '$PackId'"
} catch {
    Write-Log "ERROR: $($_.Exception.Message)"
    throw
}
