#Requires -Version 5.1
# face-module-sync.ps1 -- list / install / update / remove face-tracking modules
# from registry, folder, or GitHub sources.  Runs without elevation; all target
# directories are under %LocalAppDataLow%.
#
# Parameters:
#   -Action     add | update | install | remove
#   -Kind       registry | folder | github (required for add/update/install)
#   -SourceData '<JSON string>'          (required for add/update/install; source descriptor)
#   -SourceId   '<hex id>'               (required for remove and update)
#   -ResultPath '<file path>'            (required; result JSON is written here)
#
# Result JSON written to -ResultPath:
#   { "ok": true|false, "message": "...",
#     "installed_uuid": "...", "installed_version": "..." }

[CmdletBinding()]
param(
    [Parameter(Mandatory)][string] $Action,
    [string] $Kind       = '',
    [string] $SourceData = '',
    [string] $SourceId   = '',
    [Parameter(Mandatory)][string] $ResultPath,

    # Optional override for the public registry URL. Used by the local test
    # harness to point the registry-kind branch at a 127.0.0.1 fixture
    # server. When unset the SourceData JSON's "url" field controls the
    # registry, matching production behavior.
    [string] $RegistryUrlOverride = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# ---- helpers ---------------------------------------------------------------

$script:ResultSourceId = $SourceId

function Write-Result([bool]$ok, [string]$msg, [string]$uuid = '', [string]$ver = '', [int]$availableCount = -1) {
    $obj = [ordered]@{
        ok                = $ok
        message           = $msg
        installed_uuid    = $uuid
        installed_version = $ver
        source_id         = $script:ResultSourceId
        action            = $Action
    }
    if ($availableCount -ge 0) {
        $obj.available_count = $availableCount
    }
    $json = $obj | ConvertTo-Json -Compress
    # UTF-8 without BOM. The static [System.Text.Encoding]::UTF8 has
    # emitUTF8Identifier=true (writes a BOM), which prefixes the file with
    # 0xEF 0xBB 0xBF -- the overlay's picojson parser rejects the BOM and
    # reports "Result JSON parse error" even though the JSON itself is fine.
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($ResultPath, $json, $utf8NoBom)
}

function Get-FtModulesDir {
    $dir = Join-Path (Get-FtDataDir) 'modules'
    if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    return $dir
}

function Get-FtDataDir {
    if ($env:WKOPENVR_LOCALAPPDATA_OVERRIDE) {
        # Test harness redirect: route every module install under a sandbox
        # directory so the real %LocalAppDataLow%\WKOpenVR tree is never
        # touched. The harness sets this env var before running scenarios.
        $low = $env:WKOPENVR_LOCALAPPDATA_OVERRIDE
    } else {
        $base = [System.Environment]::GetFolderPath('LocalApplicationData')
        # LocalApplicationData is %AppData%\..\..\LocalLow on some systems; use
        # the registry key to get the real LocalAppDataLow path.
        $low = [System.Environment]::GetFolderPath('ApplicationData') -replace 'Roaming$','LocalLow'
    }
    $dir = Join-Path $low 'WKOpenVR\facetracking'
    if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    return $dir
}

function Get-FtAvailableDir {
    $dir = Join-Path (Get-FtDataDir) 'available'
    if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    return $dir
}

function Write-JsonNoBom([string]$path, $data, [int]$depth = 12) {
    $json = $data | ConvertTo-Json -Depth $depth
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($path, $json, $utf8NoBom)
}

function Get-Prop($obj, [string[]]$names, [string]$fallback = '') {
    if ($null -eq $obj) { return $fallback }
    foreach ($name in $names) {
        $prop = $obj.PSObject.Properties[$name]
        if ($null -ne $prop -and $null -ne $prop.Value) {
            $value = [string]$prop.Value
            if (-not [string]::IsNullOrWhiteSpace($value)) { return $value }
        }
    }
    return $fallback
}

function Get-BoolProp($obj, [string[]]$names, [bool]$fallback = $false) {
    if ($null -eq $obj) { return $fallback }
    foreach ($name in $names) {
        $prop = $obj.PSObject.Properties[$name]
        if ($null -eq $prop -or $null -eq $prop.Value) { continue }
        if ($prop.Value -is [bool]) { return [bool]$prop.Value }
        $value = ([string]$prop.Value).Trim().ToLowerInvariant()
        if ($value -in @('true', '1', 'yes')) { return $true }
        if ($value -in @('false', '0', 'no')) { return $false }
    }
    return $fallback
}

function Test-IsPrereleaseChannel([string]$channel) {
    if ([string]::IsNullOrWhiteSpace($channel)) { return $false }
    $normalized = $channel.Trim().ToLowerInvariant()
    return $normalized -in @('alpha', 'beta', 'nightly', 'preview', 'prerelease', 'pre-release')
}

function Read-Manifest([string]$folder) {
    $path = Join-Path $folder 'manifest.json'
    if (-not (Test-Path $path)) { return $null }
    return Get-Content $path -Raw -Encoding UTF8 | ConvertFrom-Json
}

function Write-SourceJson([string]$destDir, [hashtable]$data) {
    $json = $data | ConvertTo-Json -Compress
    # Same BOM-avoidance dance as Write-Result -- the overlay's picojson
    # reader rejects files that start with the UTF-8 BOM (EF BB BF), so
    # source.json files written with the static UTF8 encoder ended up
    # silently empty when parsed and SourceLabel reported "Unknown" for
    # every installed module.
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText((Join-Path $destDir 'source.json'),
                                    $json, $utf8NoBom)
}

function Copy-ModuleFolder([string]$srcDir, [string]$uuid, [string]$version,
                           [hashtable]$sourceInfo) {
    $modsDir = Get-FtModulesDir
    $destDir = Join-Path $modsDir "$uuid\$version"
    if (-not (Test-Path $destDir)) { New-Item -ItemType Directory -Path $destDir -Force | Out-Null }

    # Copy everything from the source folder.
    Get-ChildItem -Path $srcDir -Recurse | ForEach-Object {
        $rel     = $_.FullName.Substring($srcDir.Length).TrimStart('\','/')
        $target  = Join-Path $destDir $rel
        if ($_.PSIsContainer) {
            if (-not (Test-Path $target)) { New-Item -ItemType Directory -Path $target -Force | Out-Null }
        } else {
            Copy-Item -Path $_.FullName -Destination $target -Force
        }
    }
    Write-SourceJson -destDir $destDir -data $sourceInfo
}

function Remove-SourceModules([string]$srcId) {
    $modsDir = Get-FtModulesDir
    if (-not (Test-Path $modsDir)) { return }
    foreach ($uuidDir in Get-ChildItem -Path $modsDir -Directory) {
        foreach ($verDir in Get-ChildItem -Path $uuidDir.FullName -Directory) {
            $sourceFile = Join-Path $verDir.FullName 'source.json'
            if (Test-Path $sourceFile) {
                $s = Get-Content $sourceFile -Raw -Encoding UTF8 | ConvertFrom-Json
                if ($s.source_id -eq $srcId) {
                    Remove-Item -Recurse -Force -Path $verDir.FullName
                }
            }
        }
        # Clean up empty uuid dir.
        $remaining = Get-ChildItem -Path $uuidDir.FullName -Directory
        if ($null -eq $remaining -or @($remaining).Count -eq 0) {
            Remove-Item -Recurse -Force -Path $uuidDir.FullName -ErrorAction SilentlyContinue
        }
    }
}

function Get-FileDigest([string]$filePath, [string]$algorithm) {
    $cmd = Get-Command -Name Get-FileHash -ErrorAction SilentlyContinue
    if ($null -ne $cmd) {
        try {
            $hash = Get-FileHash -LiteralPath $filePath -Algorithm $algorithm -ErrorAction Stop
            if ($null -ne $hash -and $null -ne $hash.Hash) {
                return $hash.Hash.ToLowerInvariant()
            }
        } catch {
            # Fall back below. Some locked-down PowerShell hosts do not expose
            # Get-FileHash even when the rest of the script can run.
        }
    }

    $stream = [System.IO.File]::OpenRead($filePath)
    $hasher = $null
    try {
        switch ($algorithm.ToUpperInvariant()) {
            'SHA256' { $hasher = [System.Security.Cryptography.SHA256]::Create(); break }
            'MD5'    { $hasher = [System.Security.Cryptography.MD5]::Create(); break }
            default  { throw "Unsupported hash algorithm: $algorithm" }
        }
        $bytes = $hasher.ComputeHash($stream)
        return ([System.BitConverter]::ToString($bytes).Replace('-', '')).ToLowerInvariant()
    } finally {
        if ($null -ne $hasher) { $hasher.Dispose() }
        $stream.Dispose()
    }
}

function Get-Sha256([string]$filePath) {
    return Get-FileDigest -filePath $filePath -algorithm 'SHA256'
}

function Get-Md5([string]$filePath) {
    return Get-FileDigest -filePath $filePath -algorithm 'MD5'
}

function Find-Sha256InText([string]$text) {
    # Match "SHA-256: <64 hex>" or "SHA256=<64 hex>" etc., case-insensitive.
    $m = [regex]::Match($text, '(?i)SHA-?256[:=]?\s*([a-f0-9]{64})')
    if ($m.Success) { return $m.Groups[1].Value.ToLower() }
    return $null
}

# ---- action: remove --------------------------------------------------------

if ($Action -eq 'remove') {
    if ([string]::IsNullOrEmpty($SourceId)) {
        Write-Result $false 'SourceId required for remove.'
        exit 1
    }
    Remove-SourceModules -srcId $SourceId
    Write-Result $true "Removed modules for source $SourceId."
    exit 0
}

# ---- parse SourceData -------------------------------------------------------

if ([string]::IsNullOrEmpty($SourceData)) {
    Write-Result $false 'SourceData required for add/update.'
    exit 1
}
try {
    $src = $SourceData | ConvertFrom-Json
} catch {
    Write-Result $false "SourceData JSON parse error: $_"
    exit 1
}

$srcId   = if ($src.PSObject.Properties['id'])         { $src.id }         else { $SourceId }
$srcKind = if ($src.PSObject.Properties['kind'])       { $src.kind }       else { $Kind }
$includePrerelease = Get-BoolProp $src @('include_prerelease','include_prereleases','allow_prerelease','allow_prereleases') $false
$script:IncludePrerelease = $includePrerelease
$script:ResultSourceId = $srcId

# ---- action: add/update (folder) -------------------------------------------

if ($srcKind -eq 'folder') {
    $folderPath = if ($src.PSObject.Properties['path']) { $src.path } else { '' }
    if ([string]::IsNullOrEmpty($folderPath) -or -not (Test-Path $folderPath)) {
        Write-Result $false "Folder not found: $folderPath"
        exit 1
    }
    $manifest = Read-Manifest -folder $folderPath
    if ($null -eq $manifest) {
        Write-Result $false "No manifest.json found in $folderPath"
        exit 1
    }
    $uuid = $manifest.uuid
    $ver  = $manifest.version
    if ([string]::IsNullOrEmpty($uuid) -or [string]::IsNullOrEmpty($ver)) {
        Write-Result $false 'manifest.json must have uuid and version fields.'
        exit 1
    }

    $info = @{
        source_id    = $srcId
        source_kind  = 'folder'
        installed_at = [DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ')
    }
    Copy-ModuleFolder -srcDir $folderPath -uuid $uuid -version $ver -sourceInfo $info
    Write-Result $true "Installed from folder." $uuid $ver
    exit 0
}

# ---- action: add/update (github) -------------------------------------------

if ($srcKind -eq 'github') {
    $ownerRepo = if ($src.PSObject.Properties['owner_repo']) { $src.owner_repo } else { '' }
    if ([string]::IsNullOrEmpty($ownerRepo)) {
        Write-Result $false 'owner_repo required for github source.'
        exit 1
    }

    $apiUrl = "https://api.github.com/repos/$ownerRepo/releases/latest"
    try {
        $release = Invoke-RestMethod -Uri $apiUrl -UseBasicParsing `
                       -Headers @{ 'User-Agent' = 'WKOpenVR/1.0' }
    } catch {
        Write-Result $false "GitHub API error for ${ownerRepo}: $_"
        exit 1
    }

    $releaseTag = $release.tag_name

    # For update: skip if tag unchanged.
    if ($Action -eq 'update') {
        $lastTag = if ($src.PSObject.Properties['last_release_tag']) { $src.last_release_tag } else { '' }
        if ($lastTag -eq $releaseTag) {
            Write-Result $true "Already up to date ($releaseTag)."
            exit 0
        }
    }

    # Find the first .zip asset.
    $asset = $release.assets | Where-Object { $_.name -like '*.zip' } | Select-Object -First 1
    if ($null -eq $asset) {
        Write-Result $false "No .zip asset found in release $releaseTag for $ownerRepo"
        exit 1
    }

    # Download the zip to a temp file.
    $tmpZip = [System.IO.Path]::GetTempFileName() + '.zip'
    try {
        Invoke-WebRequest -Uri $asset.browser_download_url -OutFile $tmpZip `
            -UseBasicParsing -Headers @{ 'User-Agent' = 'WKOpenVR/1.0' }
    } catch {
        Write-Result $false "Download failed for $($asset.browser_download_url): $_"
        exit 1
    }

    # Compute SHA-256 of downloaded zip.
    $downloadedSha = Get-Sha256 -filePath $tmpZip

    # Look for SHA-256 in release body. (PS 5.1 has no null-coalescing
    # operator, so write the fallback longhand.)
    $bodyText = if ($null -ne $release.body) { $release.body } else { '' }
    $releaseSha  = Find-Sha256InText -text $bodyText
    $shaVerified = ($null -ne $releaseSha -and $releaseSha -eq $downloadedSha)

    # Extract to temp dir.
    $tmpDir = [System.IO.Path]::Combine([System.IO.Path]::GetTempPath(),
                                         [System.IO.Path]::GetRandomFileName())
    New-Item -ItemType Directory -Path $tmpDir -Force | Out-Null
    try {
        Expand-Archive -Path $tmpZip -DestinationPath $tmpDir -Force
    } catch {
        Remove-Item -Force -Path $tmpZip -ErrorAction SilentlyContinue
        Remove-Item -Recurse -Force -Path $tmpDir -ErrorAction SilentlyContinue
        Write-Result $false "Zip extraction failed: $_"
        exit 1
    }
    Remove-Item -Force -Path $tmpZip -ErrorAction SilentlyContinue

    # Find the manifest.json (may be at root or one level deep).
    $manifestFile = Get-ChildItem -Path $tmpDir -Filter 'manifest.json' -Recurse |
                    Select-Object -First 1
    if ($null -eq $manifestFile) {
        Remove-Item -Recurse -Force -Path $tmpDir -ErrorAction SilentlyContinue
        Write-Result $false "No manifest.json found in release zip for $ownerRepo"
        exit 1
    }

    $manifest = Get-Content $manifestFile.FullName -Raw -Encoding UTF8 | ConvertFrom-Json
    $uuid = $manifest.uuid
    $ver  = $manifest.version
    if ([string]::IsNullOrEmpty($uuid) -or [string]::IsNullOrEmpty($ver)) {
        Remove-Item -Recurse -Force -Path $tmpDir -ErrorAction SilentlyContinue
        Write-Result $false 'manifest.json must have uuid and version fields.'
        exit 1
    }

    # The module root is the directory containing manifest.json.
    $moduleRoot = $manifestFile.DirectoryName

    $info = @{
        source_id        = $srcId
        source_kind      = 'github'
        release_tag      = $releaseTag
        release_sha256   = $releaseSha
        verified_sha256  = $shaVerified
        installed_at     = [DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ')
    }
    Copy-ModuleFolder -srcDir $moduleRoot -uuid $uuid -version $ver -sourceInfo $info
    Remove-Item -Recurse -Force -Path $tmpDir -ErrorAction SilentlyContinue

    Write-Result $true "Installed $ownerRepo $releaseTag (sha_verified=$shaVerified)." $uuid $ver
    exit 0
}

# ---- action: update/install (registry) -------------------------------------
#
# Registry sync is metadata-only. It writes an "available modules" cache that
# the overlay displays as explicit Install/Update rows. Downloads happen only
# through Action=install for one selected module.

if ($srcKind -eq 'registry') {
    $base = if ($src.PSObject.Properties['url']) { $src.url } else { '' }
    if ([string]::IsNullOrEmpty($base)) {
        Write-Result $false 'Registry source has no url field.'
        exit 1
    }
    $base = $base.TrimEnd('/')

    $headers = @{ 'User-Agent' = 'WKOpenVR/1.0' }

    function Invoke-RegistryJson([string[]]$uris) {
        $errors = @()
        foreach ($uri in $uris) {
            try {
                $data = Invoke-RestMethod -Uri $uri -UseBasicParsing -Headers $headers
                return [pscustomobject]@{ Data = $data; Uri = $uri }
            } catch {
                $errors += "${uri}: $($_.Exception.Message)"
            }
        }
        throw ($errors -join '; ')
    }

    function Get-RegistryIndex {
        $candidateUris = @(
            "$base/v1/index",
            "$base/modules",
            "$base/index",
            "$base/index.json"
        )
        $resp = Invoke-RegistryJson -uris $candidateUris
        $data = $resp.Data
        $entries = @()
        $shape = 'unknown'
        if ($data -is [array]) {
            $entries = @($data)
            $shape = 'vrcft-modules'
        } elseif ($null -ne $data.PSObject.Properties['modules']) {
            $entries = @($data.modules)
            $shape = 'wk-index'
        } elseif ($null -ne $data.PSObject.Properties['Modules']) {
            $entries = @($data.Modules)
            $shape = 'vrcft-modules'
        }
        return [pscustomobject]@{
            Entries = $entries
            Shape   = $shape
            Uri     = $resp.Uri
        }
    }

    function Select-VersionMetadata($entry, [string]$wantedVersion) {
        $latest = Get-Prop $entry @('latest','version','Version') ''
        if ([string]::IsNullOrEmpty($wantedVersion)) { $wantedVersion = $latest }

        $selected = $null
        $versionsProp = $entry.PSObject.Properties['versions']
        if ($null -ne $versionsProp -and $null -ne $versionsProp.Value) {
            $versions = @($versionsProp.Value)
            foreach ($v in $versions) {
                $vv = Get-Prop $v @('version','Version') ''
                if (-not [string]::IsNullOrEmpty($wantedVersion) -and $vv -eq $wantedVersion) {
                    $selected = $v
                    break
                }
            }
            if ($null -eq $selected -and $versions.Count -gt 0) {
                $selected = $versions[0]
            }
        }

        if ($null -ne $selected) {
            $latest = Get-Prop $selected @('version','Version') $latest
            return [ordered]@{
                version        = $latest
                payload_url    = Get-Prop $selected @('payload_url','DownloadUrl','download_url') ''
                payload_sha256 = Get-Prop $selected @('payload_sha256','sha256','SHA256') ''
                file_hash      = Get-Prop $selected @('FileHash','file_hash','md5') ''
                dll_file_name  = Get-Prop $selected @('DllFileName','dll_file_name') ''
                prerelease     = Get-BoolProp $selected @('prerelease','is_prerelease') $false
                release_channel = Get-Prop $selected @('release_channel','channel') ''
                module_page_url = Get-Prop $selected @('ModulePageUrl','module_page_url','release_url') ''
            }
        }

        return [ordered]@{
            version        = $latest
            payload_url    = Get-Prop $entry @('payload_url','DownloadUrl','download_url') ''
            payload_sha256 = Get-Prop $entry @('payload_sha256','sha256','SHA256') ''
            file_hash      = Get-Prop $entry @('FileHash','file_hash','md5') ''
            dll_file_name  = Get-Prop $entry @('DllFileName','dll_file_name') ''
            prerelease     = Get-BoolProp $entry @('prerelease','is_prerelease') $false
            release_channel = Get-Prop $entry @('release_channel','channel') ''
            module_page_url = Get-Prop $entry @('ModulePageUrl','module_page_url','release_url') ''
        }
    }

    function Get-RegistryManifest($uuid, [string]$version) {
        if (-not [string]::IsNullOrEmpty($version)) {
            try {
                return Invoke-RestMethod -Uri "$base/v1/modules/$uuid/versions/$version/manifest" `
                    -UseBasicParsing -Headers $headers
            } catch { }
        }
        try {
            return Invoke-RestMethod -Uri "$base/v1/modules/$uuid/manifest" `
                -UseBasicParsing -Headers $headers
        } catch { }
        return $null
    }

    function Convert-RegistryEntryToAvailable($entry) {
        $uuid = Get-Prop $entry @('uuid','ModuleId','module_id','id') ''
        if ([string]::IsNullOrEmpty($uuid)) { return $null }

        $wantedVersions = @()
        $versionsProp = $entry.PSObject.Properties['versions']
        if ($null -ne $versionsProp -and $null -ne $versionsProp.Value) {
            foreach ($v in @($versionsProp.Value)) {
                $vv = Get-Prop $v @('version','Version') ''
                if (-not [string]::IsNullOrEmpty($vv)) {
                    $wantedVersions += $vv
                }
            }
        }
        if ($wantedVersions.Count -eq 0) {
            $wantedVersions += ''
        }

        $available = @()
        foreach ($wantedVersion in $wantedVersions) {
            $verMeta = Select-VersionMetadata $entry $wantedVersion
            $version = [string]$verMeta.version
            if ([string]::IsNullOrEmpty($version)) { continue }

            $name = Get-Prop $entry @('name','ModuleName','module_name','label') $uuid
            $vendor = Get-Prop $entry @('vendor','AuthorName','author_name') 'Unknown'
            $description = Get-Prop $entry @('description','ModuleDescription','module_description') ''

            $manifest = Get-RegistryManifest $uuid $version
            if ($null -ne $manifest) {
                $name = Get-Prop $manifest @('name','ModuleName') $name
                $vendor = Get-Prop $manifest @('vendor','AuthorName') $vendor
                $description = Get-Prop $manifest @('description','ModuleDescription','module_description') $description
                $version = Get-Prop $manifest @('version','Version') $version
                if ([string]::IsNullOrEmpty([string]$verMeta.payload_sha256)) {
                    $verMeta.payload_sha256 = Get-Prop $manifest @('payload_sha256') ''
                }
                if ([string]::IsNullOrEmpty([string]$verMeta.payload_url)) {
                    $verMeta.payload_url = Get-Prop $manifest @('payload_url','download_url','DownloadUrl') ''
                }
                if (-not [bool]$verMeta.prerelease) {
                    $verMeta.prerelease = Get-BoolProp $manifest @('prerelease','is_prerelease') $false
                }
                if ([string]::IsNullOrEmpty([string]$verMeta.release_channel)) {
                    $verMeta.release_channel = Get-Prop $manifest @('release_channel','channel') ''
                }
            }

            $modulePageUrl = [string]$verMeta.module_page_url
            if ([string]::IsNullOrEmpty($modulePageUrl)) {
                $modulePageUrl = Get-Prop $entry @('ModulePageUrl','module_page_url','release_url') ''
            }

            $isPrerelease = [bool]$verMeta.prerelease
            $releaseChannel = [string]$verMeta.release_channel
            if (-not $isPrerelease -and (Test-IsPrereleaseChannel $releaseChannel)) {
                $isPrerelease = $true
            }
            if ($isPrerelease -and -not $script:IncludePrerelease) {
                continue
            }

            $available += [ordered]@{
                uuid           = $uuid
                version        = $version
                name           = $name
                vendor         = $vendor
                description    = $description
                source_id      = $srcId
                source_label   = Get-Prop $src @('label') 'Registry'
                registry_url   = $base
                payload_url    = [string]$verMeta.payload_url
                payload_sha256 = [string]$verMeta.payload_sha256
                prerelease     = $isPrerelease
                release_channel = $releaseChannel
                download_url   = Get-Prop $entry @('DownloadUrl','download_url') ''
                file_hash      = [string]$verMeta.file_hash
                dll_file_name  = [string]$verMeta.dll_file_name
                module_page_url = $modulePageUrl
            }
        }
        return $available
    }

    function Copy-DirectoryContents([string]$sourceDir, [string]$destDir) {
        if (-not (Test-Path $destDir)) { New-Item -ItemType Directory -Path $destDir -Force | Out-Null }
        Get-ChildItem -Path $sourceDir -Recurse | ForEach-Object {
            $rel = $_.FullName.Substring($sourceDir.Length).TrimStart('\','/')
            $target = Join-Path $destDir $rel
            if ($_.PSIsContainer) {
                if (-not (Test-Path $target)) { New-Item -ItemType Directory -Path $target -Force | Out-Null }
            } else {
                $parent = Split-Path -Parent $target
                if ($parent -and -not (Test-Path $parent)) {
                    New-Item -ItemType Directory -Path $parent -Force | Out-Null
                }
                Copy-Item -Path $_.FullName -Destination $target -Force
            }
        }
    }

    function Find-DllFile([string]$root, [string]$preferredName) {
        if (-not [string]::IsNullOrEmpty($preferredName)) {
            $match = Get-ChildItem -Path $root -Filter $preferredName -Recurse -File |
                     Select-Object -First 1
            if ($null -ne $match) { return $match }
        }
        return Get-ChildItem -Path $root -Filter '*.dll' -Recurse -File |
               Sort-Object FullName |
               Select-Object -First 1
    }

    function Install-RegistryStage([string]$stageDir, [string]$uuid, [string]$version,
                                   [hashtable]$sourceInfo) {
        if ([string]::IsNullOrEmpty($uuid) -or [string]::IsNullOrEmpty($version)) {
            throw 'registry install is missing uuid or version.'
        }
        $modsDir = Get-FtModulesDir
        $destDir = Join-Path $modsDir "$uuid\$version"
        if (Test-Path $destDir) {
            Remove-Item -Recurse -Force -Path $destDir
        }
        Copy-ModuleFolder -srcDir $stageDir -uuid $uuid -version $version -sourceInfo $sourceInfo
    }

    function New-CompatManifest($uuid, $version, $name, $vendor, $payloadSha, $payloadSize) {
        return [ordered]@{
            schema            = 1
            uuid              = $uuid
            name              = $name
            vendor            = $vendor
            homepage          = ''
            license           = ''
            version           = $version
            sdk_version       = 'VRCFT'
            min_host_version  = '0.0.0'
            supported_hmds    = @()
            capabilities      = @('eye', 'expression')
            platforms         = @('windows-x64')
            entry_assembly    = 'WKOpenVR.FaceTracking.VrcftCompat.dll'
            entry_type        = 'WKOpenVR.FaceTracking.VrcftCompat.ReflectingExtTrackingModuleAdapter'
            dependencies      = @()
            payload_sha256    = $payloadSha
            payload_size      = $payloadSize
        }
    }

    function Install-RegistryModule {
        $uuid = Get-Prop $src @('uuid','ModuleId','module_id') ''
        $ver = Get-Prop $src @('version','Version') ''
        if ([string]::IsNullOrEmpty($uuid)) {
            Write-Result $false 'Registry install requires a selected module uuid.'
            exit 1
        }

        $name = Get-Prop $src @('name','ModuleName','module_name') $uuid
        $vendor = Get-Prop $src @('vendor','AuthorName','author_name') 'Unknown'
        $dllFileName = Get-Prop $src @('dll_file_name','DllFileName') ''
        $downloadUrl = Get-Prop $src @('payload_url','download_url','DownloadUrl') ''
        $expectedSha = (Get-Prop $src @('payload_sha256','sha256','SHA256') '').ToLower()
        $expectedMd5 = (Get-Prop $src @('file_hash','FileHash','md5') '').ToLower()
        $registryManifest = $null
        $canonicalPayloadUrl = ''

        $registryManifest = Get-RegistryManifest $uuid $ver
        if ($null -ne $registryManifest) {
            $ver = Get-Prop $registryManifest @('version','Version') $ver
            $name = Get-Prop $registryManifest @('name','ModuleName') $name
            $vendor = Get-Prop $registryManifest @('vendor','AuthorName') $vendor
            if ([string]::IsNullOrEmpty($downloadUrl)) {
                $downloadUrl = Get-Prop $registryManifest @('payload_url','download_url','DownloadUrl') ''
            }
            if ([string]::IsNullOrEmpty($expectedSha)) {
                $expectedSha = (Get-Prop $registryManifest @('payload_sha256') '').ToLower()
            }
        }

        if ([string]::IsNullOrEmpty($ver)) {
            Write-Result $false "Registry module $uuid has no version."
            exit 1
        }
        if ([string]::IsNullOrEmpty($downloadUrl)) {
            $downloadUrl = "$base/v1/modules/$uuid/versions/$ver/payload"
        }
        $canonicalPayloadUrl = "$base/v1/modules/$uuid/versions/$ver/payload"

        $downloadLower = $downloadUrl.ToLowerInvariant()
        $isDllPayload = $downloadLower -match '\.dll($|\?)'
        $tmpPayload = [System.IO.Path]::GetTempFileName() + ($(if ($isDllPayload) { '.dll' } else { '.zip' }))
        $tmpDir = [System.IO.Path]::Combine([System.IO.Path]::GetTempPath(),
                                             [System.IO.Path]::GetRandomFileName())
        $wrapperDir = ''
        try {
            $downloadCandidates = @()
            if ($null -ne $registryManifest -and -not [string]::IsNullOrEmpty($canonicalPayloadUrl)) {
                $downloadCandidates += $canonicalPayloadUrl
            }
            if (-not [string]::IsNullOrEmpty($downloadUrl) -and -not ($downloadCandidates -contains $downloadUrl)) {
                $downloadCandidates += $downloadUrl
            }
            if ($downloadCandidates.Count -eq 0) {
                throw "No download URL is available for $uuid $ver."
            }

            $actualSha = ''
            $downloadErrors = @()
            $downloaded = $false
            foreach ($candidateUrl in $downloadCandidates) {
                try {
                    Remove-Item -Force -Path $tmpPayload -ErrorAction SilentlyContinue
                    Invoke-WebRequest -Uri $candidateUrl -OutFile $tmpPayload -UseBasicParsing -Headers $headers

                    $candidateSha = Get-Sha256 -filePath $tmpPayload
                    if (-not [string]::IsNullOrEmpty($expectedSha) -and $candidateSha -ne $expectedSha) {
                        throw "SHA-256 mismatch for ${uuid} ${ver}: expected $expectedSha got $candidateSha"
                    }
                    if (-not [string]::IsNullOrEmpty($expectedMd5)) {
                        $actualMd5 = Get-Md5 -filePath $tmpPayload
                        if ($actualMd5 -ne $expectedMd5) {
                            throw "MD5 mismatch for ${uuid} ${ver}: expected $expectedMd5 got $actualMd5"
                        }
                    }

                    $downloadUrl = $candidateUrl
                    $actualSha = $candidateSha
                    $downloaded = $true
                    break
                } catch {
                    $downloadErrors += "${candidateUrl}: $_"
                    Remove-Item -Force -Path $tmpPayload -ErrorAction SilentlyContinue
                }
            }
            if (-not $downloaded) {
                throw ("Download or verification failed for ${uuid} ${ver}: " + ($downloadErrors -join '; '))
            }

            New-Item -ItemType Directory -Path $tmpDir -Force | Out-Null
            if ($isDllPayload) {
                if ([string]::IsNullOrEmpty($dllFileName)) {
                    $dllFileName = [System.IO.Path]::GetFileName(([Uri]$downloadUrl).AbsolutePath)
                }
                if ([string]::IsNullOrEmpty($dllFileName)) { $dllFileName = "$uuid.dll" }
                Copy-Item -LiteralPath $tmpPayload -Destination (Join-Path $tmpDir $dllFileName) -Force
            } else {
                Expand-Archive -Path $tmpPayload -DestinationPath $tmpDir -Force
            }

            $manifestFile = Get-ChildItem -Path $tmpDir -Filter 'manifest.json' -Recurse -File |
                            Select-Object -First 1
            $payloadManifest = $null
            if ($null -ne $manifestFile) {
                try {
                    $payloadManifest = Get-Content $manifestFile.FullName -Raw -Encoding UTF8 | ConvertFrom-Json
                } catch { $payloadManifest = $null }
            }
            if ($null -eq $manifestFile -and $null -ne $registryManifest) {
                Write-JsonNoBom -Path (Join-Path $tmpDir 'manifest.json') -Data $registryManifest -Depth 12
                $manifestFile = Get-Item -LiteralPath (Join-Path $tmpDir 'manifest.json')
                $payloadManifest = $registryManifest
            }

            $hasHostManifest = $false
            if ($null -ne $payloadManifest -and
                $null -ne $payloadManifest.PSObject.Properties['entry_assembly'] -and
                $null -ne $payloadManifest.PSObject.Properties['entry_type']) {
                $hasHostManifest = $true
            }

            $stageRoot = $tmpDir
            if ($hasHostManifest) {
                $uuid = Get-Prop $payloadManifest @('uuid') $uuid
                $ver = Get-Prop $payloadManifest @('version') $ver
                $stageRoot = $manifestFile.DirectoryName
            } else {
                $dllFile = Find-DllFile -root $tmpDir -preferredName $dllFileName
                if ($null -eq $dllFile) {
                    throw "No module DLL found in registry payload for $uuid."
                }
                if ([string]::IsNullOrEmpty($dllFileName)) {
                    $dllFileName = $dllFile.Name
                }

                $relDll = $dllFile.FullName.Substring($tmpDir.Length).TrimStart('\','/')
                $wrapperDir = [System.IO.Path]::Combine([System.IO.Path]::GetTempPath(),
                                                         [System.IO.Path]::GetRandomFileName())
                New-Item -ItemType Directory -Path $wrapperDir -Force | Out-Null
                $assembliesDir = Join-Path $wrapperDir 'assemblies'
                Copy-DirectoryContents -sourceDir $tmpDir -destDir $assembliesDir
                Write-JsonNoBom -Path (Join-Path $assembliesDir 'bridge.json') `
                    -Data ([ordered]@{ upstream_assembly = $relDll }) -Depth 4
                Write-JsonNoBom -Path (Join-Path $wrapperDir 'manifest.json') `
                    -Data (New-CompatManifest $uuid $ver $name $vendor $actualSha ((Get-Item -LiteralPath $tmpPayload).Length)) `
                    -Depth 12
                $stageRoot = $wrapperDir
            }

            $info = @{
                source_id    = $srcId
                source_kind  = 'registry'
                registry_url = $base
                download_url = $downloadUrl
                installed_at = [DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ')
                payload_sha256 = $actualSha
            }
            Install-RegistryStage -stageDir $stageRoot -uuid $uuid -version $ver -sourceInfo $info
            Write-Result $true "Installed $name $ver." $uuid $ver
            exit 0
        } catch {
            Write-Result $false "Registry install failed for ${uuid}: $_"
            exit 1
        } finally {
            Remove-Item -Force -Path $tmpPayload -ErrorAction SilentlyContinue
            Remove-Item -Recurse -Force -Path $tmpDir -ErrorAction SilentlyContinue
            if (-not [string]::IsNullOrEmpty($wrapperDir)) {
                Remove-Item -Recurse -Force -Path $wrapperDir -ErrorAction SilentlyContinue
            }
        }
    }

    if ($Action -eq 'install') {
        Install-RegistryModule
    }

    try {
        $index = Get-RegistryIndex
    } catch {
        Write-Result $false "Registry module list fetch failed: $_"
        exit 1
    }

    if ($index.Entries.Count -eq 0) {
        Write-Result $false 'Registry returned no modules.'
        exit 1
    }

    $available = @()
    $failed = 0
    foreach ($entry in $index.Entries) {
        $modules = @(Convert-RegistryEntryToAvailable $entry)
        if ($modules.Count -eq 0) {
            continue
        }
        foreach ($module in $modules) {
            $available += $module
        }
    }

    $cache = [ordered]@{
        schema_version = 1
        source_id      = $srcId
        source_label   = Get-Prop $src @('label') 'Registry'
        registry_url   = $base
        checked_at     = [DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ')
        index_url      = $index.Uri
        modules        = $available
    }
    $cachePath = Join-Path (Get-FtAvailableDir) "$srcId.json"
    Write-JsonNoBom -Path $cachePath -Data $cache -Depth 16

    $summary = "Registry sync: found=$($available.Count) failed=$failed. Select a module from the list to install."
    if ($failed -gt 0 -and $available.Count -eq 0) {
        Write-Result $false $summary '' '' $available.Count
        exit 1
    }
    Write-Result $true $summary '' '' $available.Count
    exit 0
}

Write-Result $false "Unknown kind: $srcKind"
exit 1
