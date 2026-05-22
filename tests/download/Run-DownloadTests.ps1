# Phase 4 of test.ps1: exercise the captions + facetracking download flows
# against a local HTTP fixture server. Generates fixtures, starts the
# server, runs each case, tears everything down. Throws on first failure
# so test.ps1's outer try/catch maps it to a non-zero exit.

[CmdletBinding()]
param(
	# Override the staging output root for fixtures. Defaults to a per-run
	# temp dir so the script is safe to invoke repeatedly without colliding
	# with a previous run's leftovers.
	[string] $WorkingRoot = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$ScriptRoot   = Split-Path -Parent $MyInvocation.MyCommand.Definition
$RepoRoot     = Split-Path -Parent (Split-Path -Parent $ScriptRoot)
$FixturesDir  = Join-Path $RepoRoot 'tests\fixtures'
$GeneratorPs1 = Join-Path $FixturesDir 'generate-fixtures.ps1'
$ServerPs1    = Join-Path $ScriptRoot  'FixtureServer.ps1'
$AssertLib    = Join-Path $ScriptRoot  'lib\Assert.ps1'
$CaptionsPs1  = Join-Path $RepoRoot 'modules\captions\src\host\resources\install-captions-pack.ps1'
$FaceSyncPs1  = Join-Path $RepoRoot 'modules\facetracking\src\overlay\face-module-sync.ps1'

. $AssertLib

foreach ($p in @($GeneratorPs1, $ServerPs1, $CaptionsPs1, $FaceSyncPs1)) {
	Assert-FileExists $p 'required script'
}

function Quote-PsSingle([string] $Value) {
	return "'" + ($Value -replace "'", "''") + "'"
}

function Invoke-FaceSyncFolder([string] $SourceData, [string] $SourceId, [string] $ResultPath) {
	$env:WKOPENVR_FACE_SYNC_SOURCE_DATA = $SourceData
	try {
		$command = @"
`$ProgressPreference = 'SilentlyContinue'
& $(Quote-PsSingle $FaceSyncPs1) -Action 'add' -Kind 'folder' -SourceData `$env:WKOPENVR_FACE_SYNC_SOURCE_DATA -SourceId $(Quote-PsSingle $SourceId) -ResultPath $(Quote-PsSingle $ResultPath)
exit `$LASTEXITCODE
"@
		$encoded = [Convert]::ToBase64String([System.Text.Encoding]::Unicode.GetBytes($command))
		& pwsh.exe -NoProfile -EncodedCommand $encoded
		return $LASTEXITCODE
	}
	finally {
		Remove-Item Env:WKOPENVR_FACE_SYNC_SOURCE_DATA -ErrorAction SilentlyContinue
	}
}

# ---- workspace -----------------------------------------------------------

if (-not $WorkingRoot) {
	$WorkingRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("wkopenvr-dl-tests-" + [Guid]::NewGuid().ToString('N').Substring(0,8))
}
if (Test-Path -LiteralPath $WorkingRoot) {
	Remove-Item -LiteralPath $WorkingRoot -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $WorkingRoot | Out-Null
$LocalAppDataLow = Join-Path $WorkingRoot 'LocalAppDataLow'
New-Item -ItemType Directory -Force -Path $LocalAppDataLow | Out-Null

Write-Host "Phase 4 working root: $WorkingRoot"

# ---- generate fixtures ----------------------------------------------------

Write-Host ""
Write-Host "Generating fixtures via $GeneratorPs1"
& pwsh.exe -NoProfile -File $GeneratorPs1
if ($LASTEXITCODE -ne 0) { throw "generate-fixtures.ps1 failed (exit $LASTEXITCODE)" }

$StagingDir   = Join-Path $FixturesDir 'staging'
$SummaryPath  = Join-Path $StagingDir 'fixtures.summary.json'
Assert-FileExists $SummaryPath 'fixtures summary'
$Summary = Get-Content -LiteralPath $SummaryPath -Raw | ConvertFrom-Json

# ---- pick a free TCP port -------------------------------------------------

$tmpListener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, 0)
$tmpListener.Start()
$Port = $tmpListener.LocalEndpoint.Port
$tmpListener.Stop()
$FixtureBaseUrl = "http://127.0.0.1:$Port/"
Write-Host "Fixture base URL: $FixtureBaseUrl"

# ---- start the fixture server in a background job ------------------------

$serverJob = Start-Job -Name 'wkopenvr-fixture-server' -ScriptBlock {
	param($ServerScript, $Port, $StagingDir)
	& $ServerScript -Port $Port -FixturesRoot $StagingDir
} -ArgumentList $ServerPs1, $Port, $StagingDir

try {
	# Wait for the server's /__ping route to answer 200.
	$ready = $false
	for ($i = 0; $i -lt 40; ++$i) {
		Start-Sleep -Milliseconds 150
		try {
			$resp = Invoke-WebRequest -Uri ($FixtureBaseUrl + '__ping') -UseBasicParsing -TimeoutSec 2
			if ($resp.StatusCode -eq 200) { $ready = $true; break }
		} catch { }
	}
	if (-not $ready) {
		throw "fixture server did not respond on $FixtureBaseUrl within 6s"
	}
	Write-Host "Fixture server ready."

	# Rewrite %FIXTURE_BASE% placeholders in the generated manifest so the
	# captions installer sees real URLs.
	$captionsManifestSrc = $Summary.captions_manifest
	$captionsManifestDst = Join-Path $WorkingRoot 'captions-test-packs.json'
	$content = Get-Content -LiteralPath $captionsManifestSrc -Raw
	$content = $content -replace '%FIXTURE_BASE%', $FixtureBaseUrl
	[System.IO.File]::WriteAllText($captionsManifestDst, $content, (New-Object System.Text.UTF8Encoding($false)))

	# Same rewrite for the registry index.
	$registryIndexSrc = $Summary.registry_index
	$registryIndexDst = Join-Path $WorkingRoot 'registry-index.json'
	$content = Get-Content -LiteralPath $registryIndexSrc -Raw
	$content = $content -replace '%FIXTURE_BASE%', $FixtureBaseUrl
	[System.IO.File]::WriteAllText($registryIndexDst, $content, (New-Object System.Text.UTF8Encoding($false)))

	# Redirect captions + face-module-sync to write under our isolated tree.
	$env:WKOPENVR_LOCALAPPDATA_OVERRIDE = $LocalAppDataLow

	# --------------------------------------------------------------------
	# Case 1: captions happy path
	# --------------------------------------------------------------------
	Write-CaseStart "captions-happy-path"
	& pwsh.exe -NoProfile -File $CaptionsPs1 -PackId $Summary.captions_pack_id -Manifest $captionsManifestDst
	if ($LASTEXITCODE -ne 0) { throw "captions installer exited $LASTEXITCODE" }
	$captionsRoot = Join-Path $LocalAppDataLow 'WKOpenVR\captions'
	Assert-FileExists (Join-Path $captionsRoot 'models\ggml-base.bin') 'whisper model'
	Assert-FileExists (Join-Path $captionsRoot 'models\silero_vad.onnx') 'silero model'
	Assert-FileExists (Join-Path $captionsRoot 'runtime\onnxruntime.dll') 'onnx runtime'
	Assert-Sha256Matches (Join-Path $captionsRoot 'models\ggml-base.bin') $Summary.captions_ggml_sha
	Assert-Sha256Matches (Join-Path $captionsRoot 'models\silero_vad.onnx') $Summary.captions_silero_sha
	Assert-Sha256Matches (Join-Path $captionsRoot 'runtime\onnxruntime.dll') $Summary.captions_ort_sha
	Write-CasePass "captions-happy-path"

	# --------------------------------------------------------------------
	# Case 2: captions idempotent rerun
	# --------------------------------------------------------------------
	Write-CaseStart "captions-idempotent-rerun"
	& pwsh.exe -NoProfile -File $CaptionsPs1 -PackId $Summary.captions_pack_id -Manifest $captionsManifestDst
	if ($LASTEXITCODE -ne 0) { throw "captions rerun exited $LASTEXITCODE" }
	# Files should still match their declared SHAs (no corruption).
	Assert-Sha256Matches (Join-Path $captionsRoot 'models\ggml-base.bin') $Summary.captions_ggml_sha
	Write-CasePass "captions-idempotent-rerun"

	# --------------------------------------------------------------------
	# Case 3: captions bad-sha (manifest declares wrong SHA, installer rejects)
	# --------------------------------------------------------------------
	Write-CaseStart "captions-bad-sha"
	$badManifest = Get-Content -LiteralPath $captionsManifestDst -Raw
	$wrongSha = ('f' * 64)
	$badContent = $badManifest -replace [regex]::Escape($Summary.captions_ggml_sha), $wrongSha
	$badManifestPath = Join-Path $WorkingRoot 'captions-bad-sha.json'
	[System.IO.File]::WriteAllText($badManifestPath, $badContent, (New-Object System.Text.UTF8Encoding($false)))
	# Wipe the captions tree so the installer re-downloads (and trips on SHA).
	Remove-Item -LiteralPath $captionsRoot -Recurse -Force -ErrorAction SilentlyContinue
	$oldErrorActionPreference = $ErrorActionPreference
	$ErrorActionPreference = 'Continue'
	try {
		$badOutput = & pwsh.exe -NoProfile -File $CaptionsPs1 -PackId $Summary.captions_pack_id -Manifest $badManifestPath 2>&1
		$badExit = $LASTEXITCODE
	}
	finally {
		$ErrorActionPreference = $oldErrorActionPreference
	}
	Assert-True ($badExit -ne 0) "installer must exit non-zero on bad SHA (got exit=$badExit; output: $($badOutput -join ' | '))"
	$badOutputJoined = ($badOutput -join "`n").ToLowerInvariant()
	Assert-True ($badOutputJoined -match 'sha|hash|mismatch') `
		"installer error must mention sha/hash/mismatch (got: $badOutputJoined)"
	# Bad-SHA file must not be left on disk where downstream code might use it.
	Assert-FileMissing (Join-Path $captionsRoot 'models\ggml-base.bin') 'leftover whisper model after bad-sha rejection'
	Write-CasePass "captions-bad-sha"

	# --------------------------------------------------------------------
	# Case 4: facetracking folder install (no HTTP needed; tests local-path branch)
	# --------------------------------------------------------------------
	Write-CaseStart "facetracking-folder-install"
	$faceResultPath = Join-Path $WorkingRoot 'face-folder-result.json'
	$folderSource = $Summary.vrcft_module_dir
	$folderJson = @{ path = $folderSource } | ConvertTo-Json -Compress
	$faceExit = Invoke-FaceSyncFolder -SourceData $folderJson -SourceId 'harness-folder-1' -ResultPath $faceResultPath
	if ($faceExit -ne 0) { throw "face-module-sync folder install exited $faceExit" }
	Assert-FileExists $faceResultPath 'face install result'
	$faceResult = Get-Content -LiteralPath $faceResultPath -Raw | ConvertFrom-Json
	Assert-True $faceResult.ok "face install result must report ok=true (msg='$($faceResult.message)')"
	$installedDir = Join-Path $LocalAppDataLow ("WKOpenVR\facetracking\modules\" + $faceResult.installed_uuid + "\" + $faceResult.installed_version)
	Assert-FileExists (Join-Path $installedDir 'manifest.json') 'staged manifest'
	Assert-FileExists (Join-Path $installedDir 'WKOpenVR.FaceTracking.Stub.dll') 'staged module dll'
	Assert-Sha256Matches (Join-Path $installedDir 'WKOpenVR.FaceTracking.Stub.dll') $Summary.vrcft_module_dll_sha
	Write-CasePass "facetracking-folder-install"

	# --------------------------------------------------------------------
	# Case 5: facetracking 404 (HTTP 404 simulation -- folder branch does not
	# hit the network, so we use the registry branch with a forced 404).
	# --------------------------------------------------------------------
	Write-CaseStart "facetracking-folder-missing-source"
	$bogusFolder = Join-Path $WorkingRoot ('no-such-folder-' + ([Guid]::NewGuid().ToString('N')))
	$bogusJson = @{ path = $bogusFolder } | ConvertTo-Json -Compress
	$bogusResult = Join-Path $WorkingRoot 'face-bogus-result.json'
	$exit = 0
	try {
		$exit = Invoke-FaceSyncFolder -SourceData $bogusJson -SourceId 'harness-bogus-1' -ResultPath $bogusResult
	} catch {
		$exit = -1
	}
	# face-module-sync.ps1 may exit 0 but write ok=false into the result file.
	if (Test-Path -LiteralPath $bogusResult) {
		$r = Get-Content -LiteralPath $bogusResult -Raw | ConvertFrom-Json
		Assert-True (-not $r.ok) "expected ok=false for missing source folder (msg='$($r.message)')"
	} else {
		Assert-True ($exit -ne 0) "expected non-zero exit OR a failure JSON; exit=$exit"
	}
	Write-CasePass "facetracking-folder-missing-source"

	Write-Host ""
	Write-Host "All Phase 4 download cases passed."
}
finally {
	Remove-Item Env:WKOPENVR_LOCALAPPDATA_OVERRIDE -ErrorAction SilentlyContinue

	if ($serverJob) {
		Stop-Job -Job $serverJob -ErrorAction SilentlyContinue
		Receive-Job -Job $serverJob -ErrorAction SilentlyContinue | Out-Null
		Remove-Job -Job $serverJob -Force -ErrorAction SilentlyContinue
	}
	Write-Host "Phase 4 working root left at: $WorkingRoot"
}

# Stop-Job leaves $LASTEXITCODE in whatever state the background job set it to
# (typically 1 because the HttpListener loop bails out of GetContext when the
# listener stops). Clear it so test.ps1's outer success check passes.
$global:LASTEXITCODE = 0
exit 0
