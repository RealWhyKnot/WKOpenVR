# Generate fixture artifacts for Phase 4 download tests. Idempotent: rerun
# overwrites the existing fixtures so tests stay deterministic. Outputs land
# under tests/fixtures/staging/.

[CmdletBinding()]
param(
	# Optional output root. Defaults to tests/fixtures/staging/ next to the
	# script.
	[string] $OutputRoot = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$ScriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Definition
if (-not $OutputRoot) {
	$OutputRoot = Join-Path $ScriptRoot 'staging'
}

# Wipe + recreate the output root so SHAs are reproducible across runs.
if (Test-Path -LiteralPath $OutputRoot) {
	Remove-Item -LiteralPath $OutputRoot -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $OutputRoot | Out-Null

function Write-UTF8NoBom {
	param([string] $Path, [string] $Content)
	$enc = New-Object System.Text.UTF8Encoding($false)
	$dir = Split-Path -Parent $Path
	if ($dir -and -not (Test-Path -LiteralPath $dir)) {
		New-Item -ItemType Directory -Force -Path $dir | Out-Null
	}
	[System.IO.File]::WriteAllText($Path, $Content, $enc)
}

function New-RandomBytes {
	param([string] $Path, [int] $Length, [int] $Seed)
	$rand = [System.Random]::new($Seed)
	$bytes = New-Object 'byte[]' $Length
	$rand.NextBytes($bytes)
	$dir = Split-Path -Parent $Path
	if ($dir -and -not (Test-Path -LiteralPath $dir)) {
		New-Item -ItemType Directory -Force -Path $dir | Out-Null
	}
	[System.IO.File]::WriteAllBytes($Path, $bytes)
}

function Get-Sha {
	param([string] $Path)
	return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

# 1. Captions pack fixtures: small binary blobs with stable SHAs.
$captionsDir = Join-Path $OutputRoot 'captions'
New-RandomBytes -Path (Join-Path $captionsDir 'ggml-stub.bin')     -Length 4096 -Seed 1
New-RandomBytes -Path (Join-Path $captionsDir 'silero-stub.onnx')  -Length 4096 -Seed 2
New-RandomBytes -Path (Join-Path $captionsDir 'onnxruntime-stub.dll') -Length 4096 -Seed 3

$ggmlSha    = Get-Sha (Join-Path $captionsDir 'ggml-stub.bin')
$sileroSha  = Get-Sha (Join-Path $captionsDir 'silero-stub.onnx')
$ortDllSha  = Get-Sha (Join-Path $captionsDir 'onnxruntime-stub.dll')

# 2. Captions test manifest. Points each file at a placeholder URL that the
# fixture HTTP server replaces with a 127.0.0.1 route at startup. The runner
# rewrites %FIXTURE_BASE% to http://127.0.0.1:<port>/ before invoking
# install-captions-pack.ps1.
$captionsPacks = @{
	'$schema' = 'wkopenvr-captions-packs/v1'
	packs = @(
		[ordered]@{
			id          = 'harness-base-en'
			label       = 'Harness fixture: English base'
			description = 'Synthesized fixture used by the WKOpenVR test harness; not a real model.'
			files = @(
				[ordered]@{
					name        = 'models/ggml-base.bin'
					url         = '%FIXTURE_BASE%captions/ggml-stub.bin'
					sha256      = $ggmlSha
					destination = 'models\ggml-base.bin'
				},
				[ordered]@{
					name        = 'models/silero_vad.onnx'
					url         = '%FIXTURE_BASE%captions/silero-stub.onnx'
					sha256      = $sileroSha
					destination = 'models\silero_vad.onnx'
				},
				[ordered]@{
					name        = 'runtime/onnxruntime.dll'
					url         = '%FIXTURE_BASE%captions/onnxruntime-stub.dll'
					sha256      = $ortDllSha
					destination = 'runtime\onnxruntime.dll'
				}
			)
		}
	)
}
Write-UTF8NoBom -Path (Join-Path $OutputRoot 'captions-test-packs.json') `
	-Content ($captionsPacks | ConvertTo-Json -Depth 8)

# 3. VRCFT fixture module. Folder containing manifest.json + module.dll bytes.
$vrcftDir  = Join-Path $OutputRoot 'vrcft-stub'
$dllPath   = Join-Path $vrcftDir 'WKOpenVR.FaceTracking.Stub.dll'
New-RandomBytes -Path $dllPath -Length 8192 -Seed 4
$dllSha    = Get-Sha $dllPath

$moduleUuid    = 'f1c7a000-0001-4f00-8000-fb7651000001'
$moduleVersion = '0.1.0'
$manifest = [ordered]@{
	uuid    = $moduleUuid
	version = $moduleVersion
	name    = 'WKOpenVR Test-Harness Stub Module'
	files   = @(
		[ordered]@{
			path   = 'WKOpenVR.FaceTracking.Stub.dll'
			sha256 = $dllSha
		}
	)
}
Write-UTF8NoBom -Path (Join-Path $vrcftDir 'manifest.json') `
	-Content ($manifest | ConvertTo-Json -Depth 6)

# 4. Registry-style payload: zip of the vrcft-stub folder + a manifest that
# includes the payload SHA. The fixture server serves this zip at a known
# path; face-module-sync.ps1 downloads it via -Kind registry.
$registryDir = Join-Path $OutputRoot 'registry'
New-Item -ItemType Directory -Force -Path $registryDir | Out-Null
$zipPath = Join-Path $registryDir "$moduleUuid-$moduleVersion.zip"
if (Test-Path -LiteralPath $zipPath) { Remove-Item -LiteralPath $zipPath -Force }
Compress-Archive -Path (Join-Path $vrcftDir '*') -DestinationPath $zipPath -CompressionLevel Fastest
$zipSha = Get-Sha $zipPath

$registryIndex = [ordered]@{
	'$schema' = 'wkvrcft-legacy-registry/v1'
	modules = @(
		[ordered]@{
			uuid       = $moduleUuid
			latest     = $moduleVersion
			source_id  = 'wkopenvr-harness-stub'
			label      = 'WKOpenVR harness stub'
			versions = @(
				[ordered]@{
					version       = $moduleVersion
					payload_url   = "%FIXTURE_BASE%registry/$moduleUuid-$moduleVersion.zip"
					payload_sha256 = $zipSha
				}
			)
		}
	)
}
Write-UTF8NoBom -Path (Join-Path $registryDir 'index.json') `
	-Content ($registryIndex | ConvertTo-Json -Depth 8)

# 5. Native registry fixture. The manifest advertises a bad external payload
# URL, while /v1/modules/<uuid>/versions/<version>/payload serves the verified
# registry payload. This catches installers that prefer the external URL over
# the registry-owned payload endpoint.
$nativeRegistryDir = Join-Path $OutputRoot 'native-registry'
$nativeUuid = 'f1c7a000-0002-4f00-8000-fb7651000002'
$nativeVersion = '2026.6.7.0-beta'
$nativeModuleDir = Join-Path $OutputRoot 'native-module'
$nativeAssemblyName = 'WKOpenVR.FaceTracking.NativeStub.dll'
New-RandomBytes -Path (Join-Path $nativeModuleDir $nativeAssemblyName) -Length 8192 -Seed 14

$nativePayloadDir = Join-Path $nativeRegistryDir "v1\modules\$nativeUuid\versions\$nativeVersion"
New-Item -ItemType Directory -Force -Path $nativePayloadDir | Out-Null
$nativeZipPath = Join-Path $nativePayloadDir 'payload.zip'
if (Test-Path -LiteralPath $nativeZipPath) { Remove-Item -LiteralPath $nativeZipPath -Force }
Compress-Archive -Path (Join-Path $nativeModuleDir '*') -DestinationPath $nativeZipPath -CompressionLevel Fastest
$nativeZipSha = Get-Sha $nativeZipPath
$nativeZipSize = (Get-Item -LiteralPath $nativeZipPath).Length

$nativeManifest = [ordered]@{
	schema          = 1
	uuid            = $nativeUuid
	name            = 'WKOpenVR Native Registry Stub'
	vendor          = 'WhyKnot'
	homepage        = 'https://example.invalid/native-fixture'
	license         = 'GPL-3.0-only'
	version         = $nativeVersion
	sdk_version     = '2026.6.7.0-beta'
	min_host_version = '1.0'
	supported_hmds  = @('*')
	capabilities    = @('expression', 'audio')
	platforms       = @('windows-x64')
	module_kind     = 'wkopenvr-native'
	module_api      = 'WKOpenVR.FaceTracking.Sdk/2026.6.7.0-beta'
	sdk_package     = 'WKOpenVR.FaceTracking.Sdk'
	entry_assembly  = $nativeAssemblyName
	entry_type      = 'WKOpenVR.FaceTracking.NativeStub.Module'
	dependencies    = @()
	release_tag     = 'v2026.6.7.0-beta'
	release_url     = 'https://example.invalid/native-fixture/releases/v2026.6.7.0-beta'
	release_channel = 'beta'
	prerelease      = $true
	payload_url     = '%FIXTURE_BASE%native-registry/bad-payload.zip?wrong-sha=1'
	payload_sha256  = $nativeZipSha
	payload_size    = $nativeZipSize
}
Write-UTF8NoBom -Path (Join-Path $nativePayloadDir 'manifest.json') `
	-Content ($nativeManifest | ConvertTo-Json -Depth 8)
New-Item -ItemType Directory -Force -Path (Join-Path $nativeRegistryDir "v1\modules\$nativeUuid") | Out-Null
Copy-Item -LiteralPath (Join-Path $nativePayloadDir 'manifest.json') `
	-Destination (Join-Path $nativeRegistryDir "v1\modules\$nativeUuid\manifest.json") -Force

$nativeIndex = [ordered]@{
	schema = 1
	generated_at = '2026-06-07T00:00:00Z'
	modules = @(
		[ordered]@{
			uuid            = $nativeUuid
			name            = $nativeManifest.name
			vendor          = $nativeManifest.vendor
			homepage        = $nativeManifest.homepage
			version         = $nativeVersion
			sdk_version     = $nativeManifest.sdk_version
			capabilities    = $nativeManifest.capabilities
			platforms       = $nativeManifest.platforms
			module_kind     = $nativeManifest.module_kind
			payload_url     = $nativeManifest.payload_url
			payload_sha256  = $nativeZipSha
			payload_size    = $nativeZipSize
			release_tag     = $nativeManifest.release_tag
			release_url     = $nativeManifest.release_url
			latest          = $nativeVersion
			release_channel = 'beta'
			prerelease      = $true
			versions        = @(
				[ordered]@{
					version         = $nativeVersion
					sdk_version     = $nativeManifest.sdk_version
					module_api      = $nativeManifest.module_api
					payload_url     = $nativeManifest.payload_url
					payload_sha256  = $nativeZipSha
					payload_size    = $nativeZipSize
					release_tag     = $nativeManifest.release_tag
					release_url     = $nativeManifest.release_url
					release_channel = 'beta'
					prerelease      = $true
				}
			)
		}
	)
}
Write-UTF8NoBom -Path (Join-Path $nativeRegistryDir 'index.json') `
	-Content ($nativeIndex | ConvertTo-Json -Depth 8)

# 6. Emit a small summary the test runner can read to know the fixture paths
# without re-deriving them.
$summary = [ordered]@{
	output_root             = $OutputRoot
	captions_manifest       = (Join-Path $OutputRoot 'captions-test-packs.json')
	captions_payload_dir    = $captionsDir
	captions_pack_id        = 'harness-base-en'
	captions_ggml_sha       = $ggmlSha
	captions_silero_sha     = $sileroSha
	captions_ort_sha        = $ortDllSha
	vrcft_module_dir        = $vrcftDir
	vrcft_module_uuid       = $moduleUuid
	vrcft_module_version    = $moduleVersion
	vrcft_module_dll_sha    = $dllSha
	registry_index          = (Join-Path $registryDir 'index.json')
	registry_zip            = $zipPath
	registry_zip_sha        = $zipSha
	native_registry_index   = (Join-Path $nativeRegistryDir 'index.json')
	native_module_uuid      = $nativeUuid
	native_module_version   = $nativeVersion
	native_module_assembly  = $nativeAssemblyName
	native_registry_zip     = $nativeZipPath
	native_registry_zip_sha = $nativeZipSha
}
Write-UTF8NoBom -Path (Join-Path $OutputRoot 'fixtures.summary.json') `
	-Content ($summary | ConvertTo-Json -Depth 4)

Write-Host "Fixtures generated under $OutputRoot"
Write-Host "  captions manifest: $($summary.captions_manifest)"
Write-Host "  vrcft module dir:  $($summary.vrcft_module_dir)"
Write-Host "  registry zip:      $($summary.registry_zip)"
