# Local HTTP fixture server for WKOpenVR Phase 4 download tests. Binds to
# http://127.0.0.1:<port>/ and serves files from the fixtures/staging/ tree
# the generator emitted. Designed to run as a child process so the parent
# runner can kill it when tests finish. Loopback-only binding does not need
# a Windows firewall rule.

[CmdletBinding()]
param(
	[Parameter(Mandatory = $true)][int]    $Port,
	[Parameter(Mandatory = $true)][string] $FixturesRoot
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Send-File {
	param($Response, [string] $Path, [string] $ContentType = 'application/octet-stream')
	if (-not (Test-Path -LiteralPath $Path)) {
		$Response.StatusCode = 404
		return
	}
	$bytes = [System.IO.File]::ReadAllBytes($Path)
	$Response.ContentType   = $ContentType
	$Response.ContentLength64 = $bytes.Length
	$Response.OutputStream.Write($bytes, 0, $bytes.Length)
}

function Send-Bytes {
	param($Response, [byte[]] $Bytes, [string] $ContentType)
	$Response.ContentType = $ContentType
	$Response.ContentLength64 = $Bytes.Length
	$Response.OutputStream.Write($Bytes, 0, $Bytes.Length)
}

function Send-Text {
	param($Response, [int] $Status, [string] $Body, [string] $ContentType = 'text/plain')
	$bytes = [System.Text.Encoding]::UTF8.GetBytes($Body)
	$Response.StatusCode = $Status
	Send-Bytes $Response $bytes $ContentType
}

# Resolve fixture paths once up front so each request is cheap.
$CaptionsDir = Join-Path $FixturesRoot 'captions'
$RegistryDir = Join-Path $FixturesRoot 'registry'
$NativeRegistryDir = Join-Path $FixturesRoot 'native-registry'
$RequestCounts = @{}

$listener = New-Object System.Net.HttpListener
$prefix = "http://127.0.0.1:$Port/"
$listener.Prefixes.Add($prefix)
$listener.Start()
Write-Host "[fixture-server] listening on $prefix"

try {
	while ($listener.IsListening) {
		$context = $null
		try {
			$context = $listener.GetContext()
		} catch [System.Net.HttpListenerException] {
			break
		}
		if (-not $context) { continue }
		$request  = $context.Request
		$response = $context.Response
		try {
			$path = $request.Url.AbsolutePath
			$query = $request.Url.Query

			# Inject HTTP errors via query parameters so the runner can drive
			# negative cases without re-launching the server.
			$forcedStatus = 0
			if ($query -match '[\?&]status=(\d+)') {
				$forcedStatus = [int]$Matches[1]
			}
			if ($query -match '[\?&]flaky=(\d+)') {
				$failCount = [int]$Matches[1]
				$key = $path + $query
				if (-not $RequestCounts.ContainsKey($key)) {
					$RequestCounts[$key] = 0
				}
				$RequestCounts[$key] = [int]$RequestCounts[$key] + 1
				if ([int]$RequestCounts[$key] -le $failCount) {
					Send-Text $response 503 "flaky failure $($RequestCounts[$key])/$failCount"
					continue
				}
			}
			if ($forcedStatus -ge 400) {
				Send-Text $response $forcedStatus "forced status $forcedStatus"
				continue
			}
			if ($query -match '[\?&]wrong-sha=1') {
				$bytes = New-Object 'byte[]' 256
				[void]([System.Random]::new(99)).NextBytes($bytes)
				$response.StatusCode = 200
				Send-Bytes $response $bytes 'application/octet-stream'
				continue
			}
			if ($query -match '[\?&]cut=(\d+)') {
				$cutAt = [int]$Matches[1]
				$response.SendChunked = $false
				$response.StatusCode = 200
				$response.ContentType = 'application/octet-stream'
				# Write partial data then close without finishing.
				$payload = New-Object 'byte[]' ($cutAt + 16)
				[void]([System.Random]::new(42)).NextBytes($payload)
				$response.OutputStream.Write($payload, 0, $cutAt)
				$response.OutputStream.Close()
				continue
			}

			# Healthcheck (used by the runner to wait for readiness).
			if ($path -eq '/__ping') {
				Send-Text $response 200 'ok'
				continue
			}

			# Captions routes: /captions/<filename> -> fixtures/captions/<filename>
			if ($path -like '/captions/*') {
				$file = $path.Substring('/captions/'.Length)
				Send-File $response (Join-Path $CaptionsDir $file)
				continue
			}

			# Registry routes: /registry/index, /registry/<file>.zip
			if ($path -eq '/registry/index' -or $path -eq '/registry/index.json') {
				Send-File $response (Join-Path $RegistryDir 'index.json') 'application/json'
				continue
			}
			if ($path -like '/registry/*') {
				$file = $path.Substring('/registry/'.Length)
				$ct = if ($file -like '*.zip') { 'application/zip' } else { 'application/octet-stream' }
				Send-File $response (Join-Path $RegistryDir $file) $ct
				continue
			}

			# Native registry routes mirror wkopenvr-module-registry's static
			# v1 layout.
			if ($path -eq '/native-registry/v1/index' -or $path -eq '/native-registry/v1/index.json') {
				Send-File $response (Join-Path $NativeRegistryDir 'index.json') 'application/json'
				continue
			}
			if ($path -match '^/native-registry/v1/modules/([^/]+)/manifest$') {
				$uuid = $Matches[1]
				Send-File $response (Join-Path $NativeRegistryDir "v1\modules\$uuid\manifest.json") 'application/json'
				continue
			}
			if ($path -match '^/native-registry/v1/modules/([^/]+)/versions/([^/]+)/manifest$') {
				$uuid = $Matches[1]
				$version = $Matches[2]
				Send-File $response (Join-Path $NativeRegistryDir "v1\modules\$uuid\versions\$version\manifest.json") 'application/json'
				continue
			}
			if ($path -match '^/native-registry/v1/modules/([^/]+)/versions/([^/]+)/payload$') {
				$uuid = $Matches[1]
				$version = $Matches[2]
				Send-File $response (Join-Path $NativeRegistryDir "v1\modules\$uuid\versions\$version\payload.zip") 'application/zip'
				continue
			}

			Send-Text $response 404 "no route for $path"
		} catch {
			try {
				Send-Text $response 500 ("server error: " + $_.Exception.Message)
			} catch { }
		} finally {
			try { $response.OutputStream.Close() } catch { }
		}
	}
}
finally {
	if ($listener.IsListening) { $listener.Stop() }
	$listener.Close()
	Write-Host "[fixture-server] stopped"
}
