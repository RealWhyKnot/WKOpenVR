param(
    [string]$OutputPath = ""
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$AppRoot = Join-Path $Root "app"
$BuildRoot = Join-Path $Root "build"
$GeneratedRoot = Join-Path $BuildRoot "generated"
$ClassesRoot = Join-Path $BuildRoot "classes"
$DexRoot = Join-Path $BuildRoot "dex"
$ResourceZip = Join-Path $BuildRoot "resources.zip"
$UnsignedApk = Join-Path $BuildRoot "unsigned.apk"
$AlignedApk = Join-Path $BuildRoot "aligned.apk"
$Keystore = Join-Path $BuildRoot "debug.keystore"

function Find-FirstExisting {
    param([string[]]$Paths)
    foreach ($path in $Paths) {
        if ($path -and (Test-Path -LiteralPath $path)) {
            return $path
        }
    }
    return $null
}

function Resolve-AndroidSdk {
    $candidates = @(
        $env:ANDROID_SDK_ROOT,
        $env:ANDROID_HOME,
        (Join-Path $env:LOCALAPPDATA "Android\Sdk")
    )
    return Find-FirstExisting -Paths $candidates
}

function Resolve-JdkBin {
    if ($env:JAVA_HOME) {
        $candidate = Join-Path $env:JAVA_HOME "bin\javac.exe"
        if (Test-Path -LiteralPath $candidate) {
            return (Split-Path -Parent $candidate)
        }
    }

    $roots = @(
        "C:\Program Files\Eclipse Adoptium",
        "C:\Program Files\Java",
        "C:\Program Files\Android Studio\jbr",
        "C:\Program Files\Android Studio\jre"
    )
    foreach ($root in $roots) {
        if (-not (Test-Path -LiteralPath $root)) { continue }
        $javac = Get-ChildItem -LiteralPath $root -Filter javac.exe -Recurse -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if ($javac) {
            return (Split-Path -Parent $javac.FullName)
        }
    }
    return $null
}

$SdkRoot = Resolve-AndroidSdk
if (-not $SdkRoot) {
    throw "Android SDK was not found. Set ANDROID_SDK_ROOT or install Android SDK."
}

$Platform = Get-ChildItem -LiteralPath (Join-Path $SdkRoot "platforms") -Directory |
    Sort-Object Name -Descending |
    Select-Object -First 1
if (-not $Platform) {
    throw "Android SDK platforms folder is empty."
}
$AndroidJar = Join-Path $Platform.FullName "android.jar"

$BuildTools = Get-ChildItem -LiteralPath (Join-Path $SdkRoot "build-tools") -Directory |
    Sort-Object Name -Descending |
    Select-Object -First 1
if (-not $BuildTools) {
    throw "Android SDK build-tools folder is empty."
}
$Aapt2 = Join-Path $BuildTools.FullName "aapt2.exe"
$D8 = Join-Path $BuildTools.FullName "d8.bat"
$ApkSigner = Join-Path $BuildTools.FullName "apksigner.bat"
$ZipAlign = Join-Path $BuildTools.FullName "zipalign.exe"

$JdkBin = Resolve-JdkBin
if (-not $JdkBin) {
    throw "JDK was not found. Set JAVA_HOME or install a JDK."
}
$Javac = Join-Path $JdkBin "javac.exe"
$Keytool = Join-Path $JdkBin "keytool.exe"
$Jar = Join-Path $JdkBin "jar.exe"
$env:JAVA_HOME = Split-Path -Parent $JdkBin
$env:PATH = $JdkBin + ";" + $env:PATH

$outputWasDefault = -not $OutputPath
if ($outputWasDefault) {
    $OutputPath = Join-Path $BuildRoot "WKOpenVRQuestCompanion.apk"
}
if ([System.IO.Path]::IsPathRooted($OutputPath)) {
    $OutputPath = [System.IO.Path]::GetFullPath($OutputPath)
} else {
    $basePath = $Root
    if (-not $outputWasDefault) {
        $basePath = (Get-Location).Path
    }
    $OutputPath = [System.IO.Path]::GetFullPath((Join-Path $basePath $OutputPath))
}

Remove-Item -LiteralPath $BuildRoot -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force -Path $BuildRoot, $GeneratedRoot, $ClassesRoot, $DexRoot | Out-Null

& $Aapt2 compile --dir (Join-Path $AppRoot "src\main\res") -o $ResourceZip
if ($LASTEXITCODE -ne 0) { throw "aapt2 compile failed." }

& $Aapt2 link `
    -o $UnsignedApk `
    -I $AndroidJar `
    --manifest (Join-Path $AppRoot "src\main\AndroidManifest.xml") `
    --java $GeneratedRoot `
    --min-sdk-version 26 `
    --target-sdk-version 32 `
    --version-code 1 `
    --version-name 0.1.0 `
    $ResourceZip
if ($LASTEXITCODE -ne 0) { throw "aapt2 link failed." }

$JavaFiles = @()
$JavaFiles += Get-ChildItem -LiteralPath (Join-Path $AppRoot "src\main\java") -Filter *.java -Recurse | ForEach-Object { $_.FullName }
$JavaFiles += Get-ChildItem -LiteralPath $GeneratedRoot -Filter *.java -Recurse | ForEach-Object { $_.FullName }
& $Javac -source 8 -target 8 -classpath $AndroidJar -d $ClassesRoot @JavaFiles
if ($LASTEXITCODE -ne 0) { throw "javac failed." }

$ClassFiles = Get-ChildItem -LiteralPath $ClassesRoot -Filter *.class -Recurse | ForEach-Object { $_.FullName }
& $D8 --min-api 26 --lib $AndroidJar --output $DexRoot @ClassFiles
if ($LASTEXITCODE -ne 0) { throw "d8 failed." }

Push-Location $DexRoot
try {
    & $Jar uf $UnsignedApk "classes.dex"
    if ($LASTEXITCODE -ne 0) { throw "jar update failed." }
} finally {
    Pop-Location
}

& $ZipAlign -f 4 $UnsignedApk $AlignedApk
if ($LASTEXITCODE -ne 0) { throw "zipalign failed." }

& $Keytool -genkeypair `
    -keystore $Keystore `
    -storepass android `
    -keypass android `
    -alias androiddebugkey `
    -keyalg RSA `
    -keysize 2048 `
    -validity 10000 `
    -dname "CN=Android Debug,O=Android,C=US" | Out-Null
if ($LASTEXITCODE -ne 0) { throw "keytool failed." }

New-Item -ItemType Directory -Force -Path (Split-Path -Parent $OutputPath) | Out-Null
& $ApkSigner sign `
    --ks $Keystore `
    --ks-pass pass:android `
    --key-pass pass:android `
    --v4-signing-enabled false `
    --out $OutputPath `
    $AlignedApk
if ($LASTEXITCODE -ne 0) { throw "apksigner failed." }

$idsigPath = $OutputPath + ".idsig"
if (Test-Path -LiteralPath $idsigPath) {
    Remove-Item -LiteralPath $idsigPath -Force
}

Write-Host "Built $OutputPath"
