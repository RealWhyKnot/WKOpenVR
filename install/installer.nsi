; WKOpenVR installer.
; Installs the shared overlay and SteamVR driver tree.
;
; Usage:
;   makensis installer.nsi
;       -> WKOpenVR-Installer.exe  (all features, none pre-enabled)
;
;   makensis /DFEATURE=Calibration  installer.nsi
;   makensis /DFEATURE=Smoothing    installer.nsi
;   makensis /DFEATURE=InputHealth  installer.nsi
;   makensis /DFEATURE=FaceTracking installer.nsi
;   makensis /DFEATURE=OSCRouter    installer.nsi
;   makensis /DFEATURE=QuestApp     installer.nsi
;   makensis /DFEATURE=Captions     installer.nsi
;       -> WKOpenVR-<Feature>-Setup.exe  (single feature pre-enabled)

!include "MUI2.nsh"

!ifndef ARTIFACTS_BASEDIR
    !define ARTIFACTS_BASEDIR "..\build\artifacts\Release"
!endif
!ifndef DRIVER_BASEDIR
    !define DRIVER_BASEDIR "..\build\driver_wkopenvr"
!endif
!ifndef VERSION
    !define VERSION "0.1.0.0"
!endif

; Default FEATURE to All when not supplied on the command line.
!ifndef FEATURE
    !define FEATURE "All"
!endif

; Per-feature installer name / output path differ from the umbrella.
!if "${FEATURE}" == "All"
    Name "WKOpenVR"
    OutFile "..\build\artifacts\Release\WKOpenVR-Installer.exe"
!else
    !if "${FEATURE}" == "Calibration"
        Name "WKOpenVR (Calibration)"
        OutFile "..\build\artifacts\Release\WKOpenVR-Calibration-Setup.exe"
    !endif
    !if "${FEATURE}" == "Smoothing"
        Name "WKOpenVR (Smoothing)"
        OutFile "..\build\artifacts\Release\WKOpenVR-Smoothing-Setup.exe"
    !endif
    !if "${FEATURE}" == "InputHealth"
        Name "WKOpenVR (InputHealth)"
        OutFile "..\build\artifacts\Release\WKOpenVR-InputHealth-Setup.exe"
    !endif
    !if "${FEATURE}" == "FaceTracking"
        Name "WKOpenVR (FaceTracking)"
        OutFile "..\build\artifacts\Release\WKOpenVR-FaceTracking-Setup.exe"
    !endif
    !if "${FEATURE}" == "OSCRouter"
        Name "WKOpenVR (OSCRouter)"
        OutFile "..\build\artifacts\Release\WKOpenVR-OSCRouter-Setup.exe"
    !endif
    !if "${FEATURE}" == "QuestApp"
        Name "WKOpenVR (QuestApp)"
        OutFile "..\build\artifacts\Release\WKOpenVR-QuestApp-Setup.exe"
    !endif
    !if "${FEATURE}" == "Captions"
        Name "WKOpenVR (Captions)"
        OutFile "..\build\artifacts\Release\WKOpenVR-Captions-Setup.exe"
    !endif
!endif

InstallDir "$PROGRAMFILES64\WKOpenVR"
InstallDirRegKey HKLM "Software\WKOpenVR\Main" ""
RequestExecutionLevel admin
ShowInstDetails show
CRCCheck force
SetOverwrite on

VIProductVersion "${VERSION}"
VIAddVersionKey /LANG=1033 "ProductName" "WKOpenVR"
VIAddVersionKey /LANG=1033 "FileDescription" "WKOpenVR Installer"
VIAddVersionKey /LANG=1033 "LegalCopyright" "GPL-3.0-only, https://github.com/RealWhyKnot/WKOpenVR"
VIAddVersionKey /LANG=1033 "FileVersion" "${VERSION}"
VIAddVersionKey /LANG=1033 "ProductVersion" "${VERSION}"

Var vrRuntimePath
Var skipSteamVrRegistration

!define MUI_ABORTWARNING
!insertmacro MUI_PAGE_LICENSE "..\LICENSE"
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_LANGUAGE "English"

; --------------------------------------------------------------------------
; ResolveRuntimePath -- populates $vrRuntimePath from openvrpaths.vrpath.
; Always re-resolved at install time so a moved SteamVR is picked up.
; --------------------------------------------------------------------------
!macro ResolveRuntimePath
    nsExec::ExecToStack 'powershell -NoProfile -Command "try { ((Get-Content -Raw \"$$env:LOCALAPPDATA\openvr\openvrpaths.vrpath\" | ConvertFrom-Json).runtime)[0] } catch { exit 1 }"'
    Pop $0
    Pop $vrRuntimePath
    StrCmp $0 "0" +3
        MessageBox MB_OK|MB_ICONEXCLAMATION "Could not locate the SteamVR runtime path. Launch SteamVR once, then run this installer again."
        Abort
    Push $vrRuntimePath
    Call TrimNewlines
    Pop $vrRuntimePath
    DetailPrint "SteamVR runtime path: $vrRuntimePath"
!macroend

; --------------------------------------------------------------------------
; CheckProcessNotRunning -- aborts if <ProcessName> (without .exe extension)
; is running. Uses PowerShell Get-Process; exit 0 = running, exit 1 = not found.
; Usage: ${CheckProcessNotRunning} "vrserver" "Message shown on abort."
; --------------------------------------------------------------------------
!macro CheckProcessNotRunning ProcessName AbortMessage
    nsExec::ExecToStack 'powershell -NoProfile -Command "if (Get-Process -Name ''${ProcessName}'' -ErrorAction SilentlyContinue) { exit 0 } else { exit 1 }"'
    Pop $0
    Pop $1
    StrCmp $0 "0" 0 +3
        MessageBox MB_OK|MB_ICONEXCLAMATION "${AbortMessage}"
        Abort
!macroend
!define CheckProcessNotRunning "!insertmacro CheckProcessNotRunning"

; --------------------------------------------------------------------------
; RequireInstalledFile -- aborts if a critical output file is missing after
; extraction. This catches write failures before registry state is updated.
; --------------------------------------------------------------------------
!macro RequireInstalledFile Path Label
    IfFileExists "${Path}" +3
        MessageBox MB_OK|MB_ICONSTOP "${Label} was not installed. The upgrade did not finish."
        Abort
!macroend
!define RequireInstalledFile "!insertmacro RequireInstalledFile"

; Shared body for TrimNewlines. NSIS keeps install and uninstall functions in
; separate namespaces ("un." prefix), so each section needs its own copy of
; the function. Defining the body once as a macro avoids duplication drift.
!macro TrimNewlinesBody
    Exch $R0
    Push $R1
    Push $R2
    StrCpy $R1 0
    loop:
        IntOp $R1 $R1 - 1
        StrCpy $R2 $R0 1 $R1
        StrCmp $R2 "$\r" loop
        StrCmp $R2 "$\n" loop
        IntOp $R1 $R1 + 1
        IntCmp $R1 0 noTrim
        StrCpy $R0 $R0 $R1
    noTrim:
    Pop $R2
    Pop $R1
    Exch $R0
!macroend

Function TrimNewlines
    !insertmacro TrimNewlinesBody
FunctionEnd

Function un.TrimNewlines
    !insertmacro TrimNewlinesBody
FunctionEnd

; --------------------------------------------------------------------------
Section "Install"
; --------------------------------------------------------------------------
    ; Resolve $SMPROGRAMS / $APPDATA against the all-users location. The
    ; installer runs elevated and lands in Program Files (all-users), so
    ; the Start Menu shortcut belongs in the common Start Menu too --
    ; without this NSIS would put it in the elevating account's roaming
    ; profile, hidden from other Windows users on the machine.
    SetShellVarContext all

    ; Pre-install checks: SteamVR window, vrserver, WKOpenVR.
    FindWindow $0 "Qt5QWindowIcon" "SteamVR Status"
    StrCmp $0 0 +3
        MessageBox MB_OK|MB_ICONEXCLAMATION "SteamVR is still running. Close SteamVR and try again."
        Abort

    ${CheckProcessNotRunning} "vrserver" "vrserver.exe is still running. Close SteamVR completely and try again."
    ${CheckProcessNotRunning} "WKOpenVR" "WKOpenVR is still running. Close WKOpenVR and try again."

    ; Always re-resolve the runtime path so a moved SteamVR is picked up
    ; even when upgrading over a previous install whose registry value is stale.
    !insertmacro ResolveRuntimePath

    ReadEnvStr $skipSteamVrRegistration "WKOPENVR_SKIP_STEAMVR_REGISTRATION"

    ; Remove owned resource folders that can contain stale files from an older
    ; version. Feature enable flags are kept, except for dev-only/retired public
    ; flags that should not survive a public installer upgrade.
    RMDir /r "$INSTDIR\resources"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr\resources\facetracking\host"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr\resources\captions\host"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr\resources\translator"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr\resources\phantom"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\enable_phantom.flag"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\enable_translator.flag"

    SetOutPath "$INSTDIR"
    File /oname=WKOpenVR.exe "${ARTIFACTS_BASEDIR}\WKOpenVR.exe"
    File "${ARTIFACTS_BASEDIR}\openvr_api.dll"
    ; manifest.vrmanifest sits next to the exe so RegisterApplicationManifest
    ; in WKOpenVR can resolve it via GetModuleFileName + replace_filename
    ; on first launch and register the overlay with SteamVR for auto-start.
    File "${ARTIFACTS_BASEDIR}\manifest.vrmanifest"
    File "${ARTIFACTS_BASEDIR}\dashboard_icon.png"
    File "${ARTIFACTS_BASEDIR}\boundary_icon.png"
    File "..\LICENSE"
    File /oname=README.md "..\README.md"

    SetOutPath "$INSTDIR\resources"
    File "${ARTIFACTS_BASEDIR}\resources\face-module-sync.ps1"

    SetOutPath "$INSTDIR\resources\questapp"
    File "${ARTIFACTS_BASEDIR}\resources\questapp\install-platform-tools.ps1"
    File "${ARTIFACTS_BASEDIR}\resources\questapp\uninstall-questapp.ps1"
    File "${ARTIFACTS_BASEDIR}\resources\questapp\WKOpenVRQuestCompanion.apk"

    ; Start Menu shortcuts. The umbrella shortcut is always present.
    ; Per-feature aliases are dropped here only when the installer is
    ; pre-enabling exactly one feature; the umbrella installer (FEATURE=All)
    ; leaves all features disabled by default, so its only alias is the
    ; umbrella one -- the Modules tab adds / removes the per-feature
    ; shortcuts as the user toggles features at runtime.
    ;
    ; Each shortcut gets a unique `--launch=<feature>` argument. Windows
    ; Search dedupes Start-Menu entries by exact (target + arguments)
    ; tuple; without a distinct argument, searching "Smoothing" or
    ; "Input Health" would only surface one shortcut (the most-recently-
    ; modified .lnk that points at WKOpenVR.exe). The exe ignores
    ; unknown args today, so the launch path stays identical.
    CreateDirectory "$SMPROGRAMS\WKOpenVR"
    CreateShortCut "$SMPROGRAMS\WKOpenVR\WKOpenVR.lnk" "$INSTDIR\WKOpenVR.exe" "--launch=umbrella" "$INSTDIR\WKOpenVR.exe" 0 SW_SHOWNORMAL "" "Open WKOpenVR"

    !if "${FEATURE}" == "Calibration"
        CreateShortCut "$SMPROGRAMS\WKOpenVR\WKOpenVR - Space Calibrator.lnk" "$INSTDIR\WKOpenVR.exe" "--launch=calibration"  "$INSTDIR\WKOpenVR.exe" 0 SW_SHOWNORMAL "" "Space Calibrator in WKOpenVR"
    !endif
    !if "${FEATURE}" == "Smoothing"
        CreateShortCut "$SMPROGRAMS\WKOpenVR\WKOpenVR - Smoothing.lnk"        "$INSTDIR\WKOpenVR.exe" "--launch=smoothing"    "$INSTDIR\WKOpenVR.exe" 0 SW_SHOWNORMAL "" "Tracker smoothing in WKOpenVR"
    !endif
    !if "${FEATURE}" == "InputHealth"
        CreateShortCut "$SMPROGRAMS\WKOpenVR\WKOpenVR - Input Health.lnk"     "$INSTDIR\WKOpenVR.exe" "--launch=inputhealth"  "$INSTDIR\WKOpenVR.exe" 0 SW_SHOWNORMAL "" "Input Health in WKOpenVR"
    !endif
    !if "${FEATURE}" == "FaceTracking"
        CreateShortCut "$SMPROGRAMS\WKOpenVR\WKOpenVR - Face Tracking.lnk"    "$INSTDIR\WKOpenVR.exe" "--launch=facetracking" "$INSTDIR\WKOpenVR.exe" 0 SW_SHOWNORMAL "" "Face Tracking in WKOpenVR"
    !endif
    !if "${FEATURE}" == "OSCRouter"
        CreateShortCut "$SMPROGRAMS\WKOpenVR\WKOpenVR - OSC Router.lnk"       "$INSTDIR\WKOpenVR.exe" "--launch=oscrouter"    "$INSTDIR\WKOpenVR.exe" 0 SW_SHOWNORMAL "" "OSC Router in WKOpenVR"
    !endif
    !if "${FEATURE}" == "QuestApp"
        CreateShortCut "$SMPROGRAMS\WKOpenVR\WKOpenVR - Quest App.lnk"        "$INSTDIR\WKOpenVR.exe" "--launch=questapp"     "$INSTDIR\WKOpenVR.exe" 0 SW_SHOWNORMAL "" "Quest App in WKOpenVR"
    !endif
    !if "${FEATURE}" == "Captions"
        CreateShortCut "$SMPROGRAMS\WKOpenVR\WKOpenVR - Captions.lnk"         "$INSTDIR\WKOpenVR.exe" "--launch=captions"     "$INSTDIR\WKOpenVR.exe" 0 SW_SHOWNORMAL "" "Live Captions in WKOpenVR"
    !endif

    SetOutPath "$vrRuntimePath\drivers\01wkopenvr"
    File "${DRIVER_BASEDIR}\driver.vrdrivermanifest"
    SetOutPath "$vrRuntimePath\drivers\01wkopenvr\resources"
    File "${DRIVER_BASEDIR}\resources\driver.vrresources"
    SetOutPath "$vrRuntimePath\drivers\01wkopenvr\resources\settings"
    File "${DRIVER_BASEDIR}\resources\settings\default.vrsettings"
    SetOutPath "$vrRuntimePath\drivers\01wkopenvr\bin\win64"
    File /oname=driver_01wkopenvr.dll "${DRIVER_BASEDIR}\bin\win64\driver_wkopenvr.dll"

    ; FaceTracking host sidecar (.NET 10). Driver's HostSupervisor spawns
    ; WKOpenVR.FaceModuleHost.exe from this directory when the
    ; enable_facetracking.flag is present. The /r flag pulls in the entire
    ; published .NET tree (exe + .deps.json + .runtimeconfig.json + dependent
    ; DLLs). The whole folder is missing when the build host doesn't have the
    ; .NET 10 SDK -- in that case the !if /FileExists guard omits the File
    ; directive entirely at compile time so makensis does not abort with
    ; "no files found", and the feature simply runs inert at runtime
    ; (driver still loads, just no host process).
    !if /FileExists "${DRIVER_BASEDIR}\resources\facetracking\host\WKOpenVR.FaceModuleHost.exe"
        SetOutPath "$vrRuntimePath\drivers\01wkopenvr\resources\facetracking\host"
        File /r "${DRIVER_BASEDIR}\resources\facetracking\host\*.*"
    !else
        DetailPrint "FaceModuleHost not embedded in this installer (build host had no .NET 10 SDK); FaceTracking feature will run inert until the host is staged manually."
    !endif

    ; Captions host sidecar (native Win64 exe + dependencies). Driver's
    ; HostSupervisor spawns WKOpenVR.CaptionsHost.exe from this directory
    ; when enable_captions.flag is present. Same /FileExists guard as
    ; FaceModuleHost: omits the File directive silently when the build host
    ; produced no captions host (OPENVR_PAIR_BUILD_CAPTIONS_HOST=OFF),
    ; rather than aborting makensis with "no files found".
    !if /FileExists "${DRIVER_BASEDIR}\resources\captions\host\WKOpenVR.CaptionsHost.exe"
        SetOutPath "$vrRuntimePath\drivers\01wkopenvr\resources\captions\host"
        File /r "${DRIVER_BASEDIR}\resources\captions\host\*.*"
    !else
        DetailPrint "CaptionsHost not embedded in this installer (OPENVR_PAIR_BUILD_CAPTIONS_HOST=OFF or build missing); Captions feature will run inert until the host is staged manually."
    !endif

    ; Drop the feature enable flag when building a per-feature installer.
    ; Content matches what ShellContext::SetFlagPresent writes:
    ;   Set-Content -Value enabled -NoNewline
    !if "${FEATURE}" == "Calibration"
        FileOpen $0 "$vrRuntimePath\drivers\01wkopenvr\resources\enable_calibration.flag" w
        FileWrite $0 "enabled"
        FileClose $0
        DetailPrint "Enabled feature: Calibration"
    !endif
    !if "${FEATURE}" == "Smoothing"
        FileOpen $0 "$vrRuntimePath\drivers\01wkopenvr\resources\enable_smoothing.flag" w
        FileWrite $0 "enabled"
        FileClose $0
        DetailPrint "Enabled feature: Smoothing"
    !endif
    !if "${FEATURE}" == "InputHealth"
        FileOpen $0 "$vrRuntimePath\drivers\01wkopenvr\resources\enable_inputhealth.flag" w
        FileWrite $0 "enabled"
        FileClose $0
        DetailPrint "Enabled feature: InputHealth"
    !endif
    !if "${FEATURE}" == "FaceTracking"
        FileOpen $0 "$vrRuntimePath\drivers\01wkopenvr\resources\enable_facetracking.flag" w
        FileWrite $0 "enabled"
        FileClose $0
        DetailPrint "Enabled feature: FaceTracking"
    !endif
    !if "${FEATURE}" == "OSCRouter"
        FileOpen $0 "$vrRuntimePath\drivers\01wkopenvr\resources\enable_oscrouter.flag" w
        FileWrite $0 "enabled"
        FileClose $0
        DetailPrint "Enabled feature: OSCRouter"
    !endif
    !if "${FEATURE}" == "QuestApp"
        FileOpen $0 "$vrRuntimePath\drivers\01wkopenvr\resources\enable_questapp.flag" w
        FileWrite $0 "enabled"
        FileClose $0
        DetailPrint "Enabled feature: QuestApp"
    !endif
    !if "${FEATURE}" == "Captions"
        FileOpen $0 "$vrRuntimePath\drivers\01wkopenvr\resources\enable_captions.flag" w
        FileWrite $0 "enabled"
        FileClose $0
        DetailPrint "Enabled feature: Captions"
    !endif

    ${RequireInstalledFile} "$INSTDIR\WKOpenVR.exe" "WKOpenVR.exe"
    ${RequireInstalledFile} "$INSTDIR\openvr_api.dll" "openvr_api.dll"
    ${RequireInstalledFile} "$INSTDIR\manifest.vrmanifest" "manifest.vrmanifest"
    ${RequireInstalledFile} "$vrRuntimePath\drivers\01wkopenvr\driver.vrdrivermanifest" "driver.vrdrivermanifest"
    ${RequireInstalledFile} "$vrRuntimePath\drivers\01wkopenvr\resources\driver.vrresources" "driver.vrresources"
    ${RequireInstalledFile} "$vrRuntimePath\drivers\01wkopenvr\bin\win64\driver_01wkopenvr.dll" "driver_01wkopenvr.dll"

    WriteRegStr HKLM "Software\WKOpenVR\Main" "" "$INSTDIR"
    WriteRegStr HKLM "Software\WKOpenVR\Driver" "" "$vrRuntimePath"
    WriteRegStr HKLM "Software\WKOpenVR\Main" "Version" "${VERSION}"
    WriteRegStr HKLM "Software\WKOpenVR\Main" "Features" "${FEATURE}"

    ; Register the overlay manifest with SteamVR and flip autolaunch on,
    ; so the first SteamVR start after install opens WKOpenVR without
    ; the user having to launch the exe by hand. --register-only exits as
    ; soon as the in-process registration call completes (no GLFW window).
    StrCmp $skipSteamVrRegistration "1" skipRegisterOnly
    DetailPrint "Registering WKOpenVR overlay with SteamVR..."
    ExecWait '"$INSTDIR\WKOpenVR.exe" --register-only' $0
    DetailPrint "Registration exit code: $0"
    skipRegisterOnly:

    WriteUninstaller "$INSTDIR\Uninstall.exe"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\WKOpenVR" "DisplayName" "WKOpenVR"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\WKOpenVR" "DisplayVersion" "${VERSION}"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\WKOpenVR" "Publisher" "RealWhyKnot"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\WKOpenVR" "UninstallString" "$\"$INSTDIR\Uninstall.exe$\""
SectionEnd

; --------------------------------------------------------------------------
Section "Uninstall"
; --------------------------------------------------------------------------
    ; Match the install context so the all-users Start Menu shortcut is
    ; the one we remove. Without this the uninstall would clear the
    ; running user's roaming Start Menu and leave the common shortcut.
    SetShellVarContext all

    ; Pre-uninstall checks: removing driver files while vrserver holds them
    ; open silently fails and leaves orphan DLLs in the driver directory.
    FindWindow $0 "Qt5QWindowIcon" "SteamVR Status"
    StrCmp $0 0 +3
        MessageBox MB_OK|MB_ICONEXCLAMATION "SteamVR is still running. Close SteamVR and try again."
        Abort

    ${CheckProcessNotRunning} "vrserver" "vrserver.exe is still running. Close SteamVR completely and try again."
    ${CheckProcessNotRunning} "WKOpenVR" "WKOpenVR is still running. Close WKOpenVR and try again."

    SetOutPath "$TEMP"
    ReadEnvStr $skipSteamVrRegistration "WKOPENVR_SKIP_STEAMVR_REGISTRATION"

    ; Unregister with SteamVR while the exe + manifest still exist on disk.
    ; WKOpenVR.exe must run from $INSTDIR because RemoveApplicationManifest
    ; matches registrations by the exact registered manifest path, which
    ; was $INSTDIR\manifest.vrmanifest at install time.
    ;
    ; The 2-second sleep gives Windows time to release the image-map
    ; refcount on WKOpenVR.exe + openvr_api.dll after the spawned process
    ; exits. Defender's just-touched-binary scan and lazy mapping cleanup
    ; can both keep the files locked for 500ms-1500ms past process exit;
    ; without the wait the next Delete on WKOpenVR.exe or openvr_api.dll
    ; intermittently fails with sharing violation, leaving the install
    ; dir half-removed.
    StrCmp $skipSteamVrRegistration "1" skipUnregister
    IfFileExists "$INSTDIR\WKOpenVR.exe" 0 skipUnregister
        DetailPrint "Unregistering WKOpenVR overlay from SteamVR..."
        ExecWait '"$INSTDIR\WKOpenVR.exe" --unregister-only' $0
        DetailPrint "Unregistration exit code: $0"
        Sleep 2000
    skipUnregister:

    IfFileExists "$INSTDIR\resources\questapp\uninstall-questapp.ps1" 0 skipQuestAppCleanup
        DetailPrint "Cleaning Quest App companion state..."
        nsExec::ExecToLog 'powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$INSTDIR\resources\questapp\uninstall-questapp.ps1"'
    skipQuestAppCleanup:

    ; Re-resolve SteamVR runtime path. Fall back to the registry value when
    ; openvrpaths.vrpath is missing (SteamVR uninstalled after WKOpenVR).
    ClearErrors
    nsExec::ExecToStack 'powershell -NoProfile -Command "try { ((Get-Content -Raw \"$$env:LOCALAPPDATA\openvr\openvrpaths.vrpath\" | ConvertFrom-Json).runtime)[0] } catch { exit 1 }"'
    Pop $0
    Pop $vrRuntimePath
    StrCmp $0 "0" runtimeResolved
        ; Resolution failed -- fall back to the registry value.
        ReadRegStr $vrRuntimePath HKLM "Software\WKOpenVR\Driver" ""
    runtimeResolved:
    StrCmp $vrRuntimePath "" skipDriverRemoval
    Push $vrRuntimePath
    Call un.TrimNewlines
    Pop $vrRuntimePath
    DetailPrint "SteamVR runtime path for uninstall: $vrRuntimePath"

    ; ---- Remove the 01wkopenvr driver tree --------------------------------
    ; Step 1: delete every known file individually so the uninstall log
    ; shows exactly what came off disk. Step 2: RMDir /r the dedicated
    ; driver directory to catch anything we did not predict (driver-side
    ; logs, future feature files, etc.). The 01wkopenvr directory belongs
    ; entirely to our driver, so the recursive wipe is safe.
    ClearErrors
    Delete "$vrRuntimePath\drivers\01wkopenvr\driver.vrdrivermanifest"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\driver.vrresources"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\settings\default.vrsettings"
    Delete "$vrRuntimePath\drivers\01wkopenvr\bin\win64\driver_01wkopenvr.dll"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\enable_calibration.flag"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\enable_smoothing.flag"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\enable_inputhealth.flag"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\enable_facetracking.flag"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\enable_captions.flag"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\enable_phantom.flag"
    Delete "$vrRuntimePath\drivers\01wkopenvr\resources\enable_questapp.flag"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr\resources\facetracking"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr\resources\translator"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr\resources\captions"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr\resources\input"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr\resources\phantom"
    RMDir /r "$vrRuntimePath\drivers\01wkopenvr"

    ; ---- Best-effort legacy driver cleanup (pre-rename product) -----------
    ; Same recursive-wipe pattern; the 01openvrpair directory was the
    ; pre-rename equivalent of 01wkopenvr and likewise belongs only to
    ; this product.
    ClearErrors
    IfFileExists "$vrRuntimePath\drivers\01openvrpair" 0 skipLegacyDriver
        RMDir /r "$vrRuntimePath\drivers\01openvrpair"
    skipLegacyDriver:

    skipDriverRemoval:

    ; ---- Remove $INSTDIR contents -----------------------------------------
    ; The explicit Deletes log each known file; the trailing RMDir /r
    ; catches any future additions (extra resources, .ini snippets,
    ; future helper exes). The retry loop handles the realistic case
    ; that WKOpenVR.exe or openvr_api.dll is still locked by Windows'
    ; image-mapping cleanup or Defender's just-touched-binary scan
    ; (even after the Sleep 2000 above). After 10 retries (~10 s wall
    ; clock) the script falls back to Delete /REBOOTOK so the file is
    ; queued for next-reboot deletion, the reboot flag is set, and the
    ; user gets a "Reboot now?" prompt at the end of uninstall.
    StrCpy $0 0
    deleteRetry:
        ClearErrors
        Delete "$INSTDIR\WKOpenVR.exe"
        Delete "$INSTDIR\openvr_api.dll"
        Delete "$INSTDIR\manifest.vrmanifest"
        Delete "$INSTDIR\action_manifest.json"
        Delete "$INSTDIR\bindings_dashboardinput_knuckles.json"
        Delete "$INSTDIR\bindings_dashboardinput_touch.json"
        Delete "$INSTDIR\bindings_dashboardinput_vive.json"
        Delete "$INSTDIR\bindings_dashboardinput_generic.json"
        Delete "$INSTDIR\dashboard_icon.png"
        Delete "$INSTDIR\boundary_icon.png"
        Delete "$INSTDIR\LICENSE"
        Delete "$INSTDIR\README.md"
        Delete "$INSTDIR\Uninstall.exe"
        RMDir /r "$INSTDIR\resources"
        RMDir /r "$INSTDIR"
        IfFileExists "$INSTDIR\WKOpenVR.exe" instDirLocked deleteDone
        IfFileExists "$INSTDIR\openvr_api.dll" instDirLocked deleteDone
    instDirLocked:
        IntOp $0 $0 + 1
        IntCmp $0 10 deleteRebootOk
        DetailPrint "Install dir still locked (attempt $0); waiting 1s..."
        Sleep 1000
        Goto deleteRetry
    deleteRebootOk:
        DetailPrint "Files still locked after 10 retries -- queuing for reboot."
        Delete /REBOOTOK "$INSTDIR\WKOpenVR.exe"
        Delete /REBOOTOK "$INSTDIR\openvr_api.dll"
        Delete /REBOOTOK "$INSTDIR\manifest.vrmanifest"
        Delete /REBOOTOK "$INSTDIR\action_manifest.json"
        Delete /REBOOTOK "$INSTDIR\bindings_dashboardinput_knuckles.json"
        Delete /REBOOTOK "$INSTDIR\bindings_dashboardinput_touch.json"
        Delete /REBOOTOK "$INSTDIR\bindings_dashboardinput_vive.json"
        Delete /REBOOTOK "$INSTDIR\bindings_dashboardinput_generic.json"
        Delete /REBOOTOK "$INSTDIR\dashboard_icon.png"
        Delete /REBOOTOK "$INSTDIR\boundary_icon.png"
        Delete /REBOOTOK "$INSTDIR\LICENSE"
        Delete /REBOOTOK "$INSTDIR\README.md"
        Delete /REBOOTOK "$INSTDIR\Uninstall.exe"
        RMDir /r /REBOOTOK "$INSTDIR\resources"
        RMDir /REBOOTOK "$INSTDIR"
    deleteDone:

    ; ---- Best-effort legacy install dir cleanup ---------------------------
    ClearErrors
    IfFileExists "$PROGRAMFILES64\OpenVR-Pair" 0 skipLegacyInstDir
        RMDir /r "$PROGRAMFILES64\OpenVR-Pair"
    skipLegacyInstDir:

    ; ---- User data under %USERPROFILE%\AppData\LocalLow\ ------------------
    ; LocalLow is a SIBLING of Local, not a child, so $LOCALAPPDATA\Low is
    ; the wrong path -- it would resolve to .\AppData\Local\Low\ which never
    ; exists. $PROFILE is %USERPROFILE%, so $PROFILE\AppData\LocalLow lines
    ; up with what SHGetKnownFolderPath(FOLDERID_LocalAppDataLow) returns
    ; for the same user that runs the uninstaller.
    ClearErrors
    StrCpy $0 "$PROFILE\AppData\LocalLow\WKOpenVR"
    IfFileExists "$0" 0 skipUserData
        RMDir /r "$0"
    skipUserData:

    ; ---- Best-effort legacy user data (pre-rename product) ----------------
    ClearErrors
    StrCpy $0 "$PROFILE\AppData\LocalLow\OpenVR-Pair"
    IfFileExists "$0" 0 skipLegacyUserData
        RMDir /r "$0"
    skipLegacyUserData:

    ; ---- Start Menu shortcuts ---------------------------------------------
    ; RMDir /r catches every "WKOpenVR - <Feature>.lnk" the umbrella and
    ; per-feature installers may have dropped, without needing to spell
    ; each one out here.
    ClearErrors
    RMDir /r "$SMPROGRAMS\WKOpenVR"

    ; ---- Registry cleanup -------------------------------------------------
    ClearErrors
    DeleteRegKey HKLM "Software\WKOpenVR\Driver"
    DeleteRegKey HKLM "Software\WKOpenVR\Main"
    DeleteRegKey /ifempty HKLM "Software\WKOpenVR"
    DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\WKOpenVR"
    ; Legacy registry keys from the pre-rename product.
    DeleteRegKey HKLM "Software\OpenVR-Pair"
    ; The HKCU\Software\OpenVR-WKSpaceCalibrator key is what existed before
    ; the WKOpenVR rename; Migration.cpp copies it into the WKOpenVR-prefixed
    ; key on first launch (kNewKey there is "Software\WKOpenVR-SpaceCalibrator").
    ; Once the migration ran the old key may be empty but Migration leaves it
    ; in place; the new key is what every subsequent launch writes. Delete both.
    DeleteRegKey HKCU "Software\OpenVR-WKSpaceCalibrator"
    DeleteRegKey HKCU "Software\WKOpenVR-SpaceCalibrator"

    ClearErrors
    IfRebootFlag rebootPrompt uninstallDone
    rebootPrompt:
        MessageBox MB_YESNO|MB_ICONQUESTION "Some files were locked and have been queued for deletion on the next reboot.$\r$\nReboot now to finish the uninstall?" IDNO uninstallDone
        Reboot
    uninstallDone:
        MessageBox MB_OK|MB_ICONINFORMATION "WKOpenVR uninstalled."
SectionEnd
