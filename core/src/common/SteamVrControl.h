#pragma once

#include "ModuleSafety.h"

#include <string>
#include <vector>

// Helpers for inspecting and steering a local SteamVR install from the WKOpenVR
// overlay: discovering SteamVR's config/log/launch paths, detecting and clearing
// the "safe mode" add-on lockout that SteamVR enters after a driver crash, and
// stopping / relaunching the runtime. The pure parsing + transform functions
// (ParseVrServerSafeModeBlock, ClearSafeModeInVrSettingsJson, EvaluateLoopGuard,
// attribution) carry no Win32 state so they are unit-tested directly.
namespace openvr_pair::common::steamvr_control {

struct SteamPaths
{
	bool ok = false;
	std::wstring vrSettingsPath;  // <steam>\config\steamvr.vrsettings
	std::wstring vrServerLogPath; // <steam>\logs\vrserver.txt
	std::wstring vrStartupExe;    // <runtime>\bin\win64\vrstartup.exe
};

// Resolve SteamVR paths from %LOCALAPPDATA%\openvr\openvrpaths.vrpath (the
// canonical OpenVR registration), falling back to the conventional default
// install location if that file is missing or unparseable. `ok` is false only
// when no plausible vrsettings path could be formed.
SteamPaths ResolveSteamPaths();

// Pure. Scans vrserver.txt text for the safe-mode lockout signature. Returns
// true when the log shows SteamVR started in safe mode ("Using safe mode") and
// fills `blocked` with the driver names from each
//   "Not loading driver <X> because it was blocked by a previous safe mode event"
// line. Order-preserving, de-duplicated.
bool ParseVrServerSafeModeBlock(const std::string& logText, std::vector<std::string>& blocked);

// Reads the log file at `vrServerLogPath` (tail-capped) and parses it.
bool ReadSafeModeBlockedDrivers(const std::wstring& vrServerLogPath, std::vector<std::string>& blocked);

// Pure. Rewrites a steamvr.vrsettings JSON document so safe mode is cleared:
// sets steamvr.enableSafeMode=false and driver_<name>.blocked_by_safe_mode=false
// for every name in `driverNames`, preserving all other content. Returns false
// if `inputJson` is not a JSON object.
bool ClearSafeModeInVrSettingsJson(const std::string& inputJson, const std::vector<std::string>& driverNames,
                                   std::string& outputJson);

// Copies steamvr.vrsettings to a timestamped sibling backup
// (steamvr.vrsettings.wkopenvr-safe-mode-YYYYMMDD-HHMMSS.bak). Returns the
// backup path, or empty on failure.
std::wstring BackupVrSettings(const std::wstring& vrSettingsPath);

// File-level clear: backup, read, transform, write. Caller must stop vrserver
// first so the edit is authoritative. Returns true on a successful write.
bool ClearSafeMode(const SteamPaths& paths, const std::vector<std::string>& driverNames);

// True if a vrserver.exe process is currently running (SteamVR is up).
bool IsVrServerRunning();

// Terminates the SteamVR process tree (dashboard, compositor, monitor, server).
void StopVrServer();

// Launches SteamVR via vrstartup.exe. Returns true if the process was created.
bool LaunchSteamVr(const SteamPaths& paths);

// --- Crash attribution -------------------------------------------------------

// Returns the WKOpenVR modules implicated in an uncontained crash: those left
// with a stale ModuleSafety *suspect* marker (a module mid-critical-operation
// when the process died -- contained C++ exceptions clear their suspect marker
// during unwind, so a surviving suspect marker is strong evidence the module
// crashed the host). Active-only or already-auto-disabled markers are NOT
// treated as culprits, so a crash elsewhere (another vendor's driver, SteamVR
// core) does not get blamed on WKOpenVR. Read-only: does not mutate markers.
std::vector<const module_safety::ModuleSpec*> FindUncontainedCrashCulprits();

// --- Recovery loop guard -----------------------------------------------------

struct LoopGuardState
{
	unsigned count = 0;
	long long windowStartEpoch = 0;
};

struct LoopGuardDecision
{
	bool allowed = false;
	LoopGuardState next;
};

// Pure. Given the prior guard state and the current epoch (seconds), decide
// whether another auto-recovery attempt is allowed within the window. A new
// window starts when the prior window has elapsed; within a window, attempts
// are capped at `maxAttempts`.
LoopGuardDecision EvaluateLoopGuard(LoopGuardState prev, long long nowEpoch, unsigned maxAttempts,
                                    long long windowSeconds);

// File-backed guard state under the ModuleSafety root (recovery_attempts.state).
bool ReadLoopGuardState(LoopGuardState& out);
bool WriteLoopGuardState(const LoopGuardState& state);

// Current wall-clock epoch in seconds.
long long NowEpochSeconds();

} // namespace openvr_pair::common::steamvr_control
