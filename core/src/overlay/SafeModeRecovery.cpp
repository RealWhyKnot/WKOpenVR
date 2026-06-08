#include "SafeModeRecovery.h"

#include "DiagnosticsLog.h"
#include "ModuleRegistry.h"
#include "ModuleSafety.h"
#include "SteamVrControl.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <string>
#include <vector>

namespace openvr_pair::overlay {

namespace {
namespace svc = openvr_pair::common::steamvr_control;
namespace module_safety = openvr_pair::common::module_safety;
namespace modules = openvr_pair::common::modules;
using openvr_pair::common::DiagnosticLog;

constexpr unsigned kMaxRecoveryAttempts = 2;
constexpr long long kRecoveryWindowSeconds = 600;

const char* kManageAddonsHint = "Open SteamVR > Settings > Startup/Shutdown > Manage Add-Ons to re-enable add-ons.";

} // namespace

SafeModeRecoveryResult RunSafeModeRecoveryIfNeeded()
{
	SafeModeRecoveryResult result;

	const svc::SteamPaths paths = svc::ResolveSteamPaths();
	if (!paths.ok) return result;

	// Only act when SteamVR is actually running. A bare desktop launch (no VR)
	// must never stop/relaunch SteamVR off a stale log line.
	if (!svc::IsVrServerRunning()) return result;

	std::vector<std::string> blocked;
	if (!svc::ReadSafeModeBlockedDrivers(paths.vrServerLogPath, blocked)) return result;

	const bool weAreBlocked = std::find(blocked.begin(), blocked.end(), std::string("01wkopenvr")) != blocked.end();
	if (!weAreBlocked) return result;

	// Corroborate: if our driver IPC pipe answers, the driver actually loaded
	// (the parsed line was from an earlier, already-resolved session) -- bail.
	if (const char* pipe = modules::PipeName(modules::ModuleId::Calibration)) {
		if (WaitNamedPipeA(pipe, 100)) {
			DiagnosticLog("safe-mode-recovery", "driver pipe is live despite safe-mode log line; no recovery needed");
			return result;
		}
	}

	DiagnosticLog("safe-mode-recovery", "SteamVR safe mode is blocking WKOpenVR (%zu add-on(s) blocked total)",
	              blocked.size());

	// Attribute. Self-heal only when one of OUR modules left a stale suspect
	// marker (an uncontained crash mid-operation). Otherwise the crash was not
	// provably ours (another vendor's driver, SteamVR core) -- surface a notice
	// and take no destructive action, per the chosen behaviour.
	const std::vector<const module_safety::ModuleSpec*> culprits = svc::FindUncontainedCrashCulprits();
	if (culprits.empty()) {
		DiagnosticLog("safe-mode-recovery", "no WKOpenVR module implicated; leaving safe mode for the user to clear");
		result.surfaceNotice = true;
		result.noticeMessage =
		    std::string("SteamVR safe mode is active and WKOpenVR was not the cause. ") + kManageAddonsHint;
		return result;
	}

	// Loop guard: bound auto-recovery so a module that crashes on every load
	// cannot fork-bomb SteamVR restarts.
	svc::LoopGuardState prev;
	svc::ReadLoopGuardState(prev);
	const svc::LoopGuardDecision guard =
	    svc::EvaluateLoopGuard(prev, svc::NowEpochSeconds(), kMaxRecoveryAttempts, kRecoveryWindowSeconds);
	if (!guard.allowed) {
		DiagnosticLog("safe-mode-recovery", "loop guard tripped (%u attempts in window); not relaunching", prev.count);
		result.surfaceNotice = true;
		result.noticeMessage =
		    std::string("WKOpenVR stopped auto-recovering SteamVR after repeated attempts -- a module keeps crashing "
		                "on load. ") +
		    kManageAddonsHint;
		return result;
	}
	svc::WriteLoopGuardState(guard.next);

	// Keep the culprit module(s) disabled across the relaunch. MarkFault turns
	// the stale suspect into the auto-disabled marker the driver's feature gate
	// honours at next Init, so the offending module stays off while everything
	// else comes back.
	for (const module_safety::ModuleSpec* spec : culprits) {
		module_safety::MarkFault(*spec, "safe_mode_recovery");
		DiagnosticLog("safe-mode-recovery", "disabling attributed culprit module '%s'",
		              spec->display_name ? spec->display_name : (spec->slug ? spec->slug : "(unknown)"));
	}

	// Stop SteamVR first so the vrsettings edit is authoritative (vrserver
	// rewrites the file on shutdown/startup), re-enable every blocked add-on,
	// then relaunch.
	DiagnosticLog("safe-mode-recovery", "stopping SteamVR, clearing safe mode, relaunching");
	svc::StopVrServer();

	if (!svc::ClearSafeMode(paths, blocked)) {
		result.surfaceNotice = true;
		result.noticeMessage =
		    std::string("WKOpenVR could not clear SteamVR safe mode automatically. ") + kManageAddonsHint;
		return result;
	}

	if (!svc::LaunchSteamVr(paths)) {
		result.surfaceNotice = true;
		result.noticeMessage =
		    "WKOpenVR cleared SteamVR safe mode but could not relaunch SteamVR. Start SteamVR manually.";
		return result;
	}

	result.relaunchedSteamVr = true;
	return result;
}

} // namespace openvr_pair::overlay
