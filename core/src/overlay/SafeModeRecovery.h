#pragma once

#include <string>

namespace openvr_pair::overlay {

// Outcome of the post-crash safe-mode self-heal check run once at overlay
// startup.
struct SafeModeRecoveryResult
{
	// Recovery cleared SteamVR safe mode and relaunched the runtime. The overlay
	// should exit cleanly: the freshly started SteamVR auto-launches a new
	// overlay against a healthy session.
	bool relaunchedSteamVr = false;

	// A user-facing notice should be shown (e.g. safe mode was not caused by
	// WKOpenVR, or the loop guard stopped repeated auto-recovery).
	bool surfaceNotice = false;
	std::string noticeMessage;
};

// If SteamVR is running but safe mode has blocked the WKOpenVR driver, and the
// prior crash is attributable to one of WKOpenVR's own modules, disable just
// that module, re-enable the blocked add-ons, and relaunch SteamVR. When the
// crash is not attributable to WKOpenVR (or the loop guard has tripped), do
// nothing destructive and surface a notice instead. A no-op on a normal launch.
SafeModeRecoveryResult RunSafeModeRecoveryIfNeeded();

} // namespace openvr_pair::overlay
