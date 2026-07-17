#include "UserInterfaceBanners.h"
#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "VRState.h"
#include "BuildStamp.h"
#include "UiHelpers.h"

#include <string>
#include <shellapi.h>
#include <imgui/imgui.h>

extern bool s_inUmbrella;

namespace spacecal::ui {

// One-line banner that announces the VR stack isn't connected yet. Clears
// automatically the moment SteamVR starts (the main loop retries the
// connection once per second). Kept compact because the user already
// knows what state they're in -- they launched the calibrator before
// starting SteamVR. No need to dump verbose error strings.
//
// Interface-version mismatch is surfaced as a distinct error: it requires
// user action (update SteamVR or the overlay), not just patience.
void DrawVRWaitingBanner()
{
	if (IsVRReady()) return;
	// In umbrella mode the same status is already in the footer
	// (Driver: waiting for SteamVR). Don't duplicate it at the top of
	// the calibration tab where it pushes content down.
	if (s_inUmbrella) return;

	const std::string& vrError = LastVRConnectError();

	// Detect interface version mismatch: the error string set by TryInitVRStack
	// contains "interface version" when the runtime DLL doesn't match build headers.
	const bool isMismatch = vrError.find("interface version") != std::string::npos ||
	                        vrError.find("VR_INTERFACE_VERSION") != std::string::npos;

	const bool hasDetails = !vrError.empty();

	if (isMismatch) {
		const std::string detail = "Update SteamVR or reinstall this overlay to resolve. Details: " + vrError;
		openvr_pair::overlay::ui::DrawErrorBanner("OpenVR interface version mismatch", detail.c_str());
	}
	else if (hasDetails) {
		const std::string detail = "Details: " + vrError;
		openvr_pair::overlay::ui::DrawErrorBanner(
		    "SteamVR connection failed -- calibration controls enable when tracking is live.", detail.c_str());
	}
	else {
		openvr_pair::overlay::ui::DrawWaitingBanner(
		    "Waiting for SteamVR -- calibration controls enable when tracking is live.");
	}

	ImGui::Spacing();
}

} // namespace spacecal::ui
