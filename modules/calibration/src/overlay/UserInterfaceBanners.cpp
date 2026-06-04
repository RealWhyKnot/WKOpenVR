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

// Warning banner shown when the chaperone geometry in the saved profile has a
// corrupt size (not a multiple of 12). Auto-apply is disabled when this fires.
// Dismissed implicitly once the user saves a new chaperone via "Copy bounds"
// (g_chaperoneGeometrySizeMismatch stays true for the lifetime of the process
// but chaperone.valid becomes true after the copy, so we hide the banner then).
void DrawChaperoneLoadFailedBanner()
{
	if (!g_chaperoneGeometrySizeMismatch) return;
	// Once the user copies fresh bounds the problem is resolved in-session.
	if (CalCtx.chaperone.valid) return;

	const auto& palChap = openvr_pair::overlay::ui::GetPalette();
	ImGui::PushStyleColor(ImGuiCol_ChildBg, palChap.bannerErrorBg);
	ImGui::PushStyleColor(ImGuiCol_Border, palChap.bannerErrorTitle);
	ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 6.0f));

	const float bannerHeight = ImGui::GetFrameHeightWithSpacing() * 2.0f;
	if (ImGui::BeginChild("ChaperoneFailBanner", ImVec2(ImGui::GetContentRegionAvail().x, bannerHeight),
	                      ImGuiChildFlags_Border)) {
		ImGui::TextColored(palChap.bannerErrorTitle,
		                   "Saved chaperone could not be loaded (corrupted size). Auto-apply is disabled.");
		ImGui::TextColored(palChap.bannerErrorDetail,
		                   "Press \"Copy chaperone bounds to profile\" to save a new one and restore auto-apply.");
	}
	ImGui::EndChild();

	ImGui::PopStyleVar(2);
	ImGui::PopStyleColor(2);

	ImGui::Spacing();
}

} // namespace spacecal::ui
