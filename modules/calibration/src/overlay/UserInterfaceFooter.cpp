#include "UserInterfaceFooter.h"
#include "UserInterface.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "IPCClient.h"
#include "Protocol.h"
#include "BuildStamp.h"
#include "VRState.h"
#include "UiHelpers.h"

extern SCIPCClient Driver;
extern bool runningInOverlay;
extern bool s_inUmbrella;

namespace spacecal::ui {

// Render a small filled circle aligned with the current text baseline. Used as
// the connection-status dot in the version line. Drawn directly to the window
// draw list so we don't have to fiddle with widget sizing -- the caller manages
// SameLine/spacing.
void DrawStatusDot(ImU32 color, float radiusScale) {
	ImDrawList* dl = ImGui::GetWindowDrawList();
	const float h = ImGui::GetTextLineHeight();
	const float r = h * radiusScale;
	ImVec2 cursor = ImGui::GetCursorScreenPos();
	// Center vertically on the text line; nudge left/right so the dot sits in
	// its own little gutter rather than overlapping the next character.
	ImVec2 center(cursor.x + r + 2.0f, cursor.y + h * 0.5f);
	dl->AddCircleFilled(center, r, color);
	// Reserve the space the dot occupies so SameLine() spacing works.
	ImGui::Dummy(ImVec2(r * 2.0f + 4.0f, h));
	ImGui::SameLine();
}

void ShowVersionLine() {
	// Footer is two short rows:
	//   row 1: Hover for tooltips. Right-click sliders to reset.
	//   row 2: Driver: ...   |   Space Calibrator <build>   |   mode
	// The hover tip sits above so the more-prominent status row is
	// visually anchored to the bottom edge of the window.
	//
	// Two positioning modes:
	//   - Standalone: absolute-positioned at GetWindowHeight() - footerH so
	//     the footer floats above scrolled content.
	//   - Umbrella: rendered inline at the current cursor position. Caller
	//     (BuildMainWindowContents) is expected to have ended the scrollable
	//     content child before calling us, so the footer sits as a normal
	//     layout element below the scroll region rather than floating over
	//     it. Floating mode collided with the credits panel at the bottom
	//     of the Advanced tab.
	const float lineH  = ImGui::GetTextLineHeight();
	const float footerH = lineH * 2.0f + 12.0f;
	if (s_inUmbrella) {
		ImGui::Separator();
	} else {
		ImGui::SetNextWindowPos(ImVec2(10.0f, ImGui::GetWindowHeight() - footerH));
	}
	const float footerW = s_inUmbrella ? ImGui::GetContentRegionAvail().x
	                                   : (ImGui::GetWindowWidth() - 20.0f);
	if (!ImGui::BeginChild("bottom line", ImVec2(footerW, footerH), ImGuiChildFlags_None)) {
		ImGui::EndChild();
		return;
	}

	// --- Row 1: hover-for-tooltip hint ---
	ImGui::TextDisabled("Hover any setting for help. Right-click a slider to reset it.");

	// --- Row 2: driver status + version + mode ---
	// Driver-connection dot. Three states:
	//   green  - connected, everything's working.
	//   amber  - VR stack hasn't connected yet (SteamVR not running, or the
	//            program was launched standalone). Not an error; the main
	//            loop is retrying and will flip to green automatically.
	//   red    - established connection has dropped (IPC pipe broke after
	//            initial handshake). This is the case where reinstalling the
	//            driver is genuinely the right advice.
	const auto &pal = openvr_pair::overlay::ui::GetPalette();
	const bool driverConnected = Driver.IsConnected();
	if (driverConnected) {
		DrawStatusDot(pal.dotOk);
		ImGui::TextColored(pal.statusOk,
			"Driver: connected (v%u)", (unsigned)protocol::Version);
	} else if (!IsVRReady()) {
		DrawStatusDot(pal.dotPending);
		ImGui::TextColored(pal.statusPending,
			"Driver: waiting for SteamVR");
	} else {
		DrawStatusDot(pal.dotError);
		ImGui::TextColored(pal.statusError,
			"Driver: disconnected -- reinstall the SteamVR driver");
	}

	ImGui::SameLine();
	ImGui::Text("  |  WKOpenVR " SPACECAL_BUILD_STAMP);
	if (runningInOverlay)
	{
		ImGui::SameLine();
		ImGui::Text("- close VR overlay to use mouse");
	}

	// Mode label after the version. Plain text styled the same way as the
	// driver-status text -- we apply the per-mode accent colour but no
	// rounded-pill background. Tooltip on hover for the longer
	// explanation.
	{
		const char* modeLabel = nullptr;
		const char* modeTooltip = nullptr;
		ImVec4 modeAccent;
		GetModeStatus(modeLabel, modeTooltip, modeAccent);
		ImGui::SameLine();
		ImGui::TextColored(modeAccent, "  |  %s", modeLabel);
		if (modeTooltip && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("%s", modeTooltip);
		}
	}

	ImGui::EndChild();
}

// Compute the current calibration-state label + tooltip + accent colour.
// Used by the footer where it renders as plain text styled like the
// surrounding driver-status / version row, rather than a coloured pill.
void GetModeStatus(const char*& label, const char*& tooltip, ImVec4& accent) {
	const auto state = CalCtx.state;
	const bool validProfile = CalCtx.validProfile;
	const bool enabled = CalCtx.enabled;
	const auto &pal = openvr_pair::overlay::ui::GetPalette();

	if (!validProfile) {
		label = "no profile";
		tooltip = "No saved calibration profile is loaded.\n"
		          "Hit \"Start Calibration\" or \"Continuous Calibration\" below to create one.";
		accent = pal.statusIdle;
	} else if (state == CalibrationState::ContinuousStandby) {
		label = "standby -- waiting for tracking";
		tooltip = "Continuous calibration is on, but the reference or target tracker isn't currently\n"
		          "reporting valid poses. Calibration resumes automatically when both come back online.";
		accent = pal.statusWarn;
	} else if (state == CalibrationState::Continuous) {
		const double now = ImGui::GetTime();
		const double sinceAccept = now - Metrics::error_currentCal.lastTs();
		const bool searching = Metrics::consecutiveRejections.last() > 10.0;
		const bool recentlyUpdated = sinceAccept >= 0.0 && sinceAccept < 5.0;
		if (searching) {
			label = "live -- searching";
			tooltip = "Continuous calibration is running but hasn't accepted a new estimate in a while.\n"
			          "Usually means the user isn't moving enough to give the solver useful samples.\n"
			          "Try slowly rotating + translating the target tracker through varied directions.";
			accent = pal.statusPending;
		} else if (recentlyUpdated) {
			label = "live -- updating";
			tooltip = "Continuous calibration is running and just accepted a fresh estimate.\n"
			          "The driver is blending toward the new offset; if Recalibrate-on-movement is on,\n"
			          "the blend only progresses while the device is actively moving.";
			accent = pal.statusInfo;
		} else {
			label = "live";
			tooltip = "Continuous calibration is running and the current estimate is being applied.\n"
			          "No recent updates needed -- the calibration is stable.";
			accent = pal.statusInfo;
		}
	} else if (enabled && state == CalibrationState::None) {
		label = "fixed offset active";
		tooltip = "A one-shot calibration is applied as a fixed offset. The driver applies the\n"
		          "stored transform; no continuous re-solving. Switch to Continuous mode if the\n"
		          "offset drifts over time.";
		accent = pal.statusOk;
	} else {
		label = "idle";
		tooltip = "A profile is loaded but no calibration is being applied. This usually means the\n"
		          "current HMD tracking system doesn't match the profile's reference system.";
		accent = pal.statusIdle;
	}
}

} // namespace spacecal::ui
