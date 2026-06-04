#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "DeviceFilters.h"
#include "VRState.h"
#include "CalibrationMetrics.h"
#include "IPCClient.h"
#include "Protocol.h"
#include "Version.h"
#include "BuildStamp.h"
#include "Wizard.h"
#include "UserInterfaceFooter.h"
#include "UserInterfaceBanners.h"
#include "UserInterfaceCalibrationProgress.h"
#include "HeadMountOffsetModal.h"
#include "UiHelpers.h"

#include <thread>
#include <string>
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <exception>
#include <shellapi.h>
#include <shlobj_core.h>
#include <imgui/imgui.h>
#include "imgui_extensions.h"

extern SCIPCClient Driver;

void TextWithWidth(const char* label, const char* text, float width);
void DrawVectorElement(const std::string id, const char* text, double* value, int defaultValue = 0,
                       const char* defaultValueStr = " 0 ");

VRState LoadVRState();
void BuildSystemSelection(const VRState& state);
void BuildDeviceSelections(const VRState& state);
void BuildProfileEditor();

static const ImGuiWindowFlags bareWindowFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                                ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                                                ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse;

void BuildContinuousCalDisplay();
static void BuildMainWindowContents(bool runningInOverlay_);
void BuildMenu(bool runningInOverlay);

// Forward decls for the tab content called from both modes. CCal_BasicInfo /
// CCal_DrawSettings were declared near the continuous-mode tab bar; the
// non-continuous flow needs them in scope at BuildMainWindow time too, so the
// decls live at file scope. Prediction and finger smoothing relocated to the
// Smoothing overlay (Protocol v12 migration, 2026-05-11).
void CCal_BasicInfo();
void CCal_DrawSettings();
void CCal_DrawBoundaryTab();
void CCal_TickBoundaryCapture();
static void OneShot_DrawSettings();

bool runningInOverlay;
bool s_inUmbrella = false;

void CCal_SetInUmbrella(bool inUmbrella)
{
	s_inUmbrella = inUmbrella;
}

// Shared head-mount config IPC. Previously inlined in BuildContinuousCalDisplay
// only; lifted here so the one-shot path can dispatch the same payload after
// the offset modal saves.
static void SendHeadMountConfigFromCalCtx()
{
	const auto& hm = CalCtx.headMount;
	protocol::Request req(protocol::RequestSetHeadMountConfig);
	auto& p = req.setHeadMountConfig;
	p.mode = static_cast<uint32_t>(hm.mode);
	p.deviceId = hm.deviceID;
	p.hideTracker = hm.hideTracker;
	p.offsetCalibrated = hm.offsetCalibrated;
	const auto timing = wkopenvr::headmount::ClampDriverSynthTimingConfig(hm.driverSynthTiming);
	p.driverSynthStaleLimitMs = static_cast<uint16_t>(timing.staleLimitMs);
	p.driverSynthGraceHoldMs = static_cast<uint16_t>(timing.graceHoldMs);
	p.driverSynthBlendToFallbackMs = static_cast<uint16_t>(timing.blendToFallbackMs);
	p.driverSynthStableBeforeSynthMs = static_cast<uint16_t>(timing.stableBeforeSynthMs);
	p.driverSynthBlendToSynthMs = static_cast<uint16_t>(timing.blendToSynthMs);
	{
		size_t len = hm.trackerSerial.size();
		if (len >= sizeof p.trackerSerial) len = sizeof p.trackerSerial - 1;
		memcpy(p.trackerSerial, hm.trackerSerial.data(), len);
		p.trackerSerial[len] = '\0';
	}
	{
		size_t len = hm.trackerTrackingSystem.size();
		if (len >= sizeof p.trackerTrackingSystem) len = sizeof p.trackerTrackingSystem - 1;
		memcpy(p.trackerTrackingSystem, hm.trackerTrackingSystem.data(), len);
		p.trackerTrackingSystem[len] = '\0';
	}
	Eigen::Quaterniond q(hm.headFromTracker.linear());
	q.normalize();
	const Eigen::Vector3d t = hm.headFromTracker.translation();
	p.headFromTrackerTrans[0] = t.x();
	p.headFromTrackerTrans[1] = t.y();
	p.headFromTrackerTrans[2] = t.z();
	p.headFromTrackerRot[0] = q.x();
	p.headFromTrackerRot[1] = q.y();
	p.headFromTrackerRot[2] = q.z();
	p.headFromTrackerRot[3] = q.w();
	try {
		Driver.SendBlocking(req);
	}
	catch (const std::exception& e) {
		char buf[240];
		snprintf(buf, sizeof buf, "[head-mount] config push failed: %s", e.what());
		Metrics::WriteLogAnnotation(buf);
	}
}

void BuildMainWindow(bool runningInOverlay_)
{
	auto& io = ImGui::GetIO();

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);

	// Scroll allowed on the main window: bareWindowFlags disables it (right
	// for the calibration-progress popup and the continuous-mode root,
	// where content fits by construction), but the main page mixes
	// device dropdowns + action buttons + a tab bar and can overflow on
	// short windows. Without this, content past the bottom edge gets
	// silently clipped (no scrollbar, no scroll-with-mouse), which is
	// what produced the "I can't scroll down" report.
	const ImGuiWindowFlags mainFlags =
	    bareWindowFlags & ~ImGuiWindowFlags_NoScrollbar & ~ImGuiWindowFlags_NoScrollWithMouse;
	if (!ImGui::Begin("SpaceCalibrator", nullptr, mainFlags)) {
		ImGui::End();
		return;
	}

	BuildMainWindowContents(runningInOverlay_);
	ImGui::End();
}

void CCal_DrawTab()
{
	BuildMainWindowContents(false);
}

static void BuildMainWindowContents(bool runningInOverlay_)
{
	runningInOverlay = runningInOverlay_;
	bool continuousCalibration =
	    CalCtx.state == CalibrationState::Continuous || CalCtx.state == CalibrationState::ContinuousStandby;

	ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImGui::GetStyleColorVec4(ImGuiCol_Button));

	// In umbrella mode the SC body lives inside an ImGui tab item. The
	// previous layout let SC content scroll on the umbrella's outer window
	// AND on an inner BeginChild in continuous mode -- that's the "two
	// scrollbars" report. Wrap the body in a single sized child so:
	//   - there's exactly one scrollable region inside this tab,
	//   - the ShowVersionLine footer below sits as a normal layout element
	//     (no absolute positioning), and so cannot overlap the credits
	//     panel at the bottom of the Advanced tab.
	// Standalone mode keeps the old behavior unchanged.
	const float lineH = ImGui::GetTextLineHeight();
	const float footerH = lineH * 2.0f + 12.0f;
	bool bodyChildOpen = false;
	if (s_inUmbrella) {
		const float availY = ImGui::GetContentRegionAvail().y;
		const float bodyH = (availY > footerH + 16.0f) ? (availY - footerH - 4.0f) : 0.0f;
		bodyChildOpen = ImGui::BeginChild("SCTabBody", ImVec2(0.0f, bodyH), ImGuiChildFlags_None);
		if (!bodyChildOpen) {
			ImGui::EndChild();
			spacecal::ui::ShowVersionLine();
			ImGui::PopStyleColor();
			return;
		}
	}

	// "Waiting for SteamVR" banner -- visible whenever the program is up
	// without a connected VR stack (e.g. user launched us before starting
	// SteamVR). Disappears the moment the connection lands. Renders after
	// the update banner because update checks work without VR; the VR
	// banner is the more transient state.
	spacecal::ui::DrawVRWaitingBanner();

	// Auto-recovery banner (audit UX #3). Sticky for 60 s after the auto-
	// recover fires so the user actually notices that their calibration was
	// just clobbered, with Undo + Dismiss buttons. Without this the only
	// signal was a single line in CalCtx.messages, swept on the next
	// messages.clear(), invisible on tabs other than Basic. The 2026-05-02
	// false-positive recoveries that destroyed working cals would have been
	// caught here -- the user could have hit Undo within seconds.
	{
		double recoveryAge = 0.0, recoveryDelta = 0.0;
		if (LastAutoRecoveryActive(recoveryAge, recoveryDelta)) {
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.4f, 1.0f));
			ImGui::TextWrapped(
			    "Auto-recovery cleared calibration %.0fs ago (~%.0f cm HMD jump). Recalibrating from scratch.",
			    recoveryAge, recoveryDelta * 100.0);
			ImGui::PopStyleColor();
			if (ImGui::SmallButton("Undo (restore prior calibration)")) {
				UndoLastAutoRecovery();
			}
			ImGui::SameLine();
			if (ImGui::SmallButton("Dismiss")) {
				DismissAutoRecoveryBanner();
			}
			ImGui::Separator();
		}
	}

	// First-run auto-open of the setup wizard. Defer until VR is ready --
	// the wizard's first step depends on enumerating tracking systems via
	// VRState::Load, which is empty without a live OpenVR connection. If
	// it auto-opened on a no-VR launch, the user would see an empty wizard
	// and have to dismiss it. With this gate, the wizard pops up the moment
	// VR comes online, which is what a first-time user would expect.
	{
		static bool s_firstRunChecked = false;
		if (!s_firstRunChecked && IsVRReady()) {
			s_firstRunChecked = true;
			if (!CalCtx.wizardCompleted && !spacecal::wizard::IsActive()) {
				spacecal::wizard::Open();
			}
		}
	}
	spacecal::wizard::Draw();

	if (continuousCalibration) {
		BuildContinuousCalDisplay();
	}
	else {
		// (Mode pill moved to the global footer; no longer takes a row at
		// the top of the main page.)

		auto state = LoadVRState();

		ImGui::BeginDisabled(CalCtx.state == CalibrationState::Continuous);
		BuildSystemSelection(state);
		BuildDeviceSelections(state);
		ImGui::EndDisabled();
		BuildMenu(runningInOverlay);

		// Non-continuous tabbed surface. Mirrors the depth of access
		// continuous-calibration users get -- the previous version showed
		// only the action buttons here, leaving Settings / Advanced /
		// Prediction / Logs reachable only after the user committed to
		// continuous mode. With this tab bar, a one-shot user can open
		// debug logs and tweak settings without ever clicking
		// "Continuous Calibration".
		//
		// Hidden during the in-progress calibration popup (state != None)
		// because the user is captured by the modal anyway and the tabs
		// would just clutter the background.
		if (CalCtx.state == CalibrationState::None) {
			ImGui::Spacing();
			if (ImGui::BeginTabBar("OneShotTabs", 0)) {
				if (ImGui::BeginTabItem("Settings")) {
					OneShot_DrawSettings();
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("Advanced")) {
					CCal_DrawSettings();
					ImGui::EndTabItem();
				}
				// Logs moved to the umbrella's global Logs tab.
				ImGui::EndTabBar();
			}
		}
	}

	if (s_inUmbrella && bodyChildOpen) {
		ImGui::EndChild();
	}

	spacecal::ui::ShowVersionLine();

	ImGui::PopStyleColor();
}

// DrawStatusDot, ShowVersionLine, GetModeStatus moved to UserInterfaceFooter.cpp

// ShowVersionLine, GetModeStatus bodies moved to UserInterfaceFooter.cpp

// FormatBytes and DrawVRWaitingBanner moved to UserInterfaceBanners.cpp

void BuildContinuousCalDisplay()
{
	if (!s_inUmbrella) {
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::SetNextWindowSize(ImGui::GetWindowSize());
		ImGui::SetNextWindowBgAlpha(1);
		if (!ImGui::Begin("Continuous Calibration", nullptr, bareWindowFlags & ~ImGuiWindowFlags_NoTitleBar)) {
			ImGui::End();
			return;
		}
	}

	// Standalone window draws an inner CCalDisplayFrame child sized against
	// GetWindowHeight() so the standalone window's footer has reserved
	// space below the tab bar. In umbrella mode the outer SCTabBody child
	// (BuildMainWindowContents) already reserves footer space, and the
	// caller will draw the footer afterwards -- a second nested child
	// here produces the double scrollbar + duplicate footer rendering.
	bool ccalChildOpen = true;
	if (!s_inUmbrella) {
		ImVec2 contentRegion;
		contentRegion.x = ImGui::GetWindowContentRegionWidth();
		contentRegion.y = ImGui::GetWindowHeight() - ImGui::GetFrameHeightWithSpacing() * 2.1f;

		ccalChildOpen = ImGui::BeginChild("CCalDisplayFrame", contentRegion, ImGuiChildFlags_None);
		if (!ccalChildOpen) {
			ImGui::EndChild();
			ImGui::End();
			return;
		}
	}

	// (Mode pill moved to the global footer alongside the driver-status dot;
	// no longer takes a row above the tab bar.)

	// Tab bar layout. User-facing categories:
	//   - Basic:     the currently-running calibration -- device status, action
	//                buttons, common settings.
	//   - Play Space: head-mounted tracker setup, safety boundary drawing,
	//                 and Quest Guardian controls in one room-scale flow.
	//   - Graphs:    live plots for users who want to watch the math.
	//   - Advanced:  technical knobs (speed, alignment thresholds,
	//                tuning); the only place to override AUTO defaults.
	//   - Logs:      debug-log CSV files for bug reports. Lives in the
	//                umbrella's global Logs tab now.
	if (ImGui::BeginTabBar("CCalTabs", 0)) {
		if (ImGui::BeginTabItem("Basic")) {
			CCal_BasicInfo();
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Play Space")) {
			CCal_DrawBoundaryTab();
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Graphs")) {
			ShowCalibrationDebug(2, 3);
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Advanced")) {
			CCal_DrawSettings();
			ImGui::EndTabItem();
		}

		// Logs moved to the umbrella's global Logs tab. The SC-specific panel
		// (file list, enable toggle, drift dump button, Explorer link)
		// surfaces there via SpaceCalibratorPlugin::DrawLogsSection.
		ImGui::EndTabBar();
	}

	// Head-mount offset modal. Must be called outside the tab-item scope (ImGui
	// modals are rendered at the top of the window stack, not inside the tab).
	// Returns true on the frame the user clicks Save; the shared IPC helper
	// then sends the updated config to the driver so it picks up the new
	// offset and calibrated flag.
	if (wkopenvr::headmount::DrawOffsetModal()) {
		SendHeadMountConfigFromCalCtx();
	}

	if (!s_inUmbrella) {
		ImGui::EndChild();
		spacecal::ui::ShowVersionLine();
		ImGui::End();
	}
	// Umbrella path: BuildMainWindowContents owns both the SCTabBody child
	// wrapper and the single ShowVersionLine call, so this function only
	// emits the tab bar's contents.
}

// ScaledDragFloat, AddResetContextMenu, CCal_DrawSettings moved to UserInterfaceTabsAdvanced.cpp

void DrawVectorElement(const std::string id, const char* text, double* value, int defaultValue,
                       const char* defaultValueStr)
{
	constexpr float CONTINUOUS_CALIBRATION_TRACKER_OFFSET_DELTA = 0.01f;

	ImGui::Text("%s", text);

	ImGui::SameLine();

	ImGui::PushID((id + text + "_btn_reset").c_str());
	if (ImGui::Button(defaultValueStr)) {
		*value *= defaultValue;
	}
	ImGui::PopID();
	ImGui::SameLine();
	if (ImGui::ArrowButton((id + text + "_decrease").c_str(), ImGuiDir_Down)) {
		*value -= CONTINUOUS_CALIBRATION_TRACKER_OFFSET_DELTA;
	}
	ImGui::SameLine();
	ImGui::PushItemWidth(100);
	ImGui::PushID((id + text + "_text_field").c_str());
	ImGui::InputDouble("##label", value, 0, 0, "%.2f");
	ImGui::PopID();
	ImGui::PopItemWidth();
	ImGui::SameLine();
	if (ImGui::ArrowButton((id + text + "_increase").c_str(), ImGuiDir_Up)) {
		*value += CONTINUOUS_CALIBRATION_TRACKER_OFFSET_DELTA;
	}
}

inline const char* GetPrettyTrackingSystemName(const std::string& value)
{
	// To comply with SteamVR branding guidelines (page 29), we rename devices under lighthouse tracking to SteamVR
	// Tracking.
	if (value == "lighthouse" || value == "aapvr") {
		return "SteamVR Tracking";
	}
	return value.c_str();
}

// CCal_DrawPredictionSuppression: relocated to the Smoothing overlay's
// SmoothingPrediction sub-tab as part of the Protocol v12 migration on
// 2026-05-11. Per-tracker pose-prediction smoothness is now owned by the
// Smoothing plugin and pushed via RequestSetDevicePrediction; SC no longer
// holds trackerSmoothness state or sends those fields inside
// RequestSetDeviceTransform.

// One-shot mode's Settings tab. Trimmed to what a one-shot user actually
// touches: outlier rejection, universe-shift correction, calibration speed,
// chaperone bounds, maintenance. Continuous-only knobs (Lock relative
// position, Recalibrate on movement, recalibration threshold, alignment
// thresholds) live on the continuous Basic /
// Advanced tabs where they actually do something.
//
// The reasoning for having both this AND CCal_BasicInfo's Common settings
// rather than one shared function: the surrounding contexts differ -- the
// continuous Basic tab has device-status table + Cancel/Restart/Pause action
// buttons above, this one is just the settings.
static void OneShot_DrawSettings()
{
	ImVec2 panelSize{ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x, 0};
	ImGui::BeginGroupPanel("Settings", panelSize);

	if (ImGui::BeginTable("##oneshot_settings_grid", 2,
	                      ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoBordersInBody)) {
		ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthFixed, 230.0f);
		ImGui::TableSetupColumn("##control", ImGuiTableColumnFlags_WidthStretch);

		// (Jitter threshold moved to the Advanced tab -- it's a rarely-touched
		// knob, surfaced there alongside the rest of the deeper math settings.)

		// (Lock relative position and Recalibrate on movement moved to the
		// continuous-cal Basic tab. Both knobs only behave during continuous
		// refinement -- Lock gates the relative-pose constraint inside
		// ComputeIncremental, and Recalibrate-on-movement gates per-frame
		// transform blending. Rendering them on the one-shot Settings tab
		// created the impression they affect the one-shot solve, which they
		// do not.)

		// (Hide tracker controls moved out of one-shot Settings. The
		// calibration-target hide lives on the Advanced tab (continuous-only).
		// The head-mount tracker hide lives on the Play Space tab.)

		// --- Ignore outliers ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Ignore outliers");
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Checkbox("##oneshot_ignore_outliers", &CalCtx.ignoreOutliers)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Drop sample pairs whose rotation axis disagrees with the consensus before\n"
			                  "the LS solve. Helps with intermittent USB glitches or brief tracking loss.");
		}

		// --- Base station drift correction (AUTO/OFF) ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Auto-correct universe shifts");
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Checkbox("##oneshot_base_station_drift", &CalCtx.baseStationDriftCorrectionEnabled)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("AUTO (on): when Lighthouse base stations are detected, watch for\n"
			                  "uniform pose shifts across all of them between ticks -- a SteamVR\n"
			                  "universe re-origin (chaperone reset, seated zero pose reset, etc.) --\n"
			                  "and apply the inverse to the stored calibration so body trackers stay\n"
			                  "aligned with your physical position. No-op if no base stations are\n"
			                  "present (Quest-only setups, etc.). Math is honest: requires actual\n"
			                  "evidence of a universe shift, not a heuristic guess.\n\n"
			                  "OFF: never adjust the calibration based on base station poses.");
		}

		// (Enable debug logs toggle removed -- it lives in the Logs tab now,
		// where the user can flip it on right where they're managing the
		// log files.)

		ImGui::EndTable();
	}

	ImGui::EndGroupPanel(); // Settings

	// Calibration speed -- moved here from above the tab bar (was being
	// rendered inline in BuildMenu, stacking the speed picker on top of the
	// device + action rows and pushing the tabs off-screen on small
	// windows). Lives in Settings rather than Advanced because it's a
	// common knob most users want one click away.
	ImGui::Spacing();
	ImGui::BeginGroupPanel("Calibration speed", panelSize);
	{
		// One-shot speed picker. AUTO is intentionally absent here: it only
		// re-evaluates meaningfully during continuous calibration. The
		// Advanced tab's continuous-mode panel exposes AUTO.
		auto speed = CalCtx.oneShotCalibrationSpeed;
		struct Opt
		{
			const char* label;
			CalibrationContext::Speed value;
			const char* tooltip;
		};
		const Opt opts[] = {
		    {"Fast", CalibrationContext::FAST,
		     "30 samples (~1.7 s buffer). Good for almost everyone. Pick this first."},
		    {"Slow", CalibrationContext::SLOW,
		     "100 samples (~5.5 s buffer). Try this if FAST keeps producing visibly\n"
		     "misaligned results on the same hardware."},
		    {"Very Slow", CalibrationContext::VERY_SLOW,
		     "200 samples (~11 s buffer). For IMU-based body trackers or very noisy\n"
		     "environments. Most people never need this."},
		};
		for (size_t i = 0; i < sizeof(opts) / sizeof(opts[0]); ++i) {
			if (i > 0) ImGui::SameLine();
			if (ImGui::RadioButton(opts[i].label, speed == opts[i].value)) {
				CalCtx.oneShotCalibrationSpeed = opts[i].value;
				SaveProfile(CalCtx);
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("%s", opts[i].tooltip);
			}
		}
	}
	ImGui::EndGroupPanel();

	// Chaperone -- moved here from BuildMenu for the same reason. Copy/Paste
	// + auto-apply checkbox; Paste only meaningful when the profile already
	// has stored bounds.
	spacecal::ui::DrawChaperoneLoadFailedBanner();
	ImGui::Spacing();
	ImGui::BeginGroupPanel("Chaperone bounds", panelSize);
	{
		ImGui::BeginDisabled(!IsVRReady());
		if (ImGui::Button("Copy chaperone bounds to profile")) {
			LoadChaperoneBounds();
			SaveProfile(CalCtx);
		}
		ImGui::EndDisabled();
		if (!IsVRReady() && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Waiting for SteamVR.");
		}
		if (CalCtx.chaperone.valid) {
			ImGui::SameLine();
			ImGui::BeginDisabled(!IsVRReady());
			if (ImGui::Button("Paste chaperone bounds")) {
				ApplyChaperoneBounds();
			}
			ImGui::EndDisabled();
			if (!IsVRReady() && ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Waiting for SteamVR.");
			}

			if (ImGui::Checkbox("Paste automatically when geometry resets", &CalCtx.chaperone.autoApply)) {
				SaveProfile(CalCtx);
			}
		}
		else {
			ImGui::TextDisabled("(No bounds saved in profile yet -- press Copy first.)");
		}
	}
	ImGui::EndGroupPanel();

	// (Recenter playspace removed: in the lighthouse-anchored boundary model
	// the chaperone is built from boundary vertices in lighthouse space and
	// shifting the SZP to the HMD's reported position would push the boundary
	// off the user's physical room.)

	// Wizard / reset actions, grouped in their own panel so they don't read
	// as floating buttons under the Settings table.
	ImGui::Spacing();
	ImGui::BeginGroupPanel("Maintenance", panelSize);
	if (ImGui::Button("Run setup wizard")) {
		spacecal::wizard::Open();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Re-run the first-run setup wizard. Useful after changing your hardware\n"
		                  "(adding/removing a tracking system) or if you want to start fresh.");
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset settings")) {
		CalCtx.ResetConfig();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Reset all settings (jitter / speed / lock / etc.) to defaults.\n"
		                  "Does NOT clear your calibrated profile -- only the tunables.");
	}
	ImGui::EndGroupPanel();
}

static std::string CalibrationBlockedMessage()
{
	if (IsVRReady()) return {};
	const std::string& vrError = LastVRConnectError();
	if (vrError.find("Space Calibrator driver unavailable") != std::string::npos ||
	    vrError.find("WKOpenVR-Calibration") != std::string::npos || vrError.find("Calibration") != std::string::npos) {
		return "Waiting for SteamVR to reload Space Calibrator. If you just enabled the module, restart SteamVR.";
	}
	if (vrError.find("interface version") != std::string::npos ||
	    vrError.find("VR_INTERFACE_VERSION") != std::string::npos) {
		return "OpenVR interface version mismatch. Update SteamVR or reinstall WKOpenVR.";
	}
	return "Waiting for SteamVR. Calibration controls enable when tracking is live.";
}

void BuildMenu(bool runningInOverlay)
{
	auto& io = ImGui::GetIO();
	ImGuiStyle& style = ImGui::GetStyle();
	ImGui::Text("");

	if (CalCtx.state == CalibrationState::None) {
		// (Profile-mismatch banner moved below the action buttons -- it used
		// to push the Start / Continuous / Edit / Clear row down with a
		// multi-line warning. The buttons are what the user is actually
		// reaching for; surface them first.)

		float width = ImGui::GetWindowContentRegionWidth(), scale = 1.0f;
		if (CalCtx.validProfile) {
			width -= style.FramePadding.x * 4.0f;
			scale = 1.0f / 4.0f;
		}

		// Start / Continuous Calibration both need a live VR stack to enumerate
		// devices and collect samples. Edit / Clear are pure-memory operations
		// on the saved profile and stay enabled even without SteamVR running.
		const std::string blockedMessage = CalibrationBlockedMessage();
		ImGui::BeginDisabled(!IsVRReady());
		if (ImGui::Button("Start Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
			ImGui::OpenPopup("Calibration Progress");
			StartCalibration("ui_start_button");
		}
		ImGui::EndDisabled();
		if (!IsVRReady() && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("%s", blockedMessage.c_str());
		}

		ImGui::SameLine();
		ImGui::BeginDisabled(!IsVRReady());
		if (ImGui::Button("Continuous Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
			StartContinuousCalibration("ui_continuous_button");
		}
		ImGui::EndDisabled();
		if (!IsVRReady() && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("%s", blockedMessage.c_str());
		}

		if (CalCtx.validProfile) {
			ImGui::SameLine();
			if (ImGui::Button("Edit Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
				CalCtx.state = CalibrationState::Editing;
			}

			ImGui::SameLine();
			if (ImGui::Button("Clear Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
				CalCtx.Clear();
				SaveProfile(CalCtx);
			}
		}
		if (!blockedMessage.empty()) {
			ImGui::Spacing();
			ImGui::TextDisabled("%s", blockedMessage.c_str());
		}

		// Profile-mismatch banner (relocated): renders only when the saved
		// profile expects a different HMD tracking system than the current
		// HMD reports. Sits below the action buttons so the buttons stay
		// at the top -- the banner is informative + recovery actions
		// (Clear profile / Recalibrate), not a blocking modal.
		if (CalCtx.validProfile && !CalCtx.enabled) {
			const char* refSystem = GetPrettyTrackingSystemName(CalCtx.referenceTrackingSystem);
			std::string actualSystem;
			if (auto vrSystem = vr::VRSystem()) {
				char buffer[vr::k_unMaxPropertyStringSize] = {0};
				vr::ETrackedPropertyError err = vr::TrackedProp_Success;
				vrSystem->GetStringTrackedDeviceProperty(
				    vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String, buffer, sizeof buffer, &err);
				if (err == vr::TrackedProp_Success && buffer[0] != 0) {
					actualSystem = GetPrettyTrackingSystemName(buffer);
				}
			}
			ImGui::Spacing();
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.55f, 0.45f, 1.0f));
			if (!actualSystem.empty()) {
				ImGui::TextWrapped("Profile expects %s HMD but current HMD is on %s. Calibration not applied.",
				                   refSystem, actualSystem.c_str());
			}
			else {
				ImGui::TextWrapped("Profile expects %s HMD but current HMD is unavailable or on a different tracking "
				                   "system. Calibration not applied.",
				                   refSystem);
			}
			ImGui::PopStyleColor();
			if (ImGui::Button("Clear profile")) {
				CalCtx.Clear();
				SaveProfile(CalCtx);
			}
			ImGui::SameLine();
			ImGui::BeginDisabled(!IsVRReady());
			if (ImGui::Button("Recalibrate")) {
				ImGui::OpenPopup("Calibration Progress");
				StartCalibration("ui_recalibrate_button");
			}
			ImGui::EndDisabled();
			if (!IsVRReady() && ImGui::IsItemHovered()) {
				ImGui::SetTooltip("%s", blockedMessage.c_str());
			}
		}

		// (Chaperone Copy/Paste buttons + autoApply moved to the Settings
		// tab as the "Chaperone bounds" group panel.)
		// (Calibration Speed picker moved to the Settings tab as the
		// "Calibration speed" group panel.)
		// Both used to render inline here, stacking on top of the action
		// buttons and pushing the tab bar off-screen on small windows.
	}
	else if (CalCtx.state == CalibrationState::Editing) {
		BuildProfileEditor();

		if (ImGui::Button("Save Profile",
		                  ImVec2(ImGui::GetWindowContentRegionWidth(), ImGui::GetTextLineHeight() * 2))) {
			SaveProfile(CalCtx);
			CalCtx.state = CalibrationState::None;
		}
	}
	else {
		ImGui::Button("Calibration in progress...",
		              ImVec2(ImGui::GetWindowContentRegionWidth(), ImGui::GetTextLineHeight() * 2));
	}

	spacecal::ui::DrawCalibrationProgressPopup(io.DisplaySize, bareWindowFlags);
}

void BuildSystemSelection(const VRState& state)
{
	if (state.trackingSystems.empty()) {
		ImGui::Text("No tracked devices are present");
		return;
	}

	ImGuiStyle& style = ImGui::GetStyle();
	float paneWidth = ImGui::GetWindowContentRegionWidth() / 2 - style.FramePadding.x;

	TextWithWidth("ReferenceSystemLabel", "Reference Space", paneWidth);
	ImGui::SameLine();
	TextWithWidth("TargetSystemLabel", "Target Space", paneWidth);

	int currentReferenceSystem = -1;
	int currentTargetSystem = -1;
	int firstReferenceSystemNotTargetSystem = -1;

	std::vector<const char*> referenceSystems;
	std::vector<const char*> referenceSystemsUi;
	for (const std::string& str : state.trackingSystems) {
		if (str == CalCtx.referenceTrackingSystem) {
			currentReferenceSystem = (int)referenceSystems.size();
		}
		else if (firstReferenceSystemNotTargetSystem == -1 && str != CalCtx.targetTrackingSystem) {
			firstReferenceSystemNotTargetSystem = (int)referenceSystems.size();
		}
		referenceSystems.push_back(str.c_str());
		referenceSystemsUi.push_back(GetPrettyTrackingSystemName(str));
	}

	if (currentReferenceSystem == -1 && CalCtx.referenceTrackingSystem == "") {
		if (CalCtx.state == CalibrationState::ContinuousStandby) {
			auto iter = std::find(state.trackingSystems.begin(), state.trackingSystems.end(),
			                      CalCtx.referenceStandby.trackingSystem);
			if (iter != state.trackingSystems.end()) {
				currentReferenceSystem = (int)(iter - state.trackingSystems.begin());
			}
		}
		else {
			currentReferenceSystem = firstReferenceSystemNotTargetSystem;
		}
	}

	ImGui::PushItemWidth(paneWidth);
	ImGui::Combo("##ReferenceTrackingSystem", &currentReferenceSystem, &referenceSystemsUi[0],
	             (int)referenceSystemsUi.size());

	if (currentReferenceSystem != -1 && currentReferenceSystem < (int)referenceSystems.size()) {
		CalCtx.referenceTrackingSystem = std::string(referenceSystems[currentReferenceSystem]);
		if (CalCtx.referenceTrackingSystem == CalCtx.targetTrackingSystem) CalCtx.targetTrackingSystem = "";
	}

	if (CalCtx.targetTrackingSystem == "") {
		if (CalCtx.state == CalibrationState::ContinuousStandby) {
			auto iter = std::find(state.trackingSystems.begin(), state.trackingSystems.end(),
			                      CalCtx.targetStandby.trackingSystem);
			if (iter != state.trackingSystems.end()) {
				currentTargetSystem = (int)(iter - state.trackingSystems.begin());
			}
		}
		else {
			currentTargetSystem = 0;
		}
	}

	std::vector<const char*> targetSystems;
	std::vector<const char*> targetSystemsUi;
	for (const std::string& str : state.trackingSystems) {
		if (str != CalCtx.referenceTrackingSystem) {
			if (str != "" && str == CalCtx.targetTrackingSystem) currentTargetSystem = (int)targetSystems.size();
			targetSystems.push_back(str.c_str());
			targetSystemsUi.push_back(GetPrettyTrackingSystemName(str));
		}
	}

	ImGui::SameLine();
	if (targetSystemsUi.empty()) {
		int unavailable = 0;
		const char* items[] = {"(no target space)"};
		ImGui::BeginDisabled();
		ImGui::Combo("##TargetTrackingSystem", &unavailable, items, 1);
		ImGui::EndDisabled();
		CalCtx.targetTrackingSystem = "";
	}
	else {
		ImGui::Combo("##TargetTrackingSystem", &currentTargetSystem, &targetSystemsUi[0], (int)targetSystemsUi.size());
	}

	if (currentTargetSystem != -1 && currentTargetSystem < targetSystems.size()) {
		CalCtx.targetTrackingSystem = std::string(targetSystems[currentTargetSystem]);
	}

	ImGui::PopItemWidth();
}

void AppendSeparated(std::string& buffer, const std::string& suffix)
{
	if (!buffer.empty()) buffer += " | ";
	buffer += suffix;
}

std::string LabelString(const VRDevice& device)
{
	std::string label;

	/*if (device.controllerRole == vr::TrackedControllerRole_LeftHand)
	    label = "Left Controller";
	else if (device.controllerRole == vr::TrackedControllerRole_RightHand)
	    label = "Right Controller";
	else if (device.deviceClass == vr::TrackedDeviceClass_Controller)
	    label = "Controller";
	else if (device.deviceClass == vr::TrackedDeviceClass_HMD)
	    label = "HMD";
	else if (device.deviceClass == vr::TrackedDeviceClass_GenericTracker)
	    label = "Tracker";*/

	AppendSeparated(label, device.model);
	AppendSeparated(label, device.serial);
	return label;
}

std::string LabelString(const StandbyDevice& device)
{
	std::string label("< ");

	label += device.model;
	AppendSeparated(label, device.serial);

	label += " >";
	return label;
}

void BuildDeviceSelection(const VRState& state, int& initialSelected, const std::string& system,
                          StandbyDevice& standbyDevice)
{
	int selected = initialSelected;
	ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), "Devices from: %s", GetPrettyTrackingSystemName(system));

	if (selected != -1) {
		bool matched = false;
		for (auto& device : state.devices) {
			if (device.trackingSystem != system) continue;

			if (selected == device.id) {
				matched = true;
				break;
			}
		}

		if (!matched) {
			// Device is no longer present.
			selected = -1;
		}
	}

	bool standby = CalCtx.state == CalibrationState::ContinuousStandby;

	if (selected == -1 && !standby) {
		for (auto& device : state.devices) {
			if (device.trackingSystem != system) continue;

			if (device.controllerRole == vr::TrackedControllerRole_LeftHand) {
				selected = device.id;
				break;
			}
		}

		if (selected == -1) {
			for (auto& device : state.devices) {
				if (device.trackingSystem != system) continue;

				selected = device.id;
				break;
			}
		}
	}

	uint64_t iterator = 0;
	if (selected == -1 && standby &&
	    !openvr_pair::overlay::IsInternalAuxiliaryTrackedDevice(standbyDevice.serial, standbyDevice.model)) {
		bool present = false;
		for (auto& device : state.devices) {
			if (device.trackingSystem != system) continue;

			if (standbyDevice.model != device.model) continue;
			if (standbyDevice.serial != device.serial) continue;

			present = true;
			break;
		}

		if (!present) {
			auto label = LabelString(standbyDevice);
			std::string uniqueId = label + "_pass0_" + std::to_string(iterator);
			iterator++;
			ImGui::PushID(uniqueId.c_str());
			ImGui::Selectable(label.c_str(), true);
			ImGui::PopID();
		}
	}

	iterator = 0;

	for (auto& device : state.devices) {
		if (device.trackingSystem != system) continue;

		auto label = LabelString(device);
		std::string uniqueId = label + "_pass1_" + std::to_string(iterator);
		iterator++;
		ImGui::PushID(uniqueId.c_str());
		if (ImGui::Selectable(label.c_str(), selected == device.id)) {
			selected = device.id;
		}
		ImGui::PopID();
	}
	if (selected != initialSelected) {
		const auto& device =
		    std::find_if(state.devices.begin(), state.devices.end(), [&](const auto& d) { return d.id == selected; });
		if (device == state.devices.end()) return;

		initialSelected = selected;
		standbyDevice.trackingSystem = system;
		standbyDevice.model = device->model;
		standbyDevice.serial = device->serial;
	}
}

void BuildDeviceSelections(const VRState& state)
{
	ImGuiStyle& style = ImGui::GetStyle();
	ImVec2 paneSize(ImGui::GetWindowContentRegionWidth() / 2 - style.FramePadding.x,
	                ImGui::GetTextLineHeightWithSpacing() * 5 + style.ItemSpacing.y * 4);

	ImGui::BeginChild("left device pane", paneSize, ImGuiChildFlags_Borders);
	BuildDeviceSelection(state, CalCtx.referenceID, CalCtx.referenceTrackingSystem, CalCtx.referenceStandby);
	ImGui::EndChild();

	ImGui::SameLine();

	ImGui::BeginChild("right device pane", paneSize, ImGuiChildFlags_Borders);
	BuildDeviceSelection(state, CalCtx.targetID, CalCtx.targetTrackingSystem, CalCtx.targetStandby);
	ImGui::EndChild();

	if (ImGui::Button("Identify selected devices (blinks LED or vibrates)",
	                  ImVec2(ImGui::GetWindowContentRegionWidth(), ImGui::GetTextLineHeightWithSpacing() + 4.0f))) {
		// Guard: TriggerHapticPulse with an invalid device index is undefined
		// behaviour (driver crash or silent no-op depending on the runtime).
		// Skip the entire loop if either ID hasn't been assigned yet.
		if (CalCtx.targetID != vr::k_unTrackedDeviceIndexInvalid &&
		    CalCtx.referenceID != vr::k_unTrackedDeviceIndexInvalid) {
			for (unsigned i = 0; i < 100; ++i) {
				vr::VRSystem()->TriggerHapticPulse(CalCtx.targetID, 0, 2000);
				vr::VRSystem()->TriggerHapticPulse(CalCtx.referenceID, 0, 2000);
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
		}
	}
}

VRState LoadVRState()
{
	VRState state = VRState::Load();
	auto& trackingSystems = state.trackingSystems;

	// Inject entries for continuous calibration targets which have yet to load

	if (CalCtx.state == CalibrationState::ContinuousStandby) {
		const bool referenceInternal = openvr_pair::overlay::IsInternalAuxiliaryTrackedDevice(
		    CalCtx.referenceStandby.serial, CalCtx.referenceStandby.model);
		const bool targetInternal = openvr_pair::overlay::IsInternalAuxiliaryTrackedDevice(CalCtx.targetStandby.serial,
		                                                                                   CalCtx.targetStandby.model);

		auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), CalCtx.referenceTrackingSystem);
		if (!referenceInternal && !CalCtx.referenceTrackingSystem.empty() && existing == trackingSystems.end()) {
			trackingSystems.push_back(CalCtx.referenceTrackingSystem);
		}

		existing = std::find(trackingSystems.begin(), trackingSystems.end(), CalCtx.targetTrackingSystem);
		if (!targetInternal && !CalCtx.targetTrackingSystem.empty() && existing == trackingSystems.end()) {
			trackingSystems.push_back(CalCtx.targetTrackingSystem);
		}
	}

	return state;
}

void BuildProfileEditor()
{
	ImGuiStyle& style = ImGui::GetStyle();
	float width = ImGui::GetWindowContentRegionWidth() / 3.0f - style.FramePadding.x;
	float widthF = width - style.FramePadding.x;

	TextWithWidth("YawLabel", "Yaw", width);
	ImGui::SameLine();
	TextWithWidth("PitchLabel", "Pitch", width);
	ImGui::SameLine();
	TextWithWidth("RollLabel", "Roll", width);

	ImGui::PushItemWidth(widthF);
	ImGui::InputDouble("##Yaw", &CalCtx.calibratedRotation(1), 0.1, 1.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Pitch", &CalCtx.calibratedRotation(2), 0.1, 1.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Roll", &CalCtx.calibratedRotation(0), 0.1, 1.0, "%.8f");

	TextWithWidth("XLabel", "X", width);
	ImGui::SameLine();
	TextWithWidth("YLabel", "Y", width);
	ImGui::SameLine();
	TextWithWidth("ZLabel", "Z", width);

	ImGui::InputDouble("##X", &CalCtx.calibratedTranslation(0), 1.0, 10.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Y", &CalCtx.calibratedTranslation(1), 1.0, 10.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Z", &CalCtx.calibratedTranslation(2), 1.0, 10.0, "%.8f");

	TextWithWidth("ScaleLabel", "Scale", width);

	ImGui::InputDouble("##Scale", &CalCtx.calibratedScale, 0.0001, 0.01, "%.8f");
	ImGui::PopItemWidth();
}

void TextWithWidth(const char* label, const char* text, float width)
{
	ImGui::BeginChild(label, ImVec2(width, ImGui::GetTextLineHeightWithSpacing()));
	ImGui::Text("%s", text);
	ImGui::EndChild();
}

CCalPresenceSnapshot CCal_GetPresenceSnapshot()
{
	CCalPresenceSnapshot snap{};
	snap.state = static_cast<int>(CalCtx.state);
	snap.validProfile = CalCtx.validProfile;
	snap.referencePoseOk = CalCtx.ReferencePoseIsValidSimple();
	snap.targetPoseOk = CalCtx.TargetPoseIsValidSimple();
	snap.sampleProgress = 0;
	snap.sampleTarget = 1;
	// Pull progress from the most recent Progress message, if any.
	for (const auto& msg : CalCtx.messages) {
		if (msg.type == CalibrationContext::Message::Progress) {
			snap.sampleProgress = msg.progress;
			snap.sampleTarget = msg.target > 0 ? msg.target : 1;
		}
	}
	snap.targetTrackingSystem = CalCtx.targetTrackingSystem;
	return snap;
}

// SendFingerSmoothingConfig + CCal_DrawFingerSmoothing relocated to the
// Smoothing overlay's SmoothingFingers sub-tab on 2026-05-11 (Protocol v12
// migration). The Smoothing plugin owns the on-disk config and the
// RequestSetFingerSmoothing IPC send now; SC no longer carries finger state
// in CalibrationContext.
