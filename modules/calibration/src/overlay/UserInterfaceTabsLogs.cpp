#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "BuildChannel.h"
#include "Configuration.h"
#include "DebugLogging.h"
#include "UiControls.h"
#include "UserInterfaceHeadMount.h"
#include "UserInterfaceLogFiles.h"
#include "Win32Text.h"
#if WKOPENVR_BUILD_IS_DEV
#include "DevFakeDevices.h"
#endif

#include <string>
#include <shellapi.h>
#include <imgui/imgui.h>

#if WKOPENVR_BUILD_IS_DEV
namespace {

bool DrawDriverSynthTimingControl(const char* label,
	int& value,
	int minValue,
	int maxValue,
	const char* tooltip)
{
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(0);
	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted(label);
	ImGui::TableSetColumnIndex(1);
	ImGui::PushItemWidth(-1.0f);
	const bool changed = ImGui::SliderInt("##value", &value, minValue, maxValue, "%d ms");
	ImGui::PopItemWidth();
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("%s", tooltip);
	}
	return changed;
}

} // namespace
#endif

// Release builds ship user-controlled debug logs and bug-report support.
// Replay CSV capture and simulated devices are dev-build-only surfaces.
void CCal_DrawLogsPanel() {
	auto& state = spacecal::ui_logs::LogsState();

	ImGui::TextWrapped(
		"Debug logging writes WKOpenVR diagnostics and SpaceCal annotations to "
		"%%LocalAppDataLow%%\\WKOpenVR\\Logs. Turn it on before reproducing an issue, "
		"then use Report bug.");
	ImGui::Spacing();

	const bool isDevBuild = openvr_pair::common::IsDebugLoggingForcedOn();
	Metrics::enableLogs = openvr_pair::common::IsDebugLoggingEnabled();
	if (Metrics::enableLogs) {
		Metrics::EnsureLogFileReady(nullptr);
	}
	if (isDevBuild) {
		Metrics::enableLogs = true;
		ImGui::BeginDisabled();
	}
	bool enableLogs = Metrics::enableLogs;
	if (ImGui::Checkbox("Enable debug logging", &enableLogs)) {
		openvr_pair::common::SetDebugLoggingEnabled(enableLogs);
		Metrics::enableLogs = openvr_pair::common::IsDebugLoggingEnabled();
		if (Metrics::enableLogs) {
			Metrics::EnsureLogFileReady("logs_toggle_on");
		} else {
			Metrics::CloseLogFile();
		}
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
			"Write diagnostics and SpaceCal annotations to %%LocalAppDataLow%%\\WKOpenVR\\Logs\\.\n"
			"Turn this on before reproducing an issue for a useful bug report.");
	}
	if (isDevBuild) {
		ImGui::EndDisabled();
	}
	ImGui::SameLine();
	if (isDevBuild) {
		ImGui::TextColored(ImVec4(0.55f, 0.75f, 0.95f, 1.0f),
			" -- dev build: always on");
	} else if (Metrics::enableLogs) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f),
			" -- debug log active");
	} else {
		ImGui::TextDisabled(" (off)");
	}

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::TextDisabled("SpaceCal debug log");
	const Metrics::LogHealth health = Metrics::GetLogHealth();
	if (!Metrics::enableLogs) {
		ImGui::TextWrapped(
			"Debug logging is off. Diagnostics, SpaceCal annotations, and bug-report "
			"log context are not written until it is enabled.");
	} else if (health.open) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f), "Writing to disk");
		ImGui::TextWrapped("%s", openvr_pair::common::WideToUtf8(health.path).c_str());
		ImGui::Text("Size: %s   Annotations: %llu   Open attempts: %llu",
			openvr_pair::overlay::ui::FormatByteCountOrUnknown(health.sizeBytes).c_str(),
			(unsigned long long)health.annotationsWritten,
			(unsigned long long)health.openAttempts);
#if WKOPENVR_BUILD_IS_DEV
		if (health.replayCsvEnabled) {
			ImGui::Text("Replay rows: %llu", (unsigned long long)health.rowsWritten);
		}
#endif
#if WKOPENVR_BUILD_IS_DEV
		ImGui::Text("Replay capture: %s",
			health.replayCsvEnabled ? "enabled" : "disabled");
#endif
	} else {
		ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.30f, 1.0f),
			"SpaceCal debug log is not open");
		if (health.lastErrorCode) {
			ImGui::TextWrapped("Status: %s  Error: %lu",
				health.status.empty() ? "unknown" : health.status.c_str(),
				health.lastErrorCode);
		} else {
			ImGui::TextWrapped("Status: %s",
				health.status.empty() ? "unknown" : health.status.c_str());
		}
	}

	ImGui::BeginDisabled(!health.open);
	if (ImGui::SmallButton("Flush now##spacecal_log")) {
		if (Metrics::FlushLogFile()) {
			state.copyHint = "SpaceCal log flushed to disk";
		} else {
			state.copyHint = "SpaceCal log flush failed";
		}
		state.copyHintExpireTime = ImGui::GetTime() + 2.5;
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("Copy current path##spacecal_log")) {
		if (openvr_pair::overlay::ui::CopyWideTextToClipboard(health.path)) {
			state.copyHint = "Current log path copied";
		} else {
			state.copyHint = "Failed to copy path (clipboard busy?)";
		}
		state.copyHintExpireTime = ImGui::GetTime() + 2.5;
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("Write health snapshot##spacecal_log")) {
		Metrics::WriteLogHealthSnapshot("logs_tab_button");
		state.copyHint = "Log health snapshot written";
		state.copyHintExpireTime = ImGui::GetTime() + 2.5;
	}
	ImGui::EndDisabled();

	ImGui::Spacing();
	ImGui::Separator();

	ImGui::TextDisabled("Log files");
	ImGui::SameLine();
	if (ImGui::SmallButton("Refresh##logs")) {
		spacecal::ui_logs::RebuildLogsList(state);
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("Open folder##logs")) {
		const std::wstring dir = spacecal::ui_logs::ResolveLogsDirectory(state);
		if (!dir.empty()) {
			ShellExecuteW(nullptr, L"open", dir.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
		}
	}

	spacecal::ui_logs::DrawLogFileList(state);

	ImGui::Spacing();
	spacecal::ui_logs::DrawSelectedLogActions(state);
}

#if WKOPENVR_BUILD_IS_DEV
void CCal_DrawDevToolsPanel() {
	namespace ui = openvr_pair::overlay::ui;

	ui::DrawSectionHeading("Replay CSV");
	bool replayCsv = Metrics::enableReplayCsv;
	if (ImGui::Checkbox("Enable replay CSV capture", &replayCsv)) {
		Metrics::enableReplayCsv = replayCsv;
		if (Metrics::enableLogs) {
			Metrics::CloseLogFile();
			Metrics::EnsureLogFileReady(
				replayCsv ? "replay_csv_enabled" : "replay_csv_disabled");
		}
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
			"Write per-tick calibration rows that can be replayed locally.");
	}

	ImGui::Spacing();
	ui::DrawSectionHeading("Simulation");
	bool fakeDevices = spacecal::devfake::IsEnabled();
	if (ImGui::Checkbox("Simulated tracker and HMD", &fakeDevices)) {
		spacecal::devfake::SetEnabled(fakeDevices);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
			"Adds a simulated Quest HMD and Vive tracker so calibration UI can run from the desktop.");
	}

	bool fakeControllers = spacecal::devfake::IncludeIndexControllers();
	ImGui::SameLine();
	ImGui::BeginDisabled(!fakeDevices);
	if (ImGui::Checkbox("Simulated Index controllers", &fakeControllers)) {
		spacecal::devfake::SetIncludeIndexControllers(fakeControllers);
	}
	ImGui::EndDisabled();
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
			"Adds left and right Index-style controllers to the fake lighthouse space.");
	}

	if (fakeDevices) {
		ImGui::TextWrapped(
			"Simulated devices are local to the overlay and do not push transforms to the SteamVR driver.");
	}

	ImGui::Spacing();
	ui::DrawSectionHeading("DriverSynth");
	auto& hm = CalCtx.headMount;
	const bool hasTracker = !hm.trackerSerial.empty();
	const bool offsetOk = hm.offsetCalibrated;
	const bool canUseDriverSynth = hasTracker && offsetOk;
	{
		bool driverSynthEnabled = hm.mode == HeadMountMode::DriverSynth;
		ui::DisabledSection ds(!canUseDriverSynth && !driverSynthEnabled,
			!hasTracker
				? "Start continuous calibration with the headset-mounted tracker as the target first."
				: "Calibrate the tracker-to-headset offset first.");
		if (ImGui::Checkbox("Synthesize headset pose from tracker", &driverSynthEnabled)) {
			hm.mode = driverSynthEnabled ? HeadMountMode::DriverSynth : HeadMountMode::Off;
			SaveProfile(CalCtx);
			CCal_SendHeadMountConfig();
		}
		ds.AttachReasonTooltip();
	}
	if (ImGui::IsItemHovered() && canUseDriverSynth) {
		ImGui::SetTooltip(
			"Replaces the rendered headset pose with one derived from the head-tracker.\n"
			"Known compositor and comfort risks.");
	}

	if (hm.mode == HeadMountMode::DriverSynth) {
		ImGui::Spacing();
		ImGui::TextUnformatted("DriverSynth fallback timing");
		if (ImGui::BeginTable("driver_synth_timing", 2,
			ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoSavedSettings))
		{
			auto timing = wkopenvr::headmount::ClampDriverSynthTimingConfig(
				hm.driverSynthTiming);
			bool changed = false;
			ImGui::PushID("stale_limit");
			changed |= DrawDriverSynthTimingControl("Tracker stale limit",
				timing.staleLimitMs,
				wkopenvr::headmount::kDriverSynthStaleLimitMsMin,
				wkopenvr::headmount::kDriverSynthStaleLimitMsMax,
				"How old the last tracker pose can be before it is treated as missing.");
			ImGui::PopID();
			ImGui::PushID("grace_hold");
			changed |= DrawDriverSynthTimingControl("Grace hold",
				timing.graceHoldMs,
				wkopenvr::headmount::kDriverSynthTransitionMsMin,
				wkopenvr::headmount::kDriverSynthTransitionMsMax,
				"How long to keep the last tracker-synth pose before fading to Quest tracking.");
			ImGui::PopID();
			ImGui::PushID("blend_out");
			changed |= DrawDriverSynthTimingControl("Blend to fallback",
				timing.blendToFallbackMs,
				wkopenvr::headmount::kDriverSynthTransitionMsMin,
				wkopenvr::headmount::kDriverSynthTransitionMsMax,
				"Fade duration from tracker-synth pose to Quest tracking after grace expires.");
			ImGui::PopID();
			ImGui::PushID("stable_return");
			changed |= DrawDriverSynthTimingControl("Stable before return",
				timing.stableBeforeSynthMs,
				wkopenvr::headmount::kDriverSynthTransitionMsMin,
				wkopenvr::headmount::kDriverSynthTransitionMsMax,
				"How long the tracker must be good again before WKOpenVR blends back to it.");
			ImGui::PopID();
			ImGui::PushID("blend_in");
			changed |= DrawDriverSynthTimingControl("Blend back to tracker",
				timing.blendToSynthMs,
				wkopenvr::headmount::kDriverSynthTransitionMsMin,
				wkopenvr::headmount::kDriverSynthTransitionMsMax,
				"Fade duration from Quest tracking back to tracker-synth pose.");
			ImGui::PopID();
			ImGui::EndTable();

			if (changed) {
				hm.driverSynthTiming = timing;
				SaveProfile(CalCtx);
				CCal_SendHeadMountConfig();
			}
		}
		if (ImGui::Button("Reset DriverSynth timing")) {
			hm.driverSynthTiming = {};
			SaveProfile(CalCtx);
			CCal_SendHeadMountConfig();
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Restore the default DriverSynth fallback timing values.");
		}
	}

	ImGui::Spacing();
	ui::DrawSectionHeading("Diagnostics");
	if (ImGui::Button("Dump drift state")) {
		DumpDriftSubsystemState();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
			"Write a one-shot snapshot of the relocalization state\n"
			"to the current debug log. Greppable by [drift][state-dump].");
	}
}
#endif
