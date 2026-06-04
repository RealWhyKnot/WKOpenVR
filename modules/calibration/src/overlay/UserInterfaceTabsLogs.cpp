#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "BuildChannel.h"
#include "Configuration.h"
#include "DebugLogging.h"
#include "UiControls.h"
#include "UserInterfaceLogFiles.h"
#include "Win32Text.h"

#include <string>
#include <shellapi.h>
#include <imgui/imgui.h>

extern bool s_inUmbrella;

// Release builds ship user-controlled debug logs and bug-report support.
// Replay CSV capture and simulated devices are dev-build-only surfaces.
void CCal_DrawLogsPanel()
{
	auto& state = spacecal::ui_logs::LogsState();
	const bool umbrella = s_inUmbrella;

	if (umbrella) {
		ImGui::TextWrapped("Debug logging writes WKOpenVR diagnostics to "
		                   "%%LocalAppDataLow%%\\WKOpenVR\\Logs. Turn it on before reproducing an issue, "
		                   "then use Report bug.");
	}
	else {
		ImGui::TextWrapped("Debug logging writes WKOpenVR diagnostics and SpaceCal annotations to "
		                   "%%LocalAppDataLow%%\\WKOpenVR\\Logs. Turn it on before reproducing an issue, "
		                   "then use Report bug.");
	}
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
		}
		else {
			Metrics::CloseLogFile();
		}
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
		    "%s", umbrella ? "Write diagnostics to %%LocalAppDataLow%%\\WKOpenVR\\Logs\\.\n"
		                     "Turn this on before reproducing an issue for a useful bug report."
		                   : "Write diagnostics and SpaceCal annotations to %%LocalAppDataLow%%\\WKOpenVR\\Logs\\.\n"
		                     "Turn this on before reproducing an issue for a useful bug report.");
	}
	if (isDevBuild) {
		ImGui::EndDisabled();
	}
	ImGui::SameLine();
	if (isDevBuild) {
		ImGui::TextColored(ImVec4(0.55f, 0.75f, 0.95f, 1.0f), " -- dev build: always on");
	}
	else if (Metrics::enableLogs) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f), " -- debug log active");
	}
	else {
		ImGui::TextDisabled(" (off)");
	}

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::TextDisabled(umbrella ? "Current debug log" : "SpaceCal debug log");
	const Metrics::LogHealth health = Metrics::GetLogHealth();
	if (!Metrics::enableLogs) {
		ImGui::TextWrapped(
		    "%s",
		    umbrella
		        ? "Debug logging is off. Diagnostics and bug-report log context are not written until it is enabled."
		        : "Debug logging is off. Diagnostics, SpaceCal annotations, and bug-report log context are not written "
		          "until it is enabled.");
	}
	else if (health.open) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f), "Writing to disk");
		ImGui::TextWrapped("%s", openvr_pair::common::WideToUtf8(health.path).c_str());
		ImGui::Text("Size: %s   Annotations: %llu   Open attempts: %llu",
		            openvr_pair::overlay::ui::FormatByteCountOrUnknown(health.sizeBytes).c_str(),
		            (unsigned long long)health.annotationsWritten, (unsigned long long)health.openAttempts);
#if WKOPENVR_BUILD_IS_DEV
		if (health.replayCsvEnabled) {
			ImGui::Text("Replay rows: %llu", (unsigned long long)health.rowsWritten);
		}
#endif
#if WKOPENVR_BUILD_IS_DEV
		ImGui::Text("Replay capture: %s", health.replayCsvEnabled ? "enabled" : "disabled");
#endif
	}
	else {
		ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.30f, 1.0f),
		                   umbrella ? "Current debug log is not open" : "SpaceCal debug log is not open");
		if (health.lastErrorCode) {
			ImGui::TextWrapped("Status: %s  Error: %lu", health.status.empty() ? "unknown" : health.status.c_str(),
			                   health.lastErrorCode);
		}
		else {
			ImGui::TextWrapped("Status: %s", health.status.empty() ? "unknown" : health.status.c_str());
		}
	}

	ImGui::BeginDisabled(!health.open);
	if (ImGui::SmallButton("Flush now##spacecal_log")) {
		if (Metrics::FlushLogFile()) {
			state.copyHint = umbrella ? "Debug log flushed to disk" : "SpaceCal log flushed to disk";
		}
		else {
			state.copyHint = umbrella ? "Debug log flush failed" : "SpaceCal log flush failed";
		}
		state.copyHintExpireTime = ImGui::GetTime() + 2.5;
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("Copy current path##spacecal_log")) {
		if (openvr_pair::overlay::ui::CopyWideTextToClipboard(health.path)) {
			state.copyHint = "Current log path copied";
		}
		else {
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
void CCal_DrawDevToolsPanel()
{
	namespace ui = openvr_pair::overlay::ui;

	ui::DrawSectionHeading("Replay CSV");
	bool replayCsv = Metrics::enableReplayCsv;
	if (ImGui::Checkbox("Enable replay CSV capture", &replayCsv)) {
		Metrics::enableReplayCsv = replayCsv;
		if (Metrics::enableLogs) {
			Metrics::CloseLogFile();
			Metrics::EnsureLogFileReady(replayCsv ? "replay_csv_enabled" : "replay_csv_disabled");
		}
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Write per-tick calibration rows that can be replayed locally.");
	}

	ImGui::Spacing();
	ui::DrawSectionHeading("Diagnostics");
	if (ImGui::Button("Dump drift state")) {
		DumpDriftSubsystemState();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Write a one-shot snapshot of the relocalization state\n"
		                  "to the current debug log. Greppable by [drift][state-dump].");
	}
}
#endif
