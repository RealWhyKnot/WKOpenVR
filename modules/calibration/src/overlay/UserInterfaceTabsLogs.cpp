#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "DebugLogging.h"
#include "DevFakeDevices.h"
#include "UiControls.h"
#include "UserInterfaceLogFiles.h"
#include "Win32Text.h"

#include <string>
#include <shellapi.h>
#include <imgui/imgui.h>

// === Logs panel ============================================================
// User-friendly view of the debug-log files written when "Enable debug logs"
// is on. The previous Recordings tab was dev-tooling -- it loaded a CSV and
// replayed it through the calibration math, which only made sense for the
// developer iterating on a fix. Most users just want a way to find and copy
// log files to attach to a bug report. So we surface the file list, a button
// to open the logs folder in Explorer, and a per-row copy-path action.
//
// The replay machinery (spacecal::replay::LoadRecording / RunReplay) is still
// in MotionRecording.cpp for the standalone replay CLI tool and the test
// suite; it just isn't surfaced in the overlay UI anymore.
void CCal_DrawLogsPanel() {
	auto& state = spacecal::ui_logs::LogsState();

	ImGui::TextWrapped(
		"SpaceCal writes a replayable CSV while debug logging is enabled. The file opens "
		"as soon as logging turns on, flushes each write to disk, and records raw poses "
		"so the calibration math can be replayed later.");
	ImGui::Spacing();

	// Recording status + toggle. The log file IS the recording -- there's no
	// separate "start/stop recording" notion. Putting the toggle here means
	// the user can flip logging on right where they're managing the log
	// files, instead of having to dig into Settings.
	//
	// Dev builds force logging on and disable the checkbox so a local debug
	// session always has a CSV trail without anyone having to remember to
	// flip the toggle. Release builds keep the toggle live; default-off boots
	// are handled by Metrics::enableLogs's initializer.
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
		ImGui::SetTooltip("Write a per-tick CSV of calibration state to %%LocalAppDataLow%%\\WKOpenVR\\Logs\\\n"
		                  "while this is on. The new log shows up in the list below as soon as the next calibration tick fires.");
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
			" -- a fresh CSV is being written this session");
	} else {
		ImGui::TextDisabled(" (off)");
	}

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::TextDisabled("SpaceCal replay CSV");
	const Metrics::LogHealth health = Metrics::GetLogHealth();
	if (!Metrics::enableLogs) {
		ImGui::TextWrapped(
			"Debug logging is off. No SpaceCal CSV, diagnostics log, replay rows, or "
			"calibration annotations are written until it is enabled.");
	} else if (health.open) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f), "Recording to disk");
		ImGui::TextWrapped("%s", openvr_pair::common::WideToUtf8(health.path).c_str());
		ImGui::Text("Size: %s   Rows: %llu   Annotations: %llu   Open attempts: %llu",
			openvr_pair::overlay::ui::FormatByteCountOrUnknown(health.sizeBytes).c_str(),
			(unsigned long long)health.rowsWritten,
			(unsigned long long)health.annotationsWritten,
			(unsigned long long)health.openAttempts);
		ImGui::Text("Replay: %s",
			health.devAutoRecording
				? "dev auto-recording on; newest captures retained"
				: "enabled while debug logging is on");
	} else {
		ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.30f, 1.0f),
			"SpaceCal CSV is not open");
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

	if (isDevBuild) {
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::TextDisabled("Dev simulation");

		bool fakeDevices = spacecal::devfake::IsEnabled();
		if (ImGui::Checkbox("Simulated tracker and HMD", &fakeDevices)) {
			spacecal::devfake::SetEnabled(fakeDevices);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Adds a simulated Quest HMD and Vive tracker so calibration UI can run from the desktop.");
		}

		bool fakeControllers = spacecal::devfake::IncludeIndexControllers();
		ImGui::SameLine();
		ImGui::BeginDisabled(!fakeDevices);
		if (ImGui::Checkbox("Simulated Index controllers", &fakeControllers)) {
			spacecal::devfake::SetIncludeIndexControllers(fakeControllers);
		}
		ImGui::EndDisabled();
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Adds left and right Index-style controllers to the fake lighthouse space.");
		}

		if (fakeDevices) {
			ImGui::TextWrapped(
				"Simulated devices are local to the overlay and do not push transforms to the SteamVR driver.");
		}
	}

	// Drift-subsystem state dump. Captures relocalization state to the log
	// in a single batch of [drift][state-dump] annotations.
	ImGui::Spacing();
	if (ImGui::Button("Dump drift state")) {
		DumpDriftSubsystemState();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Write a one-shot snapshot of the relocalization state\n"
			"to the current debug log. Greppable by [drift][state-dump].");
	}

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
