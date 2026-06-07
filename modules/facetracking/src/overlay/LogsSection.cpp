#include "LogsSection.h"

#include "DebugLogging.h"
#include "FacetrackingPlugin.h"
#include "Logging.h"
#include "UiHelpers.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <imgui/imgui.h>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>

#include <string>
#include <utility>
#include <vector>

namespace facetracking::ui {

using namespace openvr_pair::overlay::ui;

namespace {

std::wstring LogsDir()
{
	return openvr_pair::common::WkOpenVrLogsPath(false);
}

void OpenInExplorer(const std::wstring& dir)
{
	ShellExecuteW(nullptr, L"explore", dir.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
}

struct LogEntry
{
	std::wstring path;
	std::string nameUtf8;
};

std::vector<LogEntry> EnumerateLogs(const std::wstring& dir)
{
	std::vector<LogEntry> result;
	if (dir.empty()) return result;

	const wchar_t* prefixes[] = {
	    L"facetracking_log.*.txt",
	    L"facetracking_drv_log.*.txt",
	    L"facetracking_host_log.*.txt",
	};
	for (const wchar_t* prefix : prefixes) {
		std::wstring search = dir + L"\\" + prefix;
		WIN32_FIND_DATAW fd{};
		HANDLE h = FindFirstFileW(search.c_str(), &fd);
		if (h == INVALID_HANDLE_VALUE) continue;
		do {
			std::string name = openvr_pair::common::WideToUtf8(fd.cFileName);
			result.push_back({dir + L"\\" + fd.cFileName, std::move(name)});
		} while (FindNextFileW(h, &fd));
		FindClose(h);
	}
	return result;
}

} // namespace

void DrawLogsSection(FacetrackingPlugin& plugin)
{
	// ---- IPC state ----
	ImGui::Text("IPC: %s", plugin.ipc_.IsConnected() ? "connected" : "disconnected");

	// ---- Verbose logging toggle ----
	DrawSectionHeading("Logging");

	const bool isDev = openvr_pair::common::IsDebugLoggingForcedOn();

	if (isDev) {
		FtOverlayVerbose.store(true, std::memory_order_relaxed);
		ImGui::BeginDisabled();
	}

	bool verboseOverlay = openvr_pair::common::IsDebugLoggingEnabled();
	if (CheckboxWithTooltip("Enable debug logging", &verboseOverlay,
	                        "Write Face Tracking overlay diagnostics to the facetracking_log.* file.\n"
	                        "The shared debug toggle also gates driver and host-side logs.")) {
		openvr_pair::common::SetDebugLoggingEnabled(verboseOverlay);
		verboseOverlay = openvr_pair::common::IsDebugLoggingEnabled();
		FtOverlayVerbose.store(verboseOverlay, std::memory_order_relaxed);
	}

	if (isDev) {
		ImGui::EndDisabled();
		ImGui::SameLine();
		ImGui::TextDisabled("(forced on -- dev channel)");
	}

	// ---- Log file list ----
	DrawSectionHeading("Log files");

	std::wstring dir = LogsDir();
	auto entries = EnumerateLogs(dir);

	if (entries.empty()) {
		ImGui::TextDisabled("No face-tracking log files found.");
	}
	else {
		for (const auto& e : entries) {
			ImGui::TextUnformatted(e.nameUtf8.c_str());
		}
	}

	ImGui::Spacing();
	if (ImGui::Button("Open log folder in Explorer")) {
		OpenInExplorer(dir);
	}
	TooltipForLastItem("%LocalAppDataLow%\\WKOpenVR\\Logs\\\n"
	                   "Contains overlay, driver, and host log files for all features.");

	// ---- File paths (reference) ----
	DrawSectionHeading("Paths");
	ImGui::TextWrapped("Overlay:   %%LocalAppDataLow%%\\WKOpenVR\\Logs\\facetracking_log.<ts>.txt");
	ImGui::TextWrapped("Driver:    %%LocalAppDataLow%%\\WKOpenVR\\Logs\\facetracking_drv_log.<ts>.txt");
	ImGui::TextWrapped("Host:      %%LocalAppDataLow%%\\WKOpenVR\\Logs\\facetracking_host_log.<ts>.txt");
	ImGui::TextWrapped("Profiles:  %%LocalAppDataLow%%\\WKOpenVR\\profiles\\facetracking.json");
	ImGui::TextWrapped("Trust:     %%LocalAppDataLow%%\\WKOpenVR\\facetracking\\trust.json");

	(void)plugin; // profile_.current is not needed here at present
}

} // namespace facetracking::ui
