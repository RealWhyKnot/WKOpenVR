#include "UserInterfaceLogFiles.h"

#include "UiControls.h"
#include "Win32Text.h"
#include "Win32Paths.h"

#include <algorithm>

#include <imgui/imgui.h>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>

namespace spacecal::ui_logs {
namespace {

uint64_t FileTimeToUint64(const FILETIME& ft)
{
	return (static_cast<uint64_t>(ft.dwHighDateTime) << 32) | static_cast<uint64_t>(ft.dwLowDateTime);
}

std::vector<LogFileEntry> ListLogFiles()
{
	std::vector<LogFileEntry> files;
	std::wstring directory = openvr_pair::common::WkOpenVrLogsPath(false);
	if (directory.empty()) return files;
	if (directory.back() != L'\\' && directory.back() != L'/') {
		directory.push_back(L'\\');
	}

	WIN32_FIND_DATAW data{};
	HANDLE find = FindFirstFileW((directory + L"*.txt").c_str(), &data);
	if (find == INVALID_HANDLE_VALUE) {
		return files;
	}

	do {
		if (data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
		LogFileEntry entry;
		entry.fullPath = directory + data.cFileName;
		entry.name = openvr_pair::common::WideToUtf8(data.cFileName);
		entry.sizeBytes = (static_cast<uint64_t>(data.nFileSizeHigh) << 32) | static_cast<uint64_t>(data.nFileSizeLow);
		entry.mtimeFileTime = FileTimeToUint64(data.ftLastWriteTime);
		files.push_back(entry);
	} while (FindNextFileW(find, &data));

	FindClose(find);
	std::sort(files.begin(), files.end(),
	          [](const LogFileEntry& a, const LogFileEntry& b) { return a.mtimeFileTime > b.mtimeFileTime; });
	return files;
}

} // namespace

LogsPanelState& LogsState()
{
	static LogsPanelState s;
	return s;
}

void RebuildLogsList(LogsPanelState& state)
{
	state.files = ListLogFiles();
	state.listBuilt = true;
	if (state.selectedIdx >= static_cast<int>(state.files.size())) {
		state.selectedIdx = -1;
	}
}

std::wstring ResolveLogsDirectory(const LogsPanelState& state)
{
	if (!state.files.empty()) {
		const auto& full = state.files.front().fullPath;
		const size_t lastSlash = full.find_last_of(L"\\/");
		if (lastSlash != std::wstring::npos) {
			return full.substr(0, lastSlash);
		}
	}
	return openvr_pair::common::WkOpenVrLogsPath(true);
}

void DrawLogFileList(LogsPanelState& state)
{
	if (!state.listBuilt) {
		RebuildLogsList(state);
	}

	if (state.files.empty()) {
		ImGui::TextDisabled("(No log files found in the Logs directory.)");
		return;
	}

	const float listHeight = ImGui::GetTextLineHeightWithSpacing() * 8.0f;
	if (ImGui::BeginChild("##logs_list", ImVec2(0, listHeight), ImGuiChildFlags_Border)) {
		for (int i = 0; i < static_cast<int>(state.files.size()); ++i) {
			const auto& f = state.files[i];
			char label[512];
			snprintf(label, sizeof label, "%s   (%s, %s)", f.name.c_str(),
			         openvr_pair::overlay::ui::FormatByteCount(f.sizeBytes).c_str(),
			         openvr_pair::overlay::ui::FormatFileAgeFromFileTime(f.mtimeFileTime).c_str());

			const float deleteBtnWidth = 70.0f;
			const float rowWidth = ImGui::GetContentRegionAvail().x - deleteBtnWidth - 8.0f;
			if (ImGui::Selectable(label, state.selectedIdx == i, 0, ImVec2(rowWidth, 0))) {
				state.selectedIdx = i;
			}
			ImGui::SameLine();
			char delId[64];
			snprintf(delId, sizeof delId, "Delete##log%d", i);
			if (ImGui::SmallButton(delId)) {
				const BOOL ok = DeleteFileW(state.files[i].fullPath.c_str());
				if (ok) {
					state.copyHint = "Deleted " + state.files[i].name;
				}
				else {
					state.copyHint = "Could not delete (file may be in use)";
				}
				state.copyHintExpireTime = ImGui::GetTime() + 2.5;
				RebuildLogsList(state);
				break;
			}
		}
	}
	ImGui::EndChild();
}

void DrawSelectedLogActions(LogsPanelState& state)
{
	const bool haveSelection = state.selectedIdx >= 0 && state.selectedIdx < static_cast<int>(state.files.size());
	ImGui::BeginDisabled(!haveSelection);
	if (ImGui::Button("Open selected##logs")) {
		ShellExecuteW(nullptr, L"open", state.files[state.selectedIdx].fullPath.c_str(), nullptr, nullptr,
		              SW_SHOWNORMAL);
	}
	ImGui::SameLine();
	if (ImGui::Button("Copy path##logs")) {
		if (openvr_pair::overlay::ui::CopyWideTextToClipboard(state.files[state.selectedIdx].fullPath)) {
			state.copyHint = "Path copied to clipboard";
		}
		else {
			state.copyHint = "Failed to copy path (clipboard busy?)";
		}
		state.copyHintExpireTime = ImGui::GetTime() + 2.5;
	}
	ImGui::EndDisabled();

	if (!state.copyHint.empty() && ImGui::GetTime() < state.copyHintExpireTime) {
		ImGui::SameLine();
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextUnformatted(state.copyHint.c_str());
		ImGui::PopStyleColor();
	}
	else if (!state.copyHint.empty()) {
		state.copyHint.clear();
	}
}

} // namespace spacecal::ui_logs
