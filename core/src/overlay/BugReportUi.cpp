#include "BugReportUi.h"

#include "BugReport.h"
#include "DebugLogging.h"
#include "ShellContext.h"
#include "UiControls.h"

#include <imgui.h>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>
#endif

#ifndef OPENVR_PAIR_VERSION_STRING
#define OPENVR_PAIR_VERSION_STRING "0.0.0.0-dev"
#endif

namespace openvr_pair::overlay {

void DrawBugReportButton(ShellContext& context)
{
	auto createReport = [&]() {
		common::BugReportOptions options;
		options.logRoot = context.logRoot;
		options.version = OPENVR_PAIR_VERSION_STRING;
		const common::BugReportResult report = common::CreateBugReport(options);
		if (!report.success) {
			context.SetStatus(report.error.empty()
				? "Could not prepare a bug report."
				: report.error);
		} else {
			ImGui::SetClipboardText(report.issueBody.c_str());
#ifdef _WIN32
			const std::wstring explorerArgs = L"/select,\"" + report.reportFile + L"\"";
			ShellExecuteW(nullptr, L"open", L"explorer.exe",
				explorerArgs.c_str(), nullptr, SW_SHOWNORMAL);
			ShellExecuteA(nullptr, "open", report.issueUrl.c_str(),
				nullptr, nullptr, SW_SHOWNORMAL);
#endif
			context.SetStatus(
				"Bug report opened. A sanitized report file is selected in Explorer and copied to the clipboard.");
		}
	};

	if (ImGui::Button("Report bug##report_bug")) {
		if (!common::IsDebugLoggingEnabled()) {
			ImGui::OpenPopup("Enable debug logging first?##bug_report_debug_prompt");
		} else {
			createReport();
		}
	}
	ui::TooltipForLastItem(
		"Prepare a sanitized text report from recent WKOpenVR logs, copy it to the clipboard, "
		"select it in Explorer, and open the GitHub bug form. Turn debug logging on before reproducing "
		"the issue for a more useful report.");

	if (ImGui::BeginPopupModal(
		"Enable debug logging first?##bug_report_debug_prompt",
		nullptr,
		ImGuiWindowFlags_AlwaysAutoResize)) {
		ImGui::TextWrapped(
			"Debug logging is off. Turn it on, reproduce the issue, then click Report bug again "
			"so the report includes useful diagnostics.");
		ImGui::Spacing();
		if (ImGui::Button("Enable debug logging")) {
			if (common::SetDebugLoggingEnabled(true)) {
				context.SetStatus("Debug logging enabled. Reproduce the issue, then click Report bug again.");
			} else {
				context.SetStatus("Could not enable debug logging.");
			}
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button("Create report anyway")) {
			createReport();
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button("Cancel")) {
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}

} // namespace openvr_pair::overlay
