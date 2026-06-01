#include "ShellFooter.h"

#include "Theme.h"
#include "UiHelpers.h"
#include "UpdateNotice.h"

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

void DrawShellFooter(const ShellFooterStatus &status)
{
	const float lineH = ImGui::GetTextLineHeight();
	// Reserve two text lines so the footer still has room when the composed
	// status string wraps on a narrow window. Single-line case leaves the
	// second line as a small gap below the text.
	const float footerH = lineH * 2.0f + 8.0f;
	const float available = ImGui::GetContentRegionAvail().y;
	if (available > footerH) {
		ImGui::Dummy(ImVec2(0.0f, available - footerH));
	}
	ImGui::Separator();

	const char *label = status.driverLabel ? status.driverLabel : "Driver";
	const char *stamp = status.buildStamp ? status.buildStamp : OPENVR_PAIR_VERSION_STRING;

	const ui::SemanticPalette &pal = ui::GetPalette();
	const ShellFooterConnectionState connectionState =
		ResolveShellFooterConnectionState(status.driverConnected, status.vrConnected);

	ImU32 dotColor = pal.dotPending;
	ImVec4 textColor = pal.statusPending;
	const char *state = "waiting for SteamVR";
	if (connectionState == ShellFooterConnectionState::Connected) {
		dotColor = pal.dotOk;
		textColor = pal.statusOk;
		state = "connected";
	} else if (connectionState == ShellFooterConnectionState::Disconnected) {
		dotColor = pal.dotError;
		textColor = pal.statusError;
		state = "disconnected";
	}

	ui::DrawStatusDot(dotColor);
	ImGui::PushStyleColor(ImGuiCol_Text, textColor);
	// TextWrapped wraps at the right edge of the current content region, so
	// the "Driver: connected  |  WKOpenVR <stamp>" line reflows instead of
	// overflowing on a desktop-mode window that has been shrunk.
	ImGui::TextWrapped("%s: %s  |  WKOpenVR %s", label, state, stamp);
	ImGui::PopStyleColor();

	// Update notice. Rendered on the line below the driver status so the
	// existing single-line layout doesn't get crowded. Only shows when the
	// startup GitHub probe has finished AND a newer release exists. Click
	// the line to open the release page in the default browser; tooltip on
	// hover repeats the URL for users who want to copy it. The notice
	// stays put once shown -- there's no dismiss button by design: it
	// represents a fact ("a newer release exists"), and the user fixes
	// that fact by upgrading, not by hiding it.
	const UpdateNoticeState notice = GetUpdateNoticeState();
	if (notice.available && !notice.latestVersion.empty()) {
		ImGui::PushStyleColor(ImGuiCol_Text, pal.statusInfo);
		const std::string text =
			std::string("Update available: v") + notice.latestVersion + "  (click for release notes)";
		ImGui::TextUnformatted(text.c_str());
		ImGui::PopStyleColor();
#ifdef _WIN32
		if (ImGui::IsItemClicked() && !notice.releaseUrl.empty()) {
			ShellExecuteA(nullptr, "open", notice.releaseUrl.c_str(),
				nullptr, nullptr, SW_SHOWNORMAL);
		}
#endif
		if (ImGui::IsItemHovered() && !notice.releaseUrl.empty()) {
			ImGui::SetTooltip("%s", notice.releaseUrl.c_str());
		}
	}
}

} // namespace openvr_pair::overlay
