#include "UiControls.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace openvr_pair::overlay::ui {

ImVec4 StatusColor(StatusTone tone)
{
	const SemanticPalette& pal = GetPalette();
	switch (tone) {
		case StatusTone::Ok:
			return pal.statusOk;
		case StatusTone::Pending:
			return pal.statusPending;
		case StatusTone::Error:
			return pal.statusError;
		case StatusTone::Warn:
			return pal.statusWarn;
		case StatusTone::Info:
			return pal.statusInfo;
		case StatusTone::Idle:
			return pal.statusIdle;
	}
	return pal.statusIdle;
}

ScopedStyleColor::ScopedStyleColor(ImGuiCol idx, ImVec4 color)
{
	ImGui::PushStyleColor(idx, color);
}

ScopedStyleColor::~ScopedStyleColor()
{
	ImGui::PopStyleColor();
}

ScopedStyleColors::ScopedStyleColors(std::initializer_list<std::pair<ImGuiCol, ImVec4>> colors)
    : count_(static_cast<int>(colors.size()))
{
	for (const auto& [idx, color] : colors) {
		ImGui::PushStyleColor(idx, color);
	}
}

ScopedStyleColors::~ScopedStyleColors()
{
	if (count_ > 0) ImGui::PopStyleColor(count_);
}

DisabledSection::DisabledSection(bool disabled, const char* whyTooltip) : active(disabled), why(whyTooltip)
{
	if (active) ImGui::BeginDisabled();
}

DisabledSection::~DisabledSection()
{
	if (active) ImGui::EndDisabled();
}

void DisabledSection::AttachReasonTooltip() const
{
	if (active && why && why[0] && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
		ImGui::SetTooltip("%s", why);
	}
}

void TooltipForLastItem(const char* tooltip)
{
	if (tooltip && tooltip[0] != '\0' && ImGui::IsItemHovered()) {
		ImGui::SetTooltip("%s", tooltip);
	}
}

void TooltipOnHover(const char* tooltip)
{
	TooltipForLastItem(tooltip);
}

bool CheckboxWithTooltip(const char* label, bool* value, const char* tooltip)
{
	const bool changed = ImGui::Checkbox(label, value);
	TooltipForLastItem(tooltip);
	return changed;
}

bool SliderIntWithTooltip(const char* label, int* value, int min, int max, const char* format, const char* tooltip)
{
	const bool changed = ImGui::SliderInt(label, value, min, max, format);
	TooltipForLastItem(tooltip);
	return changed;
}

bool SliderFloatWithTooltip(const char* label, float* value, float min, float max, const char* format,
                            const char* tooltip, ImGuiSliderFlags flags)
{
	const bool changed = ImGui::SliderFloat(label, value, min, max, format, flags);
	TooltipForLastItem(tooltip);
	return changed;
}

bool RadioButtonWithTooltip(const char* label, bool active, const char* tooltip)
{
	const bool clicked = ImGui::RadioButton(label, active);
	TooltipForLastItem(tooltip);
	return clicked;
}

void DrawHelpMarker(const char* tooltip)
{
	if (!tooltip || tooltip[0] == '\0') return;
	ImGui::TextDisabled("(?)");
	TooltipForLastItem(tooltip);
}

void DrawTextWrapped(const char* text)
{
	if (!text) return;
	ImGui::TextWrapped("%s", text);
}

void DrawTextWrapped(const std::string& text)
{
	DrawTextWrapped(text.c_str());
}

void DrawColoredText(const char* text, ImVec4 color)
{
	if (!text) return;
	ImGui::TextColored(color, "%s", text);
}

void DrawStatusText(const char* text, StatusTone tone)
{
	if (!text) return;
	ImGui::TextColored(StatusColor(tone), "%s", text);
}

void DrawEmptyState(const char* text)
{
	if (!text) return;
	ImGui::TextDisabled("%s", text);
}

void DrawStatusDot(ImU32 color)
{
	ImDrawList* dl = ImGui::GetWindowDrawList();
	const float h = ImGui::GetTextLineHeight();
	const float r = h * 0.32f;
	const ImVec2 cursor = ImGui::GetCursorScreenPos();
	const ImVec2 center(cursor.x + r + 2.0f, cursor.y + h * 0.5f);
	dl->AddCircleFilled(center, r, color);
	ImGui::Dummy(ImVec2(r * 2.0f + 4.0f, h));
	ImGui::SameLine();
}

void StatusBadge(const char* text, StatusTone tone)
{
	if (!text || !text[0]) return;
	const ImVec4 col = StatusColor(tone);
	const ImVec2 textSize = ImGui::CalcTextSize(text);
	const ImVec2 pad(8.0f, 2.0f);
	const ImVec2 rectMin = ImGui::GetCursorScreenPos();
	const ImVec2 rectMax(rectMin.x + textSize.x + pad.x * 2.0f, rectMin.y + textSize.y + pad.y * 2.0f);

	ImDrawList* dl = ImGui::GetWindowDrawList();
	const float rounding = (rectMax.y - rectMin.y) * 0.5f;
	dl->AddRectFilled(rectMin, rectMax, ImGui::GetColorU32(ImVec4(col.x, col.y, col.z, 0.18f)), rounding);
	dl->AddRect(rectMin, rectMax, ImGui::GetColorU32(ImVec4(col.x, col.y, col.z, 0.55f)), rounding);
	dl->AddText(ImVec2(rectMin.x + pad.x, rectMin.y + pad.y), ImGui::GetColorU32(col), text);

	ImGui::Dummy(ImVec2(rectMax.x - rectMin.x, rectMax.y - rectMin.y));
}

void RightAlignText(const char* text, ImVec4 color, bool colored)
{
	if (!text) return;
	const float colW = ImGui::GetContentRegionAvail().x;
	const float textW = ImGui::CalcTextSize(text).x;
	const float pad = (colW > textW) ? (colW - textW) : 0.0f;
	ImGui::Dummy(ImVec2(pad, 0.0f));
	ImGui::SameLine(0.0f, 0.0f);
	if (colored) {
		ImGui::TextColored(color, "%s", text);
	}
	else {
		ImGui::TextDisabled("%s", text);
	}
}

bool CopyToClipboardButton(const char* id, const char* text)
{
	if (!id || !text) return false;
	ImGui::PushID(id);
	const bool clicked = ImGui::SmallButton("Copy");
	ImGui::PopID();
	if (clicked) {
		ImGui::SetClipboardText(text);
	}
	TooltipForLastItem("Copy this path to the clipboard");
	return clicked;
}

bool CopyWideTextToClipboard(std::wstring_view text)
{
	if (!OpenClipboard(nullptr)) return false;
	EmptyClipboard();
	const size_t bytes = (text.size() + 1) * sizeof(wchar_t);
	HGLOBAL h = GlobalAlloc(GMEM_MOVEABLE, bytes);
	if (!h) {
		CloseClipboard();
		return false;
	}
	if (auto* buf = static_cast<wchar_t*>(GlobalLock(h))) {
		memcpy(buf, text.data(), text.size() * sizeof(wchar_t));
		buf[text.size()] = L'\0';
		GlobalUnlock(h);
		SetClipboardData(CF_UNICODETEXT, h);
	}
	else {
		GlobalFree(h);
		CloseClipboard();
		return false;
	}
	CloseClipboard();
	return true;
}

void DrawFilePath(const char* path)
{
	if (!path || !path[0]) return;
	const float avail = ImGui::GetContentRegionAvail().x;
	const float full = ImGui::CalcTextSize(path).x;
	if (avail <= 1.0f || full <= avail) {
		ImGui::TextUnformatted(path);
		TooltipOnHover(path);
		return;
	}

	std::string s(path);
	const std::string ellipsis = "...";
	while (s.size() > ellipsis.size() + 4) {
		const size_t mid = s.size() / 2;
		s.erase(mid - 1, 2);
		std::string candidate = s.substr(0, s.size() / 2) + ellipsis + s.substr(s.size() / 2);
		if (ImGui::CalcTextSize(candidate.c_str()).x <= avail) {
			ImGui::TextUnformatted(candidate.c_str());
			TooltipOnHover(path);
			return;
		}
	}
	ImGui::TextUnformatted(path);
	TooltipOnHover(path);
}

std::string FormatByteCount(uint64_t bytes)
{
	char buf[64];
	if (bytes >= (1ull << 20)) {
		snprintf(buf, sizeof buf, "%.1f MB", static_cast<double>(bytes) / static_cast<double>(1ull << 20));
	}
	else if (bytes >= (1ull << 10)) {
		snprintf(buf, sizeof buf, "%.0f KB", static_cast<double>(bytes) / static_cast<double>(1ull << 10));
	}
	else {
		snprintf(buf, sizeof buf, "%llu B", static_cast<unsigned long long>(bytes));
	}
	return buf;
}

std::string FormatByteCountOrUnknown(int64_t bytes)
{
	if (bytes < 0) return "unknown";
	return FormatByteCount(static_cast<uint64_t>(bytes));
}

std::string FormatFileAgeSeconds(uint64_t ageSeconds)
{
	char buf[64];
	if (ageSeconds < 60) {
		snprintf(buf, sizeof buf, "%llus ago", static_cast<unsigned long long>(ageSeconds));
	}
	else if (ageSeconds < 3600) {
		snprintf(buf, sizeof buf, "%llum ago", static_cast<unsigned long long>(ageSeconds / 60));
	}
	else if (ageSeconds < 86400) {
		snprintf(buf, sizeof buf, "%lluh ago", static_cast<unsigned long long>(ageSeconds / 3600));
	}
	else {
		snprintf(buf, sizeof buf, "%llud ago", static_cast<unsigned long long>(ageSeconds / 86400));
	}
	return buf;
}

std::string FormatFileAgeFromFileTime(uint64_t mtimeFileTime, uint64_t nowFileTime)
{
	if (mtimeFileTime > nowFileTime) return "in the future";
	const uint64_t deltaTicks = nowFileTime - mtimeFileTime;
	return FormatFileAgeSeconds(deltaTicks / 10000000ull);
}

std::string FormatFileAgeFromFileTime(uint64_t mtimeFileTime)
{
	FILETIME nowFt{};
	GetSystemTimeAsFileTime(&nowFt);
	const uint64_t now = (static_cast<uint64_t>(nowFt.dwHighDateTime) << 32) | nowFt.dwLowDateTime;
	return FormatFileAgeFromFileTime(mtimeFileTime, now);
}

void DrawBanner(const char* title, const char* detail, ImVec4 background, ImVec4 titleColor, ImVec4 detailColor)
{
	const ImGuiStyle& style = ImGui::GetStyle();
	const float width = ImGui::GetContentRegionAvail().x;
	if (width <= 1.0f) return;

	const ImVec2 padding(12.0f, 8.0f);
	const float wrapWidth = std::max(1.0f, width - padding.x * 2.0f);
	const bool hasTitle = title && title[0] != '\0';
	const bool hasDetail = detail && detail[0] != '\0';
	const float titleHeight = hasTitle ? ImGui::CalcTextSize(title, nullptr, false, wrapWidth).y : 0.0f;
	const float detailHeight = hasDetail ? ImGui::CalcTextSize(detail, nullptr, false, wrapWidth).y : 0.0f;
	const float gap = (hasTitle && hasDetail) ? style.ItemSpacing.y * 0.5f : 0.0f;
	const float height = padding.y * 2.0f + titleHeight + gap + detailHeight;

	const ImVec2 rectMin = ImGui::GetCursorScreenPos();
	const ImVec2 rectMax(rectMin.x + width, rectMin.y + height);
	ImDrawList* dl = ImGui::GetWindowDrawList();
	dl->AddRectFilled(rectMin, rectMax, ImGui::GetColorU32(background), 6.0f);
	dl->AddRect(rectMin, rectMax, ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 0.12f)), 6.0f);

	ImGui::SetCursorScreenPos(ImVec2(rectMin.x + padding.x, rectMin.y + padding.y));
	ImGui::PushTextWrapPos(rectMax.x - padding.x);
	if (hasTitle) {
		ScopedStyleColor color(ImGuiCol_Text, titleColor);
		ImGui::TextWrapped("%s", title);
	}
	if (hasDetail) {
		ScopedStyleColor color(ImGuiCol_Text, detailColor);
		ImGui::TextWrapped("%s", detail);
	}
	ImGui::PopTextWrapPos();
	ImGui::SetCursorScreenPos(ImVec2(rectMin.x, rectMax.y));
	ImGui::Dummy(ImVec2(width, 0.0f));
}

void DrawErrorBanner(const char* title, const char* detail)
{
	const SemanticPalette& pal = GetPalette();
	DrawBanner(title, detail, pal.bannerErrorBg, pal.bannerErrorTitle, pal.bannerErrorDetail);
}

void DrawWaitingBanner(const char* message)
{
	const SemanticPalette& pal = GetPalette();
	DrawBanner("Waiting", message, pal.bannerWarnBg, pal.bannerWarnTitle, pal.bannerWarnDetail);
}

void DrawInfoBanner(const char* title, const char* detail)
{
	const SemanticPalette& pal = GetPalette();
	DrawBanner(title, detail, pal.bannerInfoBg, pal.bannerInfoText, pal.bannerInfoText);
}

void DrawSectionHeading(const char* label)
{
	if (!label) return;
	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::SeparatorText(label);
}

bool ActionButtonWidget(const ActionButton& button)
{
	DisabledSection disabled(button.disabled, button.disabledTooltip);
	const bool clicked = ImGui::Button(button.label ? button.label : "", button.size);
	if (button.disabled) {
		disabled.AttachReasonTooltip();
	}
	else {
		TooltipForLastItem(button.tooltip);
	}
	if (clicked && !button.disabled && button.onClick) {
		button.onClick();
	}
	return clicked && !button.disabled;
}

int DrawActionRow(const char* id, std::initializer_list<ActionButton> buttons)
{
	ImGui::PushID(id ? id : "##action_row");
	int clickedIndex = -1;
	int index = 0;
	for (const ActionButton& button : buttons) {
		if (index > 0) ImGui::SameLine();
		if (ActionButtonWidget(button)) clickedIndex = index;
		++index;
	}
	ImGui::PopID();
	return clickedIndex;
}

} // namespace openvr_pair::overlay::ui
