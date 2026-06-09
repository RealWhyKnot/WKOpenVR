#pragma once

#include "Theme.h"

#include <imgui.h>

#include <functional>
#include <initializer_list>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

namespace openvr_pair::overlay::ui {

enum class StatusTone
{
	Ok,
	Pending,
	Error,
	Warn,
	Info,
	Idle,
};

struct ScopedStyleColor
{
	ScopedStyleColor(ImGuiCol idx, ImVec4 color);
	~ScopedStyleColor();
	ScopedStyleColor(const ScopedStyleColor&) = delete;
	ScopedStyleColor& operator=(const ScopedStyleColor&) = delete;
};

// RAII for pushing several style colors at once -- replaces hand-balanced
// PushStyleColor/PopStyleColor chains (a common pop-count footgun).
struct ScopedStyleColors
{
	explicit ScopedStyleColors(std::initializer_list<std::pair<ImGuiCol, ImVec4>> colors);
	~ScopedStyleColors();
	ScopedStyleColors(const ScopedStyleColors&) = delete;
	ScopedStyleColors& operator=(const ScopedStyleColors&) = delete;

private:
	int count_;
};

struct DisabledSection
{
	bool active;
	const char* why;
	DisabledSection(bool disabled, const char* whyTooltip = nullptr);
	~DisabledSection();
	void AttachReasonTooltip() const;
	DisabledSection(const DisabledSection&) = delete;
	DisabledSection& operator=(const DisabledSection&) = delete;
};

struct ActionButton
{
	const char* label = nullptr;
	const char* tooltip = nullptr;
	bool disabled = false;
	const char* disabledTooltip = nullptr;
	ImVec2 size = ImVec2(0.0f, 0.0f);
	std::function<void()> onClick;
};

ImVec4 StatusColor(StatusTone tone);

void TooltipForLastItem(const char* tooltip);
void TooltipOnHover(const char* tooltip);
bool CheckboxWithTooltip(const char* label, bool* value, const char* tooltip);
bool SliderIntWithTooltip(const char* label, int* value, int min, int max, const char* format, const char* tooltip);
bool SliderFloatWithTooltip(const char* label, float* value, float min, float max, const char* format,
                            const char* tooltip, ImGuiSliderFlags flags = 0);
bool RadioButtonWithTooltip(const char* label, bool active, const char* tooltip);
void DrawHelpMarker(const char* tooltip);

void DrawTextWrapped(const char* text);
void DrawTextWrapped(const std::string& text);
void DrawColoredText(const char* text, ImVec4 color);
void DrawStatusText(const char* text, StatusTone tone);
void DrawEmptyState(const char* text);
void DrawStatusDot(ImU32 color);
// Inline pill/chip: a rounded, tone-tinted background behind tone-colored
// label text. Advances the cursor so it flows like any other inline item.
void StatusBadge(const char* text, StatusTone tone);
void RightAlignText(const char* text, ImVec4 color, bool colored = true);

bool CopyToClipboardButton(const char* id, const char* text);
bool CopyWideTextToClipboard(std::wstring_view text);
void DrawFilePath(const char* path);

std::string FormatByteCount(uint64_t bytes);
std::string FormatByteCountOrUnknown(int64_t bytes);
std::string FormatFileAgeSeconds(uint64_t ageSeconds);
std::string FormatFileAgeFromFileTime(uint64_t mtimeFileTime, uint64_t nowFileTime);
std::string FormatFileAgeFromFileTime(uint64_t mtimeFileTime);

void DrawBanner(const char* title, const char* detail, ImVec4 background, ImVec4 titleColor, ImVec4 detailColor);
void DrawErrorBanner(const char* title, const char* detail);
void DrawWaitingBanner(const char* message);
void DrawInfoBanner(const char* title, const char* detail);
void DrawSectionHeading(const char* label);

bool ActionButtonWidget(const ActionButton& button);
int DrawActionRow(const char* id, std::initializer_list<ActionButton> buttons);

} // namespace openvr_pair::overlay::ui
