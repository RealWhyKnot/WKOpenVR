#pragma once

#include <imgui.h>

// Theme + semantic-color palette for the umbrella shell and feature modules.
//
// All overlay code routes "state colors" (Enabled green, Pending amber, error
// banner red, etc.) through GetPalette() instead of hand-rolling ImVec4
// literals at the call site. This lets a theme switch (Themes tab) recolor
// the entire UI live without touching feature-module call sites.
//
// First-launch default is ThemeId::Legacy, which reproduces the exact colors
// the app shipped with before the theme system existed.
namespace openvr_pair::overlay {
struct ShellContext;
}

namespace openvr_pair::overlay::ui {

// Semantic tokens used by the shell, footer, banners, and feature tabs.
// Each token names a role ("statusOk"), not a hue ("green"), so a Light or
// HighContrast theme can re-tune the actual color while keeping the call
// site stable.
struct SemanticPalette
{
	ImVec4 statusOk;      // Enabled, connected, host running
	ImVec4 statusPending; // Enabling/Disabling in flight, waiting
	ImVec4 statusError;   // disconnected, failure, sync error
	ImVec4 statusWarn;    // drift detected, stale source
	ImVec4 statusInfo;    // dev-build badge, maintainer accent
	ImVec4 statusIdle;    // Disabled, never synced (muted)

	ImVec4 bannerErrorBg;
	ImVec4 bannerErrorTitle;
	ImVec4 bannerErrorDetail;

	ImVec4 bannerWarnBg;
	ImVec4 bannerWarnTitle;
	ImVec4 bannerWarnDetail;

	ImVec4 bannerInfoBg;
	ImVec4 bannerInfoBorder;
	ImVec4 bannerInfoText;

	// Pre-packed dot colors for ImDrawList::AddCircleFilled callers.
	ImU32 dotOk;
	ImU32 dotPending;
	ImU32 dotError;

	// Plot accents used by calibration's Graphs tab and any future plots.
	ImVec4 plotThresholdLine;
	ImVec4 plotAxisLow;
	ImVec4 plotAxisHigh;

	// FBO clear color -- the shell reads this so a Light theme does not
	// leave a dark border around the content rectangle when the window
	// is larger than the ImGui-drawn area.
	ImVec4 windowBg;
};

enum class ThemeId
{
	Legacy = 0,
	Dark,
	Light,
	HighContrast,
	PureDark,
	Count_,
};

// Read the active theme's palette. Always returns a valid reference, even
// before InitThemeFromDisk runs (defaults to the Legacy palette).
const SemanticPalette& GetPalette();

ThemeId GetCurrentThemeId();
const char* ThemeName(ThemeId id);
const char* ThemeCaption(ThemeId id);

// Apply a theme: rewrites ImGui::GetStyle().Colors[] in place, swaps the
// active palette, and (when shellProfileRoot is non-empty) persists the
// selection to shell.txt.
void SetTheme(ThemeId id);

// One-shot at startup. Reads shell.txt under context.profileRoot; on parse
// failure or missing file, defaults to ThemeId::Legacy. Also calls SetTheme
// so ImGui colors are configured before the first frame.
void InitThemeFromDisk(const ShellContext& context);

} // namespace openvr_pair::overlay::ui
