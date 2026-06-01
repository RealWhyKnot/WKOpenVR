#include "Theme.h"

#include "ShellContext.h"
#include "ShellSettings.h"

#include <imgui.h>

#include <string>

namespace openvr_pair::overlay::ui {

namespace {

// Legacy palette: matches the hardcoded colors the app shipped with before
// the theme system. Every other theme is defined as deltas off this base so
// adding a new theme stays focused on "what changes" rather than the full
// table.
SemanticPalette LegacyPalette()
{
	SemanticPalette p{};
	p.statusOk      = ImVec4(0.45f, 0.85f, 0.45f, 1.0f);
	p.statusPending = ImVec4(0.95f, 0.70f, 0.40f, 1.0f);
	p.statusError   = ImVec4(0.95f, 0.45f, 0.45f, 1.0f);
	p.statusWarn    = ImVec4(0.95f, 0.85f, 0.40f, 1.0f);
	p.statusInfo    = ImVec4(0.55f, 0.75f, 0.95f, 1.0f);
	p.statusIdle    = ImVec4(0.55f, 0.55f, 0.55f, 1.0f);

	p.bannerErrorBg     = ImVec4(0.42f, 0.06f, 0.06f, 1.0f);
	p.bannerErrorTitle  = ImVec4(1.00f, 0.88f, 0.88f, 1.0f);
	p.bannerErrorDetail = ImVec4(1.00f, 0.96f, 0.96f, 1.0f);

	p.bannerWarnBg     = ImVec4(0.45f, 0.33f, 0.08f, 1.0f);
	p.bannerWarnTitle  = ImVec4(1.00f, 0.92f, 0.72f, 1.0f);
	p.bannerWarnDetail = ImVec4(1.00f, 0.96f, 0.86f, 1.0f);

	p.bannerInfoBg     = ImVec4(0.18f, 0.30f, 0.45f, 1.0f);
	p.bannerInfoBorder = ImVec4(0.40f, 0.65f, 0.95f, 1.0f);
	p.bannerInfoText   = ImVec4(0.90f, 0.95f, 1.00f, 1.0f);

	p.dotOk      = IM_COL32(80, 200, 120, 255);
	p.dotPending = IM_COL32(220, 170, 60, 255);
	p.dotError   = IM_COL32(220, 80, 80, 255);

	p.plotThresholdLine = ImVec4(1.00f, 0.40f, 0.40f, 1.0f);
	p.plotAxisLow       = ImVec4(0.85f, 0.35f, 0.35f, 0.25f);
	p.plotAxisHigh      = ImVec4(0.35f, 0.85f, 0.35f, 0.25f);

	p.windowBg = ImVec4(0.07f, 0.07f, 0.08f, 1.00f);
	return p;
}

SemanticPalette DarkPalette()
{
	SemanticPalette p = LegacyPalette();
	// Cooler neutral background -- distinguishes Dark from Legacy at a
	// glance without retuning every semantic accent.
	p.windowBg = ImVec4(0.08f, 0.09f, 0.11f, 1.00f);
	return p;
}

SemanticPalette LightPalette()
{
	SemanticPalette p{};
	// Saturated semantic colors that read on near-white backgrounds.
	p.statusOk      = ImVec4(0.10f, 0.55f, 0.10f, 1.0f);
	p.statusPending = ImVec4(0.75f, 0.45f, 0.05f, 1.0f);
	p.statusError   = ImVec4(0.80f, 0.15f, 0.15f, 1.0f);
	p.statusWarn    = ImVec4(0.70f, 0.50f, 0.05f, 1.0f);
	p.statusInfo    = ImVec4(0.15f, 0.40f, 0.75f, 1.0f);
	p.statusIdle    = ImVec4(0.45f, 0.45f, 0.45f, 1.0f);

	p.bannerErrorBg     = ImVec4(0.97f, 0.85f, 0.85f, 1.0f);
	p.bannerErrorTitle  = ImVec4(0.55f, 0.10f, 0.10f, 1.0f);
	p.bannerErrorDetail = ImVec4(0.35f, 0.10f, 0.10f, 1.0f);

	p.bannerWarnBg     = ImVec4(0.97f, 0.91f, 0.74f, 1.0f);
	p.bannerWarnTitle  = ImVec4(0.55f, 0.40f, 0.05f, 1.0f);
	p.bannerWarnDetail = ImVec4(0.35f, 0.27f, 0.05f, 1.0f);

	p.bannerInfoBg     = ImVec4(0.85f, 0.92f, 1.00f, 1.0f);
	p.bannerInfoBorder = ImVec4(0.40f, 0.55f, 0.80f, 1.0f);
	p.bannerInfoText   = ImVec4(0.10f, 0.25f, 0.50f, 1.0f);

	p.dotOk      = IM_COL32(40, 140, 50, 255);
	p.dotPending = IM_COL32(190, 120, 20, 255);
	p.dotError   = IM_COL32(190, 40, 40, 255);

	p.plotThresholdLine = ImVec4(0.75f, 0.20f, 0.20f, 1.0f);
	p.plotAxisLow       = ImVec4(0.85f, 0.35f, 0.35f, 0.20f);
	p.plotAxisHigh      = ImVec4(0.25f, 0.65f, 0.25f, 0.20f);

	p.windowBg = ImVec4(0.94f, 0.94f, 0.95f, 1.0f);
	return p;
}

SemanticPalette HighContrastPalette()
{
	SemanticPalette p{};
	p.statusOk      = ImVec4(0.30f, 1.00f, 0.30f, 1.0f);
	p.statusPending = ImVec4(1.00f, 0.85f, 0.30f, 1.0f);
	p.statusError   = ImVec4(1.00f, 0.30f, 0.30f, 1.0f);
	p.statusWarn    = ImVec4(1.00f, 0.95f, 0.30f, 1.0f);
	p.statusInfo    = ImVec4(0.55f, 0.85f, 1.00f, 1.0f);
	p.statusIdle    = ImVec4(0.75f, 0.75f, 0.75f, 1.0f);

	p.bannerErrorBg     = ImVec4(0.55f, 0.00f, 0.00f, 1.0f);
	p.bannerErrorTitle  = ImVec4(1.00f, 1.00f, 1.00f, 1.0f);
	p.bannerErrorDetail = ImVec4(1.00f, 1.00f, 1.00f, 1.0f);

	p.bannerWarnBg     = ImVec4(0.55f, 0.40f, 0.00f, 1.0f);
	p.bannerWarnTitle  = ImVec4(1.00f, 1.00f, 1.00f, 1.0f);
	p.bannerWarnDetail = ImVec4(1.00f, 1.00f, 1.00f, 1.0f);

	p.bannerInfoBg     = ImVec4(0.05f, 0.20f, 0.55f, 1.0f);
	p.bannerInfoBorder = ImVec4(1.00f, 1.00f, 1.00f, 1.0f);
	p.bannerInfoText   = ImVec4(1.00f, 1.00f, 1.00f, 1.0f);

	p.dotOk      = IM_COL32(50, 255, 50, 255);
	p.dotPending = IM_COL32(255, 200, 50, 255);
	p.dotError   = IM_COL32(255, 50, 50, 255);

	p.plotThresholdLine = ImVec4(1.00f, 0.30f, 0.30f, 1.0f);
	p.plotAxisLow       = ImVec4(1.00f, 0.30f, 0.30f, 0.35f);
	p.plotAxisHigh      = ImVec4(0.30f, 1.00f, 0.30f, 0.35f);

	p.windowBg = ImVec4(0.00f, 0.00f, 0.00f, 1.0f);
	return p;
}

SemanticPalette PureDarkPalette()
{
	// Slightly desaturated relative to Legacy so accents do not bloom
	// against pure-black panels on OLED displays.
	SemanticPalette p{};
	p.statusOk      = ImVec4(0.40f, 0.78f, 0.45f, 1.0f);
	p.statusPending = ImVec4(0.88f, 0.65f, 0.40f, 1.0f);
	p.statusError   = ImVec4(0.88f, 0.45f, 0.45f, 1.0f);
	p.statusWarn    = ImVec4(0.88f, 0.78f, 0.40f, 1.0f);
	p.statusInfo    = ImVec4(0.55f, 0.75f, 0.95f, 1.0f);
	p.statusIdle    = ImVec4(0.50f, 0.50f, 0.50f, 1.0f);

	p.bannerErrorBg     = ImVec4(0.30f, 0.05f, 0.05f, 1.0f);
	p.bannerErrorTitle  = ImVec4(0.95f, 0.80f, 0.80f, 1.0f);
	p.bannerErrorDetail = ImVec4(0.95f, 0.88f, 0.88f, 1.0f);

	p.bannerWarnBg     = ImVec4(0.30f, 0.22f, 0.05f, 1.0f);
	p.bannerWarnTitle  = ImVec4(0.95f, 0.85f, 0.65f, 1.0f);
	p.bannerWarnDetail = ImVec4(0.95f, 0.90f, 0.78f, 1.0f);

	p.bannerInfoBg     = ImVec4(0.08f, 0.18f, 0.30f, 1.0f);
	p.bannerInfoBorder = ImVec4(0.30f, 0.50f, 0.80f, 1.0f);
	p.bannerInfoText   = ImVec4(0.80f, 0.88f, 0.95f, 1.0f);

	p.dotOk      = IM_COL32(70, 180, 100, 255);
	p.dotPending = IM_COL32(200, 150, 50, 255);
	p.dotError   = IM_COL32(200, 70, 70, 255);

	p.plotThresholdLine = ImVec4(0.90f, 0.35f, 0.35f, 1.0f);
	p.plotAxisLow       = ImVec4(0.78f, 0.32f, 0.32f, 0.25f);
	p.plotAxisHigh      = ImVec4(0.32f, 0.78f, 0.32f, 0.25f);

	p.windowBg = ImVec4(0.00f, 0.00f, 0.00f, 1.0f);
	return p;
}

// Replace the active ImGui style's color table. Called from SetTheme after
// the palette swap so feature-module DrawTab implementations see consistent
// chrome + accent colors on the very next ImGui::NewFrame.
void ApplyLegacyImGuiColors()
{
	ImGui::StyleColorsDark();
}

void ApplyDarkImGuiColors()
{
	ImGui::StyleColorsDark();
	ImVec4 *c = ImGui::GetStyle().Colors;
	// Cooler neutrals: nudge backgrounds toward blue-gray and lift panel
	// contrast a touch above StyleColorsDark's defaults.
	c[ImGuiCol_WindowBg]        = ImVec4(0.08f, 0.09f, 0.11f, 1.00f);
	c[ImGuiCol_ChildBg]         = ImVec4(0.09f, 0.10f, 0.12f, 1.00f);
	c[ImGuiCol_PopupBg]         = ImVec4(0.09f, 0.10f, 0.12f, 0.96f);
	c[ImGuiCol_FrameBg]         = ImVec4(0.16f, 0.18f, 0.22f, 1.00f);
	c[ImGuiCol_FrameBgHovered]  = ImVec4(0.22f, 0.25f, 0.30f, 1.00f);
	c[ImGuiCol_FrameBgActive]   = ImVec4(0.26f, 0.30f, 0.36f, 1.00f);
	c[ImGuiCol_TitleBgActive]   = ImVec4(0.14f, 0.16f, 0.20f, 1.00f);
	c[ImGuiCol_Tab]             = ImVec4(0.13f, 0.15f, 0.19f, 1.00f);
	c[ImGuiCol_TabHovered]      = ImVec4(0.30f, 0.34f, 0.42f, 1.00f);
	c[ImGuiCol_TabActive]       = ImVec4(0.22f, 0.26f, 0.32f, 1.00f);
	c[ImGuiCol_Header]          = ImVec4(0.18f, 0.22f, 0.28f, 1.00f);
	c[ImGuiCol_HeaderHovered]   = ImVec4(0.24f, 0.28f, 0.36f, 1.00f);
	c[ImGuiCol_HeaderActive]    = ImVec4(0.28f, 0.34f, 0.42f, 1.00f);
}

void ApplyLightImGuiColors()
{
	ImGui::StyleColorsLight();
}

void ApplyHighContrastImGuiColors()
{
	ImGui::StyleColorsDark();
	ImVec4 *c = ImGui::GetStyle().Colors;
	c[ImGuiCol_WindowBg]       = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
	c[ImGuiCol_ChildBg]        = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
	c[ImGuiCol_PopupBg]        = ImVec4(0.00f, 0.00f, 0.00f, 0.96f);
	c[ImGuiCol_Text]           = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	c[ImGuiCol_TextDisabled]   = ImVec4(0.75f, 0.75f, 0.75f, 1.00f);
	c[ImGuiCol_Border]         = ImVec4(1.00f, 1.00f, 1.00f, 0.85f);
	c[ImGuiCol_Separator]      = ImVec4(1.00f, 1.00f, 1.00f, 0.55f);
	c[ImGuiCol_FrameBg]        = ImVec4(0.05f, 0.05f, 0.05f, 1.00f);
	c[ImGuiCol_FrameBgHovered] = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
	c[ImGuiCol_FrameBgActive]  = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
	c[ImGuiCol_Button]         = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
	c[ImGuiCol_ButtonHovered]  = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
	c[ImGuiCol_ButtonActive]   = ImVec4(0.45f, 0.45f, 0.45f, 1.00f);
	c[ImGuiCol_Tab]            = ImVec4(0.05f, 0.05f, 0.05f, 1.00f);
	c[ImGuiCol_TabHovered]     = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
	c[ImGuiCol_TabActive]      = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
	c[ImGuiCol_Header]         = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
	c[ImGuiCol_HeaderHovered]  = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
	c[ImGuiCol_HeaderActive]   = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);
}

void ApplyPureDarkImGuiColors()
{
	ImGui::StyleColorsDark();
	ImVec4 *c = ImGui::GetStyle().Colors;
	c[ImGuiCol_WindowBg]       = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
	c[ImGuiCol_ChildBg]        = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
	c[ImGuiCol_PopupBg]        = ImVec4(0.02f, 0.02f, 0.02f, 0.96f);
	c[ImGuiCol_FrameBg]        = ImVec4(0.06f, 0.06f, 0.06f, 1.00f);
	c[ImGuiCol_FrameBgHovered] = ImVec4(0.12f, 0.12f, 0.12f, 1.00f);
	c[ImGuiCol_FrameBgActive]  = ImVec4(0.18f, 0.18f, 0.18f, 1.00f);
	c[ImGuiCol_TitleBg]        = ImVec4(0.04f, 0.04f, 0.04f, 1.00f);
	c[ImGuiCol_TitleBgActive]  = ImVec4(0.06f, 0.06f, 0.06f, 1.00f);
	c[ImGuiCol_Tab]            = ImVec4(0.04f, 0.04f, 0.04f, 1.00f);
	c[ImGuiCol_TabHovered]     = ImVec4(0.18f, 0.18f, 0.18f, 1.00f);
	c[ImGuiCol_TabActive]      = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
	c[ImGuiCol_Header]         = ImVec4(0.08f, 0.08f, 0.08f, 1.00f);
	c[ImGuiCol_HeaderHovered]  = ImVec4(0.16f, 0.16f, 0.16f, 1.00f);
	c[ImGuiCol_HeaderActive]   = ImVec4(0.22f, 0.22f, 0.22f, 1.00f);
}

struct ThemeDef
{
	ThemeId id;
	const char *name;
	const char *caption;
	SemanticPalette (*makePalette)();
	void (*applyImGuiColors)();
};

const ThemeDef kThemes[] = {
	{ ThemeId::Legacy,       "Legacy",        "Original Space Calibrator look. Warm dark palette.", &LegacyPalette,       &ApplyLegacyImGuiColors       },
	{ ThemeId::Dark,         "Dark",          "Cooler neutral grays. Higher panel contrast.",       &DarkPalette,         &ApplyDarkImGuiColors         },
	{ ThemeId::Light,        "Light",         "Light backgrounds. For bright rooms or screenshots.",&LightPalette,        &ApplyLightImGuiColors        },
	{ ThemeId::HighContrast, "High Contrast", "Pure black, white text and borders. Accessibility.", &HighContrastPalette, &ApplyHighContrastImGuiColors },
	{ ThemeId::PureDark,     "Pure Dark",     "True-black backgrounds. Friendly to OLED displays.", &PureDarkPalette,     &ApplyPureDarkImGuiColors     },
};
static_assert(sizeof(kThemes) / sizeof(kThemes[0]) == (size_t)ThemeId::Count_,
	"kThemes must list every ThemeId in order");

// File-scope active state. SetTheme writes both; GetPalette reads palette.
// Initialised to Legacy so calls before InitThemeFromDisk return a usable
// reference (matters when feature modules construct during plugin init).
ThemeId g_currentId = ThemeId::Legacy;
SemanticPalette g_currentPalette = LegacyPalette();
std::wstring g_profileRoot;

bool ParseThemeId(const std::string &value, ThemeId &out)
{
	for (const auto &t : kThemes) {
		if (value == t.name) {
			out = t.id;
			return true;
		}
	}
	return false;
}

bool ReadThemeFromDisk(ThemeId &out)
{
	return ParseThemeId(openvr_pair::overlay::ReadShellSetting(g_profileRoot, "theme", ""), out);
}

void WriteThemeToDisk(ThemeId id)
{
	openvr_pair::overlay::WriteShellSetting(g_profileRoot, "theme", ThemeName(id));
}

const ThemeDef &FindTheme(ThemeId id)
{
	for (const auto &t : kThemes) {
		if (t.id == id) return t;
	}
	return kThemes[0];
}

} // namespace

const SemanticPalette &GetPalette()
{
	return g_currentPalette;
}

ThemeId GetCurrentThemeId()
{
	return g_currentId;
}

const char *ThemeName(ThemeId id)
{
	return FindTheme(id).name;
}

const char *ThemeCaption(ThemeId id)
{
	return FindTheme(id).caption;
}

void SetTheme(ThemeId id)
{
	const ThemeDef &t = FindTheme(id);
	t.applyImGuiColors();
	g_currentPalette = t.makePalette();
	g_currentId = t.id;
	WriteThemeToDisk(t.id);
}

void InitThemeFromDisk(const ShellContext &context)
{
	g_profileRoot = context.profileRoot;
	ThemeId id = ThemeId::Legacy;
	ReadThemeFromDisk(id);  // failure leaves id as Legacy
	const ThemeDef &t = FindTheme(id);
	t.applyImGuiColors();
	g_currentPalette = t.makePalette();
	g_currentId = t.id;
	// Do NOT WriteThemeToDisk here: first-launch users would otherwise get
	// a shell.txt they never asked for. Persistence happens on the first
	// explicit SetTheme call from the Themes tab.
}

} // namespace openvr_pair::overlay::ui
