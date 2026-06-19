#include <gtest/gtest.h>

#include "FeaturePlugin.h"
#include "ShellFooter.h"
#include "ShellSettings.h"
#include "ShellUiLogic.h"
#include "UpdateNoticeLogic.h"
#include "UiCore.h"
#include "UiControls.h"

#include <imgui.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ui = openvr_pair::overlay::ui;

namespace {

class UiCoreTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO();
		io.IniFilename = nullptr;
		io.Fonts->AddFontDefault();
		unsigned char* pixels = nullptr;
		int width = 0;
		int height = 0;
		io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
		openvr_pair::overlay::ui::ApplyOverlayStyle();
		openvr_pair::overlay::ui::SetTheme(openvr_pair::overlay::ui::ThemeId::Legacy);
	}

	void TearDown() override { ImGui::DestroyContext(); }

	template <typename Body> void RenderAt(const ImVec2& size, Body&& body)
	{
		ImGuiIO& io = ImGui::GetIO();
		io.DisplaySize = size;
		io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

		ImGui::NewFrame();
		ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
		ImGui::SetNextWindowSize(size);
		ImGui::Begin("ui_core_test", nullptr,
		             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
		                 ImGuiWindowFlags_NoSavedSettings);
		std::forward<Body>(body)();
		ImGui::End();
		ImGui::Render();

		ASSERT_NE(nullptr, ImGui::GetDrawData());
		EXPECT_GT(ImGui::GetDrawData()->TotalVtxCount, 0);
	}
};

} // namespace

TEST(FeaturePlugin, ChannelHelpersRouteReleaseAndDevelopmentModules)
{
	using openvr_pair::overlay::FeaturePluginChannel;

	EXPECT_TRUE(openvr_pair::overlay::ShouldShowInModulesTab(FeaturePluginChannel::Release));
	EXPECT_TRUE(openvr_pair::overlay::ShouldShowInModulesTab(FeaturePluginChannel::Development));
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInModulesTab(FeaturePluginChannel::DevTools));

	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInDevModuleList(FeaturePluginChannel::Release));
	EXPECT_TRUE(openvr_pair::overlay::ShouldShowInDevModuleList(FeaturePluginChannel::Development));
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInDevModuleList(FeaturePluginChannel::DevTools));
}

TEST(ShellUiLogic, DesktopDefaultOnlySelectsInDesktopMode)
{
	EXPECT_TRUE(
	    openvr_pair::overlay::ShouldSelectDesktopDefaultTab(false, "enable_questapp.flag", "enable_questapp.flag", ""));
	EXPECT_FALSE(
	    openvr_pair::overlay::ShouldSelectDesktopDefaultTab(true, "enable_questapp.flag", "enable_questapp.flag", ""));
	EXPECT_FALSE(openvr_pair::overlay::ShouldSelectDesktopDefaultTab(false, "enable_smoothing.flag",
	                                                                 "enable_questapp.flag", ""));
	EXPECT_FALSE(openvr_pair::overlay::ShouldSelectDesktopDefaultTab(false, "enable_questapp.flag",
	                                                                 "enable_questapp.flag", "enable_questapp.flag"));
}

TEST(ShellUiLogic, FeaturePickerSelectionKeepsTopNavStable)
{
	const std::vector<std::string_view> installed = {"enable_smoothing.flag", "enable_inputhealth.flag"};

	auto selection = openvr_pair::overlay::ResolveFeaturePickerSelection(false, "enable_smoothing.flag",
	                                                                     "enable_inputhealth.flag", "", installed);
	EXPECT_EQ("enable_inputhealth.flag", std::string(selection.flag));
	EXPECT_TRUE(selection.applyDesktopDefault);

	selection = openvr_pair::overlay::ResolveFeaturePickerSelection(
	    false, "enable_smoothing.flag", "enable_inputhealth.flag", "enable_inputhealth.flag", installed);
	EXPECT_EQ("enable_smoothing.flag", std::string(selection.flag));
	EXPECT_FALSE(selection.applyDesktopDefault);

	selection = openvr_pair::overlay::ResolveFeaturePickerSelection(false, "missing.flag", "", "", installed);
	EXPECT_EQ("enable_smoothing.flag", std::string(selection.flag));
	EXPECT_FALSE(selection.applyDesktopDefault);

	const std::vector<std::string_view> none;
	selection = openvr_pair::overlay::ResolveFeaturePickerSelection(false, "missing.flag", "", "", none);
	EXPECT_TRUE(selection.flag.empty());
	EXPECT_FALSE(selection.applyDesktopDefault);
}

TEST(ShellUiLogic, FeatureContentTabsRequireEffectiveEnabledModule)
{
	EXPECT_TRUE(openvr_pair::overlay::ShouldShowFeatureContentTab(true, false, false));
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowFeatureContentTab(false, false, false));
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowFeatureContentTab(true, true, false));
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowFeatureContentTab(true, false, true));
}

TEST(ShellUiLogic, ModuleTabOrderParsesAndSerializesValidFlags)
{
	const std::vector<std::string> parsed = openvr_pair::overlay::ParseModuleTabOrderSetting(
	    " enable_smoothing.flag ; invalid ; enable_inputhealth.flag ; enable_smoothing.flag ; ../bad.flag ");

	ASSERT_EQ(2u, parsed.size());
	EXPECT_EQ("enable_smoothing.flag", parsed[0]);
	EXPECT_EQ("enable_inputhealth.flag", parsed[1]);
	EXPECT_EQ("enable_smoothing.flag;enable_inputhealth.flag",
	          openvr_pair::overlay::SerializeModuleTabOrderSetting(parsed));
}

TEST(ShellUiLogic, ModuleTabOrderAppliesSavedOrderAndAppendsNewModules)
{
	const std::vector<std::string> saved = {"enable_captions.flag", "enable_missing.flag", "enable_smoothing.flag",
	                                        "enable_smoothing.flag"};
	const std::vector<std::string_view> available = {"enable_smoothing.flag", "enable_inputhealth.flag",
	                                                 "enable_captions.flag"};

	const std::vector<std::string> resolved = openvr_pair::overlay::ResolveModuleTabOrder(saved, available);

	ASSERT_EQ(3u, resolved.size());
	EXPECT_EQ("enable_captions.flag", resolved[0]);
	EXPECT_EQ("enable_smoothing.flag", resolved[1]);
	EXPECT_EQ("enable_inputhealth.flag", resolved[2]);
}

TEST(ShellUiLogic, ModuleTabOrderMovesWithinBounds)
{
	std::vector<std::string> order = {"enable_inputhealth.flag", "enable_smoothing.flag", "enable_captions.flag"};

	EXPECT_TRUE(openvr_pair::overlay::MoveModuleTabOrder(order, "enable_smoothing.flag", -1));
	EXPECT_EQ("enable_smoothing.flag", order[0]);
	EXPECT_EQ("enable_inputhealth.flag", order[1]);
	EXPECT_EQ("enable_captions.flag", order[2]);

	EXPECT_FALSE(openvr_pair::overlay::MoveModuleTabOrder(order, "enable_smoothing.flag", -1));
	EXPECT_FALSE(openvr_pair::overlay::MoveModuleTabOrder(order, "enable_missing.flag", 1));

	EXPECT_TRUE(openvr_pair::overlay::MoveModuleTabOrder(order, "enable_inputhealth.flag", 1));
	EXPECT_EQ("enable_smoothing.flag", order[0]);
	EXPECT_EQ("enable_captions.flag", order[1]);
	EXPECT_EQ("enable_inputhealth.flag", order[2]);
}

TEST(ShellUiLogic, CalibrationPluginOwnsUnifiedLogsPanel)
{
	EXPECT_TRUE(openvr_pair::overlay::IsDefaultLogsPanelPlugin("enable_calibration.flag"));
	EXPECT_FALSE(openvr_pair::overlay::IsDefaultLogsPanelPlugin("enable_smoothing.flag"));
	EXPECT_FALSE(openvr_pair::overlay::IsDefaultLogsPanelPlugin(nullptr));
}

TEST(ShellUiLogic, TransientStatusExpiresOnlyAfterDeadline)
{
	EXPECT_FALSE(openvr_pair::overlay::ShouldClearTransientStatus(10.0, 0.0));
	EXPECT_FALSE(openvr_pair::overlay::ShouldClearTransientStatus(10.0, 10.1));
	EXPECT_TRUE(openvr_pair::overlay::ShouldClearTransientStatus(10.0, 10.0));
	EXPECT_TRUE(openvr_pair::overlay::ShouldClearTransientStatus(10.2, 10.0));
}

TEST(ShellFooter, ResolvesConnectionStateLikeSpaceCalibrator)
{
	using openvr_pair::overlay::ResolveShellFooterConnectionState;
	using openvr_pair::overlay::ShellFooterConnectionState;

	EXPECT_EQ(ShellFooterConnectionState::Connected, ResolveShellFooterConnectionState(true, false));
	EXPECT_EQ(ShellFooterConnectionState::WaitingForSteamVR, ResolveShellFooterConnectionState(false, false));
	EXPECT_EQ(ShellFooterConnectionState::Disconnected, ResolveShellFooterConnectionState(false, true));
}

TEST(ShellSettings, PreservesMultipleShellKeys)
{
	const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
	const std::filesystem::path dir =
	    std::filesystem::temp_directory_path() / ("wkopenvr-shellsettings-test-" + std::to_string(unique));

	std::filesystem::remove_all(dir);
	std::filesystem::create_directories(dir);

	EXPECT_TRUE(openvr_pair::overlay::WriteShellSetting(dir.wstring(), "theme", "Dark"));
	EXPECT_TRUE(
	    openvr_pair::overlay::WriteShellSetting(dir.wstring(), "desktop_default_module", "enable_smoothing.flag"));

	EXPECT_EQ("Dark", openvr_pair::overlay::ReadShellSetting(dir.wstring(), "theme", ""));
	EXPECT_EQ("enable_smoothing.flag",
	          openvr_pair::overlay::ReadShellSetting(dir.wstring(), "desktop_default_module", ""));

	std::filesystem::remove_all(dir);
}

TEST_F(UiCoreTest, PanelAndSettingTableRenderAtSmallAndDashboardSizes)
{
	for (const ImVec2 size : {ImVec2(640.0f, 480.0f), ImVec2(1200.0f, 780.0f)}) {
		RenderAt(size, [] {
			bool enabled = true;
			int amount = 42;

			openvr_pair::overlay::ui::DrawPanel("Settings", [&] {
				openvr_pair::overlay::ui::DrawSettingTable(
				    "settings_grid", 160.0f, [&](openvr_pair::overlay::ui::SettingTableScope& table) {
					    openvr_pair::overlay::ui::SettingRow(table, "Enabled", [&] {
						    openvr_pair::overlay::ui::CheckboxWithTooltip("##enabled", &enabled,
						                                                  "Controls whether this setting is active.");
					    });
					    openvr_pair::overlay::ui::SettingRow(table, "Amount", [&] {
						    openvr_pair::overlay::ui::SliderIntWithTooltip("##amount", &amount, 0, 100, "%d%%",
						                                                   "Shared integer slider.");
					    });
				    });
			});
		});
	}
}

TEST_F(UiCoreTest, TabHelpersRenderNestedScrollableContent)
{
	RenderAt(ImVec2(1200.0f, 780.0f), [] {
		openvr_pair::overlay::ui::TabBarScope tabs("tabs");
		ASSERT_TRUE((bool)tabs);
		openvr_pair::overlay::ui::DrawTabItem("Settings",
		                                      [] { openvr_pair::overlay::ui::DrawTextWrapped("Settings body"); });
		openvr_pair::overlay::ui::DrawScrollableTabItem(
		    "Logs", [] { openvr_pair::overlay::ui::DrawTextWrapped("Scrollable body"); });
	});
}

TEST_F(UiCoreTest, BannersDisabledStateAndActionsRender)
{
	RenderAt(ImVec2(640.0f, 480.0f), [] {
		openvr_pair::overlay::ui::DrawErrorBanner("Error", "Detail");
		openvr_pair::overlay::ui::DrawWaitingBanner("Waiting for state.");

		{
			openvr_pair::overlay::ui::DisabledSection disabled(true, "Disabled for test.");
			bool checked = false;
			openvr_pair::overlay::ui::CheckboxWithTooltip("Disabled checkbox", &checked, "Checkbox tooltip.");
			disabled.AttachReasonTooltip();
		}

		openvr_pair::overlay::ui::DrawActionRow(
		    "actions", {
		                   openvr_pair::overlay::ui::ActionButton{"Primary", "Primary action.", false, nullptr,
		                                                          ImVec2(120.0f, 0.0f), [] {}},
		                   openvr_pair::overlay::ui::ActionButton{"Blocked", nullptr, true, "Blocked action.",
		                                                          ImVec2(120.0f, 0.0f), [] {}},
		               });
	});
}

TEST_F(UiCoreTest, ExpandedComponentsRenderAtSmallAndDashboardSizes)
{
	for (const ImVec2 size : {ImVec2(640.0f, 480.0f), ImVec2(1200.0f, 780.0f)}) {
		RenderAt(size, [] {
			float ratio = 0.5f;

			ui::DrawCard("Connection", ui::StatusTone::Ok, [&] {
				ui::StatusBadge("connected", ui::StatusTone::Ok);
				ui::SliderFloatWithTooltip("##ratio", &ratio, 0.0f, 1.0f, "%.2f", "Shared float slider.");
				ui::ScopedStyleColors tint({
				    {ImGuiCol_Button, ui::StatusColor(ui::StatusTone::Warn)},
				    {ImGuiCol_Text, ui::StatusColor(ui::StatusTone::Idle)},
				});
				ImGui::Button("Tinted");
			});

			ui::DrawCard("Plain", ui::StatusTone::Idle, [] { ui::DrawTextWrapped("Untinted card body."); });

			{
				ui::TableScope table("tone_cells", 2,
				                     ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter |
				                         ImGuiTableFlags_SizingStretchProp);
				if (table) {
					ui::SetupStretchColumn("Name", 2.0f);
					ui::SetupFixedColumn("State", 90.0f);
					ui::NextRow();
					ui::SetColumn(0);
					ImGui::TextUnformatted("Tracker 0");
					ui::SetColumn(1);
					ui::SetCellToneBg(ui::StatusTone::Error);
					ui::DrawStatusCell("lost", ui::StatusTone::Error);
				}
			}

			{
				ui::ResponsiveColumnsScope cols("grid", 3, 220.0f);
				if (cols) {
					ImGui::TextUnformatted("A");
					cols.Next();
					ImGui::TextUnformatted("B");
					cols.Next();
					ImGui::TextUnformatted("C");
				}
			}

			{
				ui::FlowRowScope flow;
				for (int i = 0; i < 8; ++i) {
					flow.Item();
					ui::StatusBadge("chip", ui::StatusTone::Info);
				}
			}
		});
	}
}

TEST_F(UiCoreTest, ModernThemeAppliesAndRenders)
{
	ui::SetTheme(ui::ThemeId::Modern);
	RenderAt(ImVec2(1200.0f, 780.0f),
	         [] { ui::DrawCard("Modern", ui::StatusTone::Info, [] { ui::DrawTextWrapped("Modern theme body."); }); });
}

TEST(Theme, RegistersModernAsDefaultAndNamesEveryTheme)
{
	EXPECT_EQ(0, (int)ui::ThemeId::Modern);
	EXPECT_STREQ("Modern", ui::ThemeName(ui::ThemeId::Modern));
	EXPECT_STREQ("Legacy", ui::ThemeName(ui::ThemeId::Legacy));
	for (int i = 0; i < (int)ui::ThemeId::Count_; ++i) {
		const char* name = ui::ThemeName((ui::ThemeId)i);
		ASSERT_NE(nullptr, name);
		EXPECT_NE('\0', name[0]);
	}
}

TEST(UiSharedFormatting, ByteCountsAndFileAgesUseCompactLabels)
{
	using openvr_pair::overlay::ui::FormatByteCount;
	using openvr_pair::overlay::ui::FormatByteCountOrUnknown;
	using openvr_pair::overlay::ui::FormatFileAgeFromFileTime;
	using openvr_pair::overlay::ui::FormatFileAgeSeconds;

	EXPECT_EQ("999 B", FormatByteCount(999));
	EXPECT_EQ("1 KB", FormatByteCount(1024));
	EXPECT_EQ("1.5 MB", FormatByteCount(1536 * 1024));
	EXPECT_EQ("unknown", FormatByteCountOrUnknown(-1));

	EXPECT_EQ("42s ago", FormatFileAgeSeconds(42));
	EXPECT_EQ("5m ago", FormatFileAgeSeconds(5 * 60));
	EXPECT_EQ("2h ago", FormatFileAgeSeconds(2 * 3600));
	EXPECT_EQ("3d ago", FormatFileAgeSeconds(3 * 86400));

	const uint64_t now = 1000000000ull;
	EXPECT_EQ("10s ago", FormatFileAgeFromFileTime(now - 100000000ull, now));
	EXPECT_EQ("in the future", FormatFileAgeFromFileTime(now + 1, now));
}

TEST(UpdateNoticeLogic, NormalizesGitHubAssetDigests)
{
	const std::string sha = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
	EXPECT_EQ(sha, openvr_pair::overlay::NormalizeSha256Digest("sha256:" + sha));
	EXPECT_EQ(sha, openvr_pair::overlay::NormalizeSha256Digest(
	                   "SHA256:0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF"));
	EXPECT_EQ("", openvr_pair::overlay::NormalizeSha256Digest("sha256:not-a-valid-hash"));
}

TEST(UpdateNoticeLogic, ReleaseBodyShaMustMatchDigest)
{
	const std::string sha = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
	const std::string other = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
	EXPECT_TRUE(openvr_pair::overlay::ReleaseBodySha256Matches("SHA256: " + sha + "\n", "sha256:" + sha));
	EXPECT_FALSE(openvr_pair::overlay::ReleaseBodySha256Matches("SHA256: " + other + "\n", sha));
	EXPECT_FALSE(openvr_pair::overlay::ReleaseBodySha256Matches("No hash here", sha));
}

TEST(UpdateNoticeLogic, ReleaseAssetUrlMustMatchExpectedRepoTagAndAsset)
{
	const std::string asset = openvr_pair::overlay::ExpectedInstallerAssetName("Smoothing", "2026.6.9.0");
	EXPECT_EQ("WKOpenVR-Smoothing-v2026.6.9.0-Setup.exe", asset);

	EXPECT_TRUE(openvr_pair::overlay::IsTrustedGitHubReleaseAssetUrl(
	    "https://github.com/RealWhyKnot/WKOpenVR-Smoothing/releases/download/v2026.6.9.0/" + asset,
	    "WKOpenVR-Smoothing", "v2026.6.9.0", asset));
	EXPECT_FALSE(openvr_pair::overlay::IsTrustedGitHubReleaseAssetUrl(
	    "https://example.com/RealWhyKnot/WKOpenVR-Smoothing/releases/download/v2026.6.9.0/" + asset,
	    "WKOpenVR-Smoothing", "v2026.6.9.0", asset));
	EXPECT_FALSE(openvr_pair::overlay::IsTrustedGitHubReleaseAssetUrl(
	    "https://github.com/RealWhyKnot/WKOpenVR-Captions/releases/download/v2026.6.9.0/" + asset, "WKOpenVR-Smoothing",
	    "v2026.6.9.0", asset));
}
