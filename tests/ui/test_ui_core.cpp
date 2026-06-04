#include <gtest/gtest.h>

#include "FeaturePlugin.h"
#include "ShellFooter.h"
#include "ShellSettings.h"
#include "ShellUiLogic.h"
#include "UiCore.h"
#include "UiControls.h"

#include <imgui.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <utility>

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
