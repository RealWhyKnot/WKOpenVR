#include "SpaceCalibratorPlugin.h"

#include "CalibrationMetrics.h"
#include "DebugLogging.h"
#include "EmbeddedFiles.h"
#include "Protocol.h"
#include "ShellContext.h"
#include "SpaceCalibratorUmbrellaRuntime.h"
#include "UserInterface.h"

#include <imgui.h>

#include <memory>
#include <string>

// Defined in UserInterfaceTabsLogs.cpp. Declared here so the global Logs tab
// can surface SC's logs panel without pulling the file-scope forward decl
// out of UserInterface.cpp.
void CCal_DrawLogsPanel();
#if WKOPENVR_BUILD_IS_DEV
void CCal_DrawDevToolsPanel();
#endif

void SpaceCalibratorPlugin::OnStart(openvr_pair::overlay::ShellContext&)
{
	Metrics::enableLogs = openvr_pair::common::IsDebugLoggingEnabled();
	lastDebugLoggingEnabled_ = Metrics::enableLogs;
	if (Metrics::enableLogs) {
		Metrics::EnsureLogFileReady("spacecal_plugin_start");
	}

	// Match the standalone SpaceCalibrator binary's typography so the
	// calibration UI looks the way long-time users expect. The umbrella
	// shell otherwise falls through to ImGui's default ProggyClean, which
	// renders too small and too pixelated for this overlay's size.
	auto& io = ImGui::GetIO();
	if (io.Fonts->Fonts.empty() || (io.Fonts->Fonts.size() == 1 && io.Fonts->Fonts[0] == io.FontDefault &&
	                                io.FontDefault != nullptr && io.FontDefault->FontSize < 18.0f)) {
		io.Fonts->AddFontFromMemoryCompressedTTF(DroidSans_compressed_data, DroidSans_compressed_size, 24.0f);
	}

	CCal_SetInUmbrella(true);
	CCal_UmbrellaStart();
}

void SpaceCalibratorPlugin::OnShutdown(openvr_pair::overlay::ShellContext&)
{
	CCal_UmbrellaShutdown();
	Metrics::CloseLogFile();
}

void SpaceCalibratorPlugin::Tick(openvr_pair::overlay::ShellContext&)
{
	CCal_UmbrellaTick();
}

void SpaceCalibratorPlugin::DrawTab(openvr_pair::overlay::ShellContext&)
{
	CCal_DrawTab();
}

void SpaceCalibratorPlugin::DrawLogsSection(openvr_pair::overlay::ShellContext&)
{
	// The umbrella's global Logs tab uses the calibration log panel as the
	// single debug-log surface because calibration owns the structured session
	// log and shared log-file browser.
	CCal_DrawLogsPanel();
}

#if WKOPENVR_BUILD_IS_DEV
void SpaceCalibratorPlugin::DrawDevTools(openvr_pair::overlay::ShellContext&)
{
	CCal_DrawDevToolsPanel();
}
#endif

void SpaceCalibratorPlugin::OnDebugLoggingChanged(bool enabled)
{
	if (enabled == lastDebugLoggingEnabled_) return;
	lastDebugLoggingEnabled_ = enabled;
	Metrics::enableLogs = enabled;
	if (enabled) {
		Metrics::EnsureLogFileReady("debug_logging_enabled");
	}
	else {
		Metrics::CloseLogFile();
	}
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateSpaceCalibratorPlugin()
{
	return std::make_unique<SpaceCalibratorPlugin>();
}

} // namespace openvr_pair::overlay
