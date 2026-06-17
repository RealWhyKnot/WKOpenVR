#pragma once

#include "DynamicResolutionLogic.h"
#include "DynamicResolutionProfile.h"
#include "DynamicResolutionStreaming.h"
#include "FeaturePlugin.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace wkopenvr::dynamicres {

class DynamicResolutionPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
	const char* Name() const override { return ModuleName(ModuleId::DynamicResolution); }
	const char* FlagFileName() const override { return ModuleFlagFileName(ModuleId::DynamicResolution); }
	const char* PipeName() const override { return ModulePipeName(ModuleId::DynamicResolution); }
	openvr_pair::overlay::FeaturePluginChannel Channel() const override
	{
		return openvr_pair::overlay::FeaturePluginChannel::Development;
	}

	bool IsInstalled(openvr_pair::overlay::ShellContext& context) const override;
	void OnStart(openvr_pair::overlay::ShellContext& context) override;
	void OnShutdown(openvr_pair::overlay::ShellContext& context) override;
	void Tick(openvr_pair::overlay::ShellContext& context) override;
	void DrawTab(openvr_pair::overlay::ShellContext& context) override;
	void DrawLogsSection(openvr_pair::overlay::ShellContext& context) override;

private:
	struct SceneState
	{
		bool running = false;
		uint32_t processId = 0;
	};

	bool CaptureBaseline(uint32_t sceneProcessId);
	bool RestoreBaseline(const char* reason);
	void ClearExternalOverride();
	bool ReadSteamVrScale(double& outScale) const;
	bool ReadSteamVrManualOverride(bool& outManual) const;
	bool WriteSteamVrScale(double scale);
	bool WriteSteamVrManualOverride(bool value);
	bool CollectTiming(DynamicResolutionTiming& outTiming);
	double ReadFrameBudgetMs();
	SceneState ReadSceneState() const;
	bool DetectStreamingHeadset() const;
	void RefreshStreamingState(bool streamingDetected);
	void UpdateStatus(const DynamicResolutionControllerOutput& output, bool streamingDetected);
	void SaveProfile();
	void DrawSettings();
	void DrawStatus();

	DynamicResolutionProfile profile_;
	DynamicResolutionController controller_;
	DynamicResolutionClassification lastClassification_;
	std::chrono::steady_clock::time_point nextTick_{};
	uint32_t lastFrameIndex_ = 0;
	bool haveLastFrameIndex_ = false;
	bool externalOverride_ = false;
	bool disabledForScene_ = false;
	bool streamingDetected_ = false;
	bool streamingCodecAllowsAction_ = false;
	bool displayFrequencyFallback_ = false;
	double lastLiveScale_ = 1.0;
	double lastTargetScale_ = 1.0;
	double lastFrameBudgetMs_ = 1000.0 / 90.0;
	VirtualDesktopStreamerSettings virtualDesktopSettings_;
	ResolutionAction lastActionCode_ = ResolutionAction::None;
	std::string lastAction_ = "None";
	std::string lastReason_ = "Waiting";
	std::string lastError_;
};

} // namespace wkopenvr::dynamicres
