#pragma once

#include "BuildChannel.h"
#include "FeaturePlugin.h"
#include "Protocol.h"

class SpaceCalibratorPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
	const char *Name() const override { return ModuleName(ModuleId::Calibration); }
	const char *FlagFileName() const override { return ModuleFlagFileName(ModuleId::Calibration); }
	const char *PipeName() const override { return ModulePipeName(ModuleId::Calibration); }

	void OnStart(openvr_pair::overlay::ShellContext &context) override;
	void OnShutdown(openvr_pair::overlay::ShellContext &context) override;
	void Tick(openvr_pair::overlay::ShellContext &context) override;
	void DrawTab(openvr_pair::overlay::ShellContext &context) override;
	void DrawLogsSection(openvr_pair::overlay::ShellContext &context) override;
	void OnDebugLoggingChanged(bool enabled) override;
#if WKOPENVR_BUILD_IS_DEV
	bool HasDevTools() const override { return true; }
	void DrawDevTools(openvr_pair::overlay::ShellContext &context) override;
#endif

private:
	bool lastDebugLoggingEnabled_ = false;
};
