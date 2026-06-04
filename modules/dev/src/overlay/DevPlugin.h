#pragma once

#include "FeaturePlugin.h"

#include <vector>

class DevPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
	explicit DevPlugin(std::vector<openvr_pair::overlay::FeaturePlugin*> plugins);

	const char* Name() const override { return "Dev"; }
	const char* FlagFileName() const override { return ""; }
	const char* PipeName() const override { return ""; }
	openvr_pair::overlay::FeaturePluginChannel Channel() const override;
	bool IsInstalled(openvr_pair::overlay::ShellContext&) const override { return true; }
	void DrawTab(openvr_pair::overlay::ShellContext& context) override;

private:
	std::vector<openvr_pair::overlay::FeaturePlugin*> plugins_;
};
