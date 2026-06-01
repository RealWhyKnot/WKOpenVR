#pragma once

#include "FeaturePlugin.h"
#include "RouterTab.h"

#include <memory>

// FeaturePlugin for the OSC Router substrate. Always registered so the
// Modules tab has a router entry even when standalone OSC routing is not
// enabled.

class OscRouterPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
    OscRouterPlugin() = default;

    const char *Name()         const override { return "OSC Router"; }
    const char *FlagFileName() const override { return "enable_oscrouter.flag"; }
    const char *PipeName()     const override { return OPENVR_PAIRDRIVER_OSCROUTER_PIPE_NAME; }
    openvr_pair::overlay::FeaturePluginChannel Channel() const override
    {
        return openvr_pair::overlay::FeaturePluginChannel::Development;
    }

    bool IsInstalled(openvr_pair::overlay::ShellContext &ctx) const override;
    void Tick(openvr_pair::overlay::ShellContext &ctx) override;
    void DrawTab(openvr_pair::overlay::ShellContext &ctx) override;

private:
    RouterTab tab_;
};

namespace openvr_pair::overlay {
std::unique_ptr<FeaturePlugin> CreateOscRouterPlugin();
}
