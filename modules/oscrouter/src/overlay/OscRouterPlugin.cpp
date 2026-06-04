#include "OscRouterPlugin.h"
#include "ShellContext.h"

#include <memory>

bool OscRouterPlugin::IsInstalled(openvr_pair::overlay::ShellContext &ctx) const
{
    namespace module_registry = openvr_pair::common::modules;
    return ctx.IsFlagPresent(module_registry::FlagFileName(module_registry::ModuleId::OscRouter))
        || ctx.IsFlagPresent(module_registry::FlagFileName(module_registry::ModuleId::FaceTracking))
        || ctx.IsFlagPresent(module_registry::FlagFileName(module_registry::ModuleId::Captions));
}

void OscRouterPlugin::Tick(openvr_pair::overlay::ShellContext &ctx)
{
    tab_.Tick(ctx);
}

void OscRouterPlugin::DrawTab(openvr_pair::overlay::ShellContext &ctx)
{
    tab_.Draw(ctx);
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateOscRouterPlugin()
{
    return std::make_unique<OscRouterPlugin>();
}

} // namespace openvr_pair::overlay
