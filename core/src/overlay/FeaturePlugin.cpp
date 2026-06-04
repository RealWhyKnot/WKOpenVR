#include "FeaturePlugin.h"
#include "ShellContext.h"

namespace openvr_pair::overlay {

bool FeaturePlugin::IsInstalled(ShellContext& context) const
{
	return context.IsFlagPresent(FlagFileName());
}

} // namespace openvr_pair::overlay
