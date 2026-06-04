#include "QuestAppCatalog.h"

namespace wkopenvr::questapp {

const std::vector<QuestLaunchTarget>& CuratedLaunchTargets()
{
	static const std::vector<QuestLaunchTarget> targets = {
	    {
	        "Virtual Desktop",
	        "VirtualDesktop.Android",
	        "md59102214312e19799944a61bf7bc2f23e.VrActivity",
	        true,
	    },
	    {
	        "Steam Link VR",
	        "com.valvesoftware.steamlinkvr",
	        "com.valvesoftware.steamlink.VRLink",
	        true,
	    },
	    {
	        "Meta Quest Link",
	        "com.meta.pclinkservice.server",
	        "com.oculus.xrstreamingclient.MainActivity",
	        true,
	    },
	};
	return targets;
}

std::string LaunchTargetDisplayName(const QuestLaunchTarget& target)
{
	if (!target.label.empty()) return target.label;
	if (!target.packageName.empty()) return target.packageName;
	return "(unknown app)";
}

bool SameLaunchTarget(const QuestLaunchTarget& a, const QuestLaunchTarget& b)
{
	return a.packageName == b.packageName && a.activityName == b.activityName;
}

} // namespace wkopenvr::questapp
