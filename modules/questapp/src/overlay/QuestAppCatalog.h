#pragma once

#include <string>
#include <vector>

namespace wkopenvr::questapp {

struct QuestLaunchTarget {
    std::string label;
    std::string packageName;
    std::string activityName;
    bool curated = false;
};

const std::vector<QuestLaunchTarget>& CuratedLaunchTargets();
std::string LaunchTargetDisplayName(const QuestLaunchTarget& target);
bool SameLaunchTarget(const QuestLaunchTarget& a, const QuestLaunchTarget& b);

} // namespace wkopenvr::questapp
