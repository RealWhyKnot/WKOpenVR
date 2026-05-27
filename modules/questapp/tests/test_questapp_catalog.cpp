#include "QuestAppCatalog.h"

#include <gtest/gtest.h>

#include <algorithm>

TEST(QuestAppCatalog, CuratedTargetsIncludeCommonPcVrLaunchers)
{
    const auto& targets = wkopenvr::questapp::CuratedLaunchTargets();

    const auto hasPackage = [&](const char* packageName) {
        return std::any_of(targets.begin(), targets.end(), [&](const auto& target) {
            return target.packageName == packageName && target.curated;
        });
    };

    EXPECT_TRUE(hasPackage("VirtualDesktop.Android"));
    EXPECT_TRUE(hasPackage("com.valvesoftware.steamlinkvr"));
    EXPECT_TRUE(hasPackage("com.meta.pclinkservice.server"));
}

TEST(QuestAppCatalog, DisplayNameFallsBackToPackage)
{
    wkopenvr::questapp::QuestLaunchTarget target;
    target.packageName = "com.example.app";

    EXPECT_EQ(wkopenvr::questapp::LaunchTargetDisplayName(target), "com.example.app");
}

