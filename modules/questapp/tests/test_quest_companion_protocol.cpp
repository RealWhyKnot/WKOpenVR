#include "QuestCompanionProtocol.h"

#include <gtest/gtest.h>

TEST(QuestCompanionProtocol, ParsesRemoteVersionAndFeatures)
{
	const std::string body =
	    "{\"paired\":true,\"remoteProgramVersion\":1,\"remoteFeatures\":[\"settingsReport\"],"
	    "\"autoLaunchEnabled\":true,\"selectedPackage\":\"VirtualDesktop.Android\","
	    "\"selectedActivity\":\"md59102214312e19799944a61bf7bc2f23e.VrActivity\",\"httpPort\":39789}\n";

	wkopenvr::questapp::QuestCompanionSettings settings;
	ASSERT_TRUE(wkopenvr::questapp::ParseCompanionSettingsJson(body, settings));

	EXPECT_EQ(settings.remoteProgramVersion, 1);
	EXPECT_TRUE(settings.remoteReportsSettings);
	EXPECT_TRUE(settings.autoLaunchEnabled);
	EXPECT_EQ(settings.selectedPackage, "VirtualDesktop.Android");
	EXPECT_EQ(settings.selectedActivity, "md59102214312e19799944a61bf7bc2f23e.VrActivity");
}

TEST(QuestCompanionProtocol, ParsesUnversionedRemoteAsLegacy)
{
	const std::string body =
	    "{\"paired\":true,\"autoLaunchEnabled\":false,\"selectedPackage\":\"com.valvesoftware.steamlinkvr\","
	    "\"selectedActivity\":\"com.valvesoftware.steamlink.VRLink\",\"httpPort\":39789}\n";

	wkopenvr::questapp::QuestCompanionSettings settings;
	ASSERT_TRUE(wkopenvr::questapp::ParseCompanionSettingsJson(body, settings));

	EXPECT_EQ(settings.remoteProgramVersion, 0);
	EXPECT_FALSE(settings.remoteReportsSettings);
	EXPECT_FALSE(settings.autoLaunchEnabled);
	EXPECT_EQ(settings.selectedPackage, "com.valvesoftware.steamlinkvr");
	EXPECT_EQ(settings.selectedActivity, "com.valvesoftware.steamlink.VRLink");
}

TEST(QuestCompanionProtocol, RejectsInvalidJson)
{
	wkopenvr::questapp::QuestCompanionSettings settings;
	settings.selectedPackage = "unchanged";

	EXPECT_FALSE(wkopenvr::questapp::ParseCompanionSettingsJson("{not-json", settings));
	EXPECT_EQ(settings.selectedPackage, "unchanged");
}
