#include "DynamicResolutionProfile.h"

#include <gtest/gtest.h>

#include <sstream>
#include <string>

namespace dynamicres = wkopenvr::dynamicres;

TEST(DynamicResolutionProfile, ParsesSettingsAndRestoreState)
{
	std::istringstream in("min_scale_fraction=0.55\n"
	                      "step_fraction=0.07\n"
	                      "allow_raise_back=0\n"
	                      "restore_pending=1\n"
	                      "baseline_scale=1.25\n"
	                      "baseline_manual_override=1\n"
	                      "active_scene_pid=42\n"
	                      "last_written_scale=0.95\n");

	const auto profile = dynamicres::ParseDynamicResolutionProfile(in);

	EXPECT_DOUBLE_EQ(profile.settings.minScaleFraction, 0.55);
	EXPECT_DOUBLE_EQ(profile.settings.stepFraction, 0.07);
	EXPECT_FALSE(profile.settings.allowRaiseBack);
	EXPECT_TRUE(profile.restore.restorePending);
	EXPECT_DOUBLE_EQ(profile.restore.baselineScale, 1.25);
	EXPECT_TRUE(profile.restore.baselineManualOverride);
	EXPECT_EQ(profile.restore.sceneProcessId, 42u);
	EXPECT_DOUBLE_EQ(profile.restore.lastWrittenScale, 0.95);
}

TEST(DynamicResolutionProfile, ClampsUnsafeValues)
{
	std::istringstream in("min_scale_fraction=9.0\n"
	                      "step_fraction=0.90\n"
	                      "baseline_scale=-1.0\n"
	                      "last_written_scale=-2.0\n");

	const auto profile = dynamicres::ParseDynamicResolutionProfile(in);

	EXPECT_DOUBLE_EQ(profile.settings.minScaleFraction, 1.5);
	EXPECT_DOUBLE_EQ(profile.settings.stepFraction, 0.25);
	EXPECT_DOUBLE_EQ(profile.restore.baselineScale, 0.1);
	EXPECT_DOUBLE_EQ(profile.restore.lastWrittenScale, 0.0);
}

TEST(DynamicResolutionProfile, RoundTripsKeyValueState)
{
	dynamicres::DynamicResolutionProfile profile;
	profile.settings.minScaleFraction = 0.64;
	profile.settings.stepFraction = 0.04;
	profile.settings.allowRaiseBack = false;
	profile.restore.restorePending = true;
	profile.restore.baselineScale = 1.1;
	profile.restore.baselineManualOverride = false;
	profile.restore.sceneProcessId = 1234;
	profile.restore.lastWrittenScale = 0.9;

	std::ostringstream out;
	dynamicres::WriteDynamicResolutionProfile(profile, out);
	std::istringstream in(out.str());
	const auto parsed = dynamicres::ParseDynamicResolutionProfile(in);

	EXPECT_DOUBLE_EQ(parsed.settings.minScaleFraction, profile.settings.minScaleFraction);
	EXPECT_DOUBLE_EQ(parsed.settings.stepFraction, profile.settings.stepFraction);
	EXPECT_EQ(parsed.settings.allowRaiseBack, profile.settings.allowRaiseBack);
	EXPECT_EQ(parsed.restore.restorePending, profile.restore.restorePending);
	EXPECT_DOUBLE_EQ(parsed.restore.baselineScale, profile.restore.baselineScale);
	EXPECT_EQ(parsed.restore.baselineManualOverride, profile.restore.baselineManualOverride);
	EXPECT_EQ(parsed.restore.sceneProcessId, profile.restore.sceneProcessId);
	EXPECT_DOUBLE_EQ(parsed.restore.lastWrittenScale, profile.restore.lastWrittenScale);
}

TEST(DynamicResolutionProfile, IgnoresLegacyStreamingKeys)
{
	std::istringstream in("streaming_min_scale_fraction=0.82\n"
	                      "act_under_streaming=1\n"
	                      "conservative_streaming=0\n"
	                      "min_scale_fraction=0.66\n"
	                      "step_fraction=0.12\n");

	const auto profile = dynamicres::ParseDynamicResolutionProfile(in);

	EXPECT_DOUBLE_EQ(profile.settings.minScaleFraction, 0.66);
	EXPECT_DOUBLE_EQ(profile.settings.stepFraction, 0.12);

	std::ostringstream out;
	dynamicres::WriteDynamicResolutionProfile(profile, out);
	const std::string body = out.str();
	EXPECT_EQ(body.find("streaming_min_scale_fraction"), std::string::npos);
	EXPECT_EQ(body.find("act_under_streaming"), std::string::npos);
	EXPECT_EQ(body.find("conservative_streaming"), std::string::npos);
}
