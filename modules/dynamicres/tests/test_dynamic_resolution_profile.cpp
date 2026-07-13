#include "DynamicResolutionProfile.h"

#include <gtest/gtest.h>

#include <sstream>
#include <string>

namespace dynamicres = wkopenvr::dynamicres;

TEST(DynamicResolutionProfile, ParsesSettingsAndRestoreState)
{
	std::istringstream in("quality_preset=2\n"
	                      "min_scale_fraction=0.55\n"
	                      "max_scale_fraction=1.40\n"
	                      "step_fraction=0.07\n"
	                      "gpu_harm_rate_fraction=0.025\n"
	                      "raise_harm_rate_fraction=0.004\n"
	                      "motion_rate_fraction=0.08\n"
	                      "lower_target_fraction=0.88\n"
	                      "raise_safety_fraction=0.87\n"
	                      "burned_decay_ticks=60\n"
	                      "raise_above_baseline_fraction=0.60\n"
	                      "cpu_stall_fraction=1.20\n"
	                      "lower_required_ticks=3\n"
	                      "raise_required_ticks=6\n"
	                      "raise_above_baseline_ticks=10\n"
	                      "allow_raise_back=0\n"
	                      "release_on_cpu_bound=0\n"
	                      "cpu_release_ticks=9\n"
	                      "restore_pending=1\n"
	                      "baseline_scale=1.25\n"
	                      "baseline_manual_override=1\n"
	                      "active_scene_pid=42\n"
	                      "last_written_scale=0.95\n");

	const auto profile = dynamicres::ParseDynamicResolutionProfile(in);

	EXPECT_EQ(profile.settings.qualityPreset, dynamicres::QualityPreset::Balanced);
	EXPECT_DOUBLE_EQ(profile.settings.minScaleFraction, 0.55);
	EXPECT_DOUBLE_EQ(profile.settings.maxScaleFraction, 1.40);
	EXPECT_DOUBLE_EQ(profile.settings.stepFraction, 0.07);
	EXPECT_DOUBLE_EQ(profile.settings.gpuHarmRateFraction, 0.025);
	EXPECT_DOUBLE_EQ(profile.settings.raiseHarmRateFraction, 0.004);
	EXPECT_DOUBLE_EQ(profile.settings.motionRateFraction, 0.08);
	EXPECT_DOUBLE_EQ(profile.settings.lowerTargetFraction, 0.88);
	EXPECT_DOUBLE_EQ(profile.settings.raiseSafetyFraction, 0.87);
	EXPECT_EQ(profile.settings.burnedDecayTicks, 60);
	EXPECT_DOUBLE_EQ(profile.settings.raiseAboveBaselineFraction, 0.60);
	EXPECT_DOUBLE_EQ(profile.settings.cpuStallFraction, 1.20);
	EXPECT_EQ(profile.settings.lowerRequiredTicks, 3);
	EXPECT_EQ(profile.settings.raiseRequiredTicks, 6);
	EXPECT_EQ(profile.settings.raiseAboveBaselineTicks, 10);
	EXPECT_FALSE(profile.settings.allowRaiseBack);
	EXPECT_FALSE(profile.settings.releaseOnCpuBound);
	EXPECT_EQ(profile.settings.cpuReleaseTicks, 9);
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
	                      "max_scale_fraction=9.0\n"
	                      "gpu_harm_rate_fraction=9.0\n"
	                      "raise_harm_rate_fraction=9.0\n"
	                      "motion_rate_fraction=9.0\n"
	                      "lower_target_fraction=9.0\n"
	                      "raise_safety_fraction=0.0\n"
	                      "burned_decay_ticks=99999\n"
	                      "cpu_stall_fraction=9.0\n"
	                      "raise_above_baseline_ticks=999\n"
	                      "quality_preset=99\n"
	                      "cpu_release_ticks=999\n"
	                      "baseline_scale=-1.0\n"
	                      "last_written_scale=-2.0\n");

	const auto profile = dynamicres::ParseDynamicResolutionProfile(in);

	EXPECT_DOUBLE_EQ(profile.settings.minScaleFraction, 1.5);
	EXPECT_DOUBLE_EQ(profile.settings.stepFraction, 0.25);
	EXPECT_DOUBLE_EQ(profile.settings.maxScaleFraction, 2.0);
	EXPECT_DOUBLE_EQ(profile.settings.gpuHarmRateFraction, 0.20);
	EXPECT_DOUBLE_EQ(profile.settings.raiseHarmRateFraction, 0.05);
	EXPECT_DOUBLE_EQ(profile.settings.motionRateFraction, 1.0);
	EXPECT_DOUBLE_EQ(profile.settings.lowerTargetFraction, 0.98);
	EXPECT_DOUBLE_EQ(profile.settings.raiseSafetyFraction, 0.70);
	EXPECT_EQ(profile.settings.burnedDecayTicks, 600);
	EXPECT_DOUBLE_EQ(profile.settings.cpuStallFraction, 2.0);
	EXPECT_EQ(profile.settings.raiseAboveBaselineTicks, 60);
	EXPECT_EQ(profile.settings.qualityPreset, dynamicres::QualityPreset::Custom);
	EXPECT_EQ(profile.settings.cpuReleaseTicks, 60);
	EXPECT_DOUBLE_EQ(profile.restore.baselineScale, 0.1);
	EXPECT_DOUBLE_EQ(profile.restore.lastWrittenScale, 0.0);
}

TEST(DynamicResolutionProfile, RoundTripsKeyValueState)
{
	dynamicres::DynamicResolutionProfile profile;
	profile.settings.qualityPreset = dynamicres::QualityPreset::Custom;
	profile.settings.minScaleFraction = 0.64;
	profile.settings.maxScaleFraction = 1.35;
	profile.settings.stepFraction = 0.04;
	profile.settings.gpuHarmRateFraction = 0.021;
	profile.settings.raiseHarmRateFraction = 0.003;
	profile.settings.motionRateFraction = 0.07;
	profile.settings.lowerTargetFraction = 0.89;
	profile.settings.raiseSafetyFraction = 0.86;
	profile.settings.burnedDecayTicks = 45;
	profile.settings.raiseAboveBaselineFraction = 0.62;
	profile.settings.cpuStallFraction = 1.10;
	profile.settings.lowerRequiredTicks = 3;
	profile.settings.raiseRequiredTicks = 5;
	profile.settings.raiseAboveBaselineTicks = 9;
	profile.settings.allowRaiseBack = false;
	profile.settings.releaseOnCpuBound = false;
	profile.settings.cpuReleaseTicks = 7;
	profile.restore.restorePending = true;
	profile.restore.baselineScale = 1.1;
	profile.restore.baselineManualOverride = false;
	profile.restore.sceneProcessId = 1234;
	profile.restore.lastWrittenScale = 0.9;

	std::ostringstream out;
	dynamicres::WriteDynamicResolutionProfile(profile, out);
	std::istringstream in(out.str());
	const auto parsed = dynamicres::ParseDynamicResolutionProfile(in);

	EXPECT_EQ(parsed.settings.qualityPreset, profile.settings.qualityPreset);
	EXPECT_DOUBLE_EQ(parsed.settings.minScaleFraction, profile.settings.minScaleFraction);
	EXPECT_DOUBLE_EQ(parsed.settings.maxScaleFraction, profile.settings.maxScaleFraction);
	EXPECT_DOUBLE_EQ(parsed.settings.stepFraction, profile.settings.stepFraction);
	EXPECT_DOUBLE_EQ(parsed.settings.gpuHarmRateFraction, profile.settings.gpuHarmRateFraction);
	EXPECT_DOUBLE_EQ(parsed.settings.raiseHarmRateFraction, profile.settings.raiseHarmRateFraction);
	EXPECT_DOUBLE_EQ(parsed.settings.motionRateFraction, profile.settings.motionRateFraction);
	EXPECT_DOUBLE_EQ(parsed.settings.lowerTargetFraction, profile.settings.lowerTargetFraction);
	EXPECT_DOUBLE_EQ(parsed.settings.raiseSafetyFraction, profile.settings.raiseSafetyFraction);
	EXPECT_EQ(parsed.settings.burnedDecayTicks, profile.settings.burnedDecayTicks);
	EXPECT_DOUBLE_EQ(parsed.settings.raiseAboveBaselineFraction, profile.settings.raiseAboveBaselineFraction);
	EXPECT_DOUBLE_EQ(parsed.settings.cpuStallFraction, profile.settings.cpuStallFraction);
	EXPECT_EQ(parsed.settings.lowerRequiredTicks, profile.settings.lowerRequiredTicks);
	EXPECT_EQ(parsed.settings.raiseRequiredTicks, profile.settings.raiseRequiredTicks);
	EXPECT_EQ(parsed.settings.raiseAboveBaselineTicks, profile.settings.raiseAboveBaselineTicks);
	EXPECT_EQ(parsed.settings.allowRaiseBack, profile.settings.allowRaiseBack);
	EXPECT_EQ(parsed.settings.releaseOnCpuBound, profile.settings.releaseOnCpuBound);
	EXPECT_EQ(parsed.settings.cpuReleaseTicks, profile.settings.cpuReleaseTicks);
	EXPECT_EQ(parsed.restore.restorePending, profile.restore.restorePending);
	EXPECT_DOUBLE_EQ(parsed.restore.baselineScale, profile.restore.baselineScale);
	EXPECT_EQ(parsed.restore.baselineManualOverride, profile.restore.baselineManualOverride);
	EXPECT_EQ(parsed.restore.sceneProcessId, profile.restore.sceneProcessId);
	EXPECT_DOUBLE_EQ(parsed.restore.lastWrittenScale, profile.restore.lastWrittenScale);
}

TEST(DynamicResolutionProfile, IgnoresLegacyKeys)
{
	std::istringstream in("streaming_min_scale_fraction=0.82\n"
	                      "act_under_streaming=1\n"
	                      "conservative_streaming=0\n"
	                      "lower_gpu_budget_fraction=0.83\n"
	                      "gpu_safety_margin_fraction=0.91\n"
	                      "over_budget_fraction=0.20\n"
	                      "headroom_gpu_budget_fraction=0.68\n"
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
	EXPECT_EQ(body.find("lower_gpu_budget_fraction"), std::string::npos);
	EXPECT_EQ(body.find("gpu_safety_margin_fraction"), std::string::npos);
	EXPECT_EQ(body.find("over_budget_fraction"), std::string::npos);
	EXPECT_EQ(body.find("headroom_gpu_budget_fraction"), std::string::npos);
}

TEST(DynamicResolutionProfile, CustomProfileMigration)
{
	// A field profile saved by the previous build: Custom preset with hand-tuned knobs plus the
	// four retired metric keys. The kept knobs must load unchanged and the retired keys must
	// vanish silently, leaving the new metric knobs at their defaults.
	std::istringstream in("quality_preset=4\n"
	                      "min_scale_fraction=0.8\n"
	                      "max_scale_fraction=1.5\n"
	                      "step_fraction=0.25\n"
	                      "lower_gpu_budget_fraction=0.93\n"
	                      "gpu_safety_margin_fraction=0.95\n"
	                      "over_budget_fraction=0.15\n"
	                      "headroom_gpu_budget_fraction=0.85\n"
	                      "raise_above_baseline_fraction=0.85\n"
	                      "cpu_stall_fraction=1.05\n"
	                      "lower_required_ticks=10\n"
	                      "raise_required_ticks=4\n"
	                      "raise_above_baseline_ticks=4\n"
	                      "allow_raise_back=1\n"
	                      "release_on_cpu_bound=1\n"
	                      "cpu_release_ticks=4\n"
	                      "restore_pending=0\n"
	                      "baseline_scale=1\n"
	                      "baseline_manual_override=0\n"
	                      "active_scene_pid=0\n"
	                      "last_written_scale=0\n");

	const auto profile = dynamicres::ParseDynamicResolutionProfile(in);
	const dynamicres::DynamicResolutionSettings defaults;

	EXPECT_EQ(profile.settings.qualityPreset, dynamicres::QualityPreset::Custom);
	EXPECT_DOUBLE_EQ(profile.settings.minScaleFraction, 0.8);
	EXPECT_DOUBLE_EQ(profile.settings.stepFraction, 0.25);
	EXPECT_EQ(profile.settings.lowerRequiredTicks, 10);
	EXPECT_DOUBLE_EQ(profile.settings.raiseAboveBaselineFraction, 0.85);
	EXPECT_EQ(profile.settings.raiseAboveBaselineTicks, 4);
	EXPECT_DOUBLE_EQ(profile.settings.gpuHarmRateFraction, defaults.gpuHarmRateFraction);
	EXPECT_DOUBLE_EQ(profile.settings.raiseHarmRateFraction, defaults.raiseHarmRateFraction);
	EXPECT_DOUBLE_EQ(profile.settings.motionRateFraction, defaults.motionRateFraction);
	EXPECT_DOUBLE_EQ(profile.settings.lowerTargetFraction, defaults.lowerTargetFraction);
	EXPECT_DOUBLE_EQ(profile.settings.raiseSafetyFraction, defaults.raiseSafetyFraction);
	EXPECT_EQ(profile.settings.burnedDecayTicks, defaults.burnedDecayTicks);

	std::ostringstream out;
	dynamicres::WriteDynamicResolutionProfile(profile, out);
	const std::string body = out.str();
	EXPECT_NE(body.find("gpu_harm_rate_fraction"), std::string::npos);
	EXPECT_EQ(body.find("lower_gpu_budget_fraction"), std::string::npos);
	EXPECT_EQ(body.find("headroom_gpu_budget_fraction"), std::string::npos);
}
