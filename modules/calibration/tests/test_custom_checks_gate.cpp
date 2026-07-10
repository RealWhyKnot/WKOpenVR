// Tests for the enhanced-tracking master switch (enhancedTrackingChecks):
// profile persistence, defaults, and the telemetry bit assignment. The switch
// gates every non-upstream check in the pipeline; off = classic upstream
// behaviour, so the default and its persistence are load-bearing.

#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include "Calibration.h"
#include "CalibrationExperimentFlags.h"
#include "Configuration.h"
#include "TrackingStyle.h"

namespace {

std::string MakeMinimalProfileJson()
{
	std::ostringstream o;
	o << "[{";
	o << "\"schema_version\":7,";
	o << "\"reference_tracking_system\":\"lighthouse\",";
	o << "\"target_tracking_system\":\"oculus\",";
	o << "\"roll\":0.0,\"yaw\":0.0,\"pitch\":0.0,";
	o << "\"x\":0.0,\"y\":0.0,\"z\":0.0,";
	o << "\"continuous_calibration_target_offset_x\":0.0,";
	o << "\"continuous_calibration_target_offset_y\":0.0,";
	o << "\"continuous_calibration_target_offset_z\":0.0,";
	o << "\"tracking_style\":1";
	o << "}]";
	return o.str();
}

} // namespace

TEST(CustomChecksGateTest, DefaultsOffWhenKeyAbsent)
{
	// Profiles written before the switch existed carry no key; those loads
	// take the default (off = classic upstream pipeline).
	CalibrationContext ctx;
	std::stringstream io(MakeMinimalProfileJson());
	ParseProfile(ctx, io);

	EXPECT_FALSE(ctx.enhancedTrackingChecks);
	EXPECT_FALSE(ctx.CustomChecksActive());
}

TEST(CustomChecksGateTest, ExplicitOnRoundTrips)
{
	CalibrationContext src;
	src.referenceTrackingSystem = "lighthouse";
	src.targetTrackingSystem = "oculus";
	src.validProfile = true;
	ApplyTrackingStylePreset(src, TrackingStyle::Continuous);
	src.enhancedTrackingChecks = true; // explicit on must survive

	std::stringstream io;
	WriteProfile(src, io);
	EXPECT_NE(io.str().find("enhanced_tracking_checks"), std::string::npos)
	    << "always written: an absent key means the off-default";

	CalibrationContext dst;
	ParseProfile(dst, io);
	EXPECT_TRUE(dst.enhancedTrackingChecks);
}

TEST(CustomChecksGateTest, ExplicitOffRoundTrips)
{
	CalibrationContext src;
	src.referenceTrackingSystem = "lighthouse";
	src.targetTrackingSystem = "oculus";
	src.validProfile = true;
	ApplyTrackingStylePreset(src, TrackingStyle::Continuous);
	src.enhancedTrackingChecks = false;

	std::stringstream io;
	WriteProfile(src, io);

	CalibrationContext dst;
	dst.enhancedTrackingChecks = true; // must be overwritten by the load
	ParseProfile(dst, io);
	EXPECT_FALSE(dst.enhancedTrackingChecks);
}

TEST(CustomChecksGateTest, ResetConfigDefaultsOff)
{
	CalibrationContext ctx;
	ctx.enhancedTrackingChecks = true;
	ctx.ResetConfig();
	EXPECT_FALSE(ctx.enhancedTrackingChecks);
}

TEST(CustomChecksGateTest, TelemetryBitIsPinned)
{
	// The experimental_flags column of archived v5 recordings persists these
	// values; the assignments must never move. Bits 1<<0,1,2,4,5 are retired.
	EXPECT_EQ(static_cast<uint32_t>(spacecal::calibration_experiments::ConfidenceFusion), 1u << 3);
	EXPECT_EQ(static_cast<uint32_t>(spacecal::calibration_experiments::EnhancedTrackingChecks), 1u << 6);
}
