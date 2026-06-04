#include "HeadMountSourceGuard.h"

#include <gtest/gtest.h>

namespace hm = spacecal::headmount;

TEST(HeadMountSourceGuard, SelectsHeadProxyOnlyWhenOffsetIsCalibrated)
{
	EXPECT_EQ(HeadMountSampleSource::PhysicalTracker, hm::SelectHeadMountSampleSource(HeadMountMode::Off, false));
	EXPECT_EQ(HeadMountSampleSource::PhysicalTracker,
	          hm::SelectHeadMountSampleSource(HeadMountMode::AutoPaired, false));
	EXPECT_EQ(HeadMountSampleSource::HeadProxy, hm::SelectHeadMountSampleSource(HeadMountMode::AutoPaired, true));
	EXPECT_EQ(HeadMountSampleSource::HeadProxy, hm::SelectHeadMountSampleSource(HeadMountMode::DriverSynth, true));
}

TEST(HeadMountSourceGuard, InitialHeadProxyWithRelativePoseForcesReset)
{
	hm::HeadMountSourceFingerprint previous;
	hm::HeadMountSourceFingerprint current;
	current.source = HeadMountSampleSource::HeadProxy;
	current.mode = HeadMountMode::DriverSynth;
	current.offsetVersion = 2;

	const auto decision = hm::EvaluateHeadMountSourceTransition(false, previous, current, true);

	EXPECT_TRUE(decision.reset);
	EXPECT_STREQ("initial_head_proxy", decision.reason);
}

TEST(HeadMountSourceGuard, PhysicalTrackerToHeadProxyForcesReset)
{
	hm::HeadMountSourceFingerprint previous;
	previous.source = HeadMountSampleSource::PhysicalTracker;
	previous.mode = HeadMountMode::Off;
	previous.offsetVersion = 1;
	previous.deviceID = 4;
	previous.targetSerial = "LHR-HEAD";
	previous.targetTrackingSystem = "lighthouse";

	hm::HeadMountSourceFingerprint current = previous;
	current.source = HeadMountSampleSource::HeadProxy;
	current.mode = HeadMountMode::DriverSynth;

	const auto decision = hm::EvaluateHeadMountSourceTransition(true, previous, current, true);

	EXPECT_TRUE(decision.reset);
	EXPECT_STREQ("sample_source_changed", decision.reason);
}

TEST(HeadMountSourceGuard, OffsetVersionChangeForcesReset)
{
	hm::HeadMountSourceFingerprint previous;
	previous.source = HeadMountSampleSource::HeadProxy;
	previous.mode = HeadMountMode::DriverSynth;
	previous.offsetVersion = 3;
	previous.deviceID = 4;
	previous.targetSerial = "LHR-HEAD";
	previous.targetTrackingSystem = "lighthouse";

	hm::HeadMountSourceFingerprint current = previous;
	current.offsetVersion = 4;

	const auto decision = hm::EvaluateHeadMountSourceTransition(true, previous, current, true);

	EXPECT_TRUE(decision.reset);
	EXPECT_STREQ("offset_changed", decision.reason);
}

TEST(HeadMountSourceGuard, SameFingerprintDoesNotReset)
{
	hm::HeadMountSourceFingerprint previous;
	previous.source = HeadMountSampleSource::HeadProxy;
	previous.mode = HeadMountMode::DriverSynth;
	previous.offsetVersion = 3;
	previous.deviceID = 4;
	previous.targetSerial = "LHR-HEAD";
	previous.targetTrackingSystem = "lighthouse";

	const auto decision = hm::EvaluateHeadMountSourceTransition(true, previous, previous, true);

	EXPECT_FALSE(decision.reset);
	EXPECT_STREQ("none", decision.reason);
}
