#include "CalibrationAnchor.h"

#include <gtest/gtest.h>

#include <string>

TEST(CalibrationAnchorTest, HeadsetSynthesisTrackerSerialIsOptional)
{
	using namespace openvr_pair::overlay;

	SetHeadsetSynthesisTrackerSerial({});

	std::string serial;
	EXPECT_FALSE(TryGetHeadsetSynthesisTrackerSerial(serial));
	EXPECT_FALSE(IsHeadsetSynthesisTracker("LHR-head"));

	SetHeadsetSynthesisTrackerSerial("LHR-head");
	ASSERT_TRUE(TryGetHeadsetSynthesisTrackerSerial(serial));
	EXPECT_EQ(serial, "LHR-head");
	EXPECT_TRUE(IsHeadsetSynthesisTracker("LHR-head"));
	EXPECT_FALSE(IsHeadsetSynthesisTracker("LHR-other"));

	SetHeadsetSynthesisTrackerSerial({});
	EXPECT_FALSE(TryGetHeadsetSynthesisTrackerSerial(serial));
	EXPECT_FALSE(IsHeadsetSynthesisTracker("LHR-head"));
}

TEST(CalibrationAnchorTest, CalibrationLocksRemainIndependentFromHeadsetSynthesisMarker)
{
	using namespace openvr_pair::overlay;

	SetCalibrationDeviceLocks({{"LHR-ref", CalibrationDeviceLockKind::Reference}});
	SetHeadsetSynthesisTrackerSerial("LHR-head");

	CalibrationDeviceLockKind kind{};
	EXPECT_TRUE(TryGetCalibrationDeviceLockKind("LHR-ref", kind));
	EXPECT_EQ(kind, CalibrationDeviceLockKind::Reference);
	EXPECT_FALSE(TryGetCalibrationDeviceLockKind("LHR-head", kind));
	EXPECT_TRUE(IsHeadsetSynthesisTracker("LHR-head"));

	SetCalibrationDeviceLocks({});
	SetHeadsetSynthesisTrackerSerial({});
}
