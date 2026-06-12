#include "CalibrationAnchor.h"

#include <gtest/gtest.h>

#include <string>

namespace {

int g_lastSmoothingValue = -1;
std::string g_lastSmoothingReason;

bool CaptureSmoothingUpdate(int smoothness, const char* reason)
{
	g_lastSmoothingValue = smoothness;
	g_lastSmoothingReason = reason ? reason : "";
	return true;
}

bool RejectSmoothingUpdate(int, const char*)
{
	return false;
}

} // namespace

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

TEST(CalibrationAnchorTest, HeadsetSynthesisSmoothingStateIsOptionalAndClamped)
{
	using namespace openvr_pair::overlay;

	SetHeadsetSynthesisState("LHR-head", 125, CaptureSmoothingUpdate);

	std::string serial;
	int smoothness = -1;
	ASSERT_TRUE(TryGetHeadsetSynthesisTrackerSerial(serial));
	EXPECT_EQ(serial, "LHR-head");
	ASSERT_TRUE(TryGetHeadsetSynthesisLockedSmoothing(smoothness));
	EXPECT_EQ(smoothness, 100);

	g_lastSmoothingValue = -1;
	g_lastSmoothingReason.clear();
	EXPECT_TRUE(TrySetHeadsetSynthesisLockedSmoothing(-5, "unit-test"));
	EXPECT_EQ(g_lastSmoothingValue, 0);
	EXPECT_EQ(g_lastSmoothingReason, "unit-test");
	ASSERT_TRUE(TryGetHeadsetSynthesisLockedSmoothing(smoothness));
	EXPECT_EQ(smoothness, 0);

	SetHeadsetSynthesisState({}, 50, CaptureSmoothingUpdate);
	EXPECT_FALSE(TryGetHeadsetSynthesisTrackerSerial(serial));
	EXPECT_FALSE(TryGetHeadsetSynthesisLockedSmoothing(smoothness));
	EXPECT_FALSE(TrySetHeadsetSynthesisLockedSmoothing(20, "inactive"));
}

TEST(CalibrationAnchorTest, FailedSmoothingUpdateDoesNotChangeCachedValue)
{
	using namespace openvr_pair::overlay;

	SetHeadsetSynthesisState("LHR-head", 25, RejectSmoothingUpdate);

	int smoothness = -1;
	ASSERT_TRUE(TryGetHeadsetSynthesisLockedSmoothing(smoothness));
	EXPECT_EQ(smoothness, 25);
	EXPECT_FALSE(TrySetHeadsetSynthesisLockedSmoothing(60, "unit-test"));
	ASSERT_TRUE(TryGetHeadsetSynthesisLockedSmoothing(smoothness));
	EXPECT_EQ(smoothness, 25);

	SetHeadsetSynthesisTrackerSerial("LHR-legacy");
	EXPECT_FALSE(TrySetHeadsetSynthesisLockedSmoothing(60, "legacy-serial-only"));

	SetHeadsetSynthesisState({}, 0, nullptr);
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
