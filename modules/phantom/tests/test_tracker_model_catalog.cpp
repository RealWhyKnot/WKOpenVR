#include <gtest/gtest.h>

#include "TrackerModelCatalog.h"

#include <cstring>

// The wire-zero default must be Vive Tracker 3.0: PhantomConfig.render_model
// repurposes a pad byte, so a zero from an older overlay (or a fresh config)
// has to decode to the new default model.
TEST(TrackerModelCatalogTest, DefaultIsViveTracker3AtZero)
{
	EXPECT_EQ(static_cast<uint8_t>(phantom::TrackerModel::ViveTracker3), 0u);
	EXPECT_STREQ(phantom::TrackerModelRenderName(phantom::TrackerModel::ViveTracker3), "{htc}vr_tracker_vive_3_0");
}

TEST(TrackerModelCatalogTest, RenderNamesAndLabelsAreNonEmptyForEveryModel)
{
	for (uint8_t i = 0; i < phantom::kTrackerModelCount; ++i) {
		const auto m = static_cast<phantom::TrackerModel>(i);
		const char* render = phantom::TrackerModelRenderName(m);
		const char* label = phantom::TrackerModelLabel(m);
		ASSERT_NE(render, nullptr);
		ASSERT_NE(label, nullptr);
		EXPECT_GT(std::strlen(render), 0u);
		EXPECT_GT(std::strlen(label), 0u);
	}
}

TEST(TrackerModelCatalogTest, RenderNamesMatchInstalledSteamVrModels)
{
	EXPECT_STREQ(phantom::TrackerModelRenderName(phantom::TrackerModel::ViveTracker1), "{htc}vr_tracker_vive_1_0");
	EXPECT_STREQ(phantom::TrackerModelRenderName(phantom::TrackerModel::GenericTracker), "generic_tracker");
}

TEST(TrackerModelCatalogTest, KeyRoundTripsThroughFromKey)
{
	for (uint8_t i = 0; i < phantom::kTrackerModelCount; ++i) {
		const auto m = static_cast<phantom::TrackerModel>(i);
		const char* k = phantom::TrackerModelToKey(m);
		ASSERT_NE(k, nullptr);
		EXPECT_EQ(phantom::TrackerModelFromKey(k), m) << "round-trip failed for " << phantom::TrackerModelLabel(m);
	}
}

TEST(TrackerModelCatalogTest, UnknownKeyFallsBackToDefault)
{
	EXPECT_EQ(phantom::TrackerModelFromKey(nullptr), phantom::TrackerModel::ViveTracker3);
	EXPECT_EQ(phantom::TrackerModelFromKey(""), phantom::TrackerModel::ViveTracker3);
	EXPECT_EQ(phantom::TrackerModelFromKey("nope"), phantom::TrackerModel::ViveTracker3);
	EXPECT_EQ(phantom::TrackerModelFromKey("vive_3"), phantom::TrackerModel::ViveTracker3);
}
