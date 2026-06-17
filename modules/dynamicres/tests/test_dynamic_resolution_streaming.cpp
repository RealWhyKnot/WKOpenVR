#include "DynamicResolutionStreaming.h"

#include <gtest/gtest.h>

namespace dynamicres = wkopenvr::dynamicres;

namespace {

dynamicres::VirtualDesktopStreamerSettings ParseSettings(const char* codecName)
{
	dynamicres::VirtualDesktopStreamerSettings settings;
	const std::string json =
	    std::string("{\"PreferredCodec\":5,\"CodecName\":\"") + codecName + "\",\"DeviceName\":\"Meta Quest Pro\"}";
	EXPECT_TRUE(dynamicres::ParseVirtualDesktopStreamerSettings(json, settings));
	return settings;
}

} // namespace

TEST(DynamicResolutionStreaming, ParsesVirtualDesktopCodecAndDevice)
{
	dynamicres::VirtualDesktopStreamerSettings settings;

	ASSERT_TRUE(dynamicres::ParseVirtualDesktopStreamerSettings(
	    "{\"PreferredCodec\":5,\"CodecName\":\"H.264+\",\"DeviceName\":\"Meta Quest Pro\"}", settings));

	EXPECT_TRUE(settings.parsed);
	EXPECT_EQ(settings.preferredCodec, 5);
	EXPECT_EQ(settings.codecName, "H.264+");
	EXPECT_EQ(settings.deviceName, "Meta Quest Pro");
}

TEST(DynamicResolutionStreaming, AllowsOnlyH264FamilyByDefault)
{
	EXPECT_TRUE(dynamicres::VirtualDesktopCodecAllowsDefaultAction(ParseSettings("H.264")));
	EXPECT_TRUE(dynamicres::VirtualDesktopCodecAllowsDefaultAction(ParseSettings("H.264+")));
	EXPECT_TRUE(dynamicres::VirtualDesktopCodecAllowsDefaultAction(ParseSettings("x264")));
	EXPECT_TRUE(dynamicres::VirtualDesktopCodecAllowsDefaultAction(ParseSettings("AVC")));

	EXPECT_FALSE(dynamicres::VirtualDesktopCodecAllowsDefaultAction(ParseSettings("HEVC")));
	EXPECT_FALSE(dynamicres::VirtualDesktopCodecAllowsDefaultAction(ParseSettings("HEVC 10-bit")));
	EXPECT_FALSE(dynamicres::VirtualDesktopCodecAllowsDefaultAction(ParseSettings("AV1")));
	EXPECT_FALSE(dynamicres::VirtualDesktopCodecAllowsDefaultAction(ParseSettings("AV1 10-bit")));
}

TEST(DynamicResolutionStreaming, RejectsInvalidSettingsJson)
{
	dynamicres::VirtualDesktopStreamerSettings settings;
	std::string error;

	EXPECT_FALSE(dynamicres::ParseVirtualDesktopStreamerSettings("{not-json", settings, &error));
	EXPECT_FALSE(settings.parsed);
	EXPECT_FALSE(error.empty());
	EXPECT_FALSE(dynamicres::VirtualDesktopCodecAllowsDefaultAction(settings));
}
