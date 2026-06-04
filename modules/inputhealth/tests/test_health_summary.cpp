#include <gtest/gtest.h>

#include "InputHealthHealthSummary.h"
#include "JsonUtil.h"
#include "picojson.h"

TEST(InputHealthHealthSummary, JsonIsValidAndHasRequiredFields)
{
	InputHealthHealthSummarySnapshot snapshot;
	snapshot.overlay_started = true;
	snapshot.ipc_connected = true;
	snapshot.shmem_opened = true;
	snapshot.publish_tick = 42;
	snapshot.live_components = 7;
	snapshot.profiles_loaded = 2;
	snapshot.path_families.finger_capsense = 1;
	snapshot.path_families.trackpad_axis = 2;
	snapshot.learning.drift_suppressed_policy = 3;
	snapshot.learning.compensation_push_success = 4;
	snapshot.learning.compensation_push_rejected = 5;
	snapshot.learning.profile_sync_attempts = 6;
	snapshot.learning.diagnostic_path_counts["/input/finger/index"] = 3;
	snapshot.profile_io.attempted_saves = 8;
	snapshot.profile_io.skipped_unchanged = 9;
	snapshot.profile_io.actual_writes = 10;
	snapshot.profile_io.last_save_reason = "ready_transition";

	const std::string json = BuildInputHealthHealthSummaryJson(snapshot);
	picojson::value parsed;
	std::string err;
	ASSERT_TRUE(openvr_pair::common::json::Parse(parsed, json, &err)) << err;
	ASSERT_TRUE(parsed.is<picojson::object>());
	const auto& root = parsed.get<picojson::object>();

	EXPECT_NE(root.find("schema"), root.end());
	EXPECT_NE(root.find("pipeline"), root.end());
	EXPECT_NE(root.find("path_families"), root.end());
	EXPECT_NE(root.find("learning"), root.end());
	EXPECT_NE(root.find("profile_io"), root.end());

	const auto& learning = root.at("learning").get<picojson::object>();
	EXPECT_DOUBLE_EQ(learning.at("drift_suppressed_policy").get<double>(), 3.0);
	EXPECT_DOUBLE_EQ(learning.at("compensation_push_success").get<double>(), 4.0);
	EXPECT_DOUBLE_EQ(learning.at("compensation_push_rejected").get<double>(), 5.0);
	EXPECT_TRUE(learning.at("top_drift_paths").is<picojson::array>());

	const auto& profileIo = root.at("profile_io").get<picojson::object>();
	EXPECT_DOUBLE_EQ(profileIo.at("attempted_saves").get<double>(), 8.0);
	EXPECT_DOUBLE_EQ(profileIo.at("actual_writes").get<double>(), 10.0);
	EXPECT_EQ(profileIo.at("last_save_reason").get<std::string>(), "ready_transition");
}
