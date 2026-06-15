#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include "RoleCatalog.h"

#include <openvr_driver.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace openvr_pair::overlay::testharness {

struct PhantomReplayDevice
{
	uint32_t replay_id = 0;
	std::string serial;
	vr::ETrackedDeviceClass device_class = vr::TrackedDeviceClass_Invalid;
	vr::ETrackedControllerRole controller_role = vr::TrackedControllerRole_Invalid;
	phantom::BodyRole body_role = phantom::BodyRole::None;
	bool dropout_enabled = false;
};

struct PhantomReplaySample
{
	double time_ms = 0.0;
	uint32_t replay_device_id = 0;
	vr::DriverPose_t pose{};
};

struct PhantomReplayRecording
{
	std::string source_format;
	std::vector<PhantomReplayDevice> devices;
	std::vector<PhantomReplaySample> samples;
	double duration_ms = 0.0;
};

struct PhantomReplayLoadResult
{
	bool ok = false;
	std::string error;
	PhantomReplayRecording recording;
};

PhantomReplayLoadResult LoadPhantomReplay(const std::filesystem::path& path);

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
