#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include "HarnessScenario.h"
#include "MockOpenVR/MockDriverContext.h"
#include "MockOpenVR/MockServerDriverHost.h"
#include "MockOpenVR/MockDriverInput.h"
#include "MockOpenVR/MockProperties.h"
#include "MockOpenVR/MockSettings.h"

#include <openvr_driver.h>

#include <filesystem>
#include <memory>
#include <mutex>
#include <vector>

namespace openvr_pair::overlay::testharness {

// Umbrella for all mock OpenVR interfaces. The InProcessDriverLoader
// constructs one of these, passes the contained MockDriverContext into
// driver_wkopenvr.dll's HmdDriverFactory entry, and exposes per-interface
// pointers via accessors so scenarios can read recorded calls + seed state.
class MockOpenVRRuntime
{
public:
	explicit MockOpenVRRuntime(std::filesystem::path driver_resources);

	BarrierQueue& recorder() noexcept { return recorder_; }
	const std::filesystem::path& driver_resources() const noexcept { return driver_resources_; }

	MockDriverContext& context() noexcept { return *context_; }
	MockServerDriverHost& server_driver_host() noexcept { return *server_driver_host_; }
	MockDriverInput& driver_input() noexcept { return *driver_input_; }
	MockProperties& properties() noexcept { return *properties_; }
	MockSettings& settings() noexcept { return *settings_; }
	MockDriverLog& driver_log() noexcept { return *driver_log_; }
	MockResources& resources() noexcept { return *resources_; }
	MockDriverManager& driver_manager() noexcept { return *driver_manager_; }

private:
	std::filesystem::path driver_resources_;
	BarrierQueue recorder_;

	// Order matters: context must be last-initialized because it registers
	// pointers into all of the others. unique_ptr lets us use forward decls
	// in the header without #include cycles.
	std::unique_ptr<MockServerDriverHost> server_driver_host_;
	std::unique_ptr<MockDriverInput> driver_input_;
	std::unique_ptr<MockProperties> properties_;
	std::unique_ptr<MockSettings> settings_;
	std::unique_ptr<MockDriverLog> driver_log_;
	std::unique_ptr<MockResources> resources_;
	std::unique_ptr<MockDriverManager> driver_manager_;
	std::unique_ptr<MockDriverContext> context_;
};

// Helper used by scenarios to feed synthetic device poses into the driver.
// The pose flow is: scenario -> MockPoseSource::PushPose -> the mock's
// TrackedDevicePoseUpdated -> MinHook detour -> driver's internal pose
// pipeline. The mock records every pose into the BarrierQueue so scenarios
// can also assert on what the driver eventually emits.
class MockPoseSource
{
public:
	explicit MockPoseSource(MockOpenVRRuntime& runtime);

	// Register a synthetic tracked device. Returns the assigned device id
	// (the driver receives this id in subsequent pose updates).
	uint32_t AddDevice(const std::string& serial, vr::ETrackedDeviceClass device_class);

	// Push a pose for the named device. Pose timestamps + qpcOffset are
	// auto-filled if zero on the input.
	void PushPose(uint32_t device_id, vr::DriverPose_t pose);

	// Build a default valid DriverPose_t with the given world position +
	// identity orientation. Saves boilerplate inside scenarios.
	static vr::DriverPose_t MakeIdentityPose(double x, double y, double z);

private:
	MockOpenVRRuntime& runtime_;
	std::mutex mu_;
};

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
