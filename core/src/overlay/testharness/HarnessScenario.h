#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace vr {
class IServerTrackedDeviceProvider;
}

namespace openvr_pair::overlay::testharness {

class MockOpenVRRuntime;
class MockPoseSource;

// Captures the call sites the driver hits via our mock vtables. The harness
// records every interesting call (poses, scalar/boolean updates, device adds)
// so scenarios can assert by predicate rather than poll for state.
enum class MockCallKind : uint32_t
{
	TrackedDeviceAdded,
	TrackedDevicePoseUpdated,
	UpdateBooleanComponent,
	UpdateScalarComponent,
	UpdateSkeletonComponent,
	CreateBooleanComponent,
	CreateScalarComponent,
	CreateSkeletonComponent,
	WritePropertyBatch,
	ReadPropertyBatch,
	LogMessage,
	RequestRestart,
};

struct MockCall
{
	uint64_t seq = 0;       // monotonic per-runtime, assigned at push time
	uint64_t qpc_ticks = 0; // QueryPerformanceCounter at push time
	MockCallKind kind = MockCallKind::LogMessage;

	// Polymorphic fields. Only the fields relevant to `kind` are populated;
	// others stay default. Kept as flat fields rather than std::variant so the
	// recorder stays trivially copyable and predicate lambdas read clean.
	uint32_t device_id = 0;
	uint64_t component_handle = 0;
	uint64_t property_container = 0;
	double f_value = 0.0;
	bool b_value = false;
	double time_offset_sec = 0.0;
	uint32_t aux_int = 0;
	std::string text;

	// Full pose snapshot for TrackedDevicePoseUpdated. The legacy scalar
	// fields above stay populated for older scenarios, but replay diagnostics
	// need exact position, velocity, orientation, and tracking result.
	bool has_pose = false;
	bool pose_device_connected = false;
	int32_t pose_tracking_result = 0;
	double pose_position[3]{};
	double pose_velocity[3]{};
	double pose_rotation[4]{};
};

// Thread-safe append-only ledger of MockCalls. Scenarios block on
// WaitFor(predicate, timeout) instead of polling, eliminating race-prone
// "sleep 100ms and hope" patterns. The runtime appends; scenarios snapshot.
class BarrierQueue
{
public:
	BarrierQueue() = default;

	// Append a new call. Returns the assigned sequence number.
	uint64_t Push(MockCall call);

	// Snapshot the entire ledger to date. Cheap for the small N we expect
	// (<10k entries per scenario).
	std::vector<MockCall> Snapshot() const;

	// Block until predicate(call) returns true for at least one call appended
	// after the current cursor, or timeout elapses. Returns nullopt on timeout,
	// or the matching MockCall on success. cursor is advanced past the match
	// so the same call is not double-counted across waits.
	struct WaitOutcome
	{
		bool matched = false;
		MockCall call;
	};
	WaitOutcome WaitFor(std::function<bool(const MockCall&)> predicate, std::chrono::milliseconds timeout,
	                    uint64_t& cursor);

	// Count matching calls in the snapshot (advances cursor past the last match
	// so successive calls return only new matches).
	size_t CountSince(std::function<bool(const MockCall&)> predicate, uint64_t& cursor) const;

	// Reset cursor and (optionally) clear the underlying ledger. Tests use
	// this between scenarios when they want a clean slate.
	void Clear();
	uint64_t LatestSeq() const;

private:
	mutable std::mutex mu_;
	mutable std::condition_variable cv_;
	std::atomic<uint64_t> next_seq_{1};
	std::deque<MockCall> calls_;
};

// Per-scenario logger. Writes plain-text [SCENARIO][LEVEL] messages to stdout
// and tags them so the orchestrator can collate per-scenario output.
class HarnessLogger
{
public:
	HarnessLogger(std::string scenario_name);
	void Info(std::string msg);
	void Warn(std::string msg);
	void Error(std::string msg);
	void Step(std::string msg); // bold-prefix structural step

	const std::string& name() const noexcept { return name_; }

private:
	std::string name_;
};

struct ScenarioContext
{
	MockOpenVRRuntime& mock;
	vr::IServerTrackedDeviceProvider* provider; // owned by InProcessDriverLoader
	MockPoseSource& pose_source;
	std::filesystem::path sandbox_root;
	std::filesystem::path driver_root;      // sandbox_root\drivers\01wkopenvr
	std::filesystem::path driver_resources; // sandbox_root\drivers\01wkopenvr\resources
	HarnessLogger& log;
	std::filesystem::path phantom_replay_path;
	std::filesystem::path phantom_replay_report_path;
	std::string phantom_replay_dropout_role;
	double phantom_replay_dropout_start_ms = -1.0;
	double phantom_replay_dropout_end_ms = -1.0;
	double phantom_replay_speed = 1.0;
};

struct ScenarioResult
{
	std::string name;
	bool passed = false;
	std::string failure_reason;
	std::chrono::milliseconds duration{0};
};

using ScenarioFn = ScenarioResult (*)(ScenarioContext&);

struct ScenarioEntry
{
	const char* slug;
	const char* feature_flag; // "enable_<slug>.flag"
	ScenarioFn run;
};

// Scenario_<slug>.cpp each define one of these and the runner stitches them
// into the kAllScenarios table via this declaration block.
ScenarioResult RunScenario_calibration(ScenarioContext&);
ScenarioResult RunScenario_smoothing(ScenarioContext&);
ScenarioResult RunScenario_dashboardinput(ScenarioContext&);
ScenarioResult RunScenario_inputhealth(ScenarioContext&);
ScenarioResult RunScenario_facetracking(ScenarioContext&);
ScenarioResult RunScenario_oscrouter(ScenarioContext&);
ScenarioResult RunScenario_captions(ScenarioContext&);
ScenarioResult RunScenario_phantom(ScenarioContext&);
ScenarioResult RunScenario_phantom_replay(ScenarioContext&);

// Convenience: PASS with no failure_reason; FAIL with the given message.
ScenarioResult Pass(const std::string& name, std::chrono::milliseconds duration);
ScenarioResult Fail(const std::string& name, std::chrono::milliseconds duration, std::string reason);

// QueryPerformanceCounter wrapper for timestamping MockCalls.
uint64_t QpcNow();

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
