#include <gtest/gtest.h>

#include "inputhealth/PresenceCounting.h"

#include <cstring>
#include <unordered_map>
#include <vector>

using namespace inputhealth;
using protocol::INPUTHEALTH_PATH_LEN;
using protocol::InputHealthSnapshotBody;

namespace {

// Mimic SnapshotReader::Entry without dragging the overlay headers into the
// test binary. CountInputHealthPresence pulls .body off the map's value type.
struct FakeEntry
{
	InputHealthSnapshotBody body{};
};

// Build a body with the given handle/serial/path. Other fields default to
// zero; tests that need ph_triggered or scalar/boolean flags set them
// explicitly afterwards.
InputHealthSnapshotBody MakeBody(uint64_t handle, uint64_t serial_hash, const char* path)
{
	InputHealthSnapshotBody b{};
	b.handle = handle;
	b.device_serial_hash = serial_hash;
	std::memset(b.path, 0, INPUTHEALTH_PATH_LEN);
	if (path) {
		size_t n = std::strlen(path);
		if (n >= INPUTHEALTH_PATH_LEN) n = INPUTHEALTH_PATH_LEN - 1;
		std::memcpy(b.path, path, n);
	}
	return b;
}

std::unordered_map<uint64_t, FakeEntry> MakeMap(std::initializer_list<InputHealthSnapshotBody> bodies)
{
	std::unordered_map<uint64_t, FakeEntry> m;
	for (const auto& b : bodies) {
		m[b.handle].body = b;
	}
	return m;
}

} // namespace

// ---------------------------------------------------------------------------
// Empty and trivial inputs.
// ---------------------------------------------------------------------------

TEST(PresenceCounting, EmptyMapAllZeros)
{
	std::unordered_map<uint64_t, FakeEntry> m;
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.devices, 0);
	EXPECT_EQ(c.compensation_paths, 0);
	EXPECT_EQ(c.diagnostic_paths, 0);
	EXPECT_EQ(c.warnings, 0);
	EXPECT_EQ(c.zero_hash_slots, 0);
	EXPECT_EQ(c.unsupported_slots, 0);
}

TEST(PresenceCounting, ZeroHandleSlotsSkipped)
{
	// handle == 0 means the slot is empty / retired; the count helper must
	// not consider it at all (no path classification, no serial dedupe).
	auto m = MakeMap({
	    MakeBody(0, 0xAAAA, "/input/trigger/value"),
	    MakeBody(0, 0xBBBB, "/proximity"),
	});
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.devices, 0);
	EXPECT_EQ(c.compensation_paths, 0);
	EXPECT_EQ(c.unsupported_slots, 0);
}

// ---------------------------------------------------------------------------
// Distinct-device counting.
// ---------------------------------------------------------------------------

TEST(PresenceCounting, OneDeviceMultiplePaths)
{
	auto m = MakeMap({
	    MakeBody(1, 0xAAAA, "/input/trigger/value"),
	    MakeBody(2, 0xAAAA, "/input/grip/value"),
	    MakeBody(3, 0xAAAA, "/input/joystick/x"),
	});
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.devices, 1);
	EXPECT_EQ(c.compensation_paths, 3);
}

TEST(PresenceCounting, TwoDevicesThreePathsEach)
{
	auto m = MakeMap({
	    MakeBody(1, 0xAAAA, "/input/trigger/value"),
	    MakeBody(2, 0xAAAA, "/input/grip/value"),
	    MakeBody(3, 0xAAAA, "/input/a/click"),
	    MakeBody(4, 0xBBBB, "/input/trigger/value"),
	    MakeBody(5, 0xBBBB, "/input/grip/value"),
	    MakeBody(6, 0xBBBB, "/input/a/click"),
	});
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.devices, 2);
	EXPECT_EQ(c.compensation_paths, 6);
}

// ---------------------------------------------------------------------------
// Unsupported / diagnostic paths must not inflate the device count.
// ---------------------------------------------------------------------------

TEST(PresenceCounting, ProximitySlotsCountedAsUnsupported)
{
	// The real "18 controllers" bug regressed from /proximity slots having
	// their serial_hash inserted into the dedupe set. After the fix,
	// /proximity must land in unsupported_slots and contribute nothing to
	// devices or compensation_paths.
	auto m = MakeMap({
	    MakeBody(1, 0xAAAA, "/input/trigger/value"),
	    MakeBody(2, 0xCCCC, "/proximity"),
	    MakeBody(3, 0xDDDD, "/proximity"),
	});
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.devices, 1);
	EXPECT_EQ(c.compensation_paths, 1);
	EXPECT_EQ(c.unsupported_slots, 2);
}

TEST(PresenceCounting, EyeSlotsCountedAsDiagnosticOnly)
{
	auto m = MakeMap({
	    MakeBody(1, 0xAAAA, "/input/trigger/value"),
	    MakeBody(2, 0xEEEE, "/input/eye/openness"),
	    MakeBody(3, 0xEEEE, "/input/face/jawOpen"),
	});
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.devices, 1);
	EXPECT_EQ(c.compensation_paths, 1);
	EXPECT_EQ(c.diagnostic_paths, 2);
	EXPECT_EQ(c.unsupported_slots, 0);
}

TEST(PresenceCounting, PupilImuClassifiedUnsupported)
{
	auto m = MakeMap({
	    MakeBody(1, 0xAAAA, "/input/pupil/left"),
	    MakeBody(2, 0xAAAA, "/input/imu/raw"),
	});
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.devices, 0);
	EXPECT_EQ(c.compensation_paths, 0);
	EXPECT_EQ(c.unsupported_slots, 2);
}

// ---------------------------------------------------------------------------
// Zero-hash slots count separately, not as a phantom device.
// ---------------------------------------------------------------------------

TEST(PresenceCounting, ZeroSerialHashCountedSeparately)
{
	auto m = MakeMap({
	    MakeBody(1, 0, "/input/trigger/value"), // serial not yet resolved
	    MakeBody(2, 0xAAAA, "/input/trigger/value"),
	});
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.devices, 1);
	EXPECT_EQ(c.compensation_paths, 2);
	EXPECT_EQ(c.zero_hash_slots, 1);
}

// ---------------------------------------------------------------------------
// ph_triggered alarm count.
// ---------------------------------------------------------------------------

TEST(PresenceCounting, WarningsCountTriggeredFlag)
{
	auto m = MakeMap({
	    MakeBody(1, 0xAAAA, "/input/trigger/value"),
	    MakeBody(2, 0xAAAA, "/input/grip/value"),
	    MakeBody(3, 0xAAAA, "/input/joystick/x"),
	    MakeBody(4, 0xBBBB, "/input/trigger/value"),
	    MakeBody(5, 0xBBBB, "/input/grip/value"),
	});
	// Trigger ph_triggered on two slots.
	m[1].body.ph_triggered = 1;
	m[4].body.ph_triggered = 1;
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.compensation_paths, 5);
	EXPECT_EQ(c.warnings, 2);
}

TEST(PresenceCounting, WarningsIgnoredOnDiagnosticPaths)
{
	auto m = MakeMap({
	    MakeBody(1, 0xEEEE, "/input/eye/openness"),
	});
	m[1].body.ph_triggered = 1; // alarm on a diagnostics-only path is filtered
	const auto c = CountInputHealthPresence(m);
	EXPECT_EQ(c.warnings, 0);
	EXPECT_EQ(c.diagnostic_paths, 1);
}

// ---------------------------------------------------------------------------
// Realistic mixed bag mirroring the bug-report shape.
// ---------------------------------------------------------------------------

TEST(PresenceCounting, RealisticMixedSession)
{
	// Two hand controllers (5 paths each) + four trackers, each registered
	// with one /input/trigger/value-shaped pseudopath SteamVR exposes on
	// Vive pucks. A face-tracking sink registers eye/face paths. The
	// proximity sensor on each controller registers a /proximity slot.
	auto m = MakeMap({
	    // Controller A
	    MakeBody(10, 0xA001, "/input/trigger/value"),
	    MakeBody(11, 0xA001, "/input/grip/value"),
	    MakeBody(12, 0xA001, "/input/joystick/x"),
	    MakeBody(13, 0xA001, "/input/joystick/y"),
	    MakeBody(14, 0xA001, "/input/a/click"),
	    MakeBody(15, 0xA001, "/proximity"),
	    // Controller B
	    MakeBody(20, 0xA002, "/input/trigger/value"),
	    MakeBody(21, 0xA002, "/input/grip/value"),
	    MakeBody(22, 0xA002, "/input/joystick/x"),
	    MakeBody(23, 0xA002, "/input/joystick/y"),
	    MakeBody(24, 0xA002, "/input/a/click"),
	    MakeBody(25, 0xA002, "/proximity"),
	    // Trackers (each one path)
	    MakeBody(30, 0xB001, "/input/system/click"),
	    MakeBody(31, 0xB002, "/input/system/click"),
	    MakeBody(32, 0xB003, "/input/system/click"),
	    MakeBody(33, 0xB004, "/input/system/click"),
	    // Face/eye sink
	    MakeBody(40, 0xC001, "/input/eye/openness"),
	    MakeBody(41, 0xC001, "/input/face/jawOpen"),
	    MakeBody(42, 0xC001, "/input/face/smile"),
	});
	const auto c = CountInputHealthPresence(m);
	// devices = 2 controllers + 4 trackers (the face sink does not
	// contribute because all its paths are DiagnosticsOnly).
	EXPECT_EQ(c.devices, 6);
	// compensation paths = 5*2 + 1*4 = 14
	EXPECT_EQ(c.compensation_paths, 14);
	EXPECT_EQ(c.diagnostic_paths, 3);
	EXPECT_EQ(c.unsupported_slots, 2);
	EXPECT_EQ(c.zero_hash_slots, 0);
	EXPECT_EQ(c.warnings, 0);
}
