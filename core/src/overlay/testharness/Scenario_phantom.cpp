#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "HarnessIpcClient.h"
#include "HarnessScenario.h"
#include "MockPoseSource.h"
#include "PhantomStateShmem.h"
#include "Protocol.h"

#include <chrono>
#include <cstring>
#include <thread>

namespace openvr_pair::overlay::testharness {

namespace {

uint64_t Fnv1a64(const std::string& s)
{
	uint64_t h = 0xcbf29ce484222325ULL;
	for (char c : s) {
		h ^= (uint64_t)(uint8_t)c;
		h *= 0x100000001b3ULL;
	}
	return h;
}

bool ProbePhantomShmem(uint32_t& magic, uint32_t& version, uint32_t& device_count)
{
	HANDLE h = ::OpenFileMappingA(FILE_MAP_READ, FALSE, "WKOpenVRPhantomStateV2");
	if (h == nullptr) return false;
	void* view = ::MapViewOfFile(h, FILE_MAP_READ, 0, 0, 0);
	if (view == nullptr) {
		::CloseHandle(h);
		return false;
	}
	uint32_t hdr[3]{};
	std::memcpy(hdr, view, sizeof(hdr));
	::UnmapViewOfFile(view);
	::CloseHandle(h);
	magic = hdr[0];
	version = hdr[1];
	device_count = hdr[2];
	return true;
}

} // namespace

ScenarioResult RunScenario_phantom(ScenarioContext& ctx)
{
	const auto t0 = std::chrono::steady_clock::now();
	auto duration_now = [&]() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	};

	HarnessIpcClient client;
	try {
		ctx.log.Step("opening phantom pipe");
		client.OpenWithRetries(OPENVR_PAIRDRIVER_PHANTOM_PIPE_NAME);
	}
	catch (const std::exception& ex) {
		return Fail("phantom", duration_now(), ex.what());
	}

	if (client.GetMismatchState() != HarnessIpcClient::MismatchState::Matching) {
		return Fail("phantom", duration_now(),
		            "protocol version mismatch driver=" + std::to_string(client.GetDriverVersion()));
	}
	ctx.log.Info("handshake ok");

	try {
		ctx.log.Step("sending SetPhantomConfig");
		protocol::Request req(protocol::RequestSetPhantomConfig);
		auto& cfg = req.setPhantomConfig;
		std::memset(&cfg, 0, sizeof(cfg));
		cfg.master_enabled = 1;
		cfg.blend_out_ms = 250;
		cfg.blend_in_ms = 250;
		cfg.reckon_hold_ms = 500;
		cfg.synth_hold_ms = 2000;
		cfg.lost_hold_ms = 5000;

		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("phantom", duration_now(),
			            "expected ResponseSuccess on SetPhantomConfig, got type=" + std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("phantom", duration_now(), std::string("SetPhantomConfig failed: ") + ex.what());
	}

	// Assign a role to a synthetic device serial and opt it in.
	const std::string serial = "PHN-HARNESS-1";
	const uint64_t serial_hash = Fnv1a64(serial);

	try {
		ctx.log.Step("sending SetPhantomDeviceOptIn");
		protocol::Request req(protocol::RequestSetPhantomDeviceOptIn);
		req.setPhantomDeviceOptIn.device_serial_hash = serial_hash;
		req.setPhantomDeviceOptIn.dropout_enabled = 1;
		std::memset(req.setPhantomDeviceOptIn._reserved, 0, sizeof(req.setPhantomDeviceOptIn._reserved));
		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("phantom", duration_now(),
			            "expected ResponseSuccess on SetPhantomDeviceOptIn, got type=" +
			                std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("phantom", duration_now(), std::string("SetPhantomDeviceOptIn failed: ") + ex.what());
	}

	try {
		ctx.log.Step("sending SetPhantomDeviceRole");
		protocol::Request req(protocol::RequestSetPhantomDeviceRole);
		req.setPhantomDeviceRole.device_serial_hash = serial_hash;
		req.setPhantomDeviceRole.body_role = 3; // any plausible role index
		std::memset(req.setPhantomDeviceRole._reserved, 0, sizeof(req.setPhantomDeviceRole._reserved));
		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("phantom", duration_now(),
			            "expected ResponseSuccess on SetPhantomDeviceRole, got type=" + std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("phantom", duration_now(), std::string("SetPhantomDeviceRole failed: ") + ex.what());
	}

	// Probe the phantom-state shmem.
	uint32_t magic = 0, version = 0, device_count = 0;
	bool found = false;
	for (int attempt = 0; attempt < 30; ++attempt) {
		if (ProbePhantomShmem(magic, version, device_count)) {
			found = true;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	if (!found) {
		return Fail("phantom", duration_now(), "WKOpenVRPhantomStateV2 shmem segment not visible");
	}
	if (magic != phantom::kPhantomStateShmemMagic) {
		return Fail("phantom", duration_now(),
		            "WKOpenVRPhantomStateV2 magic=0x" + std::to_string(magic) + " (expected 0x54534850)");
	}
	if (version != phantom::kPhantomStateShmemVersion) {
		return Fail("phantom", duration_now(),
		            "WKOpenVRPhantomStateV2 version=" + std::to_string(version) + " (expected " +
		                std::to_string(phantom::kPhantomStateShmemVersion) + ")");
	}
	if (device_count != phantom::kMaxPhantomDevices) {
		return Fail("phantom", duration_now(),
		            "WKOpenVRPhantomStateV2 device_count=" + std::to_string(device_count) + " (expected " +
		                std::to_string(phantom::kMaxPhantomDevices) + ")");
	}
	ctx.log.Info("phantom shmem header OK (magic=PHST, ver=" + std::to_string(phantom::kPhantomStateShmemVersion) +
	             ", devices=" + std::to_string(phantom::kMaxPhantomDevices) + ")");

	// Push a couple of poses on a fake device so the phantom module sees
	// "real" tracking. The phantom module hooks via the calibration pose
	// detour; we are exercising the wire here, not the dropout ladder math.
	const uint32_t devId = ctx.pose_source.AddDevice(serial, vr::TrackedDeviceClass_GenericTracker);
	for (int i = 0; i < 8; ++i) {
		auto pose = MockPoseSource::MakeIdentityPose(0.0, 1.0, 0.0);
		ctx.pose_source.PushPose(devId, pose);
	}

	client.Close();
	return Pass("phantom", duration_now());
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
