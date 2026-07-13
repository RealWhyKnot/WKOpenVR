#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "HarnessIpcClient.h"
#include "HarnessScenario.h"
#include "Protocol.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <thread>

namespace fs = std::filesystem;

namespace openvr_pair::overlay::testharness {

namespace {

// FaceTrackingFrameShmem (WKOpenVRFaceTrackingFrameRingV2) header layout per
// the survey: u32 magic, u32 shmem_version, u32 ring_size, u32 host_state,
// atomic<u64> host_heartbeat_qpc, ...
struct FaceTrackingShmemHeader
{
	uint32_t magic;
	uint32_t shmem_version;
	uint32_t ring_size;
	uint32_t host_state;
	uint64_t host_heartbeat_qpc;
};

bool ReadFaceTrackingHeader(FaceTrackingShmemHeader& out)
{
	HANDLE h = ::OpenFileMappingA(FILE_MAP_READ, FALSE, "WKOpenVRFaceTrackingFrameRingV2");
	if (h == nullptr) return false;
	void* view = ::MapViewOfFile(h, FILE_MAP_READ, 0, 0, 0);
	if (view == nullptr) {
		::CloseHandle(h);
		return false;
	}
	std::memcpy(&out, view, sizeof(out));
	::UnmapViewOfFile(view);
	::CloseHandle(h);
	return true;
}

} // namespace

ScenarioResult RunScenario_facetracking(ScenarioContext& ctx)
{
	const auto t0 = std::chrono::steady_clock::now();
	auto duration_now = [&]() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	};

	const fs::path host_exe = ctx.driver_resources / "facetracking" / "host" / "WKOpenVR.FaceModuleHost.exe";
	std::error_code ec;
	const bool host_present = fs::exists(host_exe, ec);
	if (!host_present) {
		ctx.log.Warn("FaceModuleHost.exe missing at " + host_exe.string() +
		             "; skipping host-launch portion (driver wiring still exercised)");
	}

	HarnessIpcClient client;
	try {
		ctx.log.Step("opening face-tracking pipe");
		client.OpenWithRetries(OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME);
	}
	catch (const std::exception& ex) {
		return Fail("facetracking", duration_now(), ex.what());
	}

	if (client.GetMismatchState() != HarnessIpcClient::MismatchState::Matching) {
		return Fail("facetracking", duration_now(),
		            "protocol version mismatch driver=" + std::to_string(client.GetDriverVersion()));
	}
	ctx.log.Info("handshake ok");

	try {
		ctx.log.Step("sending SetFaceTrackingConfig");
		protocol::Request req(protocol::RequestSetFaceTrackingConfig);
		auto& cfg = req.setFaceTrackingConfig;
		std::memset(&cfg, 0, sizeof(cfg));
		cfg.master_enabled = 1;
		cfg.eyelid_sync_enabled = 1;
		cfg.eyelid_sync_strength = 75;
		cfg.vergence_lock_enabled = 1;
		cfg.vergence_lock_strength = 50;
		cfg.output_osc_enabled = 1;
		cfg.gaze_smoothing = 50;
		cfg.openness_smoothing = 50;
		cfg.osc_port = 9000;
		std::snprintf(cfg.osc_host, sizeof(cfg.osc_host), "%s", "127.0.0.1");

		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("facetracking", duration_now(),
			            "expected ResponseSuccess, got type=" + std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("facetracking", duration_now(), std::string("SetFaceTrackingConfig failed: ") + ex.what());
	}

	// The driver creates WKOpenVRFaceTrackingFrameRingV2 during module Init,
	// before any host runs. Probe it.
	FaceTrackingShmemHeader header{};
	bool found = false;
	for (int attempt = 0; attempt < 30; ++attempt) {
		if (ReadFaceTrackingHeader(header)) {
			found = true;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	if (!found) {
		return Fail("facetracking", duration_now(), "WKOpenVRFaceTrackingFrameRingV2 shmem segment not visible");
	}
	if (header.magic != 0x46544652u /* 'FTFR' */) {
		return Fail("facetracking", duration_now(),
		            "WKOpenVRFaceTrackingFrameRingV2 magic=0x" + std::to_string(header.magic) +
		                " (expected 0x46544652)");
	}
	if (header.shmem_version != 3u) {
		return Fail("facetracking", duration_now(),
		            "WKOpenVRFaceTrackingFrameRingV2 version=" + std::to_string(header.shmem_version) +
		                " (expected 3)");
	}
	if (header.ring_size != 32u) {
		return Fail("facetracking", duration_now(),
		            "WKOpenVRFaceTrackingFrameRingV2 ring_size=" + std::to_string(header.ring_size));
	}
	ctx.log.Info("face shmem header OK (magic=FTFR, ver=3, ring=32)");

	// If the host is present, observe heartbeat progression. The driver's
	// HostSupervisor spawned it during module Init.
	if (host_present) {
		uint64_t hb0 = header.host_heartbeat_qpc;
		uint64_t hb1 = hb0;
		for (int attempt = 0; attempt < 60; ++attempt) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			FaceTrackingShmemHeader h2{};
			if (ReadFaceTrackingHeader(h2)) hb1 = h2.host_heartbeat_qpc;
			if (hb1 != hb0) break;
		}
		if (hb1 == hb0) {
			ctx.log.Warn("host heartbeat did not advance within 6s -- driver spawned "
			             "the host but the host process may have failed to start cleanly");
		}
		else {
			ctx.log.Info("host heartbeat advanced (qpc " + std::to_string(hb0) + " -> " + std::to_string(hb1) + ")");
		}
	}

	client.Close();
	return Pass("facetracking", duration_now());
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
