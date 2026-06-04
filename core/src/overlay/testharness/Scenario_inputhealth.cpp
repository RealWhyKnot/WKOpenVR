#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "HarnessIpcClient.h"
#include "HarnessScenario.h"
#include "MockPoseSource.h" // brings in the full MockOpenVRRuntime definition
#include "Protocol.h"

#include <chrono>
#include <cstring>
#include <thread>

namespace openvr_pair::overlay::testharness {

namespace {

// Read the first 12 bytes of the InputHealth shmem segment. Returns true and
// fills [magic, version, slot_count] on success; false if the segment is
// missing or the header looks wrong.
bool ProbeInputHealthShmem(uint32_t& magic, uint32_t& version, uint32_t& slot_count)
{
	HANDLE h = ::OpenFileMappingA(FILE_MAP_READ, FALSE, "WKOpenVRInputHealthMemoryV1");
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
	slot_count = hdr[2];
	return true;
}

} // namespace

ScenarioResult RunScenario_inputhealth(ScenarioContext& ctx)
{
	const auto t0 = std::chrono::steady_clock::now();
	auto duration_now = [&]() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	};

	HarnessIpcClient client;
	try {
		ctx.log.Step("opening inputhealth pipe");
		client.OpenWithRetries(OPENVR_PAIRDRIVER_INPUTHEALTH_PIPE_NAME);
	}
	catch (const std::exception& ex) {
		return Fail("inputhealth", duration_now(), ex.what());
	}

	if (client.GetMismatchState() != HarnessIpcClient::MismatchState::Matching) {
		return Fail("inputhealth", duration_now(),
		            "protocol version mismatch driver=" + std::to_string(client.GetDriverVersion()));
	}
	ctx.log.Info("handshake ok");

	try {
		ctx.log.Step("sending SetInputHealthConfig");
		protocol::Request req(protocol::RequestSetInputHealthConfig);
		req.setInputHealthConfig.master_enabled = true;
		req.setInputHealthConfig.diagnostics_only = false;
		req.setInputHealthConfig.enable_rest_recenter = true;
		req.setInputHealthConfig.enable_trigger_remap = true;
		req.setInputHealthConfig._reserved = 0;
		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("inputhealth", duration_now(),
			            "expected ResponseSuccess, got type=" + std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("inputhealth", duration_now(), std::string("SetInputHealthConfig failed: ") + ex.what());
	}

	// Verify the snapshot shmem is mapped + correctly initialized. The
	// driver-side InputHealthSnapshotStaging creates this segment during
	// module Init; if our handshake succeeded the segment MUST be present.
	uint32_t magic = 0, version = 0, slot_count = 0;
	bool found = false;
	for (int attempt = 0; attempt < 20; ++attempt) {
		if (ProbeInputHealthShmem(magic, version, slot_count)) {
			found = true;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	if (!found) {
		return Fail("inputhealth", duration_now(), "WKOpenVRInputHealthMemoryV1 shmem segment not visible");
	}
	if (magic != 0x494E4848u /* 'INHH' */) {
		return Fail("inputhealth", duration_now(),
		            "WKOpenVRInputHealthMemoryV1 magic=0x" + std::to_string(magic) + " (expected 0x494E4848)");
	}
	if (version != 3u) {
		return Fail("inputhealth", duration_now(),
		            "WKOpenVRInputHealthMemoryV1 version=" + std::to_string(version) + " (expected 3)");
	}
	if (slot_count != 256u) {
		return Fail("inputhealth", duration_now(),
		            "WKOpenVRInputHealthMemoryV1 slot_count=" + std::to_string(slot_count) + " (expected 256)");
	}
	ctx.log.Info("inputhealth shmem header OK (magic=INHH, ver=3, slots=256)");

	// Exercise the mock IVRDriverInput path: simulate a scalar update on a
	// trigger-like path; the InputHealth MinHook detour observes it and
	// updates its learning state. We do not assert the learned value here
	// (covered by inputhealth_tests.exe) but we do assert the call chain ran.
	auto& input = ctx.mock.driver_input();
	vr::PropertyContainerHandle_t container =
	    (vr::PropertyContainerHandle_t)ctx.mock.properties().TrackedDeviceToPropertyContainer(0);
	vr::VRInputComponentHandle_t handle = 0;
	if (input.CreateScalarComponent(container, "/user/hand/left/input/trigger/value", &handle,
	                                vr::VRScalarType_Absolute,
	                                vr::VRScalarUnits_NormalizedOneSided) != vr::VRInputError_None) {
		return Fail("inputhealth", duration_now(), "CreateScalarComponent failed on mock IVRDriverInput");
	}
	uint64_t cursor = 0;
	for (int i = 0; i < 25; ++i) {
		input.UpdateScalarComponent(handle, 0.05f + (i * 0.001f), 0.0);
	}
	const auto observed = ctx.mock.recorder().CountSince(
	    [](const MockCall& c) { return c.kind == MockCallKind::UpdateScalarComponent; }, cursor);
	if (observed < 25) {
		return Fail("inputhealth", duration_now(),
		            "expected >=25 UpdateScalarComponent records, observed " + std::to_string(observed));
	}

	client.Close();
	ctx.log.Info("inputhealth scenario complete");
	return Pass("inputhealth", duration_now());
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
