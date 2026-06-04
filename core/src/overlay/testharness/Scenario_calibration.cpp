#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include "HarnessIpcClient.h"
#include "HarnessScenario.h"
#include "MockPoseSource.h"
#include "Protocol.h"

#include <chrono>

namespace openvr_pair::overlay::testharness {

ScenarioResult RunScenario_calibration(ScenarioContext& ctx)
{
	const auto t0 = std::chrono::steady_clock::now();
	auto duration_now = [&]() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	};

	HarnessIpcClient client;
	try {
		ctx.log.Step("opening calibration pipe");
		client.OpenWithRetries(OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME);
	}
	catch (const std::exception& ex) {
		return Fail("calibration", duration_now(), ex.what());
	}

	if (client.GetMismatchState() != HarnessIpcClient::MismatchState::Matching) {
		return Fail("calibration", duration_now(),
		            "protocol version mismatch driver=" + std::to_string(client.GetDriverVersion()) +
		                " expected=" + std::to_string(client.GetExpectedVersion()));
	}
	ctx.log.Info("handshake ok, protocol v" + std::to_string(client.GetDriverVersion()));

	// Drive a SetDeviceTransform request through the wire. The driver
	// validates the payload and writes telemetry into the pose shmem.
	try {
		ctx.log.Step("sending SetDeviceTransform");
		protocol::SetDeviceTransform xfm(0u, true);
		xfm.updateTranslation = true;
		xfm.translation = vr::HmdVector3d_t{0.1, 0.0, 0.0};
		xfm.predictionSmoothness = 0;

		protocol::Request req(protocol::RequestSetDeviceTransform);
		req.setDeviceTransform = xfm;

		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("calibration", duration_now(),
			            "expected ResponseSuccess, got type=" + std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("calibration", duration_now(), std::string("SetDeviceTransform failed: ") + ex.what());
	}

	// Push a couple of synthetic poses on a fake reference tracker, so the
	// driver's pose path runs at least once. This exercises the calibration
	// detour without requiring a full Activate() handshake from a fake
	// ITrackedDeviceServerDriver -- we are testing wire reachability + pose
	// emission, not the calibration math itself (covered by spacecal_tests).
	const uint32_t ref_id = ctx.pose_source.AddDevice("CAL-REF-1", vr::TrackedDeviceClass_GenericTracker);
	for (int i = 0; i < 5; ++i) {
		auto pose = MockPoseSource::MakeIdentityPose(0.5, 1.0, -0.5);
		ctx.pose_source.PushPose(ref_id, pose);
	}

	uint64_t cursor = 0;
	const auto added_match = ctx.mock.recorder().CountSince(
	    [ref_id](const MockCall& c) { return c.kind == MockCallKind::TrackedDeviceAdded && c.device_id == ref_id; },
	    cursor);
	if (added_match == 0) {
		return Fail("calibration", duration_now(),
		            "mock did not record TrackedDeviceAdded for the fake reference tracker");
	}

	client.Close();
	ctx.log.Info("calibration pipe roundtrip complete");
	return Pass("calibration", duration_now());
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
