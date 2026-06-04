#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include "HarnessIpcClient.h"
#include "HarnessScenario.h"
#include "MockPoseSource.h"
#include "Protocol.h"

#include <chrono>

namespace openvr_pair::overlay::testharness {

ScenarioResult RunScenario_smoothing(ScenarioContext& ctx)
{
	const auto t0 = std::chrono::steady_clock::now();
	auto duration_now = [&]() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	};

	HarnessIpcClient client;
	try {
		ctx.log.Step("opening smoothing pipe");
		client.OpenWithRetries(OPENVR_PAIRDRIVER_SMOOTHING_PIPE_NAME);
	}
	catch (const std::exception& ex) {
		return Fail("smoothing", duration_now(), ex.what());
	}

	if (client.GetMismatchState() != HarnessIpcClient::MismatchState::Matching) {
		return Fail("smoothing", duration_now(),
		            "protocol version mismatch driver=" + std::to_string(client.GetDriverVersion()));
	}
	ctx.log.Info("handshake ok");

	// SetDevicePrediction: clamp predictionSmoothness on device 0 to 80, which
	// the driver records and applies on the next pose forward. We then push
	// jittery poses and rely on the calibration detour to propagate them; the
	// quantitative variance check lives in spacecal_tests' smoothing suite.
	try {
		ctx.log.Step("sending SetDevicePrediction");
		protocol::Request req(protocol::RequestSetDevicePrediction);
		req.setDevicePrediction.openVRID = 0;
		req.setDevicePrediction.predictionSmoothness = 80;
		req.setDevicePrediction.smart_enabled = 0;
		req.setDevicePrediction._reserved[0] = 0;
		req.setDevicePrediction._reserved[1] = 0;

		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("smoothing", duration_now(),
			            "expected ResponseSuccess, got type=" + std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("smoothing", duration_now(), std::string("SetDevicePrediction failed: ") + ex.what());
	}

	// Also exercise the finger smoothing config path -- different opcode,
	// different union member, sharing the same wire format.
	try {
		ctx.log.Step("sending FingerSmoothingConfig");
		protocol::Request req(protocol::RequestSetFingerSmoothing);
		auto& cfg = req.setFingerSmoothing;
		cfg.master_enabled = true;
		cfg.smoothness = 50;
		cfg.finger_mask = protocol::kAllFingersMask;
		cfg._reserved = 0;
		for (int i = 0; i < 10; ++i)
			cfg.per_finger_smoothness[i] = 0;
		cfg._reserved2[0] = cfg._reserved2[1] = 0;

		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("smoothing", duration_now(),
			            "expected ResponseSuccess on RequestSetFingerSmoothing, got type=" +
			                std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("smoothing", duration_now(), std::string("RequestSetFingerSmoothing failed: ") + ex.what());
	}

	client.Close();
	ctx.log.Info("smoothing pipe roundtrip complete");
	return Pass("smoothing", duration_now());
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
