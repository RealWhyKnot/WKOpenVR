#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include "HarnessIpcClient.h"
#include "HarnessScenario.h"
#include "Protocol.h"

#include <chrono>
#include <exception>
#include <string>

namespace openvr_pair::overlay::testharness {

ScenarioResult RunScenario_dashboardinput(ScenarioContext& ctx)
{
	const auto t0 = std::chrono::steady_clock::now();
	auto duration_now = [&]() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	};

	HarnessIpcClient client;
	try {
		ctx.log.Step("opening dashboard input pipe");
		client.OpenWithRetries(OPENVR_PAIRDRIVER_DASHBOARDINPUT_PIPE_NAME);
	}
	catch (const std::exception& ex) {
		return Fail("dashboardinput", duration_now(), ex.what());
	}

	if (client.GetMismatchState() != HarnessIpcClient::MismatchState::Matching) {
		return Fail("dashboardinput", duration_now(),
		            "protocol version mismatch driver=" + std::to_string(client.GetDriverVersion()));
	}
	ctx.log.Info("handshake ok");

	try {
		ctx.log.Step("sending dashboard hand state");
		protocol::Request req(protocol::RequestSetDashboardHandTrackingState);
		req.setDashboardHandTrackingState.enabled = 1;
		req.setDashboardHandTrackingState.dashboard_visible = 1;
		req.setDashboardHandTrackingState.primary_hand = protocol::DashboardHandTrackingHandRight;
		req.setDashboardHandTrackingState._reserved = 0;
		req.setDashboardHandTrackingState.update_mono_ms = 1234567;

		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("dashboardinput", duration_now(),
			            "expected ResponseSuccess, got type=" + std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("dashboardinput", duration_now(), std::string("dashboard hand state failed: ") + ex.what());
	}

	client.Close();
	ctx.log.Info("dashboard input pipe roundtrip complete");
	return Pass("dashboardinput", duration_now());
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
