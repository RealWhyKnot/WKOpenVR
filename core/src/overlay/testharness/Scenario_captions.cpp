#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "HarnessIpcClient.h"
#include "HarnessScenario.h"
#include "Protocol.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>

namespace fs = std::filesystem;

namespace openvr_pair::overlay::testharness {

ScenarioResult RunScenario_captions(ScenarioContext& ctx)
{
	const auto t0 = std::chrono::steady_clock::now();
	auto duration_now = [&]() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	};

	const fs::path host_exe = ctx.driver_resources / "captions" / "host" / "WKOpenVR.CaptionsHost.exe";
	std::error_code ec;
	const bool host_present = fs::exists(host_exe, ec);
	if (!host_present) {
		ctx.log.Warn("CaptionsHost.exe missing at " + host_exe.string() + "; supervisor will report not-running");
	}

	HarnessIpcClient client;
	try {
		ctx.log.Step("opening captions pipe");
		client.OpenWithRetries(OPENVR_PAIRDRIVER_CAPTIONS_PIPE_NAME);
	}
	catch (const std::exception& ex) {
		return Fail("captions", duration_now(), ex.what());
	}

	if (client.GetMismatchState() != HarnessIpcClient::MismatchState::Matching) {
		return Fail("captions", duration_now(),
		            "protocol version mismatch driver=" + std::to_string(client.GetDriverVersion()));
	}
	ctx.log.Info("handshake ok");

	try {
		ctx.log.Step("sending SetCaptionsConfig");
		protocol::Request req(protocol::RequestSetCaptionsConfig);
		auto& cfg = req.setCaptionsConfig;
		std::memset(&cfg, 0, sizeof(cfg));
		cfg.master_enabled = 0; // keep disabled in the harness; we are exercising wire format
		cfg.mode = 0;
		cfg.notify_sound = 0;
		cfg.transcript_logging = 0;
		cfg.chatbox_port = 9000;
		std::snprintf(cfg.source_lang, sizeof(cfg.source_lang), "%s", "en");
		std::snprintf(cfg.target_lang, sizeof(cfg.target_lang), "%s", "en");
		std::snprintf(cfg.chatbox_address, sizeof(cfg.chatbox_address), "%s", "/chatbox/input");

		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("captions", duration_now(),
			            "expected ResponseSuccess on SetCaptionsConfig, got type=" + std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("captions", duration_now(), std::string("SetCaptionsConfig failed: ") + ex.what());
	}

	// Query the supervisor status. Even with master_enabled=0 the driver
	// keeps the supervisor block updated and the response should carry the
	// CaptionsSupervisorStatus payload.
	try {
		ctx.log.Step("sending CaptionsGetSupervisorStatus");
		protocol::Request req(protocol::RequestCaptionsGetSupervisorStatus);
		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseCaptionsSupervisorStatus) {
			return Fail("captions", duration_now(),
			            "expected ResponseCaptionsSupervisorStatus, got type=" + std::to_string((int)resp.type));
		}
		ctx.log.Info("supervisor: host_halted=" + std::to_string((int)resp.captionsSupervisorStatus.host_halted) +
		             ", last_exit_code=" + std::to_string(resp.captionsSupervisorStatus.last_exit_code));
	}
	catch (const std::exception& ex) {
		return Fail("captions", duration_now(), std::string("CaptionsGetSupervisorStatus failed: ") + ex.what());
	}

	client.Close();
	return Pass("captions", duration_now());
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
