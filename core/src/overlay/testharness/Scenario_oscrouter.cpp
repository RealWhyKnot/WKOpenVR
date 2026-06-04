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
#include <thread>

namespace openvr_pair::overlay::testharness {

namespace {

// Write a single fire-and-forget datagram to the OSC publish pipe in the
// format the driver expects (per Protocol.h kOscRouterPubPipe comments):
//   [ 32 bytes: source-id, NUL-padded ]
//   [  4 bytes: LE uint32 frame length ]
//   [  N bytes: raw OSC packet ]
bool WriteOscPublishDatagram(HANDLE pipe, const char* source_id, const void* osc_bytes, uint32_t osc_len)
{
	char source[32]{};
	std::snprintf(source, sizeof(source), "%s", source_id ? source_id : "");
	if (!pipe || pipe == INVALID_HANDLE_VALUE) return false;

	DWORD written = 0;
	if (!::WriteFile(pipe, source, sizeof(source), &written, nullptr) || written != sizeof(source)) return false;
	if (!::WriteFile(pipe, &osc_len, sizeof(osc_len), &written, nullptr) || written != sizeof(osc_len)) return false;
	if (osc_len > 0) {
		if (!::WriteFile(pipe, osc_bytes, osc_len, &written, nullptr) || written != osc_len) return false;
	}
	return true;
}

// Tiny stub OSC packet: address "/test/harness", typetag ",i", arg=42.
// The driver parses for routing only; we do not need a fully spec-compliant
// payload, just enough bytes to survive boundary checks.
struct StubOscPacket
{
	char address[16]; // "/test/harness\0\0\0" (padded to 16 = multiple of 4)
	char typetag[4];  // ",i\0\0"
	uint32_t arg_be;  // 42 in big-endian
};

StubOscPacket MakeStubPacket()
{
	StubOscPacket p{};
	std::memcpy(p.address, "/test/harness\0\0", 16);
	std::memcpy(p.typetag, ",i\0\0", 4);
	const uint32_t v = 42u;
	p.arg_be = ((v & 0xFFu) << 24) | (((v >> 8) & 0xFFu) << 16) | (((v >> 16) & 0xFFu) << 8) | ((v >> 24) & 0xFFu);
	return p;
}

} // namespace

ScenarioResult RunScenario_oscrouter(ScenarioContext& ctx)
{
	const auto t0 = std::chrono::steady_clock::now();
	auto duration_now = [&]() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	};

	HarnessIpcClient client;
	try {
		ctx.log.Step("opening oscrouter pipe");
		client.OpenWithRetries(OPENVR_PAIRDRIVER_OSCROUTER_PIPE_NAME);
	}
	catch (const std::exception& ex) {
		return Fail("oscrouter", duration_now(), ex.what());
	}

	if (client.GetMismatchState() != HarnessIpcClient::MismatchState::Matching) {
		return Fail("oscrouter", duration_now(),
		            "protocol version mismatch driver=" + std::to_string(client.GetDriverVersion()));
	}
	ctx.log.Info("handshake ok");

	// Subscribe to a glob pattern.
	try {
		ctx.log.Step("sending OscRouteSubscribe");
		protocol::Request req(protocol::RequestOscRouteSubscribe);
		auto& sub = req.oscRouteSubscribe;
		sub.subscriber_id = 0xCAFE0001u;
		sub.enabled = 1;
		sub._reserved[0] = sub._reserved[1] = sub._reserved[2] = 0;
		std::snprintf(sub.pattern, sizeof(sub.pattern), "%s", "/test/*");
		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseSuccess) {
			return Fail("oscrouter", duration_now(),
			            "expected ResponseSuccess on Subscribe, got type=" + std::to_string((int)resp.type));
		}
	}
	catch (const std::exception& ex) {
		return Fail("oscrouter", duration_now(), std::string("OscRouteSubscribe failed: ") + ex.what());
	}

	// Push a handful of datagrams via the publish pipe.
	ctx.log.Step("publishing 5 synthetic OSC datagrams");
	const auto packet = MakeStubPacket();
	HANDLE pub = INVALID_HANDLE_VALUE;
	for (int attempt = 0; attempt < 20 && pub == INVALID_HANDLE_VALUE; ++attempt) {
		pub = ::CreateFileA(OPENVR_PAIRDRIVER_OSCROUTER_PUB_PIPE_NAME, GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0,
		                    nullptr);
		if (pub == INVALID_HANDLE_VALUE) std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	if (pub == INVALID_HANDLE_VALUE) {
		return Fail("oscrouter", duration_now(), "could not open OscRouterPub pipe after retries");
	}
	DWORD mode = PIPE_READMODE_MESSAGE;
	::SetNamedPipeHandleState(pub, &mode, nullptr, nullptr);
	int published = 0;
	for (int i = 0; i < 5; ++i) {
		if (WriteOscPublishDatagram(pub, "harness-osc-source-id-padded-12", &packet, sizeof(packet))) ++published;
	}
	::CloseHandle(pub);
	if (published < 5) {
		return Fail("oscrouter", duration_now(),
		            "OSC publish writes did not all succeed (" + std::to_string(published) + "/5)");
	}

	// Brief wait for the driver to ingest + tally.
	std::this_thread::sleep_for(std::chrono::milliseconds(300));

	// Query stats.
	try {
		ctx.log.Step("sending OscGetStats");
		protocol::Request req(protocol::RequestOscGetStats);
		const auto resp = client.SendBlocking(req);
		if (resp.type != protocol::ResponseOscRouterStats) {
			return Fail("oscrouter", duration_now(),
			            "expected ResponseOscRouterStats, got type=" + std::to_string((int)resp.type));
		}
		ctx.log.Info("router stats: packets_sent=" + std::to_string(resp.oscRouterStats.packets_sent) +
		             ", bytes_sent=" + std::to_string(resp.oscRouterStats.bytes_sent) +
		             ", active_routes=" + std::to_string(resp.oscRouterStats.active_routes));
	}
	catch (const std::exception& ex) {
		return Fail("oscrouter", duration_now(), std::string("OscGetStats failed: ") + ex.what());
	}

	client.Close();
	return Pass("oscrouter", duration_now());
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
