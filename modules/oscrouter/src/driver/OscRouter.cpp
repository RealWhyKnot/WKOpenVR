#define _CRT_SECURE_NO_DEPRECATE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "OscRouter.h"
#include "OscWire.h"
#include "Logging.h"
#include "FeatureFlags.h"
#include "ModulePerf.h"
#include "ModuleRegistry.h"
#include "Protocol.h"

#include <chrono>
#include <cstring>
#include <memory>

namespace oscrouter {

std::atomic<OscRouter*> g_activeRouter{nullptr};

static bool ShouldLogCounter(uint64_t value);

namespace {

using RouterClock = std::chrono::steady_clock;

struct SendWorkerDiagSample
{
	uint64_t sent = 0;
	uint64_t bytes = 0;
	uint64_t dropped = 0;
	uint64_t queued = 0;
	uint64_t oversize = 0;
	uint64_t unmatched = 0;
	uint64_t send_fail = 0;
	uint64_t socket_drop = 0;
	size_t queue_depth = 0;
	uint32_t active_routes = 0;
};

struct SendWorkerDiagnostics
{
	RouterClock::time_point next_log = RouterClock::now() + std::chrono::seconds(5);
	SendWorkerDiagSample last{};

	bool IsDue(RouterClock::time_point now) const { return now >= next_log; }

	void MaybeLog(RouterClock::time_point now, const SendWorkerDiagSample& sample, const char* host, uint16_t port)
	{
		if (now < next_log) return;
		next_log = now + std::chrono::seconds(5);

		const bool changed = sample.sent != last.sent || sample.dropped != last.dropped ||
		                     sample.queued != last.queued || sample.oversize != last.oversize ||
		                     sample.unmatched != last.unmatched || sample.send_fail != last.send_fail ||
		                     sample.socket_drop != last.socket_drop || sample.queue_depth != 0;
		if (changed) {
			OR_LOG("[OscRouter][diag] period queued=%llu sent=%llu bytes=%llu dropped=%llu "
			       "oversize=%llu unmatched=%llu send_fail=%llu socket_drop=%llu q_depth=%zu routes=%u target=%s:%u",
			       static_cast<unsigned long long>(sample.queued - last.queued),
			       static_cast<unsigned long long>(sample.sent - last.sent),
			       static_cast<unsigned long long>(sample.bytes - last.bytes),
			       static_cast<unsigned long long>(sample.dropped - last.dropped),
			       static_cast<unsigned long long>(sample.oversize - last.oversize),
			       static_cast<unsigned long long>(sample.unmatched - last.unmatched),
			       static_cast<unsigned long long>(sample.send_fail - last.send_fail),
			       static_cast<unsigned long long>(sample.socket_drop - last.socket_drop), sample.queue_depth,
			       static_cast<unsigned>(sample.active_routes), host, static_cast<unsigned>(port));
		}
		last = sample;
	}
};

struct PubPipeClientDiagnostics
{
	char source[33] = {};
	uint64_t frames = 0;
	uint64_t bytes = 0;
	uint64_t bad_lengths = 0;
	uint64_t invalid_osc = 0;
	uint64_t queue_drops = 0;
	uint64_t read_breaks = 0;
	bool source_logged = false;

	const char* SourceForLog() const { return source[0] ? source : "(unknown)"; }

	void RememberSource(const char* source_id)
	{
		memcpy(source, source_id, 32);
		source[32] = '\0';
		if (!source_logged) {
			OR_LOG("OscRouter pub-pipe source='%.32s'", source);
			source_logged = true;
		}
	}

	void RecordQueued(uint32_t frame_len)
	{
		++frames;
		bytes += frame_len;
	}

	void RecordReadBreak(std::atomic<uint64_t>& aggregate)
	{
		++read_breaks;
		aggregate.fetch_add(1, std::memory_order_relaxed);
	}

	void RecordBadLength(std::atomic<uint64_t>& aggregate, uint32_t frame_len)
	{
		++bad_lengths;
		aggregate.fetch_add(1, std::memory_order_relaxed);
		OR_LOG("OscRouter pub-pipe: invalid frame length %u from %.32s", static_cast<unsigned>(frame_len),
		       SourceForLog());
	}

	void RecordInvalidOsc(std::atomic<uint64_t>& aggregate, uint32_t frame_len)
	{
		++invalid_osc;
		aggregate.fetch_add(1, std::memory_order_relaxed);
		if (ShouldLogCounter(invalid_osc)) {
			OR_LOG("OscRouter pub-pipe: invalid OSC frame source='%.32s' bytes=%u invalid_total=%llu", SourceForLog(),
			       static_cast<unsigned>(frame_len), static_cast<unsigned long long>(invalid_osc));
		}
	}

	void RecordQueueDrop(std::atomic<uint64_t>& aggregate, uint64_t total_drops, size_t queue_depth)
	{
		++queue_drops;
		aggregate.fetch_add(1, std::memory_order_relaxed);
		if (ShouldLogCounter(total_drops)) {
			OR_LOG("OscRouter pub-pipe: queue full source='%.32s' total_dropped=%llu q_count=%zu", SourceForLog(),
			       static_cast<unsigned long long>(total_drops), queue_depth);
		}
	}

	void LogDisconnect() const
	{
		OR_LOG("OscRouter pub-pipe client disconnected source='%.32s' frames=%llu bytes=%llu "
		       "bad_len=%llu invalid_osc=%llu queue_drop=%llu read_break=%llu",
		       SourceForLog(), static_cast<unsigned long long>(frames), static_cast<unsigned long long>(bytes),
		       static_cast<unsigned long long>(bad_lengths), static_cast<unsigned long long>(invalid_osc),
		       static_cast<unsigned long long>(queue_drops), static_cast<unsigned long long>(read_breaks));
	}
};

} // namespace

// ---------------------------------------------------------------------------
// DriverModule factory
// ---------------------------------------------------------------------------

std::unique_ptr<DriverModule> CreateDriverModule()
{
	return std::make_unique<OscRouter>();
}

// ---------------------------------------------------------------------------
// OscRouter
// ---------------------------------------------------------------------------

OscRouter::OscRouter() = default;

OscRouter::~OscRouter()
{
	Shutdown();
}

void OscRouter::SetSendEndpointForTesting(const char* host, uint16_t port)
{
	if (host && host[0] != '\0') sendHost_ = host;
	if (port != 0) sendPort_ = port;
	if (sender_.IsOpen()) {
		sender_.Open(sendHost_.c_str(), sendPort_);
	}
}

uint32_t OscRouter::FeatureMask() const
{
	return pairdriver::kFeatureOscRouter;
}

const char* OscRouter::PipeName() const
{
	return openvr_pair::common::modules::PipeName(openvr_pair::common::modules::ModuleId::OscRouter);
}

bool OscRouter::Init(DriverModuleContext& ctx)
{
	(void)ctx;
	OrDrvOpenLogFile();
	OR_LOG("OscRouter::Init target=%s:%u enabled=%d", sendHost_.c_str(), (unsigned)sendPort_, enabled_ ? 1 : 0);

	// Create stats shmem.
	if (!statsShmem_.Create(OPENVR_PAIRDRIVER_OSCROUTER_SHMEM_NAME)) {
		OR_LOG("OscRouter: shmem create failed (GetLastError=%u); stats disabled", (unsigned)GetLastError());
	}
	else {
		shmemReady_ = true;
	}

	// Open UDP send socket.
	if (!sender_.Open(sendHost_.c_str(), sendPort_)) {
		OR_LOG("OscRouter: UDP open failed; will retry on config push");
	}

	stop_ = false;

	sendWorker_ = std::thread(&OscRouter::SendWorkerMain, this);
	pubPipeWorker_ = std::thread(&OscRouter::PubPipeWorkerMain, this);

	// Register the singleton so RouterPublishApi can reach us.
	g_activeRouter.store(this, std::memory_order_release);
	OR_LOG("OscRouter: initialized, sending to %s:%u", sendHost_.c_str(), (unsigned)sendPort_);
	return true;
}

void OscRouter::Shutdown()
{
	g_activeRouter.store(nullptr, std::memory_order_release);

	stop_.store(true, std::memory_order_release);
	queueCv_.notify_all();

	if (sendWorker_.joinable()) sendWorker_.join();
	if (pubPipeWorker_.joinable()) pubPipeWorker_.join();

	sender_.Close();
	statsShmem_.Close();
	OR_LOG("OscRouter: shutdown complete sent=%llu bytes=%llu queued=%llu dropped=%llu "
	       "oversize=%llu unmatched=%llu send_fail=%llu pub_frames=%llu pub_bad=%llu",
	       static_cast<unsigned long long>(packetsSent_.load(std::memory_order_relaxed)),
	       static_cast<unsigned long long>(bytesSent_.load(std::memory_order_relaxed)),
	       static_cast<unsigned long long>(packetsQueued_.load(std::memory_order_relaxed)),
	       static_cast<unsigned long long>(droppedCount_.load(std::memory_order_relaxed)),
	       static_cast<unsigned long long>(oversizedPackets_.load(std::memory_order_relaxed)),
	       static_cast<unsigned long long>(unmatchedRoutes_.load(std::memory_order_relaxed)),
	       static_cast<unsigned long long>(sendFailures_.load(std::memory_order_relaxed)),
	       static_cast<unsigned long long>(pubPipeFrames_.load(std::memory_order_relaxed)),
	       static_cast<unsigned long long>(pubPipeInvalidFrames_.load(std::memory_order_relaxed)));
}

// ---------------------------------------------------------------------------
// Queue helpers
// ---------------------------------------------------------------------------

bool OscRouter::EnqueueLocked(const void* data, size_t size)
{
	if (qCount_ >= kSendQueueCapacity) return false;
	if (size > kSendQueueEntryMaxSize) return false;
	SendEntry& e = queue_[qTail_];
	memcpy(e.data, data, size);
	e.size = size;
	qTail_ = (qTail_ + 1) % kSendQueueCapacity;
	++qCount_;
	return true;
}

bool OscRouter::BuildAndEnqueue(const char* address, const char* typetag, const void* args, size_t arg_len)
{
	OscPacket<kSendQueueEntryMaxSize> pkt;
	pkt.Begin(address, typetag);
	if (args && arg_len > 0) pkt.WriteRawArgs(args, arg_len);
	if (!pkt.Ok()) {
		const uint64_t count = oversizedPackets_.fetch_add(1, std::memory_order_relaxed) + 1;
		if (ShouldLogCounter(count)) {
			OR_LOG("OscRouter: packet too large for address=%s (arg_len=%zu total_oversize=%llu)",
			       address ? address : "(null)", arg_len, static_cast<unsigned long long>(count));
		}
		return false;
	}
	std::lock_guard<std::mutex> lk(queueMutex_);
	bool queued = EnqueueLocked(pkt.Data(), pkt.Size());
	if (!queued) {
		const uint64_t drops = droppedCount_.fetch_add(1, std::memory_order_relaxed) + 1;
		routeTable_.BumpDropCount(address);
		if (shmemReady_) statsShmem_.AddDropped();
		if (ShouldLogCounter(drops)) {
			OR_LOG("OscRouter: send queue full, dropped address=%s total_dropped=%llu q_count=%zu",
			       address ? address : "(null)", static_cast<unsigned long long>(drops), qCount_);
		}
	}
	else {
		packetsQueued_.fetch_add(1, std::memory_order_relaxed);
	}
	if (queued) queueCv_.notify_one();
	return queued;
}

// ---------------------------------------------------------------------------
// In-process publish API
// ---------------------------------------------------------------------------

bool OscRouter::PublishOsc(const char* source_id, const char* address, const char* typetag, const void* args,
                           size_t arg_len)
{
	(void)source_id; // used for stats attribution in future
	if (stop_.load(std::memory_order_acquire)) return false;
	return BuildAndEnqueue(address, typetag, args, arg_len);
}

// ---------------------------------------------------------------------------
// HandleRequest -- IPC server thread
// ---------------------------------------------------------------------------

bool OscRouter::HandleRequest(const protocol::Request& req, protocol::Response& resp)
{
	switch (req.type) {
		case protocol::RequestOscRouteSubscribe:
			return HandleSubscribe(req.oscRouteSubscribe, resp);
		case protocol::RequestOscRouteUnsubscribe:
			return HandleUnsubscribe(req.oscRouteUnsubscribe, resp);
		case protocol::RequestOscPublish:
			return HandlePublish(req.oscPublish, resp);
		case protocol::RequestOscGetStats:
			return HandleGetStats(resp);
		case protocol::RequestSetOscRouterConfig:
			return HandleSetConfig(req.setOscRouterConfig, resp);
		default:
			return false;
	}
}

bool OscRouter::HandleSubscribe(const protocol::OscRouteSubscribe& req, protocol::Response& resp)
{
	char label[32] = {};
	snprintf(label, sizeof(label), "sub-%u", (unsigned)req.subscriber_id);
	bool ok = routeTable_.Subscribe(req.subscriber_id, req.pattern, label);
	char pattern[sizeof(req.pattern) + 1] = {};
	memcpy(pattern, req.pattern, sizeof(req.pattern));
	OR_LOG("[OscRouter] subscribe id=%u pattern='%s' ok=%d active_routes=%u", (unsigned)req.subscriber_id, pattern,
	       ok ? 1 : 0, (unsigned)routeTable_.ActiveCount());
	resp.type = ok ? protocol::ResponseSuccess : protocol::ResponseInvalid;
	return true;
}

bool OscRouter::HandleUnsubscribe(const protocol::OscRouteUnsubscribe& req, protocol::Response& resp)
{
	routeTable_.Unsubscribe(req.subscriber_id);
	OR_LOG("[OscRouter] unsubscribe id=%u active_routes=%u", (unsigned)req.subscriber_id,
	       (unsigned)routeTable_.ActiveCount());
	resp.type = protocol::ResponseSuccess;
	return true;
}

bool OscRouter::HandlePublish(const protocol::OscPublish& req, protocol::Response& resp)
{
	char address[sizeof(req.address) + 1];
	memcpy(address, req.address, sizeof(req.address));
	address[sizeof(req.address)] = '\0';

	char typetag[sizeof(req.typetag) + 1];
	memcpy(typetag, req.typetag, sizeof(req.typetag));
	typetag[sizeof(req.typetag)] = '\0';

	bool ok = BuildAndEnqueue(address, typetag, req.arg_bytes, req.arg_len);
	resp.type = ok ? protocol::ResponseSuccess : protocol::ResponseInvalid;
	return true;
}

bool OscRouter::HandleGetStats(protocol::Response& resp)
{
	resp.type = protocol::ResponseOscRouterStats;
	if (!statsShmem_.ReadGlobalStats(resp.oscRouterStats)) {
		resp.oscRouterStats = {};
		resp.oscRouterStats.active_routes = routeTable_.ActiveCount();
	}
	return true;
}

bool OscRouter::HandleSetConfig(const protocol::OscRouterConfig& req, protocol::Response& resp)
{
	// Defer the socket re-open to the send-worker thread -- UdpSender is
	// documented as single-thread-owned, and the worker checks pendingSendPort_
	// once per loop iteration. Reject obvious garbage (0 / >65535 handled by
	// type, but 0 here means "no port" so treat as invalid).
	if (req.send_port == 0) {
		OR_LOG("[OscRouter] HandleSetConfig rejected invalid send_port=0", 0);
		resp.type = protocol::ResponseInvalid;
		return true;
	}
	pendingSendPort_.store(req.send_port, std::memory_order_release);
	OR_LOG("[OscRouter] HandleSetConfig: queued send_port=%u (was %u)", (unsigned)req.send_port, (unsigned)sendPort_);
	resp.type = protocol::ResponseSuccess;
	return true;
}

// ---------------------------------------------------------------------------
// Send worker
// ---------------------------------------------------------------------------

static uint64_t QpcNow()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return static_cast<uint64_t>(li.QuadPart);
}

static bool ShouldLogCounter(uint64_t value)
{
	return value == 1 || (value % 1024) == 0;
}

void OscRouter::SendWorkerMain()
{
	openvr_pair::common::moduleperf::ScopedThreadRegistration perfRegistration(
	    openvr_pair::common::modules::ModuleId::OscRouter, "osc-send");
	OR_LOG("OscRouter send worker started");

	using clock = std::chrono::steady_clock;
	auto nextStats = clock::now() + std::chrono::milliseconds(100);
	SendWorkerDiagnostics diag;
	bool socket_not_open_logged_ = false;

	SendEntry entry;

	while (!stop_.load(std::memory_order_acquire)) {
		// Live send-port re-open. Checked before each send attempt so the
		// first packet after a UI edit lands on the new port. Single-thread
		// owned socket -- this is the only place that closes/reopens it.
		uint16_t newPort = pendingSendPort_.exchange(0, std::memory_order_acq_rel);
		if (newPort != 0 && newPort != sendPort_) {
			OR_LOG("[OscRouter] send worker: re-opening socket %u -> %u", (unsigned)sendPort_, (unsigned)newPort);
			sender_.Close();
			if (sender_.Open(sendHost_.c_str(), newPort)) {
				sendPort_ = newPort;
				socket_not_open_logged_ = false;
				OR_LOG("[OscRouter] send worker: re-opened socket on port %u", (unsigned)newPort);
			}
			else {
				OR_LOG("[OscRouter] send worker: re-open to port %u failed; old socket closed", (unsigned)newPort);
			}
		}

		{
			std::unique_lock<std::mutex> lk(queueMutex_);
			queueCv_.wait_for(lk, std::chrono::milliseconds(50),
			                  [this] { return qCount_ > 0 || stop_.load(std::memory_order_relaxed); });
			if (qCount_ == 0) {
				// Timeout or spurious wake -- publish stats and loop.
				goto maybe_stats;
			}
			entry = queue_[qHead_];
			qHead_ = (qHead_ + 1) % kSendQueueCapacity;
			--qCount_;
		}

		if (sender_.IsOpen() && entry.size > 0) {
			int sent = sender_.Send(entry.data, entry.size);
			if (sent > 0) {
				uint64_t prev = packetsSent_.fetch_add(1, std::memory_order_relaxed);
				uint64_t total = prev + 1;
				bytesSent_.fetch_add(static_cast<uint64_t>(sent), std::memory_order_relaxed);
				if (shmemReady_) statsShmem_.AddSent(static_cast<uint64_t>(sent));

				if (prev == 0) {
					OscMessage firstMsg = OscParseMessage(entry.data, entry.size);
					OR_LOG("[OscRouter] first packet emitted: addr='%s' bytes=%d target=%s:%u",
					       firstMsg.valid ? firstMsg.address : "(unknown)", sent, sendHost_.c_str(),
					       (unsigned)sendPort_);
				}
				if (total % 100000 == 0) {
					OR_LOG("[OscRouter] sent %llu packets target=%s:%u", (unsigned long long)total, sendHost_.c_str(),
					       (unsigned)sendPort_);
				}

				// Dispatch for route stats.
				OscMessage msg = OscParseMessage(entry.data, entry.size);
				if (msg.valid) {
					bool matched = false;
					routeTable_.Dispatch(msg.address, QpcNow(), matched);
					if (!matched) {
						unmatchedRoutes_.fetch_add(1, std::memory_order_relaxed);
					}
				}
			}
			else {
				const uint64_t failures = sendFailures_.fetch_add(1, std::memory_order_relaxed) + 1;
				if (ShouldLogCounter(failures)) {
					OR_LOG("[OscRouter] send worker: UDP send failed total=%llu target=%s:%u",
					       static_cast<unsigned long long>(failures), sendHost_.c_str(), (unsigned)sendPort_);
				}
			}
		}
		else if (!sender_.IsOpen() && !socket_not_open_logged_) {
			socketClosedDrops_.fetch_add(1, std::memory_order_relaxed);
			OR_LOG("[OscRouter] send worker: socket not open, packets will be dropped");
			socket_not_open_logged_ = true;
		}
		else if (!sender_.IsOpen()) {
			socketClosedDrops_.fetch_add(1, std::memory_order_relaxed);
		}

	maybe_stats:
		auto now = clock::now();
		if (now >= nextStats) {
			nextStats = now + std::chrono::milliseconds(100);
			if (shmemReady_) routeTable_.PublishToShmem(statsShmem_);
		}
		if (diag.IsDue(now)) {
			SendWorkerDiagSample sample;
			sample.sent = packetsSent_.load(std::memory_order_relaxed);
			sample.bytes = bytesSent_.load(std::memory_order_relaxed);
			sample.dropped = droppedCount_.load(std::memory_order_relaxed);
			sample.queued = packetsQueued_.load(std::memory_order_relaxed);
			sample.oversize = oversizedPackets_.load(std::memory_order_relaxed);
			sample.unmatched = unmatchedRoutes_.load(std::memory_order_relaxed);
			sample.send_fail = sendFailures_.load(std::memory_order_relaxed);
			sample.socket_drop = socketClosedDrops_.load(std::memory_order_relaxed);
			sample.active_routes = routeTable_.ActiveCount();
			{
				std::lock_guard<std::mutex> lk(queueMutex_);
				sample.queue_depth = qCount_;
			}
			diag.MaybeLog(now, sample, sendHost_.c_str(), sendPort_);
		}
	}

	OR_LOG("OscRouter send worker stopped");
}

// ---------------------------------------------------------------------------
// Publish pipe server (\\.\pipe\WKOpenVR-OscRouterPub)
//
// Wire format per connection:
//   [32-byte source-id]
//   repeated frames: [4-byte LE length][length bytes OSC]
//
// Fire-and-forget; no response written. Multiple concurrent clients supported
// via PIPE_UNLIMITED_INSTANCES + overlapped IO.
// ---------------------------------------------------------------------------

void OscRouter::PubPipeWorkerMain()
{
	openvr_pair::common::moduleperf::ScopedThreadRegistration perfRegistration(
	    openvr_pair::common::modules::ModuleId::OscRouter, "osc-pub-pipe");
	OR_LOG("OscRouter pub-pipe worker started");

	// Each client connection is handled synchronously in a blocking read loop.
	// With PIPE_UNLIMITED_INSTANCES the kernel can accept concurrent clients;
	// for v1 we handle one at a time per thread -- sidecars are few and their
	// publish rate is bounded (~120 Hz face tracking). A single thread is
	// simpler and avoids the overlapped-IO complexity for fire-and-forget.

	while (!stop_.load(std::memory_order_acquire)) {
		HANDLE pipe = CreateNamedPipeA(
		    OPENVR_PAIRDRIVER_OSCROUTER_PUB_PIPE_NAME, PIPE_ACCESS_INBOUND | FILE_FLAG_OVERLAPPED,
		    PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT, PIPE_UNLIMITED_INSTANCES, 0, 4096, 1000, nullptr);

		if (pipe == INVALID_HANDLE_VALUE) {
			OR_LOG("OscRouter pub-pipe CreateNamedPipe failed: %u", (unsigned)GetLastError());
			Sleep(500);
			continue;
		}

		// Wait for a client to connect (overlapped with 500 ms timeout so we
		// check stop_ periodically).
		OVERLAPPED ov{};
		ov.hEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
		if (!ov.hEvent) {
			CloseHandle(pipe);
			break;
		}

		ConnectNamedPipe(pipe, &ov);
		DWORD err = GetLastError();
		if (err == ERROR_IO_PENDING) {
			DWORD r = WaitForSingleObject(ov.hEvent, 500);
			if (r != WAIT_OBJECT_0) {
				// Timeout or error -- no client yet; loop.
				CloseHandle(ov.hEvent);
				DisconnectNamedPipe(pipe);
				CloseHandle(pipe);
				continue;
			}
		}
		else if (err != ERROR_PIPE_CONNECTED) {
			if (!stop_) {
				OR_LOG("OscRouter pub-pipe ConnectNamedPipe failed: %u", (unsigned)err);
			}
			CloseHandle(ov.hEvent);
			DisconnectNamedPipe(pipe);
			CloseHandle(pipe);
			if (!stop_) Sleep(50);
			continue;
		}
		CloseHandle(ov.hEvent);

		char source_id[32];
		PubPipeClientDiagnostics client;
		OR_LOG("OscRouter pub-pipe client connected", 0);

		// Read the source identifier once per connection. The packet stream
		// that follows is length-prefixed; reading another source id per frame
		// consumes the next frame's length and corrupts every subsequent packet
		// from sidecars that keep the pipe open.
		DWORD got = 0;
		if (!ReadFile(pipe, source_id, 32, &got, nullptr) || got != 32) {
			client.RecordReadBreak(pubPipeReadBreaks_);
		}
		else {
			client.RememberSource(source_id);
		}

		// Read frames from this client until it disconnects or stop_.
		while (!stop_.load(std::memory_order_acquire) && client.source_logged) {
			// Read 4-byte LE frame length.
			uint8_t lenBuf[4];
			if (!ReadFile(pipe, lenBuf, 4, &got, nullptr) || got != 4) {
				client.RecordReadBreak(pubPipeReadBreaks_);
				break;
			}
			uint32_t frameLen = (uint32_t)lenBuf[0] | ((uint32_t)lenBuf[1] << 8) | ((uint32_t)lenBuf[2] << 16) |
			                    ((uint32_t)lenBuf[3] << 24);

			if (frameLen == 0 || frameLen > kSendQueueEntryMaxSize) {
				client.RecordBadLength(pubPipeInvalidFrames_, frameLen);
				break;
			}

			// Read the OSC frame.
			uint8_t frameBuf[kSendQueueEntryMaxSize];
			DWORD total = 0;
			while (total < frameLen && !stop_) {
				DWORD chunk = 0;
				if (!ReadFile(pipe, frameBuf + total, frameLen - total, &chunk, nullptr)) {
					total = 0;
					break;
				}
				total += chunk;
			}
			if (total != frameLen) {
				client.RecordReadBreak(pubPipeReadBreaks_);
				break;
			}

			OscMessage msg = OscParseMessage(frameBuf, frameLen);
			if (!msg.valid) {
				client.RecordInvalidOsc(pubPipeInvalidFrames_, frameLen);
			}

			// Enqueue the raw OSC packet directly.
			{
				std::lock_guard<std::mutex> lk(queueMutex_);
				bool ok = EnqueueLocked(frameBuf, frameLen);
				if (!ok) {
					const uint64_t drops = droppedCount_.fetch_add(1, std::memory_order_relaxed) + 1;
					if (msg.valid) routeTable_.BumpDropCount(msg.address);
					if (shmemReady_) statsShmem_.AddDropped();
					client.RecordQueueDrop(pubPipeQueueDrops_, drops, qCount_);
				}
				else {
					packetsQueued_.fetch_add(1, std::memory_order_relaxed);
					pubPipeFrames_.fetch_add(1, std::memory_order_relaxed);
					client.RecordQueued(frameLen);
					queueCv_.notify_one();
				}
			}
		}

		client.LogDisconnect();
		DisconnectNamedPipe(pipe);
		CloseHandle(pipe);
	}

	OR_LOG("OscRouter pub-pipe worker stopped");
}

} // namespace oscrouter
