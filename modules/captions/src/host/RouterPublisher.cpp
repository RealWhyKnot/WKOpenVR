#define _CRT_SECURE_NO_DEPRECATE
#include "RouterPublisher.h"
#include "Logging.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

// Source identifier sent to the router as the 32-byte header.
const char RouterPublisher::kSourceId[32] = "translator\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

RouterPublisher::RouterPublisher() = default;

RouterPublisher::~RouterPublisher()
{
	Disconnect();
}

void RouterPublisher::Disconnect()
{
	if (pipe_ != INVALID_HANDLE_VALUE) {
		CloseHandle(pipe_);
		pipe_ = INVALID_HANDLE_VALUE;
	}
	identified_ = false;
}

bool RouterPublisher::Connect()
{
	Disconnect();

	pipe_ = CreateFileA("\\\\.\\pipe\\WKOpenVR-OscRouterPub", GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
	                    FILE_ATTRIBUTE_NORMAL, nullptr);

	if (pipe_ == INVALID_HANDLE_VALUE) {
		return false;
	}

	// Send 32-byte source identifier.
	if (!Write(kSourceId, 32)) {
		Disconnect();
		return false;
	}

	identified_ = true;
	backoff_ms_ = 500;
	TH_LOG("[publisher] connected to OSC router pub pipe");
	return true;
}

bool RouterPublisher::Write(const void* data, size_t size)
{
	const uint8_t* ptr = reinterpret_cast<const uint8_t*>(data);
	size_t remaining = size;
	while (remaining > 0) {
		DWORD written = 0;
		if (!WriteFile(pipe_, ptr, static_cast<DWORD>(remaining), &written, nullptr)) {
			return false;
		}
		ptr += written;
		remaining -= written;
	}
	return true;
}

// ---------------------------------------------------------------------------
// OSC packet encoding
// ---------------------------------------------------------------------------
//
// <address> ,sTT or ,T/,F
//   Address:   OSC address padded to 4-byte boundary
//   TypeTag:   ",sTT" padded to 4-byte boundary
//   Arg0 (s):  text as NUL-terminated string padded to 4-byte boundary
//   Arg1 (T/F): sendImmediate -- OSC True/False type tag (no data bytes)
//   Arg2 (T/F): playNotification -- OSC True/False type tag (no data bytes)
//
// Note: the OSC 1.0 True/False tags ('T'/'F') are encoded in the type-tag
// string itself and carry no additional argument bytes.

static size_t Pad4(size_t n)
{
	return (n + 3) & ~3u;
}

static void WriteOscString(uint8_t*& out, const char* s)
{
	size_t len = strlen(s) + 1; // include NUL
	memcpy(out, s, len);
	out += len;
	size_t pad = Pad4(len) - len;
	memset(out, 0, pad);
	out += pad;
}

size_t RouterPublisher::EncodeChatboxPacket(uint8_t* buf, size_t buf_size, const char* address, const char* text,
                                            bool send_immediate, bool notify)
{
	if (!address || address[0] != '/' || !text) return 0;

	// Build the typetag string with T/F for the bool args.
	char typetag[8];
	snprintf(typetag, sizeof(typetag), ",s%c%c", send_immediate ? 'T' : 'F', notify ? 'T' : 'F');

	// Estimate packet size.
	size_t text_len = strlen(text) + 1;
	size_t addr_bytes = Pad4(strlen(address) + 1);
	size_t tag_bytes = Pad4(strlen(typetag) + 1);
	size_t str_bytes = Pad4(text_len);
	size_t total = addr_bytes + tag_bytes + str_bytes;

	if (total > buf_size) return 0;

	uint8_t* out = buf;
	WriteOscString(out, address);
	WriteOscString(out, typetag);
	WriteOscString(out, text);

	return static_cast<size_t>(out - buf);
}

size_t RouterPublisher::EncodeBoolPacket(uint8_t* buf, size_t buf_size, const char* address, bool value)
{
	if (!address || address[0] != '/') return 0;

	char typetag[4];
	snprintf(typetag, sizeof(typetag), ",%c", value ? 'T' : 'F');

	size_t addr_bytes = Pad4(strlen(address) + 1);
	size_t tag_bytes = Pad4(strlen(typetag) + 1);
	size_t total = addr_bytes + tag_bytes;

	if (total > buf_size) return 0;

	uint8_t* out = buf;
	WriteOscString(out, address);
	WriteOscString(out, typetag);

	return static_cast<size_t>(out - buf);
}

// ---------------------------------------------------------------------------
// Truncate text to at most 144 UTF-8 bytes at a whitespace boundary.
// ---------------------------------------------------------------------------

static std::string TruncateToVrchatLimit(const std::string& text)
{
	static constexpr size_t kLimit = 144;
	if (text.size() <= kLimit) return text;

	// Find last whitespace at or before byte 144.
	size_t cut = kLimit;
	while (cut > 0 && text[cut] != ' ' && text[cut] != '\t' && text[cut] != '\r' && text[cut] != '\n') {
		--cut;
	}
	if (cut == 0) cut = kLimit; // no whitespace found; hard truncate

	std::string result = text.substr(0, cut);
	result += "...";
	return result;
}

bool RouterPublisher::PublishPacket(const uint8_t* packet, size_t packet_size)
{
	if (!packet || packet_size == 0) return false;

	// Try to send; reconnect on failure.
	for (int attempt = 0; attempt < 2; ++attempt) {
		if (pipe_ == INVALID_HANDLE_VALUE || !identified_) {
			if (!Connect()) {
				Sleep(static_cast<DWORD>(backoff_ms_));
				backoff_ms_ = std::min(backoff_ms_ * 2, kMaxBackoffMs);
				return false;
			}
		}

		// 4-byte LE length prefix.
		uint8_t len_buf[4];
		uint32_t len32 = static_cast<uint32_t>(packet_size);
		len_buf[0] = (uint8_t)(len32 & 0xFF);
		len_buf[1] = (uint8_t)((len32 >> 8) & 0xFF);
		len_buf[2] = (uint8_t)((len32 >> 16) & 0xFF);
		len_buf[3] = (uint8_t)((len32 >> 24) & 0xFF);

		if (Write(len_buf, 4) && Write(packet, packet_size)) {
			return true;
		}

		// Pipe broke; disconnect and retry once.
		TH_LOG("[publisher] pipe write failed; reconnecting");
		Disconnect();
	}

	backoff_ms_ = std::min(backoff_ms_ * 2, kMaxBackoffMs);
	return false;
}

bool RouterPublisher::PublishChatbox(const std::string& address, const std::string& text_in, bool send_immediate,
                                     bool notify)
{
	std::string text = TruncateToVrchatLimit(text_in);
	const char* packet_address = (!address.empty() && address.front() == '/') ? address.c_str() : "/chatbox/input";

	uint8_t pkt[1024];
	size_t pkt_size = EncodeChatboxPacket(pkt, sizeof(pkt), packet_address, text.c_str(), send_immediate, notify);
	if (pkt_size == 0) {
		TH_LOG("[publisher] failed to encode chatbox packet (addr='%s')", packet_address);
		return false;
	}

	return PublishPacket(pkt, pkt_size);
}

bool RouterPublisher::PublishTyping(bool active)
{
	uint8_t pkt[128];
	size_t pkt_size = EncodeBoolPacket(pkt, sizeof(pkt), "/chatbox/typing", active);
	if (pkt_size == 0) {
		TH_LOG("[publisher] failed to encode chatbox typing packet");
		return false;
	}

	return PublishPacket(pkt, pkt_size);
}
