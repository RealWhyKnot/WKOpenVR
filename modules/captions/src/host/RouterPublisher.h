#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstddef>
#include <cstdint>
#include <string>

// Connects to the OSC router's out-of-process publish pipe
// (\\.\pipe\WKOpenVR-OscRouterPub) and sends chatbox OSC packets.
//
// Wire protocol (per the router's PubPipeWorkerMain):
//   1. Connect.
//   2. Write 32-byte source identifier (NUL-padded ASCII).
//   3. Per packet: 4-byte LE uint32 length + `length` bytes of raw OSC.
//
// Reconnects automatically on broken-pipe with bounded exponential backoff.
class RouterPublisher
{
public:
	RouterPublisher();
	~RouterPublisher();

	// Send an OSC chatbox ,sTT packet. Connects (or reconnects) on demand.
	// text must be <= 144 UTF-8 bytes; longer strings are truncated at the
	// last whitespace boundary before byte 144.
	// address defaults to VRChat's /chatbox/input; custom endpoints must be
	// absolute OSC addresses.
	// send_immediate: chatbox sendImmediate flag (true = PTT / final).
	// notify: chatbox playNotification flag.
	bool PublishChatbox(const std::string& address, const std::string& text, bool send_immediate, bool notify);

	// Toggle VRChat's chatbox typing indicator.
	bool PublishTyping(bool active);

	// Disconnect and release the pipe handle.
	void Disconnect();

private:
	HANDLE pipe_ = INVALID_HANDLE_VALUE;
	bool identified_ = false; // true after source-id has been written
	int backoff_ms_ = 500;

	static constexpr int kMaxBackoffMs = 15000;
	static const char kSourceId[32];

	bool Connect();
	bool Write(const void* data, size_t size);
	bool PublishPacket(const uint8_t* packet, size_t packet_size);

	// Encode a chatbox ,sTT OSC packet into buf.
	// Returns the number of bytes written, or 0 on error.
	static size_t EncodeChatboxPacket(uint8_t* buf, size_t buf_size, const char* address, const char* text,
	                                  bool send_immediate, bool notify);

	// Encode a single-boolean OSC packet into buf using T/F typetags.
	static size_t EncodeBoolPacket(uint8_t* buf, size_t buf_size, const char* address, bool value);
};
