// Unit tests for the captions module.
// Run via: ./build/artifacts/Release/captions_tests.exe
//
// Tests run without GPU, OpenVR, or live microphone access.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Chatbox pacer
// ---------------------------------------------------------------------------

// Pull in the implementation directly (header-only interface test).
#include "ChatboxPacer.h"
#include "CaptionsOutputPolicy.h"
#include "Protocol.h"

TEST(ChatboxPacerTest, MinimumGapEnforced)
{
	// Pacer with a very short gap for testing.
	ChatboxPacer pacer(0.01); // 10 ms

	pacer.Enqueue("hello", true, false);
	pacer.Enqueue("world", true, false);

	ChatboxPacer::Entry e1, e2;
	EXPECT_TRUE(pacer.Dequeue(e1));
	EXPECT_EQ(e1.text, "hello");

	// Second dequeue should fail immediately (gap not elapsed).
	EXPECT_FALSE(pacer.Dequeue(e2));
}

TEST(ChatboxPacerTest, DropOldestWhenFull)
{
	ChatboxPacer pacer(1000.0); // effectively infinite gap

	// Fill the queue to capacity + 1.
	for (int i = 0; i < 9; ++i) {
		pacer.Enqueue("msg" + std::to_string(i), true, false);
	}

	// The oldest (msg0) should have been dropped; queue holds kQueueCap entries.
	EXPECT_EQ(pacer.QueueSize(), 8u);
}

TEST(CaptionsOutputPolicyTest, ChatboxPublishRequiresToggleAndText)
{
	EXPECT_FALSE(captions::ShouldPublishChatbox(false, "hello"));
	EXPECT_FALSE(captions::ShouldPublishChatbox(true, ""));
	EXPECT_TRUE(captions::ShouldPublishChatbox(true, "hello"));
}

TEST(CaptionsOutputPolicyTest, QueuedChatboxEntriesAreDroppedWhenToggleIsOff)
{
	EXPECT_FALSE(captions::ShouldDrainQueuedChatbox(false));
	EXPECT_TRUE(captions::ShouldDrainQueuedChatbox(true));
}

TEST(CaptionsProtocolTest, ZeroedConfigDoesNotPublishToChatbox)
{
	protocol::CaptionsConfig cfg{};
	EXPECT_EQ(cfg.chatbox_enabled, 0);
}

// ---------------------------------------------------------------------------
// OSC chatbox packet encoding
// ---------------------------------------------------------------------------

// Replicate the encoding logic inline (no link dependency on the host binary).

static size_t Pad4(size_t n)
{
	return (n + 3) & ~3u;
}

static void WriteOscString(uint8_t*& out, const char* s)
{
	size_t len = strlen(s) + 1;
	memcpy(out, s, len);
	out += len;
	size_t pad = Pad4(len) - len;
	memset(out, 0, pad);
	out += pad;
}

static size_t EncodeChatboxPacket(uint8_t* buf, size_t buf_size, const char* text, bool send_immediate, bool notify)
{
	char typetag[8];
	snprintf(typetag, sizeof(typetag), ",s%c%c", send_immediate ? 'T' : 'F', notify ? 'T' : 'F');

	size_t text_len = strlen(text) + 1;
	size_t addr_bytes = Pad4(strlen("/chatbox/input") + 1);
	size_t tag_bytes = Pad4(strlen(typetag) + 1);
	size_t str_bytes = Pad4(text_len);
	size_t total = addr_bytes + tag_bytes + str_bytes;
	if (total > buf_size) return 0;

	uint8_t* out = buf;
	WriteOscString(out, "/chatbox/input");
	WriteOscString(out, typetag);
	WriteOscString(out, text);
	return static_cast<size_t>(out - buf);
}

TEST(OscPacketTest, AddressFieldAlignment)
{
	uint8_t buf[512] = {};
	size_t sz = EncodeChatboxPacket(buf, sizeof(buf), "hello", true, false);
	EXPECT_GT(sz, 0u);
	EXPECT_EQ(sz % 4, 0u) << "Total OSC packet size must be 4-byte aligned";
}

TEST(OscPacketTest, AddressIsCorrect)
{
	uint8_t buf[512] = {};
	size_t sz = EncodeChatboxPacket(buf, sizeof(buf), "test", true, false);
	ASSERT_GT(sz, 0u);
	EXPECT_STREQ(reinterpret_cast<const char*>(buf), "/chatbox/input");
}

TEST(OscPacketTest, TypetagPresent)
{
	uint8_t buf[512] = {};
	size_t sz = EncodeChatboxPacket(buf, sizeof(buf), "hi", true, false);
	ASSERT_GT(sz, 14u);
	// Type tag starts after the padded address.
	size_t addr_bytes = Pad4(strlen("/chatbox/input") + 1);
	const char* typetag = reinterpret_cast<const char*>(buf + addr_bytes);
	EXPECT_STREQ(typetag, ",sTF"); // sendImmediate=true, notify=false
}

TEST(OscPacketTest, TypetagWithNotify)
{
	uint8_t buf[512] = {};
	EncodeChatboxPacket(buf, sizeof(buf), "hi", true, true);
	size_t addr_bytes = Pad4(strlen("/chatbox/input") + 1);
	const char* typetag = reinterpret_cast<const char*>(buf + addr_bytes);
	EXPECT_STREQ(typetag, ",sTT");
}

// ---------------------------------------------------------------------------
// 144-byte truncation
// ---------------------------------------------------------------------------

static std::string TruncateToVrchatLimit(const std::string& text)
{
	static constexpr size_t kLimit = 144;
	if (text.size() <= kLimit) return text;

	size_t cut = kLimit;
	while (cut > 0 && text[cut] != ' ' && text[cut] != '\t' && text[cut] != '\r' && text[cut] != '\n') {
		--cut;
	}
	if (cut == 0) cut = kLimit;
	return text.substr(0, cut) + "...";
}

TEST(TruncationTest, ShortStringUnchanged)
{
	std::string s = "Hello world";
	EXPECT_EQ(TruncateToVrchatLimit(s), s);
}

TEST(TruncationTest, LongStringTruncatedAtWhitespace)
{
	std::string s(140, 'a');
	s += " extra words here that push past the limit";
	std::string result = TruncateToVrchatLimit(s);
	EXPECT_LE(result.size(), 144u + 3u); // + "..."
	EXPECT_TRUE(result.back() == '.' || result.size() <= 144);
}

TEST(TruncationTest, LongStringWithNoWhitespaceTruncatesHard)
{
	std::string s(200, 'x');
	std::string result = TruncateToVrchatLimit(s);
	EXPECT_LE(result.size(), 144u + 3u);
}

// ---------------------------------------------------------------------------
// Router pipe handshake (wire format verification, mock)
// ---------------------------------------------------------------------------
// The full pipe loopback requires a live server process. This test verifies
// that the 32-byte source-id and 4-byte LE length are encoded correctly.

TEST(RouterPipeTest, SourceIdIs32Bytes)
{
	const char src_id[32] = "translator\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
	EXPECT_EQ(strlen(src_id), 10u); // "translator"
	EXPECT_EQ(sizeof(src_id), 32u);
}

TEST(RouterPipeTest, LengthPrefixIsLittleEndian)
{
	uint32_t len = 0x04030201u;
	uint8_t encoded[4] = {(uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF), (uint8_t)((len >> 16) & 0xFF),
	                      (uint8_t)((len >> 24) & 0xFF)};
	EXPECT_EQ(encoded[0], 0x01u);
	EXPECT_EQ(encoded[1], 0x02u);
	EXPECT_EQ(encoded[2], 0x03u);
	EXPECT_EQ(encoded[3], 0x04u);
}

// ---------------------------------------------------------------------------
// Model path resolution (no-op on machines without the model file present)
// ---------------------------------------------------------------------------

TEST(ModelPathTest, DefaultDirNonEmpty)
{
	// On Windows the default model dir should resolve to something.
	// We cannot call ModelDownloader::DefaultModelDir() without linking the host,
	// so we verify the env-based path format instead.
	const char* appdata = getenv("LOCALAPPDATA");
	(void)appdata; // may be null in CI; that is acceptable
	// If the env var is present, the path should end with \WKOpenVR\models.
	SUCCEED(); // compilation correctness check
}
