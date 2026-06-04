#define OSCROUTER_TESTS 1

#include <gtest/gtest.h>

// Pull in the implementation files directly so we don't need the full
// driver link chain (UdpSender, logging, shmem).
#include "OscWire.h"
#include "OscWire.cpp"
#include "RouteTable.h"
#include "RouteTable.cpp"

#include <algorithm>
#include <cstring>
#include <vector>

using namespace oscrouter;

// ---------------------------------------------------------------------------
// OSC packet roundtrip
// ---------------------------------------------------------------------------

TEST(OscWire, FloatRoundtrip)
{
	OscPacket<256> pkt;
	pkt.Begin("/avatar/parameters/JawOpen", ",f");
	pkt.WriteFloat(0.75f);
	ASSERT_TRUE(pkt.Ok());
	ASSERT_GT(pkt.Size(), 0u);

	OscMessage msg = OscParseMessage(pkt.Data(), pkt.Size());
	ASSERT_TRUE(msg.valid);
	EXPECT_STREQ(msg.address, "/avatar/parameters/JawOpen");
	EXPECT_STREQ(msg.typetag, ",f");
	ASSERT_EQ(msg.arg_size, 4u);

	OscReader r(msg.arg_data, msg.arg_size);
	float v = r.ReadFloat();
	EXPECT_TRUE(r.IsValid());
	EXPECT_NEAR(v, 0.75f, 1e-6f);
}

TEST(OscWire, Int32Roundtrip)
{
	OscPacket<256> pkt;
	pkt.Begin("/test/int", ",i");
	pkt.WriteInt32(-42);
	ASSERT_TRUE(pkt.Ok());

	OscMessage msg = OscParseMessage(pkt.Data(), pkt.Size());
	ASSERT_TRUE(msg.valid);
	EXPECT_STREQ(msg.typetag, ",i");

	OscReader r(msg.arg_data, msg.arg_size);
	EXPECT_EQ(r.ReadInt32(), -42);
}

TEST(OscWire, BundleDispatch)
{
	// Build a bundle containing two float messages.
	uint8_t bundle[512];
	size_t pos = 0;

	// Write "#bundle\0" + 8-byte timetag (zeroed).
	memcpy(bundle + pos, "#bundle\0", 8);
	pos += 8;
	memset(bundle + pos, 0, 8);
	pos += 8;

	// Helper: write a sub-message and prefix it with its 4-byte big-endian length.
	auto AppendSubMsg = [&](const char* addr, float val) {
		OscPacket<256> pkt;
		pkt.Begin(addr, ",f");
		pkt.WriteFloat(val);
		uint32_t sz = static_cast<uint32_t>(pkt.Size());
		bundle[pos++] = (uint8_t)(sz >> 24);
		bundle[pos++] = (uint8_t)(sz >> 16);
		bundle[pos++] = (uint8_t)(sz >> 8);
		bundle[pos++] = (uint8_t)(sz);
		memcpy(bundle + pos, pkt.Data(), pkt.Size());
		pos += pkt.Size();
	};
	AppendSubMsg("/a", 1.0f);
	AppendSubMsg("/b", 2.0f);

	int dispatchCount = 0;
	float vals[2] = {0.0f, 0.0f};
	OscDispatch(bundle, pos, [&](const char* addr, const char* /*tag*/, const uint8_t* args, size_t arg_size) {
		OscReader r(args, arg_size);
		float v = r.ReadFloat();
		if (dispatchCount < 2) vals[dispatchCount] = v;
		++dispatchCount;
		(void)addr;
	});

	EXPECT_EQ(dispatchCount, 2);
	EXPECT_NEAR(vals[0], 1.0f, 1e-6f);
	EXPECT_NEAR(vals[1], 2.0f, 1e-6f);
}

// ---------------------------------------------------------------------------
// Pattern matching
// ---------------------------------------------------------------------------

TEST(OscPattern, Exact)
{
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/JawOpen", "/avatar/parameters/JawOpen"));
	EXPECT_FALSE(OscPatternMatch("/avatar/parameters/JawOpen", "/avatar/parameters/JawClose"));
}

TEST(OscPattern, QuestionMark)
{
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/JawOpe?", "/avatar/parameters/JawOpen"));
	EXPECT_FALSE(OscPatternMatch("/avatar/parameters/JawOpe?", "/avatar/parameters/JawOpenWide"));
}

TEST(OscPattern, Star)
{
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/*", "/avatar/parameters/JawOpen"));
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/*", "/avatar/parameters/EyesDilation"));
	EXPECT_FALSE(OscPatternMatch("/avatar/parameters/Jaw*", "/avatar/parameters/EyesDilation"));
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/Jaw*", "/avatar/parameters/JawOpen"));
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/Jaw*", "/avatar/parameters/JawForward"));
}

TEST(OscPattern, CharClass)
{
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/[LR]eftEyeLid", "/avatar/parameters/LeftEyeLid"));
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/[LR]eftEyeLid", "/avatar/parameters/ReftEyeLid"));
	EXPECT_FALSE(OscPatternMatch("/avatar/parameters/[LR]eftEyeLid", "/avatar/parameters/XeftEyeLid"));
}

TEST(OscPattern, CharRange)
{
	EXPECT_TRUE(OscPatternMatch("/test/[a-z]", "/test/b"));
	EXPECT_FALSE(OscPatternMatch("/test/[a-z]", "/test/B"));
}

TEST(OscPattern, Alternates)
{
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/{LeftEyeLid,RightEyeLid}", "/avatar/parameters/LeftEyeLid"));
	EXPECT_TRUE(OscPatternMatch("/avatar/parameters/{LeftEyeLid,RightEyeLid}", "/avatar/parameters/RightEyeLid"));
	EXPECT_FALSE(OscPatternMatch("/avatar/parameters/{LeftEyeLid,RightEyeLid}", "/avatar/parameters/EyesDilation"));
}

// ---------------------------------------------------------------------------
// RouteTable
// ---------------------------------------------------------------------------

TEST(RouteTable, SubscribeAndDispatch)
{
	RouteTable table;
	EXPECT_TRUE(table.Subscribe(1, "/avatar/parameters/*", "test1"));
	EXPECT_TRUE(table.Subscribe(2, "/avatar/parameters/JawOpen", "test2"));
	EXPECT_TRUE(table.Subscribe(3, "/chatbox/*", "test3"));

	bool matched = false;
	table.Dispatch("/avatar/parameters/JawOpen", 12345ULL, matched);
	EXPECT_TRUE(matched);

	// Non-matching route (id=3) should not have been matched.
	matched = false;
	table.Dispatch("/avatar/parameters/JawOpen", 12345ULL, matched);
	EXPECT_TRUE(matched);

	// Route not matched by anything.
	matched = false;
	table.Dispatch("/unrelated/path", 12345ULL, matched);
	EXPECT_FALSE(matched);
}

TEST(RouteTable, Unsubscribe)
{
	RouteTable table;
	EXPECT_TRUE(table.Subscribe(99, "/test/*", "sub99"));
	EXPECT_EQ(table.ActiveCount(), 1u);
	table.Unsubscribe(99);
	EXPECT_EQ(table.ActiveCount(), 0u);

	bool matched = false;
	table.Dispatch("/test/foo", 0ULL, matched);
	EXPECT_FALSE(matched);
}

TEST(RouteTable, TableFull)
{
	RouteTable table;
	// Fill to capacity.
	for (uint32_t i = 0; i < protocol::OSC_ROUTER_ROUTE_SLOTS; ++i) {
		char pat[32];
		snprintf(pat, sizeof(pat), "/slot/%u", i);
		EXPECT_TRUE(table.Subscribe(i, pat, "label"));
	}
	// Next insert should fail.
	EXPECT_FALSE(table.Subscribe(999, "/overflow", "overflow"));
	EXPECT_EQ(table.ActiveCount(), protocol::OSC_ROUTER_ROUTE_SLOTS);
}

TEST(RouteTable, DropCounter)
{
	RouteTable table;
	EXPECT_TRUE(table.Subscribe(7, "/test/*", "sub7"));

	// Bump drop count.
	table.BumpDropCount("/test/foo");

	// Verify via shmem snapshot.
	protocol::OscRouterStatsShmem shmem;
	bool created = shmem.Create("OscRouterTestShmem_DropCounter");
	ASSERT_TRUE(created);

	table.PublishToShmem(shmem);

	// Find the active slot and check drop_count.
	bool found = false;
	for (uint32_t i = 0; i < protocol::OSC_ROUTER_ROUTE_SLOTS; ++i) {
		protocol::OscRouterRouteSlot slot;
		if (!shmem.TryReadRoute(i, slot)) continue;
		if (!slot.active) continue;
		if (strcmp(slot.address_pattern, "/test/*") == 0) {
			EXPECT_EQ(slot.drop_count.load(std::memory_order_relaxed), 1u);
			found = true;
		}
	}
	EXPECT_TRUE(found);
}

// ---------------------------------------------------------------------------
// Stats shmem write/read roundtrip
// ---------------------------------------------------------------------------

TEST(OscRouterStatsShmem, WriteReadRoundtrip)
{
	protocol::OscRouterStatsShmem shmem;
	bool created = shmem.Create("OscRouterTestShmem_Roundtrip");
	ASSERT_TRUE(created);

	shmem.AddSent(128);
	shmem.AddSent(64);
	shmem.AddDropped();

	protocol::OscRouterStats stats;
	EXPECT_TRUE(shmem.ReadGlobalStats(stats));
	EXPECT_EQ(stats.packets_sent, 2u);
	EXPECT_EQ(stats.bytes_sent, 192u);
	EXPECT_EQ(stats.packets_dropped, 1u);
}

// ---------------------------------------------------------------------------
// Publish pipe wire format
// ---------------------------------------------------------------------------

TEST(OscWire, PubPipeConnectionSourceThenMultipleFrames)
{
	// Verify the wire format: one 32-byte source-id per connection, followed
	// by any number of 4-byte LE length + raw OSC frames.
	OscPacket<256> pkt1;
	pkt1.Begin("/chatbox/input", ",s");
	pkt1.WriteStr("Hello");
	ASSERT_TRUE(pkt1.Ok());

	OscPacket<256> pkt2;
	pkt2.Begin("/chatbox/input", ",s");
	pkt2.WriteStr("Again");
	ASSERT_TRUE(pkt2.Ok());

	std::vector<uint8_t> wire;
	wire.resize(32);

	// Write 32-byte source-id (NUL-padded).
	const char* srcId = "translator";
	memcpy(wire.data(), srcId, std::min<size_t>(strlen(srcId), 31));

	auto appendFrame = [&](const OscPacket<256>& pkt) {
		uint32_t len = static_cast<uint32_t>(pkt.Size());
		wire.push_back((uint8_t)(len));
		wire.push_back((uint8_t)(len >> 8));
		wire.push_back((uint8_t)(len >> 16));
		wire.push_back((uint8_t)(len >> 24));
		wire.insert(wire.end(), pkt.Data(), pkt.Data() + pkt.Size());
	};
	appendFrame(pkt1);
	appendFrame(pkt2);

	// Parse back: read source-id.
	char parsedSrc[33] = {};
	memcpy(parsedSrc, wire.data(), 32);
	EXPECT_STREQ(parsedSrc, "translator");

	size_t offset = 32;
	for (int i = 0; i < 2; ++i) {
		ASSERT_LE(offset + 4, wire.size());
		uint32_t parsedLen = (uint32_t)wire[offset] | ((uint32_t)wire[offset + 1] << 8) |
		                     ((uint32_t)wire[offset + 2] << 16) | ((uint32_t)wire[offset + 3] << 24);
		offset += 4;
		ASSERT_LE(offset + parsedLen, wire.size());

		OscMessage msg = OscParseMessage(wire.data() + offset, parsedLen);
		ASSERT_TRUE(msg.valid);
		EXPECT_STREQ(msg.address, "/chatbox/input");
		EXPECT_STREQ(msg.typetag, ",s");
		offset += parsedLen;
	}
	EXPECT_EQ(offset, wire.size());
}
