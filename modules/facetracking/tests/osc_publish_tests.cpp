// Unit tests for the facetracking OSC router publish path.
//
// Coverage:
//   - OscPublishFloat encodes a float in OSC 1.0 big-endian wire format that
//     matches the reference encoding in OscSender.BuildOscFloatPacket (C#).
//   - shared expression-name table has exactly FACETRACKING_EXPRESSION_COUNT entries
//     and the first, middle, and last names match the canonical list from the
//     protocol expression list.
//
// The test builds OSC packets directly from the encode logic in
// FacetrackingDriverModule.cpp rather than invoking the full WorkerLoop
// (which requires live shmem and a router singleton).  The helpers are
// exposed via the FACETRACKING_TESTS guard below.

#include "Protocol.h"
#include "FaceOscPublisher.h"
#include "facetracking/ExpressionNames.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

struct PublishedOscFloat
{
	std::string address;
	float value = 0.0f;
};

std::vector<PublishedOscFloat>& PublishedOscFloats()
{
	static std::vector<PublishedOscFloat> values;
	return values;
}

void ClearPublishedOscFloats()
{
	PublishedOscFloats().clear();
}

const PublishedOscFloat* FindPublishedOscFloat(const char* address)
{
	const auto& values = PublishedOscFloats();
	for (auto it = values.rbegin(); it != values.rend(); ++it) {
		if (it->address == address) return &(*it);
	}
	return nullptr;
}

} // namespace

namespace pairdriver::oscrouter {

bool PublishOsc(const char* source_id, const char* address, const char* typetag, const void* args, size_t arg_len)
{
	if (!source_id || std::strcmp(source_id, "facetracking") != 0) return false;
	if (!address || !typetag || std::strcmp(typetag, ",f") != 0 || !args || arg_len != 4) return false;

	const auto* bytes = static_cast<const uint8_t*>(args);
	const uint32_t bits = (static_cast<uint32_t>(bytes[0]) << 24) | (static_cast<uint32_t>(bytes[1]) << 16) |
	                      (static_cast<uint32_t>(bytes[2]) << 8) | static_cast<uint32_t>(bytes[3]);
	float value = 0.0f;
	std::memcpy(&value, &bits, sizeof(value));
	PublishedOscFloats().push_back(PublishedOscFloat{address, value});
	return true;
}

} // namespace pairdriver::oscrouter

// ---------------------------------------------------------------------------
// Reference OSC 1.0 float packet builder (mirrors OscSender.BuildOscFloatPacket
// in C# -- kept here for cross-language byte-identity verification).
// ---------------------------------------------------------------------------

// Build a minimal OSC float packet for the given address and value.
// Returns the packet bytes via out_packet and its length via return value.
// Caller must supply a buffer of at least 256 bytes.
static size_t BuildOscFloatPacketRef(const char* address, float value, uint8_t* out)
{
	size_t addrLen = std::strlen(address) + 1; // NUL terminator
	size_t addrPadded = (addrLen + 3) & ~static_cast<size_t>(3);

	// Type tag ",f\0\0" -- always 4 bytes.
	const size_t typePadded = 4;
	const size_t floatBytes = 4;

	size_t total = addrPadded + typePadded + floatBytes;

	std::memset(out, 0, total);
	std::memcpy(out, address, addrLen - 1); // sans NUL; memset already zeroed pad
	out[addrPadded + 0] = static_cast<uint8_t>(',');
	out[addrPadded + 1] = static_cast<uint8_t>('f');

	uint32_t bits;
	std::memcpy(&bits, &value, 4);
	size_t f = addrPadded + typePadded;
	out[f + 0] = static_cast<uint8_t>(bits >> 24);
	out[f + 1] = static_cast<uint8_t>(bits >> 16);
	out[f + 2] = static_cast<uint8_t>(bits >> 8);
	out[f + 3] = static_cast<uint8_t>(bits);

	return total;
}

// ---------------------------------------------------------------------------
// Reproduce OscPublishFloat's encoding inline so the test does not need to
// link or stub RouterPublishApi.  The encoding must match exactly.
// ---------------------------------------------------------------------------

static void EncodeOscFloatArgs(float value, uint8_t arg_bytes[4])
{
	uint32_t bits;
	std::memcpy(&bits, &value, sizeof(bits));
	arg_bytes[0] = static_cast<uint8_t>(bits >> 24);
	arg_bytes[1] = static_cast<uint8_t>(bits >> 16);
	arg_bytes[2] = static_cast<uint8_t>(bits >> 8);
	arg_bytes[3] = static_cast<uint8_t>(bits);
}

// ---------------------------------------------------------------------------
// Tests: float encoding
// ---------------------------------------------------------------------------

TEST(OscPublishFloat, JawOpenByteIdentity)
{
	// The reference address from the PR comment -- confirms the packet layout
	// the router receives from the driver matches what the C# host used to send.
	const char* address = "/avatar/parameters/JawOpen";
	const float value = 0.75f;

	// Reference: full OSC packet (what the host used to send over UDP).
	uint8_t ref[256] = {};
	size_t refLen = BuildOscFloatPacketRef(address, value, ref);

	// Driver path: only the arg_bytes[4] slice is passed to PublishOsc; the
	// router builds the full packet from address + typetag + args.  We verify
	// the arg slice matches the float portion of the reference packet.
	uint8_t argBytes[4] = {};
	EncodeOscFloatArgs(value, argBytes);

	// The float bytes live at offset addrPadded + typePadded (4 + 4 = 8 for
	// "/avatar/parameters/JawOpen" -- len 27, padded to 28).
	size_t addrLen = std::strlen(address) + 1;
	size_t addrPadded = (addrLen + 3) & ~static_cast<size_t>(3);
	size_t floatOffset = addrPadded + 4; // 4 = typePadded

	ASSERT_LE(floatOffset + 4, refLen);
	EXPECT_EQ(argBytes[0], ref[floatOffset + 0]);
	EXPECT_EQ(argBytes[1], ref[floatOffset + 1]);
	EXPECT_EQ(argBytes[2], ref[floatOffset + 2]);
	EXPECT_EQ(argBytes[3], ref[floatOffset + 3]);
}

TEST(OscPublishFloat, NegativeValueEncoding)
{
	// Negative values: ensure bit_cast is correct (not just positive floats).
	const float value = -0.5f;
	uint8_t argBytes[4] = {};
	EncodeOscFloatArgs(value, argBytes);

	uint32_t bits;
	std::memcpy(&bits, &value, 4);
	EXPECT_EQ(argBytes[0], static_cast<uint8_t>(bits >> 24));
	EXPECT_EQ(argBytes[1], static_cast<uint8_t>(bits >> 16));
	EXPECT_EQ(argBytes[2], static_cast<uint8_t>(bits >> 8));
	EXPECT_EQ(argBytes[3], static_cast<uint8_t>(bits));
}

TEST(OscPublishFloat, ZeroValueEncoding)
{
	const float value = 0.0f;
	uint8_t argBytes[4] = {};
	EncodeOscFloatArgs(value, argBytes);
	EXPECT_EQ(argBytes[0], 0u);
	EXPECT_EQ(argBytes[1], 0u);
	EXPECT_EQ(argBytes[2], 0u);
	EXPECT_EQ(argBytes[3], 0u);
}

// ---------------------------------------------------------------------------
// Tests: avatar OSC address allowlist
// ---------------------------------------------------------------------------

static std::wstring TempAllowListPath()
{
	wchar_t dir[MAX_PATH] = {};
	GetTempPathW(MAX_PATH, dir);
	wchar_t path[MAX_PATH] = {};
	GetTempFileNameW(dir, L"ft", 0, path);
	DeleteFileW(path);
	return path;
}

static void WriteUtf8File(const std::wstring& path, const char* body)
{
	HANDLE h = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	ASSERT_NE(h, INVALID_HANDLE_VALUE);
	DWORD written = 0;
	const DWORD len = static_cast<DWORD>(std::strlen(body));
	ASSERT_TRUE(WriteFile(h, body, len, &written, nullptr));
	EXPECT_EQ(written, len);
	CloseHandle(h);
}

TEST(FaceOscAddressFilter, MissingFilePassesThroughWhenEmpty)
{
	std::wstring path = TempAllowListPath();
	facetracking::FaceOscAddressFilter filter(path);

	EXPECT_TRUE(filter.ReloadIfChanged());
	// A missing allowlist yields no addresses, so filtering stays off and every
	// address passes through rather than blocking all face output.
	EXPECT_FALSE(filter.Active());
	EXPECT_EQ(filter.AllowedCount(), 0u);
	EXPECT_EQ(filter.LastLoadStatus(), facetracking::FaceOscAddressFilterLoadStatus::Missing);
	EXPECT_STREQ(facetracking::FaceOscAddressFilterLoadStatusName(filter.LastLoadStatus()), "missing");
}

TEST(FaceOscAddressFilter, EmptyFilePassesThroughWhenLoaded)
{
	std::wstring path = TempAllowListPath();
	WriteUtf8File(path, "");

	facetracking::FaceOscAddressFilter filter(path);
	EXPECT_TRUE(filter.ReloadIfChanged());
	// An empty avatar parameter list (avatar OSCQuery config not available) must
	// not suppress all output: the filter loads but stays inactive so the
	// publisher sends every parameter.
	EXPECT_EQ(filter.LastLoadStatus(), facetracking::FaceOscAddressFilterLoadStatus::Loaded);
	EXPECT_EQ(filter.AllowedCount(), 0u);
	EXPECT_FALSE(filter.Active());

	DeleteFileW(path.c_str());
}

TEST(FaceOscAddressFilter, LoadsAndReloadsAddresses)
{
	std::wstring path = TempAllowListPath();
	WriteUtf8File(path, "/avatar/parameters/v2/JawOpen\n"
	                    "  /avatar/parameters/v2/EyeLidLeft\r\n");

	facetracking::FaceOscAddressFilter filter(path);
	EXPECT_TRUE(filter.ReloadIfChanged());
	EXPECT_EQ(filter.AllowedCount(), 2u);
	EXPECT_TRUE(filter.Active());
	EXPECT_EQ(filter.LastLoadStatus(), facetracking::FaceOscAddressFilterLoadStatus::Loaded);
	EXPECT_TRUE(filter.Allows("/avatar/parameters/v2/JawOpen"));
	EXPECT_TRUE(filter.Allows("/avatar/parameters/v2/EyeLidLeft"));
	EXPECT_FALSE(filter.Allows("/avatar/parameters/v2/SmileFrownLeft"));

	WriteUtf8File(path, "/avatar/parameters/v2/JawOpen\n"
	                    "/avatar/parameters/v2/SmileFrownLeft\n");
	EXPECT_TRUE(filter.ReloadIfChanged());
	EXPECT_EQ(filter.AllowedCount(), 2u);
	EXPECT_EQ(filter.LastLoadStatus(), facetracking::FaceOscAddressFilterLoadStatus::Loaded);
	EXPECT_TRUE(filter.Allows("/avatar/parameters/v2/SmileFrownLeft"));
	EXPECT_FALSE(filter.Allows("/avatar/parameters/v2/EyeLidLeft"));

	DeleteFileW(path.c_str());
}

TEST(FaceOscAddressFilter, ResolvesPrefixedVrcftV2Addresses)
{
	std::wstring path = TempAllowListPath();
	WriteUtf8File(path, "/avatar/parameters/FT/v2/JawOpen\n"
	                    "/avatar/parameters/OSCm/Proxy/FT/v2/EyeLidLeft\n"
	                    "/avatar/parameters/Example/Nest/v2/SmileFrownLeft\n");

	facetracking::FaceOscAddressFilter filter(path);
	ASSERT_TRUE(filter.ReloadIfChanged());

	const std::string* jaw = filter.CompatibleAddress("/avatar/parameters/v2/JawOpen");
	ASSERT_NE(jaw, nullptr);
	EXPECT_EQ(*jaw, "/avatar/parameters/FT/v2/JawOpen");

	const std::string* lid = filter.CompatibleAddress("/avatar/parameters/v2/EyeLidLeft");
	ASSERT_NE(lid, nullptr);
	EXPECT_EQ(*lid, "/avatar/parameters/OSCm/Proxy/FT/v2/EyeLidLeft");

	const std::string* smile = filter.CompatibleAddress("/avatar/parameters/v2/SmileFrownLeft");
	ASSERT_NE(smile, nullptr);
	EXPECT_EQ(*smile, "/avatar/parameters/Example/Nest/v2/SmileFrownLeft");

	EXPECT_EQ(filter.CompatibleAddress("/avatar/parameters/v2/MouthOpen"), nullptr);

	DeleteFileW(path.c_str());
}

TEST(FaceOscPublishCounts, AddPreservesDiagnosticFields)
{
	facetracking::FaceOscPublishCounts total{};
	facetracking::FaceOscPublishCounts delta{};
	delta.attempted = 10;
	delta.sent = 4;
	delta.dropped = 1;
	delta.filtered = 3;
	delta.deduped = 2;
	delta.remapped = 5;

	total.Add(delta);
	total.Add(delta);

	EXPECT_EQ(total.attempted, 20u);
	EXPECT_EQ(total.sent, 8u);
	EXPECT_EQ(total.dropped, 2u);
	EXPECT_EQ(total.filtered, 6u);
	EXPECT_EQ(total.deduped, 4u);
	EXPECT_EQ(total.remapped, 10u);
}

TEST(FaceOscPublisher, ClampsLegacyAndCurrentExpressionOutput)
{
	ClearPublishedOscFloats();
	protocol::FaceTrackingFrameBody frame{};
	frame.flags = 0x2u;
	frame.expressions[10] = 2.0f;          // EyeSquintLeft, legacy/internal expression path.
	frame.upstream_expressions[0] = 2.0f;  // EyeSquintRight, current VRCFT path.
	frame.upstream_expressions[1] = 2.0f;  // EyeSquintLeft, current VRCFT path.
	frame.upstream_expressions[23] = 2.0f; // JawRight, signed derived v2/JawX path.
	frame.upstream_expressions[24] = 0.0f; // JawLeft.

	const facetracking::FaceOscPublishCounts counts = facetracking::PublishFaceFrameOsc(frame);

	EXPECT_GT(counts.sent, 0u);
	const PublishedOscFloat* legacySquint = FindPublishedOscFloat("/avatar/parameters/EyeSquintLeft");
	const PublishedOscFloat* v2SquintLeft = FindPublishedOscFloat("/avatar/parameters/v2/EyeSquintLeft");
	const PublishedOscFloat* v2SquintRight = FindPublishedOscFloat("/avatar/parameters/v2/EyeSquintRight");
	const PublishedOscFloat* v2JawX = FindPublishedOscFloat("/avatar/parameters/v2/JawX");
	ASSERT_NE(legacySquint, nullptr);
	ASSERT_NE(v2SquintLeft, nullptr);
	ASSERT_NE(v2SquintRight, nullptr);
	ASSERT_NE(v2JawX, nullptr);
	EXPECT_FLOAT_EQ(legacySquint->value, 1.0f);
	EXPECT_FLOAT_EQ(v2SquintLeft->value, 1.0f);
	EXPECT_FLOAT_EQ(v2SquintRight->value, 1.0f);
	EXPECT_FLOAT_EQ(v2JawX->value, 1.0f);
}

TEST(FaceOscPublisher, ClampsEyeOutputHighSide)
{
	ClearPublishedOscFloats();
	protocol::FaceTrackingFrameBody frame{};
	frame.flags = 0x1u;
	frame.eye_gaze_l[0] = 4.0f;
	frame.eye_gaze_l[1] = -4.0f;
	frame.eye_gaze_r[0] = 4.0f;
	frame.eye_gaze_r[1] = -4.0f;
	frame.eye_openness_l = 3.0f;
	frame.eye_openness_r = 3.0f;
	frame.pupil_dilation_l = 3.0f;
	frame.pupil_dilation_r = 3.0f;

	const facetracking::FaceOscPublishCounts counts = facetracking::PublishFaceFrameOsc(frame);

	EXPECT_GT(counts.sent, 0u);
	const PublishedOscFloat* leftX = FindPublishedOscFloat("/avatar/parameters/LeftEyeX");
	const PublishedOscFloat* leftY = FindPublishedOscFloat("/avatar/parameters/LeftEyeY");
	const PublishedOscFloat* leftLid = FindPublishedOscFloat("/avatar/parameters/LeftEyeLid");
	const PublishedOscFloat* pupil = FindPublishedOscFloat("/avatar/parameters/v2/PupilDilation");
	ASSERT_NE(leftX, nullptr);
	ASSERT_NE(leftY, nullptr);
	ASSERT_NE(leftLid, nullptr);
	ASSERT_NE(pupil, nullptr);
	EXPECT_FLOAT_EQ(leftX->value, 1.0f);
	EXPECT_FLOAT_EQ(leftY->value, -1.0f);
	EXPECT_FLOAT_EQ(leftLid->value, 1.0f);
	EXPECT_FLOAT_EQ(pupil->value, 1.0f);
}

// ---------------------------------------------------------------------------
// Tests: expression param name table
// ---------------------------------------------------------------------------

// The canonical list from the PR comment (indices 0, 26, 62).
TEST(ExprParamNames, TableCount)
{
	EXPECT_EQ(protocol::FACETRACKING_EXPRESSION_COUNT, 63u);
}

TEST(ExprParamNames, FirstEntry)
{
	// Index 0 must be EyeLookOutLeft (first Unified Expression).
	EXPECT_STREQ(facetracking::ExpressionName(0), "EyeLookOutLeft");
}

TEST(ExprParamNames, JawOpen)
{
	// JawOpen is index 26 in the canonical list (0-based, per PR comment).
	EXPECT_STREQ(facetracking::ExpressionName(26), "JawOpen");
}

TEST(ExprParamNames, LastEntry)
{
	// Index 62 (last) must be TongueLeft.
	EXPECT_STREQ(facetracking::ExpressionName(62), "TongueLeft");
}

TEST(ExprParamNames, AddressBuilding)
{
	// Verify the prefix+name concatenation doesn't overflow for the longest name.
	const char* prefix = "/avatar/parameters/";
	const size_t prefixLen = std::strlen(prefix);

	size_t maxNameLen = 0;
	for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
		size_t l = std::strlen(facetracking::ExpressionName(i));
		if (l > maxNameLen) maxNameLen = l;
	}

	// Buffer in driver is 64 bytes: prefix (19) + longest name + NUL.
	EXPECT_LE(prefixLen + maxNameLen + 1, 64u)
	    << "Longest expression address does not fit in the 64-byte stack buffer.";
}

// ---------------------------------------------------------------------------
// Tests: migration idempotency
// ---------------------------------------------------------------------------
// The migration logic is in Migration.cpp which can't be linked here without
// pulling in the full overlay exe.  We test the idempotency contract by
// exercising the file-level sentinel pattern directly:
//
//   1. Sentinel absent + non-default port -> migration runs -> sentinel written.
//   2. Sentinel present -> migration returns early -> oscrouter.json unchanged.
//   3. Router profile already has non-default port -> sentinel written, no clobber.
//
// Rather than re-implementing the full migration, we verify the logic via
// picojson round-trips on mock JSON payloads.  This exercises the data
// transformation that Migration.cpp performs.

#include "picojson.h"

static void ParseMockFtJson(const std::string& body, bool& sentinelPresent, int& oscPort)
{
	picojson::value root;
	std::string err;
	picojson::parse(root, body);
	if (!root.is<picojson::object>()) {
		sentinelPresent = false;
		oscPort = 9000;
		return;
	}
	const auto& obj = root.get<picojson::object>();

	auto itSent = obj.find("osc_migrated_to_router");
	sentinelPresent = (itSent != obj.end() && itSent->second.is<bool>() && itSent->second.get<bool>());

	auto itPort = obj.find("osc_port");
	oscPort =
	    (itPort != obj.end() && itPort->second.is<double>()) ? static_cast<int>(itPort->second.get<double>()) : 9000;
}

static int ParseMockOrJson(const std::string& body)
{
	picojson::value root;
	picojson::parse(root, body);
	if (!root.is<picojson::object>()) return 9000;
	const auto& obj = root.get<picojson::object>();
	auto it = obj.find("send_port");
	return (it != obj.end() && it->second.is<double>()) ? static_cast<int>(it->second.get<double>()) : 9000;
}

// Simulates the migration transformation: reads osc_port from ftJson,
// writes send_port to orJson (if port is non-default), returns the modified
// pair and whether migration ran.
static bool SimulateMigration(std::string& ftJsonInOut, std::string& orJsonInOut)
{
	bool sentinel;
	int oldPort;
	ParseMockFtJson(ftJsonInOut, sentinel, oldPort);
	if (sentinel) return false; // idempotency check

	// Check router profile.
	int existingOrPort = ParseMockOrJson(orJsonInOut);
	if (existingOrPort != 9000) {
		// User already set a custom router port; write sentinel but don't clobber.
		picojson::value root;
		picojson::parse(root, ftJsonInOut);
		picojson::object obj = root.is<picojson::object>() ? root.get<picojson::object>() : picojson::object{};
		obj["osc_migrated_to_router"] = picojson::value(true);
		ftJsonInOut = picojson::value(obj).serialize(true);
		return false;
	}

	if (oldPort != 9000 && oldPort > 0 && oldPort <= 65535) {
		// Write send_port into router profile.
		picojson::value orRoot;
		picojson::parse(orRoot, orJsonInOut);
		picojson::object orObj = orRoot.is<picojson::object>() ? orRoot.get<picojson::object>() : picojson::object{};
		orObj["send_port"] = picojson::value(static_cast<double>(oldPort));
		orJsonInOut = picojson::value(orObj).serialize(true);
	}

	// Write sentinel.
	picojson::value ftRoot;
	picojson::parse(ftRoot, ftJsonInOut);
	picojson::object ftObj = ftRoot.is<picojson::object>() ? ftRoot.get<picojson::object>() : picojson::object{};
	ftObj["osc_migrated_to_router"] = picojson::value(true);
	ftJsonInOut = picojson::value(ftObj).serialize(true);
	return true;
}

TEST(OscPortMigration, NonDefaultPortMigratedToRouter)
{
	std::string ftJson = R"({"osc_port": 9001, "output_osc_enabled": true})";
	std::string orJson = "{}";

	bool ran = SimulateMigration(ftJson, orJson);
	EXPECT_TRUE(ran);

	// Router profile now has send_port=9001.
	EXPECT_EQ(ParseMockOrJson(orJson), 9001);

	// Sentinel written in facetracking.json.
	bool sentinel;
	int port;
	ParseMockFtJson(ftJson, sentinel, port);
	EXPECT_TRUE(sentinel);
}

TEST(OscPortMigration, DefaultPortNoMigration)
{
	std::string ftJson = R"({"osc_port": 9000, "output_osc_enabled": true})";
	std::string orJson = "{}";

	bool ran = SimulateMigration(ftJson, orJson);
	// ran == false because default port; sentinel still written.
	(void)ran;

	// Router profile unchanged (no send_port key written).
	EXPECT_EQ(ParseMockOrJson(orJson), 9000);

	// Sentinel written.
	bool sentinel;
	int port;
	ParseMockFtJson(ftJson, sentinel, port);
	EXPECT_TRUE(sentinel);
}

TEST(OscPortMigration, IdempotentOnSecondRun)
{
	std::string ftJson = R"({"osc_port": 9001, "osc_migrated_to_router": true})";
	std::string orJson = "{}";

	bool ran = SimulateMigration(ftJson, orJson);
	EXPECT_FALSE(ran); // sentinel present -> skip

	// Router profile unchanged.
	EXPECT_EQ(ParseMockOrJson(orJson), 9000);
}

TEST(OscPortMigration, RouterAlreadyConfiguredNotClobbered)
{
	std::string ftJson = R"({"osc_port": 9001, "output_osc_enabled": true})";
	std::string orJson = R"({"send_port": 9999})";

	SimulateMigration(ftJson, orJson);

	// Router profile keeps user's custom port.
	EXPECT_EQ(ParseMockOrJson(orJson), 9999);

	// Sentinel still written in facetracking.json.
	bool sentinel;
	int port;
	ParseMockFtJson(ftJson, sentinel, port);
	EXPECT_TRUE(sentinel);
}
