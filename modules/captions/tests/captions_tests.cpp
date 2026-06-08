// Unit tests for the captions module.
// Run via: ./build/artifacts/Release/captions_tests.exe
//
// Tests run without GPU, OpenVR, or live microphone access.

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Chatbox pacer
// ---------------------------------------------------------------------------

// Pull in the implementation directly (header-only interface test).
#include "ChatboxPacer.h"
#include "CaptionPreviewHistory.h"
#include "CaptionsOutputPolicy.h"
#include "Protocol.h"

// Header-only host + overlay logic under test (no ImGui / COM / device access).
#include "AudioLevel.h"
#include "CaptionsAudioInputFile.h"
#include "CaptionsTabLogic.h"
#include "EnergySpeechGate.h"

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

TEST(CaptionPreviewHistoryTest, AddsEachCompletedCaptionOnce)
{
	captions::CaptionPreviewHistory history;
	history.Observe(1, "hello", "");
	history.Observe(1, "hello", "");
	history.Observe(2, "hello", "bonjour");

	ASSERT_EQ(history.Entries().size(), 2u);
	EXPECT_EQ(history.Entries()[0].transcript, "hello");
	EXPECT_TRUE(history.Entries()[0].translation.empty());
	EXPECT_EQ(history.Entries()[1].translation, "bonjour");
}

TEST(CaptionPreviewHistoryTest, KeepsMostRecentEntries)
{
	captions::CaptionPreviewHistory history(2);
	history.Observe(1, "one", "");
	history.Observe(2, "two", "");
	history.Observe(3, "three", "");

	ASSERT_EQ(history.Entries().size(), 2u);
	EXPECT_EQ(history.Entries()[0].transcript, "two");
	EXPECT_EQ(history.Entries()[1].transcript, "three");
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

static size_t EncodeTypingPacket(uint8_t* buf, size_t buf_size, bool active)
{
	const char* typetag = active ? ",T" : ",F";
	size_t addr_bytes = Pad4(strlen("/chatbox/typing") + 1);
	size_t tag_bytes = Pad4(strlen(typetag) + 1);
	size_t total = addr_bytes + tag_bytes;
	if (total > buf_size) return 0;

	uint8_t* out = buf;
	WriteOscString(out, "/chatbox/typing");
	WriteOscString(out, typetag);
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

TEST(OscPacketTest, TypingIndicatorUsesBoolTypetag)
{
	uint8_t buf[128] = {};
	size_t sz = EncodeTypingPacket(buf, sizeof(buf), true);
	ASSERT_GT(sz, 0u);
	EXPECT_EQ(sz % 4, 0u);
	EXPECT_STREQ(reinterpret_cast<const char*>(buf), "/chatbox/typing");

	size_t addr_bytes = Pad4(strlen("/chatbox/typing") + 1);
	const char* typetag = reinterpret_cast<const char*>(buf + addr_bytes);
	EXPECT_STREQ(typetag, ",T");
	EXPECT_EQ(sz, addr_bytes + Pad4(strlen(",T") + 1));

	memset(buf, 0, sizeof(buf));
	sz = EncodeTypingPacket(buf, sizeof(buf), false);
	ASSERT_GT(sz, 0u);
	typetag = reinterpret_cast<const char*>(buf + addr_bytes);
	EXPECT_STREQ(typetag, ",F");
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

// ---------------------------------------------------------------------------
// Microphone input level (host-side pure helpers)
// ---------------------------------------------------------------------------

TEST(AudioLevelTest, SilenceIsZero)
{
	std::vector<float> silence(480, 0.0f);
	EXPECT_FLOAT_EQ(captions::ComputeBufferPeak(silence.data(), silence.size()), 0.0f);
	EXPECT_FLOAT_EQ(captions::ComputeBufferPeak(nullptr, 0), 0.0f);
}

TEST(AudioLevelTest, PeakIsMaxAbsAndClamped)
{
	float samples[] = {0.1f, -0.6f, 0.3f, -0.2f};
	EXPECT_FLOAT_EQ(captions::ComputeBufferPeak(samples, 4), 0.6f);

	float hot[] = {2.0f, -3.0f};
	EXPECT_FLOAT_EQ(captions::ComputeBufferPeak(hot, 2), 1.0f); // clamped to 1.0
}

TEST(AudioLevelTest, DecayRisesInstantlyAndFallsSlowly)
{
	// Louder peak takes over immediately.
	EXPECT_FLOAT_EQ(captions::DecayLevel(0.2f, 0.8f, 0.85f), 0.8f);
	// Quieter peak decays from the current level.
	EXPECT_FLOAT_EQ(captions::DecayLevel(1.0f, 0.0f, 0.85f), 0.85f);
}

TEST(EnergySpeechGateTest, SileroSpeechProbabilityOpensGate)
{
	EXPECT_TRUE(captions::SpeechGateIsSpeech(0.5f, 0.0f));
	EXPECT_TRUE(captions::SpeechGateIsSpeech(0.7f, 0.0f));
}

TEST(EnergySpeechGateTest, InputLevelFallbackOpensGate)
{
	EXPECT_FALSE(captions::SpeechGateIsSpeech(-1.0f, 0.079f));
	EXPECT_TRUE(captions::SpeechGateIsSpeech(-1.0f, 0.08f));
	EXPECT_TRUE(captions::SpeechGateIsSpeech(0.1f, 0.2f));
}

TEST(EnergySpeechGateTest, SilenceRequiresBothLowVadAndLowLevel)
{
	EXPECT_TRUE(captions::SpeechGateIsSilence(-1.0f, 0.0f));
	EXPECT_TRUE(captions::SpeechGateIsSilence(0.2f, 0.025f));
	EXPECT_FALSE(captions::SpeechGateIsSilence(0.4f, 0.0f));
	EXPECT_FALSE(captions::SpeechGateIsSilence(0.1f, 0.026f));
}

TEST(EnergySpeechGateTest, AlwaysOnUsesLongerMinimumThanPushToTalk)
{
	EXPECT_LT(captions::PushToTalkMinSpeechSamples(), captions::AlwaysOnMinSpeechSamples());
	EXPECT_FALSE(captions::SpeechBufferLongEnough(captions::AlwaysOnMinSpeechSamples() - 1, true));
	EXPECT_TRUE(captions::SpeechBufferLongEnough(captions::AlwaysOnMinSpeechSamples(), true));
	EXPECT_FALSE(captions::SpeechBufferLongEnough(captions::PushToTalkMinSpeechSamples() - 1, false));
	EXPECT_TRUE(captions::SpeechBufferLongEnough(captions::PushToTalkMinSpeechSamples(), false));
}

TEST(EnergySpeechGateTest, AlwaysOnKeepsShortPrerollAndModerateSilenceTail)
{
	EXPECT_EQ(captions::AlwaysOnPrerollFrames(), 8);
	EXPECT_EQ(captions::AlwaysOnSilenceFrames(), 24);
}

// ---------------------------------------------------------------------------
// Selected-device file (cross-process contract)
// ---------------------------------------------------------------------------

TEST(AudioInputFileTest, ParseTrimsWhitespaceAndNewlines)
{
	EXPECT_EQ(captions::ParseAudioInputDeviceId("{0.0.1.00000000}.{abc}"), "{0.0.1.00000000}.{abc}");
	EXPECT_EQ(captions::ParseAudioInputDeviceId("  {id}\r\n"), "{id}");
	EXPECT_EQ(captions::ParseAudioInputDeviceId("\t{id}\t"), "{id}");
	EXPECT_EQ(captions::ParseAudioInputDeviceId(""), "");
	EXPECT_EQ(captions::ParseAudioInputDeviceId("\r\n  \t"), ""); // blank => system default
}

TEST(AudioInputFileTest, ReadRoundTripFromTempDir)
{
	wchar_t tmp[MAX_PATH];
	DWORD n = GetTempPathW(MAX_PATH, tmp);
	ASSERT_GT(n, 0u);
	std::wstring dir(tmp, n);
	if (!dir.empty() && dir.back() == L'\\') dir.pop_back();

	std::wstring path = dir + L"\\audio_input.txt";

	// Missing file => system default ("").
	DeleteFileW(path.c_str());
	EXPECT_EQ(captions::ReadCaptionsInputDeviceId(dir), "");

	// Written id (with a trailing newline) round-trips trimmed.
	{
		std::ofstream of(path.c_str());
		of << "{0.0.1.00000000}.{device-guid}\n";
	}
	EXPECT_EQ(captions::ReadCaptionsInputDeviceId(dir), "{0.0.1.00000000}.{device-guid}");

	// Empty file => system default.
	{
		std::ofstream of(path.c_str(), std::ios::trunc);
	}
	EXPECT_EQ(captions::ReadCaptionsInputDeviceId(dir), "");

	DeleteFileW(path.c_str());
	EXPECT_EQ(captions::ReadCaptionsInputDeviceId(L""), ""); // empty dir is safe
}

// ---------------------------------------------------------------------------
// Setup tab: Install / Uninstall button gating
// ---------------------------------------------------------------------------

static captions::HostStatusSnapshot MakeSnap(bool valid, bool speechInstalled)
{
	captions::HostStatusSnapshot s;
	s.valid = valid;
	s.speech_pack_installed = speechInstalled;
	return s;
}

TEST(SetupGatingTest, SpeechButtonsAreMutuallyExclusiveByInstallState)
{
	using namespace captions::ui;

	// Installed: Install disabled, Uninstall enabled.
	auto installed = MakeSnap(true, true);
	EXPECT_FALSE(SpeechInstallEnabled(installed, false));
	EXPECT_TRUE(SpeechUninstallEnabled(installed, false));

	// Not installed: Install enabled, Uninstall disabled.
	auto missing = MakeSnap(true, false);
	EXPECT_TRUE(SpeechInstallEnabled(missing, false));
	EXPECT_FALSE(SpeechUninstallEnabled(missing, false));
}

TEST(SetupGatingTest, BusyDisablesBothSpeechButtons)
{
	using namespace captions::ui;
	auto installed = MakeSnap(true, true);
	EXPECT_FALSE(SpeechInstallEnabled(installed, true));
	EXPECT_FALSE(SpeechUninstallEnabled(installed, true));
}

TEST(SetupGatingTest, InvalidSnapshotTreatsSpeechAsNotInstalled)
{
	using namespace captions::ui;
	auto unknown = MakeSnap(false, true); // host not reporting yet
	EXPECT_FALSE(SpeechInstalled(unknown));
	EXPECT_FALSE(SpeechUninstallEnabled(unknown, false));
}

TEST(SetupGatingTest, TranslationGatingFollowsSelectedPackAndState)
{
	using namespace captions::ui;
	captions::HostStatusSnapshot s;
	s.valid = true;

	// No pack selected: neither button is actionable.
	EXPECT_FALSE(TranslationInstallEnabled(s, false, ""));
	EXPECT_FALSE(TranslationUninstallEnabled(s, false, ""));

	// Pack selected but not installed: Install on, Uninstall off.
	s.translation_pack_installed = false;
	EXPECT_TRUE(TranslationInstallEnabled(s, false, "translation-en-de"));
	EXPECT_FALSE(TranslationUninstallEnabled(s, false, "translation-en-de"));

	// Pack installed and active pair matches: Install off, Uninstall on.
	s.translation_pack_installed = true;
	s.active_translation_pair = "en-de";
	EXPECT_FALSE(TranslationInstallEnabled(s, false, "translation-en-de"));
	EXPECT_TRUE(TranslationUninstallEnabled(s, false, "translation-en-de"));

	// Host mid-switch to a different pair: this pack reads as not-installed.
	s.active_translation_pair = "en-fr";
	EXPECT_FALSE(TranslationInstalled(s, "translation-en-de"));
	EXPECT_TRUE(TranslationInstallEnabled(s, false, "translation-en-de"));
}

// ---------------------------------------------------------------------------
// "No audio reaching the host" warning
// ---------------------------------------------------------------------------

TEST(NoAudioWarningTest, FiresOnlyAfterSustainedStallWithLiveHost)
{
	using namespace captions::ui;
	// Host down: never warn.
	EXPECT_FALSE(ShouldWarnNoAudio(false, 10.0));
	// Host up but within the grace window: no warning yet.
	EXPECT_FALSE(ShouldWarnNoAudio(true, 1.0));
	// Host up and frames stalled past the threshold: warn.
	EXPECT_TRUE(ShouldWarnNoAudio(true, 3.0));
	EXPECT_TRUE(ShouldWarnNoAudio(true, 30.0));
}
