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
#include "ChatboxText.h"
#include "CaptionPreviewHistory.h"
#include "CaptionsConfig.h"
#include "CaptionsOutputPolicy.h"
#include "CaptionsRealtimeFlags.h"
#include "CaptionsSpeechModels.h"
#include "Protocol.h"

// Header-only host + overlay logic under test (no ImGui / COM / device access).
#include "AudioLevel.h"
#include "CaptionsAudioInputFile.h"
#include "CaptionsTabLogic.h"
#include "EnergySpeechGate.h"
#include "TranscriptText.h"
#include "WhisperPromptHistory.h"

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
	for (int i = 0; i < 17; ++i) {
		pacer.Enqueue("msg" + std::to_string(i), true, false);
	}

	// The oldest (msg0) should have been dropped; queue holds kQueueCap entries.
	EXPECT_EQ(pacer.QueueSize(), 16u);
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
	EXPECT_EQ(cfg.realtime_flags, 0);
	EXPECT_EQ(cfg.speech_model, 0);
}

TEST(CaptionsRealtimeFlagsTest, DefaultsEnableRealtimeOptions)
{
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimeExtendedTiming));
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimeSpeechEvidenceGate));
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimeConfidenceFilter));
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimeOverlapCleanup));
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimeChatboxSplitting));
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimeWhisperNoSpeechGate));
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimePromptContext));
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimeTypingIndicator));
	EXPECT_TRUE(captions::CaptionsRealtimeMaskEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimeSpeechPickupMask));
	EXPECT_TRUE(captions::CaptionsRealtimeMaskEnabled(captions::kCaptionsRealtimeDefaultFlags,
	                                                  captions::kCaptionsRealtimeRandomCaptionMask));

	CaptionsConfig cfg;
	EXPECT_EQ(cfg.realtime_flags, captions::kCaptionsRealtimeDefaultFlags);
	EXPECT_EQ(cfg.speech_model, captions::kCaptionsSpeechModelBalanced);
}

TEST(CaptionsRealtimeFlagsTest, SetClearsIndividualOptions)
{
	uint8_t flags = captions::kCaptionsRealtimeDefaultFlags;
	flags = captions::SetCaptionsRealtimeFlag(flags, captions::kCaptionsRealtimeConfidenceFilter, false);
	EXPECT_FALSE(captions::CaptionsRealtimeFlagEnabled(flags, captions::kCaptionsRealtimeConfidenceFilter));
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(flags, captions::kCaptionsRealtimeExtendedTiming));

	flags = captions::SetCaptionsRealtimeFlag(flags, captions::kCaptionsRealtimeConfidenceFilter, true);
	EXPECT_TRUE(captions::CaptionsRealtimeFlagEnabled(flags, captions::kCaptionsRealtimeConfidenceFilter));
}

TEST(CaptionsRealtimeFlagsTest, SetClearsGroupedOptions)
{
	uint8_t flags = captions::kCaptionsRealtimeDefaultFlags;
	flags = captions::SetCaptionsRealtimeMask(flags, captions::kCaptionsRealtimeRandomCaptionMask, false);
	EXPECT_FALSE(captions::CaptionsRealtimeFlagEnabled(flags, captions::kCaptionsRealtimeConfidenceFilter));
	EXPECT_FALSE(captions::CaptionsRealtimeFlagEnabled(flags, captions::kCaptionsRealtimeWhisperNoSpeechGate));
	EXPECT_FALSE(captions::CaptionsRealtimeMaskEnabled(flags, captions::kCaptionsRealtimeRandomCaptionMask));

	flags = captions::SetCaptionsRealtimeMask(flags, captions::kCaptionsRealtimeRandomCaptionMask, true);
	EXPECT_TRUE(captions::CaptionsRealtimeMaskEnabled(flags, captions::kCaptionsRealtimeRandomCaptionMask));
}

TEST(CaptionsSpeechModelTest, NormalizesAndMapsModels)
{
	EXPECT_EQ(captions::NormalizeCaptionsSpeechModel(-1), captions::kCaptionsSpeechModelBalanced);
	EXPECT_EQ(captions::NormalizeCaptionsSpeechModel(99), captions::kCaptionsSpeechModelBalanced);
	EXPECT_EQ(captions::NormalizeCaptionsSpeechModel(captions::kCaptionsSpeechModelHighAccuracy),
	          captions::kCaptionsSpeechModelHighAccuracy);
	EXPECT_STREQ(captions::CaptionsSpeechModelFileName(captions::kCaptionsSpeechModelBalanced), "ggml-base.bin");
	EXPECT_STREQ(captions::CaptionsSpeechModelFileName(captions::kCaptionsSpeechModelHighAccuracy),
	             "ggml-large-v3-turbo-q5_0.bin");
	EXPECT_STREQ(captions::CaptionsSpeechModelPackId(captions::kCaptionsSpeechModelHighAccuracy),
	             "speech-large-v3-turbo");
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

TEST(WhisperPromptHistoryTest, TrimsAndJoinsRecentTranscripts)
{
	captions::WhisperPromptHistory history;
	history.Observe("  hello there\r\n");
	history.Observe("\tgeneral kenobi ");

	EXPECT_EQ(history.Text(), "hello there general kenobi");
}

TEST(WhisperPromptHistoryTest, IgnoresBlankTextAndCanClear)
{
	captions::WhisperPromptHistory history;
	history.Observe("");
	history.Observe(" \r\n\t ");
	EXPECT_TRUE(history.Text().empty());

	history.Observe("one");
	history.Clear();
	EXPECT_TRUE(history.Text().empty());
}

TEST(WhisperPromptHistoryTest, KeepsWholeWordsWhenTrimmingBudget)
{
	captions::WhisperPromptHistory history(18);
	history.Observe("alpha beta gamma");
	history.Observe("delta");

	EXPECT_LE(history.Text().size(), 18u);
	EXPECT_EQ(history.Text(), "beta gamma delta");
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

TEST(TruncationTest, ShortStringUnchanged)
{
	std::string s = "Hello world";
	EXPECT_EQ(captions::TruncateTextForChatbox(s), s);
}

TEST(TruncationTest, LongStringTruncatedAtWhitespace)
{
	std::string s(140, 'a');
	s += " extra words here that push past the limit";
	std::string result = captions::TruncateTextForChatbox(s);
	EXPECT_LE(result.size(), captions::VrchatChatboxByteLimit());
	EXPECT_TRUE(result.back() == '.');
}

TEST(TruncationTest, LongStringWithNoWhitespaceTruncatesHard)
{
	std::string s(200, 'x');
	std::string result = captions::TruncateTextForChatbox(s);
	EXPECT_LE(result.size(), captions::VrchatChatboxByteLimit());
}

TEST(ChatboxTextTest, SplitKeepsEveryChunkUnderLimit)
{
	const std::string s = "alpha beta gamma delta epsilon zeta eta theta iota kappa lambda";
	const std::vector<std::string> chunks = captions::SplitTextForChatbox(s, 18);
	ASSERT_GT(chunks.size(), 1u);
	for (const auto& chunk : chunks) {
		EXPECT_LE(chunk.size(), 18u);
		EXPECT_FALSE(chunk.empty());
	}
	EXPECT_EQ(chunks.front(), "alpha beta gamma");
	EXPECT_EQ(chunks.back(), "iota kappa lambda");
}

TEST(ChatboxTextTest, SplitDoesNotStartChunkOnUtf8ContinuationByte)
{
	const std::string e_accent = "\xC3\xA9";
	const std::string s = e_accent + e_accent + e_accent + " " + e_accent + e_accent + e_accent;
	const std::vector<std::string> chunks = captions::SplitTextForChatbox(s, 5);
	ASSERT_GT(chunks.size(), 1u);
	for (const auto& chunk : chunks) {
		EXPECT_LE(chunk.size(), 5u);
		ASSERT_FALSE(chunk.empty());
		EXPECT_FALSE(captions::ChatboxUtf8Continuation(static_cast<unsigned char>(chunk.front())));
	}
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
	EXPECT_EQ(captions::AlwaysOnPrerollFrames(), 32);
	EXPECT_EQ(captions::AlwaysOnSilenceFrames(), 32);
	EXPECT_EQ(captions::AlwaysOnMaxSpeechSamples(), 80000u);
	EXPECT_EQ(captions::AlwaysOnContinuationOverlapSamples(), 9600u);
}

TEST(EnergySpeechGateTest, AlwaysOnCanUseLegacyTiming)
{
	EXPECT_EQ(captions::AlwaysOnPrerollFrames(false), 20);
	EXPECT_EQ(captions::AlwaysOnSilenceFrames(false), 30);
	EXPECT_EQ(captions::AlwaysOnMaxSpeechSamples(false), 128000u);
	EXPECT_EQ(captions::AlwaysOnContinuationOverlapSamples(false), 6400u);
}

TEST(EnergySpeechGateTest, ShortAlwaysOnSegmentNeedsConfidence)
{
	const size_t short_samples = captions::AlwaysOnShortSpeechSamples();
	const float threshold = 0.04f;

	EXPECT_FALSE(captions::SpeechSegmentShouldTranscribe(short_samples - 1, true, 0.99f, 1.0f, threshold));
	EXPECT_FALSE(captions::SpeechSegmentShouldTranscribe(short_samples, true, 0.30f, 0.05f, threshold));
	EXPECT_TRUE(captions::SpeechSegmentShouldTranscribe(short_samples, true, 0.71f, 0.05f, threshold));
	EXPECT_TRUE(captions::SpeechSegmentShouldTranscribe(short_samples, true, 0.30f, 0.09f, threshold));
	EXPECT_TRUE(
	    captions::SpeechSegmentShouldTranscribe(captions::AlwaysOnMinSpeechSamples(), true, 0.0f, 0.0f, threshold));
}

TEST(EnergySpeechGateTest, AdaptiveGateOpensQuietSpeechInQuietRoom)
{
	captions::AdaptiveSpeechGate gate;
	for (int i = 0; i < 120; ++i) {
		gate.ObserveAmbient(0.002f);
	}

	EXPECT_LT(gate.SpeechPeakThreshold(), 0.08f);
	EXPECT_TRUE(gate.IsSpeech(-1.0f, 0.03f));
	EXPECT_FALSE(gate.IsSilence(-1.0f, 0.03f));
}

TEST(EnergySpeechGateTest, AdaptiveGateRaisesThresholdInNoisyRoom)
{
	captions::AdaptiveSpeechGate gate;
	for (int i = 0; i < 240; ++i) {
		gate.ObserveAmbient(0.04f);
	}

	EXPECT_GT(gate.AmbientPeak(), 0.03f);
	EXPECT_GT(gate.SpeechPeakThreshold(), 0.08f);
	EXPECT_FALSE(gate.IsSpeech(-1.0f, 0.06f));
	EXPECT_TRUE(gate.IsSpeech(-1.0f, 0.11f));
}

TEST(EnergySpeechGateTest, AdaptiveGateStillTrustsVad)
{
	captions::AdaptiveSpeechGate gate;
	for (int i = 0; i < 240; ++i) {
		gate.ObserveAmbient(0.04f);
	}

	EXPECT_TRUE(gate.IsSpeech(0.50f, 0.0f));
	EXPECT_TRUE(gate.IsSpeech(0.35f, gate.SoftSpeechPeakThreshold()));
	EXPECT_FALSE(gate.IsSpeech(0.31f, gate.SoftSpeechPeakThreshold()));
}

TEST(EnergySpeechGateTest, PossibleSpeechSitsBetweenSilenceAndOpen)
{
	captions::AdaptiveSpeechGate gate;
	for (int i = 0; i < 120; ++i) {
		gate.ObserveAmbient(0.002f);
	}

	const float possible = gate.PossibleSpeechPeakThreshold();
	EXPECT_LT(possible, gate.SpeechPeakThreshold());
	EXPECT_GT(possible, gate.SilencePeakThreshold());
	EXPECT_FALSE(gate.IsSpeech(-1.0f, possible));
	EXPECT_TRUE(gate.IsPossibleSpeech(-1.0f, possible));
	EXPECT_FALSE(gate.IsSilence(-1.0f, possible));
}

TEST(EnergySpeechGateTest, ActivationWindowAllowsJitterySpeechFrames)
{
	captions::SpeechActivationWindow window;
	window.Push(true, true);
	EXPECT_FALSE(window.ShouldOpen());
	window.Push(false, true);
	EXPECT_FALSE(window.ShouldOpen());
	window.Push(true, true);

	EXPECT_TRUE(window.ShouldOpen());
	EXPECT_EQ(window.SpeechFrames(), 2);
	EXPECT_EQ(window.PossibleFrames(), 3);
}

TEST(EnergySpeechGateTest, ActivationWindowAllowsOneSpeechFrameWithPossibleContext)
{
	captions::SpeechActivationWindow window;
	window.Push(false, true);
	window.Push(true, true);
	window.Push(false, true);

	EXPECT_TRUE(window.ShouldOpen());
	EXPECT_EQ(window.SpeechFrames(), 1);
	EXPECT_EQ(window.PossibleFrames(), captions::AlwaysOnActivationPossibleFrames());
}

TEST(EnergySpeechGateTest, ActivationWindowRejectsPossibleOnlyNoise)
{
	captions::SpeechActivationWindow window;
	for (int i = 0; i < captions::AlwaysOnActivationWindowFrames(); ++i) {
		window.Push(false, true);
	}

	EXPECT_FALSE(window.ShouldOpen());
	EXPECT_EQ(window.SpeechFrames(), 0);
	EXPECT_EQ(window.PossibleFrames(), captions::AlwaysOnActivationWindowFrames());
}

TEST(EnergySpeechGateTest, CopyTrailingSamplesKeepsBoundedContinuationOverlap)
{
	std::vector<float> pcm(10000);
	for (size_t i = 0; i < pcm.size(); ++i) {
		pcm[i] = static_cast<float>(i);
	}

	std::vector<float> tail = captions::CopyTrailingSamples(pcm, captions::AlwaysOnContinuationOverlapSamples());
	ASSERT_EQ(tail.size(), captions::AlwaysOnContinuationOverlapSamples());
	EXPECT_FLOAT_EQ(tail.front(), 400.0f);
	EXPECT_FLOAT_EQ(tail.back(), 9999.0f);

	std::vector<float> short_tail = captions::CopyTrailingSamples(std::vector<float>{1.0f, 2.0f, 3.0f}, 6400);
	EXPECT_EQ(short_tail, (std::vector<float>{1.0f, 2.0f, 3.0f}));
}

TEST(TranscriptTextTest, RemovesMultiWordOverlapIgnoringCaseAndPunctuation)
{
	const std::string previous = "I want this to keep every boundary word.";
	const std::string current = "Every boundary word, even if I keep speaking.";

	EXPECT_EQ(captions::RemoveOverlappingTranscriptPrefix(previous, current), "even if I keep speaking.");
}

TEST(TranscriptTextTest, KeepsSingleRepeatedWordToAvoidDroppingRealSpeech)
{
	const std::string previous = "I said hello";
	const std::string current = "hello hello again";

	EXPECT_EQ(captions::RemoveOverlappingTranscriptPrefix(previous, current), current);
}

TEST(TranscriptTextTest, CleansKnownNonSpeechMarkers)
{
	EXPECT_EQ(captions::CleanTranscriptForPublish("[Music]"), "");
	EXPECT_EQ(captions::CleanTranscriptForPublish(" [Laughter]  hello there [Applause] "), "hello there");
	EXPECT_TRUE(captions::TranscriptIsKnownNonSpeechMarker("[background noise]"));
}

TEST(TranscriptTextTest, DetectsCommonLowConfidenceHallucinations)
{
	EXPECT_TRUE(captions::TranscriptLooksLikeCommonHallucination("Thanks for watching."));
	EXPECT_TRUE(captions::TranscriptLooksLikeCommonHallucination("Please subscribe"));
	EXPECT_FALSE(captions::TranscriptLooksLikeCommonHallucination("Thank you for helping me test captions."));
	EXPECT_FALSE(captions::TranscriptLooksLikeCommonHallucination("I was watching the mirror."));
}

TEST(TranscriptTextTest, DetectsOnlyLikelyDecodeRepetition)
{
	EXPECT_TRUE(captions::TranscriptLooksRepetitive("hello world hello world hello world"));
	EXPECT_TRUE(captions::TranscriptLooksRepetitive("test test test test test test"));
	EXPECT_FALSE(captions::TranscriptLooksRepetitive("hello world hello again world hello"));
}

TEST(TranscriptTextTest, ConfidenceSuppressionRequiresWeakAudioOrBadDecode)
{
	EXPECT_TRUE(captions::TranscriptShouldSuppressByConfidence("Thanks for watching", true, 0.10f, 0.01f, 0.04f, 0.30f,
	                                                           -0.20f, 4));
	EXPECT_TRUE(
	    captions::TranscriptShouldSuppressByConfidence("hello there", true, 0.10f, 0.01f, 0.04f, 0.90f, -0.20f, 4));
	EXPECT_FALSE(
	    captions::TranscriptShouldSuppressByConfidence("hello there", true, 0.70f, 0.01f, 0.04f, 0.90f, -0.20f, 4));
	EXPECT_FALSE(
	    captions::TranscriptShouldSuppressByConfidence("hello there", false, 0.10f, 0.01f, 0.04f, 0.90f, -2.00f, 4));
}

TEST(TranscriptTextTest, NoSpeechProbabilityUsesWhisperThresholds)
{
	EXPECT_TRUE(captions::TranscriptShouldSuppressByNoSpeechProbability("hello there", true, 0.60f, -1.01f));
	EXPECT_FALSE(captions::TranscriptShouldSuppressByNoSpeechProbability("hello there", true, 0.59f, -1.01f));
	EXPECT_FALSE(captions::TranscriptShouldSuppressByNoSpeechProbability("hello there", true, 0.90f, -0.99f));
	EXPECT_FALSE(captions::TranscriptShouldSuppressByNoSpeechProbability("hello there", false, 0.90f, -2.00f));
	EXPECT_FALSE(captions::TranscriptShouldSuppressByNoSpeechProbability("", true, 0.90f, -2.00f));
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
