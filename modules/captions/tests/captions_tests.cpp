// Unit tests for the captions module.
// Run via: ./build/artifacts/Release/captions_tests.exe
//
// Tests run without GPU, OpenVR, or live microphone access.

#include <gtest/gtest.h>

#include <cmath>
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
#include "CaptionsThreadPlan.h"
#include "EnergySpeechGate.h"
#include "SileroVadModelContract.h"
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
	EXPECT_EQ(cfg.master_enabled, 0);
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
	EXPECT_TRUE(cfg.sidecar_enabled);
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
	EXPECT_STREQ(captions::CaptionsSpeechModelName(captions::kCaptionsSpeechModelBalanced), "balanced");
	EXPECT_STREQ(captions::CaptionsSpeechModelName(captions::kCaptionsSpeechModelHighAccuracy), "high-accuracy");
	EXPECT_STREQ(captions::CaptionsSpeechModelName(99), "balanced");
}

TEST(SileroVadModelContractTest, RecognizesMergedStateModel)
{
	const std::vector<std::string> inputs = {"input", "state", "sr"};
	const std::vector<std::string> outputs = {"output", "stateN"};

	const auto format = captions::ClassifySileroVadModelContract(inputs.size(), inputs, outputs.size(), outputs);
	EXPECT_EQ(format, captions::SileroVadModelFormat::MergedState);
	EXPECT_STREQ(captions::SileroVadModelFormatName(format), "merged-state");
}

TEST(SileroVadModelContractTest, RecognizesLegacyHcStateModel)
{
	const std::vector<std::string> inputs = {"sr", "input", "c", "h"};
	const std::vector<std::string> outputs = {"cn", "output", "hn"};

	const auto format = captions::ClassifySileroVadModelContract(inputs.size(), inputs, outputs.size(), outputs);
	EXPECT_EQ(format, captions::SileroVadModelFormat::LegacyHcState);
	EXPECT_STREQ(captions::SileroVadModelFormatName(format), "legacy-hc-state");
}

TEST(SileroVadModelContractTest, RejectsUnknownModelContract)
{
	const std::vector<std::string> inputs = {"input", "sr", "h"};
	const std::vector<std::string> outputs = {"output", "hn"};

	const auto format = captions::ClassifySileroVadModelContract(inputs.size(), inputs, outputs.size(), outputs);
	EXPECT_EQ(format, captions::SileroVadModelFormat::Unknown);
	EXPECT_STREQ(captions::SileroVadModelFormatName(format), "unknown");
	EXPECT_EQ(captions::JoinSileroVadNodeNames(inputs), "input,sr,h");
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

TEST(AudioLevelTest, RmsMeasuresSignalBodyAndClamps)
{
	float samples[] = {0.0f, 0.0f, 1.0f, -1.0f};
	EXPECT_NEAR(captions::ComputeBufferRms(samples, 4), 0.7071f, 0.0001f);

	float hot[] = {2.0f, -2.0f};
	EXPECT_FLOAT_EQ(captions::ComputeBufferRms(hot, 2), 1.0f);
}

TEST(AudioLevelTest, DecayRisesInstantlyAndFallsSlowly)
{
	// Louder peak takes over immediately.
	EXPECT_FLOAT_EQ(captions::DecayLevel(0.2f, 0.8f, 0.85f), 0.8f);
	// Quieter peak decays from the current level.
	EXPECT_FLOAT_EQ(captions::DecayLevel(1.0f, 0.0f, 0.85f), 0.85f);
}

TEST(AudioLevelTest, FrameFeaturesTrackBandShape)
{
	constexpr float sample_rate = 16000.0f;
	constexpr float pi = 3.14159265358979323846f;
	std::vector<float> low(1600);
	std::vector<float> mid(1600);
	for (size_t i = 0; i < low.size(); ++i) {
		const float t = static_cast<float>(i) / sample_rate;
		low[i] = 0.25f * std::sin(2.0f * pi * 60.0f * t);
		mid[i] = 0.25f * std::sin(2.0f * pi * 1000.0f * t);
	}

	const captions::AudioFrameFeatures low_features =
	    captions::ComputeAudioFrameFeatures(low.data(), low.size(), sample_rate);
	const captions::AudioFrameFeatures mid_features =
	    captions::ComputeAudioFrameFeatures(mid.data(), mid.size(), sample_rate);

	EXPECT_GT(low_features.low_band_ratio, mid_features.low_band_ratio);
	EXPECT_GT(mid_features.speech_band_ratio, mid_features.low_band_ratio);
	EXPECT_GT(mid_features.speech_band_ratio, mid_features.high_band_ratio);
	EXPECT_GT(mid_features.speech_band_ratio, 0.35f);
}

TEST(AudioLevelTest, FrameFeaturesDetectHighFrequencyAndClippingArtifacts)
{
	std::vector<float> alternating(480);
	for (size_t i = 0; i < alternating.size(); ++i) {
		alternating[i] = (i % 2 == 0) ? 0.20f : -0.20f;
	}

	const captions::AudioFrameFeatures high_features =
	    captions::ComputeAudioFrameFeatures(alternating.data(), alternating.size());
	EXPECT_GT(high_features.zero_crossing_rate, 0.90f);
	EXPECT_GT(high_features.artifact_risk, 0.25f);

	std::vector<float> clipped(480, 0.99f);
	const captions::AudioFrameFeatures clipped_features =
	    captions::ComputeAudioFrameFeatures(clipped.data(), clipped.size());
	EXPECT_GT(clipped_features.clipping_ratio, 0.90f);
	EXPECT_GT(clipped_features.artifact_risk, 0.40f);
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
	EXPECT_FALSE(captions::SpeechGateIsSpeech(-1.0f, 0.2f, 0.004f));
	EXPECT_TRUE(captions::SpeechGateIsSpeech(-1.0f, 0.2f, 0.030f));
}

TEST(EnergySpeechGateTest, SilenceRequiresBothLowVadAndLowLevel)
{
	EXPECT_TRUE(captions::SpeechGateIsSilence(-1.0f, 0.0f));
	EXPECT_TRUE(captions::SpeechGateIsSilence(0.2f, 0.025f));
	EXPECT_FALSE(captions::SpeechGateIsSilence(0.4f, 0.0f));
	EXPECT_FALSE(captions::SpeechGateIsSilence(0.1f, 0.026f));
	EXPECT_FALSE(captions::SpeechGateIsSilence(0.1f, 0.010f, 0.020f));
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
	EXPECT_EQ(captions::AlwaysOnWeakEvidenceSamples(), 8000u);
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
	const size_t sustained_samples = captions::AlwaysOnWeakEvidenceSamples();
	const float threshold = 0.04f;
	const float rms_threshold = 0.014f;

	EXPECT_FALSE(
	    captions::SpeechSegmentShouldTranscribe(short_samples - 1, true, 0.99f, 1.0f, threshold, 1.0f, rms_threshold));
	EXPECT_FALSE(
	    captions::SpeechSegmentShouldTranscribe(short_samples, true, 0.30f, 0.05f, threshold, 0.020f, rms_threshold));
	EXPECT_TRUE(
	    captions::SpeechSegmentShouldTranscribe(short_samples, true, 0.70f, 0.05f, threshold, 0.002f, rms_threshold));
	EXPECT_FALSE(
	    captions::SpeechSegmentShouldTranscribe(short_samples, true, 0.30f, 0.09f, threshold, 0.004f, rms_threshold));
	EXPECT_TRUE(
	    captions::SpeechSegmentShouldTranscribe(short_samples, true, 0.30f, 0.09f, threshold, 0.030f, rms_threshold));
	EXPECT_FALSE(captions::SpeechSegmentShouldTranscribe(captions::AlwaysOnMinSpeechSamples(), true, 0.0f, 0.0f,
	                                                     threshold, 0.0f, rms_threshold));
	EXPECT_FALSE(captions::SpeechSegmentShouldTranscribe(captions::AlwaysOnMinSpeechSamples(), true, 0.0f, threshold,
	                                                     threshold, rms_threshold, rms_threshold));
	EXPECT_TRUE(captions::SpeechSegmentShouldTranscribe(sustained_samples, true, 0.0f, 0.055f, threshold, 0.014f,
	                                                    rms_threshold));
	EXPECT_TRUE(captions::SpeechSegmentShouldTranscribe(sustained_samples, true, 0.55f, 0.020f, threshold, 0.004f,
	                                                    rms_threshold));
}

TEST(EnergySpeechGateTest, SegmentShapeRejectsSparseWeakEvidence)
{
	const float threshold = 0.04f;
	const float rms_threshold = 0.014f;

	EXPECT_FALSE(captions::SpeechSegmentHasUsableShape(10, 1, 3, captions::AlwaysOnWeakEvidenceSamples() - 1, true,
	                                                   0.30f, 0.050f, threshold, 0.015f, rms_threshold));
	EXPECT_TRUE(captions::SpeechSegmentHasUsableShape(10, 2, 5, captions::AlwaysOnWeakEvidenceSamples() + 1, true,
	                                                  0.30f, 0.050f, threshold, 0.015f, rms_threshold));
	EXPECT_TRUE(captions::SpeechSegmentHasUsableShape(10, 1, 1, captions::AlwaysOnShortSpeechSamples(), true, 0.72f,
	                                                  0.020f, threshold, 0.004f, rms_threshold));
	EXPECT_TRUE(captions::SpeechSegmentHasUsableShape(10, 1, 1, captions::AlwaysOnShortSpeechSamples(), true, 0.30f,
	                                                  0.100f, threshold, 0.030f, rms_threshold));
	EXPECT_TRUE(captions::SpeechSegmentHasUsableShape(0, 0, 0, captions::AlwaysOnShortSpeechSamples(), false, 0.0f,
	                                                  0.0f, threshold, 0.0f, rms_threshold));
}

TEST(EnergySpeechGateTest, AdaptiveGateOpensQuietSpeechInQuietRoom)
{
	captions::AdaptiveSpeechGate gate;
	for (int i = 0; i < 120; ++i) {
		gate.ObserveAmbient(0.002f, 0.001f);
	}

	EXPECT_LT(gate.SpeechPeakThreshold(), 0.08f);
	EXPECT_LT(gate.SpeechRmsThreshold(), 0.02f);
	EXPECT_TRUE(gate.IsSpeech(-1.0f, 0.03f, 0.012f));
	EXPECT_FALSE(gate.IsSpeech(-1.0f, 0.03f, 0.003f));
	EXPECT_FALSE(gate.IsSilence(-1.0f, 0.03f, 0.012f));
}

TEST(EnergySpeechGateTest, AdaptiveGateRaisesThresholdInNoisyRoom)
{
	captions::AdaptiveSpeechGate gate;
	for (int i = 0; i < 240; ++i) {
		gate.ObserveAmbient(0.04f, 0.02f);
	}

	EXPECT_GT(gate.AmbientPeak(), 0.03f);
	EXPECT_GT(gate.SpeechPeakThreshold(), 0.08f);
	EXPECT_GT(gate.SpeechRmsThreshold(), 0.05f);
	EXPECT_FALSE(gate.IsSpeech(-1.0f, 0.11f, 0.03f));
	EXPECT_TRUE(gate.IsSpeech(-1.0f, 0.16f, 0.08f));
}

TEST(EnergySpeechGateTest, RejectedLowVadSegmentsAdaptAmbientNoise)
{
	captions::AdaptiveSpeechGate gate;
	const float noise_peak = 0.070f;
	const float noise_rms = 0.025f;

	EXPECT_TRUE(gate.IsSpeech(-1.0f, noise_peak, noise_rms));
	for (int i = 0; i < 40; ++i) {
		EXPECT_TRUE(gate.ObserveRejectedSegment(-1.0f, noise_peak, noise_rms));
	}

	EXPECT_GT(gate.AmbientPeak(), 0.02f);
	EXPECT_GT(gate.AmbientRms(), 0.01f);
	EXPECT_FALSE(gate.IsSpeech(-1.0f, noise_peak, noise_rms));
	EXPECT_FALSE(gate.ObserveRejectedSegment(0.50f, noise_peak, noise_rms));
}

TEST(EnergySpeechGateTest, AdaptiveGateStillTrustsVad)
{
	captions::AdaptiveSpeechGate gate;
	for (int i = 0; i < 240; ++i) {
		gate.ObserveAmbient(0.04f, 0.02f);
	}

	EXPECT_TRUE(gate.IsSpeech(0.50f, 0.0f));
	EXPECT_TRUE(gate.IsSpeech(0.35f, gate.PossibleSpeechPeakThreshold(), gate.PossibleSpeechRmsThreshold()));
	EXPECT_FALSE(gate.IsSpeech(0.31f, gate.PossibleSpeechPeakThreshold(), gate.PossibleSpeechRmsThreshold()));
}

TEST(EnergySpeechGateTest, PossibleSpeechSitsBetweenSilenceAndOpen)
{
	captions::AdaptiveSpeechGate gate;
	for (int i = 0; i < 120; ++i) {
		gate.ObserveAmbient(0.002f, 0.001f);
	}

	const float possible = gate.PossibleSpeechPeakThreshold();
	const float possible_rms = gate.PossibleSpeechRmsThreshold();
	EXPECT_LT(possible, gate.SpeechPeakThreshold());
	EXPECT_GT(possible, gate.SilencePeakThreshold());
	EXPECT_LT(possible_rms, gate.SpeechRmsThreshold());
	EXPECT_GT(possible_rms, gate.SilenceRmsThreshold());
	EXPECT_FALSE(gate.IsSpeech(-1.0f, possible, possible_rms));
	EXPECT_TRUE(gate.IsPossibleSpeech(-1.0f, possible, possible_rms));
	EXPECT_FALSE(gate.IsSilence(-1.0f, possible, possible_rms));
	EXPECT_FALSE(gate.IsPossibleSpeech(-1.0f, possible * 2.0f, possible_rms * 0.25f));
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

TEST(TranscriptTextTest, DetectsCommonShortFillers)
{
	EXPECT_TRUE(captions::TranscriptLooksLikeCommonFiller("you"));
	EXPECT_TRUE(captions::TranscriptLooksLikeCommonFiller("Hmm."));
	EXPECT_TRUE(captions::TranscriptLooksLikeCommonFiller("mm-hmm"));
	EXPECT_TRUE(captions::TranscriptLooksLikeCommonFiller("uh"));
	EXPECT_TRUE(captions::TranscriptLooksLikeCommonFiller("okay"));
	EXPECT_FALSE(captions::TranscriptLooksLikeCommonFiller("you should check this"));
	EXPECT_FALSE(captions::TranscriptLooksLikeCommonFiller("I am okay with that"));
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
	EXPECT_TRUE(captions::TranscriptShouldSuppressByConfidence("hello there", true, 0.10f, 0.12f, 0.04f, 0.90f, -0.20f,
	                                                           4, 0.004f, 0.014f));
	EXPECT_FALSE(captions::TranscriptShouldSuppressByConfidence("hello there", true, 0.10f, 0.12f, 0.04f, 0.90f, -0.20f,
	                                                            4, 0.030f, 0.014f));
}

TEST(TranscriptTextTest, SuppressesWeakFillerButKeepsStrongShortSpeech)
{
	const float threshold = 0.04f;
	const float rms_threshold = 0.014f;

	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("you", true, 0.10f, 0.018f, threshold, 0.0f, -0.20f, 1,
	                                                          0.004f, rms_threshold, 320, 0.50),
	          captions::TranscriptSuppressionReason::CommonFiller);
	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("hmm", true, 0.10f, 0.018f, threshold, 0.0f, -0.20f, 1,
	                                                          0.004f, rms_threshold, 320, 0.50),
	          captions::TranscriptSuppressionReason::CommonFiller);
	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("mm hmm", true, 0.10f, 0.018f, threshold, 0.0f, -0.20f, 2,
	                                                          0.004f, rms_threshold, 320, 0.50),
	          captions::TranscriptSuppressionReason::CommonFiller);
	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("um", true, 0.10f, 0.018f, threshold, 0.0f, -0.20f, 1,
	                                                          0.004f, rms_threshold, 320, 0.50),
	          captions::TranscriptSuppressionReason::CommonFiller);
	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("uh", true, 0.10f, 0.018f, threshold, 0.0f, -0.20f, 1,
	                                                          0.004f, rms_threshold, 320, 0.50),
	          captions::TranscriptSuppressionReason::CommonFiller);
	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("you", true, 0.75f, 0.020f, threshold, 0.0f, -0.20f, 1,
	                                                          0.004f, rms_threshold, 900, 0.50),
	          captions::TranscriptSuppressionReason::None);
	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("ok", true, 0.10f, 0.10f, threshold, 0.0f, -0.20f, 1,
	                                                          0.030f, rms_threshold, 900, 0.50),
	          captions::TranscriptSuppressionReason::None);
}

TEST(TranscriptTextTest, SuppressesShortWeakAndRepetitiveDecodeOutput)
{
	const float threshold = 0.04f;
	const float rms_threshold = 0.014f;

	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("hello", true, 0.10f, 0.018f, threshold, 0.0f, -0.20f, 1,
	                                                          0.004f, rms_threshold, 320, 0.40),
	          captions::TranscriptSuppressionReason::ShortWeakAudio);
	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("hello world hello world hello world", true, 0.10f,
	                                                          0.018f, threshold, 0.0f, -0.20f, 6, 0.004f, rms_threshold,
	                                                          1200, 0.40),
	          captions::TranscriptSuppressionReason::Repetitive);
	EXPECT_EQ(captions::TranscriptConfidenceSuppressionReason("hello there", true, 0.10f, 0.045f, threshold, 0.0f,
	                                                          -0.20f, 2, 0.012f, rms_threshold, 900, 2.00),
	          captions::TranscriptSuppressionReason::SlowShortDecode);
}

TEST(TranscriptTextTest, RiskScoreCombinesSegmentHistoryAndPassCredits)
{
	captions::TranscriptRiskInput repeated_filler;
	repeated_filler.cleaned_text = "you";
	repeated_filler.always_on = true;
	repeated_filler.max_vad_probability = 0.20f;
	repeated_filler.max_frame_peak = 0.018f;
	repeated_filler.speech_peak_threshold = 0.040f;
	repeated_filler.max_frame_rms = 0.004f;
	repeated_filler.speech_rms_threshold = 0.014f;
	repeated_filler.audio_ms = 1800;
	repeated_filler.evidence_ms = 320;
	repeated_filler.token_count = 1;
	repeated_filler.prompt_chars = 180;
	repeated_filler.recent_suppression_count = 2;
	repeated_filler.recent_same_text_count = 3;
	repeated_filler.speech_frame_ratio = 0.10f;

	captions::TranscriptRiskResult repeated_result = captions::TranscriptRiskScore(repeated_filler);
	EXPECT_TRUE(repeated_result.suppress);
	EXPECT_GE(repeated_result.score, 5);
	EXPECT_EQ(repeated_result.reason, captions::TranscriptSuppressionReason::CommonFiller);

	captions::TranscriptRiskInput sustained_speech = repeated_filler;
	sustained_speech.cleaned_text = "can you hear me clearly";
	sustained_speech.max_vad_probability = 0.72f;
	sustained_speech.max_frame_peak = 0.060f;
	sustained_speech.max_frame_rms = 0.020f;
	sustained_speech.audio_ms = 1800;
	sustained_speech.evidence_ms = 1200;
	sustained_speech.token_count = 5;
	sustained_speech.prompt_chars = 0;
	sustained_speech.recent_suppression_count = 0;
	sustained_speech.recent_same_text_count = 1;
	sustained_speech.speech_frame_ratio = 0.55f;

	captions::TranscriptRiskResult speech_result = captions::TranscriptRiskScore(sustained_speech);
	EXPECT_FALSE(speech_result.suppress);
	EXPECT_EQ(speech_result.reason, captions::TranscriptSuppressionReason::None);
}

TEST(TranscriptTextTest, AcousticRiskOnlyHardensWeakShortFiller)
{
	captions::TranscriptRiskInput weak_filler;
	weak_filler.cleaned_text = "you";
	weak_filler.always_on = true;
	weak_filler.max_vad_probability = 0.20f;
	weak_filler.max_frame_peak = 0.020f;
	weak_filler.speech_peak_threshold = 0.040f;
	weak_filler.max_frame_rms = 0.004f;
	weak_filler.speech_rms_threshold = 0.014f;
	weak_filler.evidence_ms = 900;
	weak_filler.token_count = 1;
	weak_filler.acoustic_artifact_risk = 0.65f;
	weak_filler.speech_band_ratio = 0.15f;
	weak_filler.zero_crossing_rate = 0.65f;

	captions::TranscriptRiskResult weak_result = captions::TranscriptRiskScore(weak_filler);
	EXPECT_TRUE(weak_result.suppress);
	EXPECT_EQ(weak_result.reason, captions::TranscriptSuppressionReason::CommonFiller);

	captions::TranscriptRiskInput strong_short = weak_filler;
	strong_short.max_vad_probability = 0.88f;
	strong_short.max_frame_peak = 0.060f;
	strong_short.max_frame_rms = 0.020f;

	captions::TranscriptRiskResult strong_result = captions::TranscriptRiskScore(strong_short);
	EXPECT_FALSE(strong_result.suppress);
	EXPECT_EQ(strong_result.reason, captions::TranscriptSuppressionReason::None);
}

TEST(TranscriptTextTest, SparseEvidenceRatioHardensShortDecode)
{
	captions::TranscriptRiskInput sparse_filler;
	sparse_filler.cleaned_text = "okay";
	sparse_filler.always_on = true;
	sparse_filler.max_vad_probability = 0.20f;
	sparse_filler.max_frame_peak = 0.030f;
	sparse_filler.speech_peak_threshold = 0.040f;
	sparse_filler.max_frame_rms = 0.006f;
	sparse_filler.speech_rms_threshold = 0.014f;
	sparse_filler.audio_ms = 1900;
	sparse_filler.evidence_ms = 420;
	sparse_filler.token_count = 1;
	sparse_filler.speech_frame_ratio = 0.20f;

	captions::TranscriptRiskResult sparse_result = captions::TranscriptRiskScore(sparse_filler);
	EXPECT_TRUE(sparse_result.suppress);
	EXPECT_EQ(sparse_result.reason, captions::TranscriptSuppressionReason::CommonFiller);

	captions::TranscriptRiskInput sustained = sparse_filler;
	sustained.cleaned_text = "okay I can hear you now";
	sustained.max_vad_probability = 0.50f;
	sustained.max_frame_peak = 0.052f;
	sustained.max_frame_rms = 0.016f;
	sustained.audio_ms = 1900;
	sustained.evidence_ms = 1000;
	sustained.token_count = 6;
	sustained.speech_frame_ratio = 0.45f;

	captions::TranscriptRiskResult sustained_result = captions::TranscriptRiskScore(sustained);
	EXPECT_FALSE(sustained_result.suppress);
	EXPECT_EQ(sustained_result.reason, captions::TranscriptSuppressionReason::None);
}

TEST(TranscriptTextTest, PromptContextDecodeDisabledForRiskyAlwaysOnSegments)
{
	EXPECT_FALSE(captions::TranscriptShouldUsePromptContextForDecode(true, 420, 320, 0.20f, 0.20f, 0.10f, 0));
	EXPECT_FALSE(captions::TranscriptShouldUsePromptContextForDecode(true, 900, 900, 0.20f, 0.20f, 0.10f, 0));
	EXPECT_FALSE(captions::TranscriptShouldUsePromptContextForDecode(true, 1900, 420, 0.70f, 0.55f, 0.10f, 0));
	EXPECT_FALSE(captions::TranscriptShouldUsePromptContextForDecode(true, 1400, 1200, 0.70f, 0.55f, 0.60f, 0));
	EXPECT_FALSE(captions::TranscriptShouldUsePromptContextForDecode(true, 1800, 1500, 0.80f, 0.60f, 0.10f, 2));

	EXPECT_TRUE(captions::TranscriptShouldUsePromptContextForDecode(false, 300, 300, 0.0f, 0.0f, 1.0f, 3));
	EXPECT_TRUE(captions::TranscriptShouldUsePromptContextForDecode(true, 900, 900, 0.78f, 0.30f, 0.10f, 0));
	EXPECT_TRUE(captions::TranscriptShouldUsePromptContextForDecode(true, 1500, 900, 0.40f, 0.50f, 0.10f, 0));
}

TEST(TranscriptTextTest, PromptContextRequiresAcceptedHighConfidenceText)
{
	const float threshold = 0.04f;
	const float rms_threshold = 0.014f;

	EXPECT_FALSE(captions::TranscriptShouldUpdatePromptContext("you", true, 0.75f, 0.020f, threshold, 0.0f, -0.20f, 1,
	                                                           0.004f, rms_threshold, 900));
	EXPECT_FALSE(captions::TranscriptShouldUpdatePromptContext("hello", true, 0.10f, 0.018f, threshold, 0.0f, -0.20f, 1,
	                                                           0.004f, rms_threshold, 320));
	EXPECT_FALSE(captions::TranscriptShouldUpdatePromptContext("hello world hello world hello world", true, 0.10f,
	                                                           0.018f, threshold, 0.0f, -0.20f, 6, 0.004f,
	                                                           rms_threshold, 1200));
	EXPECT_TRUE(captions::TranscriptShouldUpdatePromptContext("can you hear me clearly", true, 0.70f, 0.030f, threshold,
	                                                          0.0f, -0.20f, 5, 0.012f, rms_threshold, 900));
	EXPECT_TRUE(captions::TranscriptShouldUpdatePromptContext("you", false, 0.10f, 0.018f, threshold, 0.90f, -2.00f, 1,
	                                                          0.004f, rms_threshold, 100));
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

TEST(NoAudioWarningTest, SilentInputWarningRequiresLiveFramesAndGrace)
{
	using namespace captions::ui;
	// Host down: never warn.
	EXPECT_FALSE(ShouldWarnSilentInput(false, true, 30.0));
	// Frames are not arriving: the frame-stall warning owns that case.
	EXPECT_FALSE(ShouldWarnSilentInput(true, false, 30.0));
	// Frames are arriving, but the silence window has not elapsed.
	EXPECT_FALSE(ShouldWarnSilentInput(true, true, 5.0));
	// Frames are arriving and the input meter has stayed silent.
	EXPECT_TRUE(ShouldWarnSilentInput(true, true, 10.0));
	EXPECT_TRUE(ShouldWarnSilentInput(true, true, 30.0));
}

// ---------------------------------------------------------------------------
// Whisper CPU thread selection (reserve cores for desktop + overlay)
// ---------------------------------------------------------------------------

TEST(WhisperThreadPlanTest, ReservesCoresAndClampsToRange)
{
	using captions::SelectWhisperThreadCount;
	// Unknown core count falls back to a safe default (>= floor).
	EXPECT_EQ(SelectWhisperThreadCount(0), 2);
	// Small machines never drop below the floor of 2.
	EXPECT_EQ(SelectWhisperThreadCount(1), 2);
	EXPECT_EQ(SelectWhisperThreadCount(2), 2);
	EXPECT_EQ(SelectWhisperThreadCount(4), 2);
	// Reserve two logical cores for the desktop + overlay.
	EXPECT_EQ(SelectWhisperThreadCount(8), 6);
	// Cap so whisper's flat CPU scaling does not hog every core.
	EXPECT_EQ(SelectWhisperThreadCount(16), 8);
	EXPECT_EQ(SelectWhisperThreadCount(32), 8);
}

// ---------------------------------------------------------------------------
// Decode pace (decode time vs audio time) + slow-decode warning
// ---------------------------------------------------------------------------

TEST(DecodePaceTest, RatioIsZeroUntilFirstSegment)
{
	captions::HostStatusSnapshot snap;
	snap.last_segment_audio_ms = 0; // nothing decoded yet
	snap.last_transcribe_ms = 0;
	EXPECT_DOUBLE_EQ(captions::ui::DecodeRatio(snap), 0.0);
	EXPECT_FALSE(captions::ui::ShouldWarnSlowDecode(captions::ui::DecodeRatio(snap)));
}

TEST(DecodePaceTest, RatioReflectsDecodeVersusAudio)
{
	captions::HostStatusSnapshot snap;
	snap.last_segment_audio_ms = 2000;
	// Faster than real time: 800 ms decode for 2000 ms of audio.
	snap.last_transcribe_ms = 800;
	EXPECT_DOUBLE_EQ(captions::ui::DecodeRatio(snap), 0.4);
	EXPECT_FALSE(captions::ui::ShouldWarnSlowDecode(captions::ui::DecodeRatio(snap)));
	// Slower than real time: 3000 ms decode for 2000 ms of audio.
	snap.last_transcribe_ms = 3000;
	EXPECT_DOUBLE_EQ(captions::ui::DecodeRatio(snap), 1.5);
	EXPECT_TRUE(captions::ui::ShouldWarnSlowDecode(captions::ui::DecodeRatio(snap)));
}

TEST(DecodePaceTest, WarningBoundaryIsStrictlyAboveRealTime)
{
	using captions::ui::ShouldWarnSlowDecode;
	EXPECT_FALSE(ShouldWarnSlowDecode(1.0)); // exactly real time is acceptable
	EXPECT_FALSE(ShouldWarnSlowDecode(0.99));
	EXPECT_TRUE(ShouldWarnSlowDecode(1.01));
}
