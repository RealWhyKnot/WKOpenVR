#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <string>
#include <vector>

namespace captions {

inline bool TranscriptIsWordChar(unsigned char ch)
{
	return std::isalnum(ch) != 0;
}

inline std::string TranscriptTrimAscii(const std::string& text)
{
	size_t begin = 0;
	while (begin < text.size()) {
		const unsigned char ch = static_cast<unsigned char>(text[begin]);
		if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') break;
		++begin;
	}
	size_t end = text.size();
	while (end > begin) {
		const unsigned char ch = static_cast<unsigned char>(text[end - 1]);
		if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') break;
		--end;
	}
	return text.substr(begin, end - begin);
}

inline std::string TranscriptLowerWords(const std::string& text)
{
	std::string out;
	bool pending_space = false;
	for (unsigned char ch : text) {
		if (TranscriptIsWordChar(ch)) {
			if (pending_space && !out.empty()) out.push_back(' ');
			out.push_back(static_cast<char>(std::tolower(ch)));
			pending_space = false;
		}
		else {
			pending_space = !out.empty();
		}
	}
	return out;
}

inline bool TranscriptIsKnownNonSpeechMarker(const std::string& text)
{
	const std::string normalized = TranscriptLowerWords(text);
	static constexpr std::array<const char*, 9> kMarkers = {
	    "music", "laughter", "laughs", "laughing", "applause", "clapping", "silence", "inaudible", "background noise"};
	for (const char* marker : kMarkers) {
		if (normalized == marker) return true;
	}
	return false;
}

inline std::string RemoveKnownNonSpeechMarkers(std::string text)
{
	size_t pos = 0;
	while (pos < text.size()) {
		const size_t open = text.find('[', pos);
		if (open == std::string::npos) break;
		const size_t close = text.find(']', open + 1);
		if (close == std::string::npos) break;
		const std::string marker = text.substr(open + 1, close - open - 1);
		if (TranscriptIsKnownNonSpeechMarker(marker)) {
			text.erase(open, close - open + 1);
			pos = open;
		}
		else {
			pos = close + 1;
		}
	}
	return text;
}

inline std::string CollapseTranscriptWhitespace(const std::string& text)
{
	std::string out;
	bool pending_space = false;
	for (unsigned char ch : text) {
		if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
			pending_space = !out.empty();
			continue;
		}
		if (pending_space) out.push_back(' ');
		out.push_back(static_cast<char>(ch));
		pending_space = false;
	}
	return TranscriptTrimAscii(out);
}

inline std::string CleanTranscriptForPublish(const std::string& text)
{
	return CollapseTranscriptWhitespace(RemoveKnownNonSpeechMarkers(text));
}

inline bool TranscriptLooksLikeCommonHallucination(const std::string& text)
{
	const std::string normalized = TranscriptLowerWords(text);
	static constexpr std::array<const char*, 8> kPhrases = {
	    "thanks for watching", "thank you for watching", "please subscribe", "subscribe to my channel",
	    "like and subscribe",  "see you next time",      "foreign",          "subtitles by"};
	for (const char* phrase : kPhrases) {
		if (normalized == phrase) return true;
	}
	return false;
}

inline bool TranscriptLooksLikeCommonFiller(const std::string& text)
{
	const std::string normalized = TranscriptLowerWords(text);
	static constexpr std::array<const char*, 19> kPhrases = {"you", "hmm", "hmmm", "hm",    "mm",  "mmm",    "mm hmm",
	                                                         "mhm", "um",  "umm",  "uh",    "uhh", "uh huh", "yeah",
	                                                         "yep", "ok",  "okay", "right", "oh"};
	for (const char* phrase : kPhrases) {
		if (normalized == phrase) return true;
	}
	return false;
}

struct TranscriptWordSpan
{
	std::string normalized;
	size_t begin = 0;
	size_t end = 0;
};

inline std::vector<TranscriptWordSpan> TranscriptWords(const std::string& text)
{
	std::vector<TranscriptWordSpan> words;
	size_t pos = 0;
	while (pos < text.size()) {
		while (pos < text.size() && !TranscriptIsWordChar(static_cast<unsigned char>(text[pos]))) {
			++pos;
		}
		if (pos >= text.size()) break;

		TranscriptWordSpan span;
		span.begin = pos;
		while (pos < text.size() && TranscriptIsWordChar(static_cast<unsigned char>(text[pos]))) {
			const auto ch = static_cast<unsigned char>(text[pos]);
			span.normalized.push_back(static_cast<char>(std::tolower(ch)));
			++pos;
		}
		span.end = pos;
		if (!span.normalized.empty()) words.push_back(span);
	}
	return words;
}

inline int TranscriptEffectiveTokenCount(const std::string& text, int tokenCount)
{
	if (tokenCount > 0) return tokenCount;
	return static_cast<int>(TranscriptWords(text).size());
}

inline bool TranscriptLooksRepetitive(const std::string& text)
{
	std::vector<std::string> words;
	for (const auto& span : TranscriptWords(text)) {
		words.push_back(span.normalized);
	}
	if (words.size() < 6) return false;

	int run = 1;
	for (size_t i = 1; i < words.size(); ++i) {
		if (words[i] == words[i - 1]) {
			++run;
			if (run >= 6) return true;
		}
		else {
			run = 1;
		}
	}

	for (size_t phraseLen = 2; phraseLen <= words.size() / 2; ++phraseLen) {
		if (words.size() % phraseLen != 0) continue;
		bool repeats = true;
		for (size_t i = phraseLen; i < words.size(); ++i) {
			if (words[i] != words[i % phraseLen]) {
				repeats = false;
				break;
			}
		}
		if (repeats) return true;
	}

	return false;
}

inline bool TranscriptHasStrongAudioEvidence(float maxVadProbability, float maxFramePeak, float speechPeakThreshold,
                                             float maxFrameRms = 1.0f, float speechRmsThreshold = 0.0f)
{
	return maxVadProbability >= 0.55f || (maxFramePeak >= std::max(0.045f, speechPeakThreshold * 1.35f) &&
	                                      maxFrameRms >= std::max(0.012f, speechRmsThreshold));
}

inline bool TranscriptHasWeakAudioEvidence(float maxVadProbability, float maxFramePeak, float speechPeakThreshold,
                                           float maxFrameRms = 1.0f, float speechRmsThreshold = 0.0f)
{
	return maxVadProbability < 0.38f && (maxFramePeak < std::max(0.032f, speechPeakThreshold * 1.10f) ||
	                                     maxFrameRms < std::max(0.008f, speechRmsThreshold * 0.80f));
}

inline float TranscriptNoSpeechProbabilityThreshold()
{
	return 0.60f;
}

inline float TranscriptNoSpeechAverageLogProbabilityThreshold()
{
	return -1.00f;
}

inline bool TranscriptShouldSuppressByNoSpeechProbability(const std::string& cleanedText, bool alwaysOn,
                                                          float noSpeechProbability, float averageTokenLogProbability)
{
	if (!alwaysOn || cleanedText.empty()) return false;
	return noSpeechProbability >= TranscriptNoSpeechProbabilityThreshold() &&
	       averageTokenLogProbability < TranscriptNoSpeechAverageLogProbabilityThreshold();
}

enum class TranscriptSuppressionReason
{
	None,
	CommonHallucination,
	CommonFiller,
	NoSpeechProbability,
	ShortWeakAudio,
	Repetitive,
	LowConfidence,
	SlowShortDecode,
};

inline const char* TranscriptSuppressionReasonName(TranscriptSuppressionReason reason)
{
	switch (reason) {
		case TranscriptSuppressionReason::CommonHallucination:
			return "common-hallucination";
		case TranscriptSuppressionReason::CommonFiller:
			return "common-filler";
		case TranscriptSuppressionReason::NoSpeechProbability:
			return "no-speech-probability";
		case TranscriptSuppressionReason::ShortWeakAudio:
			return "short-weak-audio";
		case TranscriptSuppressionReason::Repetitive:
			return "repetitive";
		case TranscriptSuppressionReason::LowConfidence:
			return "low-confidence";
		case TranscriptSuppressionReason::SlowShortDecode:
			return "slow-short-decode";
		case TranscriptSuppressionReason::None:
		default:
			return "";
	}
}

struct TranscriptRiskInput
{
	std::string cleaned_text;
	bool always_on = false;
	float max_vad_probability = -1.0f;
	float max_frame_peak = 0.0f;
	float speech_peak_threshold = 0.0f;
	float no_speech_probability = 0.0f;
	float average_token_log_probability = 0.0f;
	int token_count = 0;
	float max_frame_rms = 1.0f;
	float speech_rms_threshold = 0.0f;
	long long evidence_ms = 0;
	double decode_ratio = 0.0;
	size_t prompt_chars = 0;
	int recent_suppression_count = 0;
	int recent_same_text_count = 0;
	float speech_frame_ratio = -1.0f;
	float acoustic_artifact_risk = 0.0f;
	float speech_band_ratio = -1.0f;
	float zero_crossing_rate = -1.0f;
	float clipping_ratio = 0.0f;
};

struct TranscriptRiskResult
{
	int score = 0;
	int pass_credit = 0;
	TranscriptSuppressionReason reason = TranscriptSuppressionReason::None;
	bool suppress = false;
};

inline TranscriptRiskResult TranscriptRiskScore(const TranscriptRiskInput& in)
{
	TranscriptRiskResult result;
	if (!in.always_on || in.cleaned_text.empty()) return result;

	const bool strong_audio = TranscriptHasStrongAudioEvidence(
	    in.max_vad_probability, in.max_frame_peak, in.speech_peak_threshold, in.max_frame_rms, in.speech_rms_threshold);
	const bool weak_audio = TranscriptHasWeakAudioEvidence(
	    in.max_vad_probability, in.max_frame_peak, in.speech_peak_threshold, in.max_frame_rms, in.speech_rms_threshold);
	const int effective_token_count = TranscriptEffectiveTokenCount(in.cleaned_text, in.token_count);
	const bool short_text = effective_token_count > 0 && effective_token_count <= 4;
	const bool very_short_evidence = in.evidence_ms > 0 && in.evidence_ms < 500;
	const bool short_evidence = in.evidence_ms > 0 && in.evidence_ms < 700;
	const bool filler = TranscriptLooksLikeCommonFiller(in.cleaned_text);
	const bool poor_acoustic_shape =
	    in.acoustic_artifact_risk >= 0.55f ||
	    (in.speech_band_ratio >= 0.0f && in.speech_band_ratio < 0.24f && in.evidence_ms > 0 && in.evidence_ms < 1000) ||
	    in.clipping_ratio > 0.04f;

	auto add_risk = [&](int points, TranscriptSuppressionReason reason) {
		result.score += points;
		if (result.reason == TranscriptSuppressionReason::None) {
			result.reason = reason;
		}
	};

	if (TranscriptLooksLikeCommonHallucination(in.cleaned_text) && !strong_audio) {
		add_risk(6, TranscriptSuppressionReason::CommonHallucination);
	}
	if (filler && short_evidence) {
		add_risk(3, TranscriptSuppressionReason::CommonFiller);
	}
	if (filler && !strong_audio) {
		add_risk(2, TranscriptSuppressionReason::CommonFiller);
	}
	if (in.no_speech_probability >= 0.85f && !strong_audio) {
		add_risk(2, TranscriptSuppressionReason::NoSpeechProbability);
	}
	if (weak_audio && in.no_speech_probability >= 0.65f) {
		add_risk(2, TranscriptSuppressionReason::NoSpeechProbability);
	}
	if (short_text && !strong_audio && (weak_audio || very_short_evidence)) {
		add_risk(5, TranscriptSuppressionReason::ShortWeakAudio);
	}
	if (short_text && !strong_audio && poor_acoustic_shape) {
		add_risk(2, TranscriptSuppressionReason::ShortWeakAudio);
	}
	if (filler && !strong_audio && in.acoustic_artifact_risk >= 0.45f) {
		add_risk(1, TranscriptSuppressionReason::CommonFiller);
	}
	if (short_text && very_short_evidence && !strong_audio && in.acoustic_artifact_risk >= 0.35f) {
		add_risk(1, TranscriptSuppressionReason::ShortWeakAudio);
	}
	if (in.token_count > 0 && in.average_token_log_probability < -1.35f && in.no_speech_probability >= 0.45f &&
	    !strong_audio) {
		add_risk(5, TranscriptSuppressionReason::LowConfidence);
	}
	if (TranscriptLooksRepetitive(in.cleaned_text) && !strong_audio) {
		add_risk(5, TranscriptSuppressionReason::Repetitive);
	}
	if (short_text && in.decode_ratio >= 1.75 && !strong_audio) {
		add_risk(5, TranscriptSuppressionReason::SlowShortDecode);
	}
	if (in.speech_frame_ratio >= 0.0f && in.speech_frame_ratio < 0.25f && in.evidence_ms > 0 && in.evidence_ms < 1000) {
		add_risk(1, TranscriptSuppressionReason::ShortWeakAudio);
	}
	if (in.prompt_chars > 128 && short_text) {
		add_risk(1, TranscriptSuppressionReason::ShortWeakAudio);
	}
	if (in.recent_suppression_count >= 2) {
		add_risk(1, TranscriptSuppressionReason::ShortWeakAudio);
	}
	if (filler && in.recent_same_text_count >= 2) {
		add_risk(2, TranscriptSuppressionReason::CommonFiller);
	}

	if (in.max_vad_probability >= 0.70f) {
		result.pass_credit += 3;
	}
	if (in.speech_frame_ratio >= 0.0f && in.evidence_ms >= 900 && in.speech_frame_ratio >= 0.40f) {
		result.pass_credit += 2;
	}
	if (strong_audio && in.evidence_ms >= 700) {
		result.pass_credit += 2;
	}
	if (in.speech_band_ratio >= 0.40f && in.acoustic_artifact_risk < 0.35f && in.evidence_ms >= 700) {
		result.pass_credit += 1;
	}

	result.score = std::max(0, result.score - result.pass_credit);
	result.suppress = result.score >= 5;
	if (!result.suppress) {
		result.reason = TranscriptSuppressionReason::None;
	}
	return result;
}

inline TranscriptSuppressionReason TranscriptConfidenceSuppressionReason(
    const std::string& cleanedText, bool alwaysOn, float maxVadProbability, float maxFramePeak,
    float speechPeakThreshold, float noSpeechProbability, float averageTokenLogProbability, int tokenCount,
    float maxFrameRms = 1.0f, float speechRmsThreshold = 0.0f, long long evidenceMs = 0, double decodeRatio = 0.0)
{
	TranscriptRiskInput input;
	input.cleaned_text = cleanedText;
	input.always_on = alwaysOn;
	input.max_vad_probability = maxVadProbability;
	input.max_frame_peak = maxFramePeak;
	input.speech_peak_threshold = speechPeakThreshold;
	input.no_speech_probability = noSpeechProbability;
	input.average_token_log_probability = averageTokenLogProbability;
	input.token_count = tokenCount;
	input.max_frame_rms = maxFrameRms;
	input.speech_rms_threshold = speechRmsThreshold;
	input.evidence_ms = evidenceMs;
	input.decode_ratio = decodeRatio;
	return TranscriptRiskScore(input).reason;
}

inline bool TranscriptShouldSuppressByConfidence(const std::string& cleanedText, bool alwaysOn, float maxVadProbability,
                                                 float maxFramePeak, float speechPeakThreshold,
                                                 float noSpeechProbability, float averageTokenLogProbability,
                                                 int tokenCount, float maxFrameRms = 1.0f,
                                                 float speechRmsThreshold = 0.0f, long long evidenceMs = 0,
                                                 double decodeRatio = 0.0)
{
	return TranscriptConfidenceSuppressionReason(cleanedText, alwaysOn, maxVadProbability, maxFramePeak,
	                                             speechPeakThreshold, noSpeechProbability, averageTokenLogProbability,
	                                             tokenCount, maxFrameRms, speechRmsThreshold, evidenceMs,
	                                             decodeRatio) != TranscriptSuppressionReason::None;
}

inline bool TranscriptShouldUpdatePromptContext(const std::string& cleanedText, bool alwaysOn, float maxVadProbability,
                                                float maxFramePeak, float speechPeakThreshold,
                                                float noSpeechProbability, float averageTokenLogProbability,
                                                int tokenCount, float maxFrameRms = 1.0f,
                                                float speechRmsThreshold = 0.0f, long long evidenceMs = 0)
{
	if (cleanedText.empty()) return false;
	if (!alwaysOn) return true;

	const bool strong_audio = TranscriptHasStrongAudioEvidence(maxVadProbability, maxFramePeak, speechPeakThreshold,
	                                                           maxFrameRms, speechRmsThreshold);
	const int effective_token_count = TranscriptEffectiveTokenCount(cleanedText, tokenCount);
	if (TranscriptLooksLikeCommonHallucination(cleanedText)) return false;
	if (TranscriptLooksLikeCommonFiller(cleanedText) && effective_token_count <= 3) return false;
	if (TranscriptLooksRepetitive(cleanedText)) return false;
	if (averageTokenLogProbability < -1.05f) return false;
	if (noSpeechProbability >= 0.55f && averageTokenLogProbability < -0.50f) return false;
	if (evidenceMs > 0 && evidenceMs < 500 && !strong_audio) return false;
	if (effective_token_count > 0 && effective_token_count <= 2 && !strong_audio) return false;

	return true;
}

inline std::string TrimTranscriptLeadingSeparators(const std::string& text, size_t start)
{
	while (start < text.size() && !TranscriptIsWordChar(static_cast<unsigned char>(text[start]))) {
		++start;
	}
	return start < text.size() ? text.substr(start) : std::string();
}

inline std::string RemoveOverlappingTranscriptPrefix(const std::string& previous, const std::string& current,
                                                     int maxWords = 8)
{
	if (previous.empty() || current.empty() || maxWords < 2) return current;

	const std::vector<TranscriptWordSpan> prev_words = TranscriptWords(previous);
	const std::vector<TranscriptWordSpan> cur_words = TranscriptWords(current);
	const int max_overlap =
	    std::min({maxWords, static_cast<int>(prev_words.size()), static_cast<int>(cur_words.size())});

	for (int overlap = max_overlap; overlap >= 2; --overlap) {
		bool match = true;
		const size_t prev_start = prev_words.size() - static_cast<size_t>(overlap);
		for (int i = 0; i < overlap; ++i) {
			const size_t offset = static_cast<size_t>(i);
			if (prev_words[prev_start + offset].normalized != cur_words[offset].normalized) {
				match = false;
				break;
			}
		}
		if (match) {
			return TrimTranscriptLeadingSeparators(current, cur_words[static_cast<size_t>(overlap - 1)].end);
		}
	}

	return current;
}

} // namespace captions
