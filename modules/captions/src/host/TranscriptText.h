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
		const int prev_start = static_cast<int>(prev_words.size()) - overlap;
		for (int i = 0; i < overlap; ++i) {
			if (prev_words[static_cast<size_t>(prev_start + i)].normalized !=
			    cur_words[static_cast<size_t>(i)].normalized) {
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
