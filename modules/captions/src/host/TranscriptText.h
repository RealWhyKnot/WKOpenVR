#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <string>
#include <vector>

namespace captions {

inline bool TranscriptIsWordChar(unsigned char ch)
{
	return std::isalnum(ch) != 0;
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
