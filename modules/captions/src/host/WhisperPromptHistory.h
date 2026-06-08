#pragma once

#include <cstddef>
#include <string>

namespace captions {

class WhisperPromptHistory
{
public:
	explicit WhisperPromptHistory(size_t maxChars = 384) : max_chars_(maxChars) {}

	void Observe(const std::string& text)
	{
		const std::string trimmed = Trim(text);
		if (trimmed.empty()) return;

		if (!history_.empty()) history_ += " ";
		history_ += trimmed;
		TrimToBudget();
	}

	void Clear() { history_.clear(); }

	const std::string& Text() const { return history_; }

private:
	static bool IsSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

	static std::string Trim(const std::string& text)
	{
		size_t first = 0;
		while (first < text.size() && IsSpace(text[first])) {
			++first;
		}
		size_t last = text.size();
		while (last > first && IsSpace(text[last - 1])) {
			--last;
		}
		return text.substr(first, last - first);
	}

	void TrimToBudget()
	{
		if (history_.size() <= max_chars_) return;

		size_t keep_from = history_.size() - max_chars_;
		while (keep_from < history_.size() && !IsSpace(history_[keep_from])) {
			++keep_from;
		}
		while (keep_from < history_.size() && IsSpace(history_[keep_from])) {
			++keep_from;
		}
		if (keep_from >= history_.size()) {
			history_.clear();
			return;
		}
		history_.erase(0, keep_from);
	}

	size_t max_chars_;
	std::string history_;
};

} // namespace captions
