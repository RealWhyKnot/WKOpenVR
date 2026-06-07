#pragma once

#include <cstddef>
#include <deque>
#include <string>

namespace captions {

struct CaptionPreviewEntry
{
	long long sequence = 0;
	std::string transcript;
	std::string translation;
};

class CaptionPreviewHistory
{
public:
	explicit CaptionPreviewHistory(size_t max_entries = 6) : max_entries_(max_entries == 0 ? 1 : max_entries) {}

	void Observe(long long sequence, const std::string& transcript, const std::string& translation)
	{
		if (sequence <= 0 || sequence == last_sequence_) return;
		last_sequence_ = sequence;
		if (transcript.empty() && translation.empty()) return;

		while (entries_.size() >= max_entries_) {
			entries_.pop_front();
		}
		entries_.push_back({sequence, transcript, translation});
	}

	void Clear() { entries_.clear(); }

	const std::deque<CaptionPreviewEntry>& Entries() const noexcept { return entries_; }

private:
	size_t max_entries_;
	long long last_sequence_ = 0;
	std::deque<CaptionPreviewEntry> entries_;
};

} // namespace captions
