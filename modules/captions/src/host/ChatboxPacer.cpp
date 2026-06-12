#include "ChatboxPacer.h"

ChatboxPacer::ChatboxPacer(double min_gap_sec)
{
	SetMinGapSec(min_gap_sec);
}

void ChatboxPacer::SetMinGapSec(double min_gap_sec) noexcept
{
	min_gap_sec_ = min_gap_sec < 0.0 ? 0.0 : min_gap_sec;
}

void ChatboxPacer::Enqueue(const std::string& text, bool send_immediate, bool notify)
{
	if (queue_.size() >= kQueueCap) {
		// Drop the oldest pending entry to keep the most recent messages.
		queue_.pop_front();
	}
	queue_.push_back({text, send_immediate, notify});
}

void ChatboxPacer::Clear() noexcept
{
	queue_.clear();
	last_send_ = {};
	first_send_ = true;
}

bool ChatboxPacer::Dequeue(Entry& out)
{
	return DequeueAt(out, std::chrono::steady_clock::now());
}

bool ChatboxPacer::DequeueAt(Entry& out, std::chrono::steady_clock::time_point now)
{
	if (queue_.empty()) return false;

	if (!first_send_) {
		const double elapsed = std::chrono::duration<double>(now - last_send_).count();
		if (elapsed < min_gap_sec_) return false;
	}

	out = queue_.front();
	queue_.pop_front();
	last_send_ = now;
	first_send_ = false;
	return true;
}
