#include "ChatboxPacer.h"

ChatboxPacer::ChatboxPacer(double min_gap_sec) : min_gap_sec_(min_gap_sec) {}

void ChatboxPacer::Enqueue(const std::string& text, bool send_immediate, bool notify)
{
	if (queue_.size() >= kQueueCap) {
		// Drop the oldest pending entry to keep the most recent messages.
		queue_.pop_front();
	}
	queue_.push_back({text, send_immediate, notify});
}

bool ChatboxPacer::Dequeue(Entry& out)
{
	if (queue_.empty()) return false;

	const auto now = std::chrono::steady_clock::now();
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
