#pragma once

#include <chrono>
#include <deque>
#include <string>

// Enforces VRChat's chatbox rate limit: a minimum 1.2-second gap between sends.
// Uses a simple token-bucket model; the queue holds at most kQueueCap entries.
// When the queue fills, the oldest pending entry is dropped (newest wins).
class ChatboxPacer
{
public:
	struct Entry
	{
		std::string text;
		bool send_immediate;
		bool notify;
	};

	explicit ChatboxPacer(double min_gap_sec = 1.2);

	// Push a new chatbox message into the queue. If the queue is full, the
	// oldest pending entry is evicted (not the current message).
	void Enqueue(const std::string& text, bool send_immediate, bool notify);

	// Returns true and writes the next entry to `out` if the minimum gap has
	// elapsed and the queue is non-empty. Otherwise returns false.
	bool Dequeue(Entry& out);

	bool IsEmpty() const noexcept { return queue_.empty(); }
	size_t QueueSize() const noexcept { return queue_.size(); }

private:
	static constexpr size_t kQueueCap = 8;
	double min_gap_sec_;
	std::deque<Entry> queue_;
	std::chrono::steady_clock::time_point last_send_{};
	bool first_send_ = true;
};
