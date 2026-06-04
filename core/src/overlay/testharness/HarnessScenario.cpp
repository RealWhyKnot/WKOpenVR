#include "HarnessScenario.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <chrono>
#include <cstdio>

namespace openvr_pair::overlay::testharness {

uint64_t BarrierQueue::Push(MockCall call)
{
	const uint64_t seq = next_seq_.fetch_add(1, std::memory_order_relaxed);
	call.seq = seq;
	if (call.qpc_ticks == 0) {
		call.qpc_ticks = QpcNow();
	}
	{
		std::lock_guard<std::mutex> lock(mu_);
		calls_.push_back(std::move(call));
	}
	cv_.notify_all();
	return seq;
}

std::vector<MockCall> BarrierQueue::Snapshot() const
{
	std::lock_guard<std::mutex> lock(mu_);
	return std::vector<MockCall>(calls_.begin(), calls_.end());
}

BarrierQueue::WaitOutcome BarrierQueue::WaitFor(std::function<bool(const MockCall&)> predicate,
                                                std::chrono::milliseconds timeout, uint64_t& cursor)
{
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	std::unique_lock<std::mutex> lock(mu_);
	for (;;) {
		// Walk forward from cursor.
		for (auto it = calls_.begin(); it != calls_.end(); ++it) {
			if (it->seq <= cursor) continue;
			if (predicate(*it)) {
				cursor = it->seq;
				return WaitOutcome{true, *it};
			}
		}
		// Nothing yet -- block until a new call arrives or timeout.
		const auto status = cv_.wait_until(lock, deadline);
		if (status == std::cv_status::timeout) {
			return WaitOutcome{false, MockCall{}};
		}
	}
}

size_t BarrierQueue::CountSince(std::function<bool(const MockCall&)> predicate, uint64_t& cursor) const
{
	std::lock_guard<std::mutex> lock(mu_);
	size_t hits = 0;
	uint64_t high_water = cursor;
	for (const auto& call : calls_) {
		if (call.seq <= cursor) continue;
		if (predicate(call)) {
			++hits;
			if (call.seq > high_water) high_water = call.seq;
		}
	}
	cursor = high_water;
	return hits;
}

void BarrierQueue::Clear()
{
	std::lock_guard<std::mutex> lock(mu_);
	calls_.clear();
}

uint64_t BarrierQueue::LatestSeq() const
{
	std::lock_guard<std::mutex> lock(mu_);
	return calls_.empty() ? 0 : calls_.back().seq;
}

HarnessLogger::HarnessLogger(std::string scenario_name) : name_(std::move(scenario_name)) {}

void HarnessLogger::Info(std::string msg)
{
	std::fprintf(stdout, "[%s][info] %s\n", name_.c_str(), msg.c_str());
	std::fflush(stdout);
}

void HarnessLogger::Warn(std::string msg)
{
	std::fprintf(stdout, "[%s][warn] %s\n", name_.c_str(), msg.c_str());
	std::fflush(stdout);
}

void HarnessLogger::Error(std::string msg)
{
	std::fprintf(stdout, "[%s][error] %s\n", name_.c_str(), msg.c_str());
	std::fflush(stdout);
}

void HarnessLogger::Step(std::string msg)
{
	std::fprintf(stdout, "[%s][step] %s\n", name_.c_str(), msg.c_str());
	std::fflush(stdout);
}

ScenarioResult Pass(const std::string& name, std::chrono::milliseconds duration)
{
	ScenarioResult r;
	r.name = name;
	r.passed = true;
	r.duration = duration;
	return r;
}

ScenarioResult Fail(const std::string& name, std::chrono::milliseconds duration, std::string reason)
{
	ScenarioResult r;
	r.name = name;
	r.passed = false;
	r.failure_reason = std::move(reason);
	r.duration = duration;
	return r;
}

uint64_t QpcNow()
{
	LARGE_INTEGER li{};
	if (::QueryPerformanceCounter(&li)) return (uint64_t)li.QuadPart;
	return 0;
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
