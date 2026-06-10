#pragma once

#include "ModulePerf.h"
#include "PerfHistory.h"

#include <cstdint>
#include <memory>

namespace protocol {
class PerfStatsShmem;
}

namespace openvr_pair::overlay {

// Owns the overlay-side perf pipeline: samples this process at 1 Hz,
// mirrors the driver host's stats from the always-on perf shmem segment,
// maintains the rolling graph history, feeds the runtime health summary,
// and emits the 10 s [perf] diagnostics-log lines. The main loop calls
// Tick() every frame; everything inside is throttled.
class PerfStatsHub
{
public:
	~PerfStatsHub();

	void Tick(double nowSeconds);

	const PerfHistoryStore& History() const { return history_; }
	const PerfViewModel& ViewModel() const { return viewModel_; }

	// True once the driver's perf segment has been mapped (any driver run
	// this boot); ViewModel().driverConnected adds the freshness check.
	bool DriverSegmentOpen() const { return static_cast<bool>(shmem_); }

	// Smoothed per-slot totals for the table; raw values feed the graphs.
	double SmoothedTotalPct(uint32_t slot) const
	{
		return slot < moduleperf::kSlotCount ? smoothedTotalPct_[slot] : 0.0;
	}

private:
	bool TryOpenDriverSegment();
	void EmitLogLines(const moduleperf::PerfSampleResult& overlaySample);

	moduleperf::Sampler sampler_{1000};
	std::unique_ptr<::protocol::PerfStatsShmem> shmem_;
	uint64_t lastDriverSampleIndex_ = 0;

	PerfHistoryStore history_;
	PerfViewModel viewModel_;
	moduleperf::PerfSampleResult lastOverlaySample_{};
	moduleperf::PerfSampleResult lastDriverSample_{};

	double smoothedTotalPct_[moduleperf::kSlotCount]{};
	bool smoothedValid_[moduleperf::kSlotCount]{};

	uint64_t lastLogWallMs_ = 0;
};

PerfStatsHub& GetPerfStatsHub();

} // namespace openvr_pair::overlay
