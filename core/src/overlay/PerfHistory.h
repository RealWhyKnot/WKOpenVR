#pragma once

#include "ModulePerf.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

// Pure data layer behind the Modules-tab performance card: rolling series
// for the graphs plus the merged per-module view model for the table. No
// ImGui or Win32 calls here so the whole layer is unit-testable.
namespace openvr_pair::overlay {

namespace moduleperf = openvr_pair::common::moduleperf;

// Rolling (timestamp, value) series. Time-based eviction over a fixed
// window plus a defensive size cap, same shape as the calibration module's
// metrics buffers.
class PerfTimeSeries
{
public:
	static constexpr double kWindowSeconds = 120.0;
	static constexpr size_t kMaxPoints = 4096;

	void Push(double timestamp, float value)
	{
		points_.emplace_back(timestamp, value);
		const double cutoff = timestamp - kWindowSeconds;
		while (!points_.empty() && points_.front().first < cutoff) {
			points_.pop_front();
		}
		while (points_.size() > kMaxPoints) {
			points_.pop_front();
		}
	}

	void Clear() { points_.clear(); }
	bool Empty() const { return points_.empty(); }
	size_t Size() const { return points_.size(); }
	const std::pair<double, float>& At(size_t index) const { return points_[index]; }
	float Latest() const { return points_.empty() ? 0.0f : points_.back().second; }

	float Max() const
	{
		float best = 0.0f;
		for (const auto& p : points_) {
			best = std::max(best, p.second);
		}
		return best;
	}

private:
	std::deque<std::pair<double, float>> points_;
};

// Series for one source process (the overlay itself, or the driver host as
// mirrored over shmem). Module series carry section + thread time only;
// sidecar processes get their own series below since they are neither.
struct PerfSeriesSet
{
	PerfTimeSeries totalCpuPctOneCore;
	PerfTimeSeries workingSetMb;
	PerfTimeSeries unattributedPctOneCore;
	PerfTimeSeries moduleCpuPctOneCore[moduleperf::kSlotCount];
};

struct PerfHistoryStore
{
	PerfSeriesSet overlay;
	PerfSeriesSet driver;
	PerfTimeSeries sidecarModuleCpuPctOneCore[moduleperf::kSlotCount];
	PerfTimeSeries sidecarTotalCpuPctOneCore;
};

inline double ModuleInProcessPct(const moduleperf::ModuleSample& m)
{
	return moduleperf::ModuleInProcessCpuPercentOneCore(m);
}

// Process total minus everything attributed to modules, floored at zero
// (scheduler-tick quantization can push the module sum past the process
// total by a hair). This is the runtime's own cost: render loop, SteamVR
// callbacks, shell UI.
inline double UnattributedPct(const moduleperf::PerfSampleResult& sample)
{
	return moduleperf::UnattributedProcessCpuPercentOneCore(sample);
}

// Appends one 1 Hz sample to the rolling series. Pushes nothing for a
// source whose CPU delta is not valid yet (first sample after start), so
// the graphs never show a bogus zero spike.
inline void PushPerfHistory(PerfHistoryStore& store, double nowSeconds, const moduleperf::PerfSampleResult& overlay,
                            bool haveDriver, const moduleperf::PerfSampleResult& driver)
{
	if (overlay.process.cpuValid) {
		store.overlay.totalCpuPctOneCore.Push(nowSeconds, static_cast<float>(overlay.process.cpuPctOneCore));
		store.overlay.unattributedPctOneCore.Push(nowSeconds, static_cast<float>(UnattributedPct(overlay)));
		for (uint32_t slot = 0; slot < moduleperf::kSlotCount; ++slot) {
			store.overlay.moduleCpuPctOneCore[slot].Push(nowSeconds,
			                                             static_cast<float>(ModuleInProcessPct(overlay.modules[slot])));
		}
	}
	if (overlay.process.snapshot.memoryValid) {
		store.overlay.workingSetMb.Push(nowSeconds, static_cast<float>(overlay.process.snapshot.workingSetBytes) /
		                                                (1024.0f * 1024.0f));
	}

	if (!haveDriver) return;

	if (driver.process.cpuValid) {
		store.driver.totalCpuPctOneCore.Push(nowSeconds, static_cast<float>(driver.process.cpuPctOneCore));
		store.driver.unattributedPctOneCore.Push(nowSeconds, static_cast<float>(UnattributedPct(driver)));
		double sidecarTotal = 0.0;
		for (uint32_t slot = 0; slot < moduleperf::kSlotCount; ++slot) {
			const moduleperf::ModuleSample& m = driver.modules[slot];
			store.driver.moduleCpuPctOneCore[slot].Push(nowSeconds, static_cast<float>(ModuleInProcessPct(m)));
			store.sidecarModuleCpuPctOneCore[slot].Push(nowSeconds, static_cast<float>(m.sidecarCpuPctOneCore));
			sidecarTotal += m.sidecarCpuPctOneCore;
		}
		store.sidecarTotalCpuPctOneCore.Push(nowSeconds, static_cast<float>(sidecarTotal));
	}
	if (driver.process.snapshot.memoryValid) {
		store.driver.workingSetMb.Push(nowSeconds, static_cast<float>(driver.process.snapshot.workingSetBytes) /
		                                               (1024.0f * 1024.0f));
	}
}

// One table row: a module's cost across every process that runs its code.
struct PerfModuleRow
{
	moduleperf::ModuleId id{};
	bool overlayActive = false;
	bool driverActive = false;
	bool sidecarPresent = false;
	double overlayPct = 0.0; // section + thread time inside WKOpenVR.exe
	double driverPct = 0.0;  // section + thread time inside the driver host
	double sidecarPct = 0.0; // sidecar whole-process CPU
	double totalPct = 0.0;   // sum of the three, raw
	uint32_t overlayThreads = 0;
	uint32_t driverThreads = 0;
	uint32_t sidecarThreads = 0;
	uint32_t sidecarPid = 0;
	uint32_t sidecarProcessCount = 0;
	uint64_t sidecarWorkingSetBytes = 0;
};

struct PerfViewModel
{
	bool driverConnected = false;
	double driverSnapshotAgeSec = 0.0;

	std::vector<PerfModuleRow> rows;

	double overlayTotalPct = 0.0;
	double driverTotalPct = 0.0;
	double sidecarTotalPct = 0.0;
	double overlayUnattributedPct = 0.0;
	double driverUnattributedPct = 0.0;

	double overlayWorkingSetMb = 0.0;
	double driverWorkingSetMb = 0.0;
	uint32_t overlayThreadCount = 0;
	uint32_t driverThreadCount = 0;
	uint32_t overlayHandleCount = 0;
	uint32_t driverHandleCount = 0;
	uint32_t overlayPid = 0;
	uint32_t driverPid = 0;
};

// Merges the overlay's own latest sample with the driver's mirrored one.
// Rows appear for every module either process has seen activity from, in
// ModuleId order, so row order and series colors stay stable.
inline PerfViewModel BuildPerfViewModel(const moduleperf::PerfSampleResult& overlay, bool driverConnected,
                                        const moduleperf::PerfSampleResult& driver, double driverSnapshotAgeSec)
{
	PerfViewModel vm;
	vm.driverConnected = driverConnected;
	vm.driverSnapshotAgeSec = driverSnapshotAgeSec;

	vm.overlayTotalPct = overlay.process.cpuPctOneCore;
	vm.overlayUnattributedPct = UnattributedPct(overlay);
	vm.overlayWorkingSetMb = static_cast<double>(overlay.process.snapshot.workingSetBytes) / (1024.0 * 1024.0);
	vm.overlayThreadCount = overlay.processThreadCount;
	vm.overlayHandleCount = overlay.process.snapshot.handleCount;
	vm.overlayPid = overlay.process.snapshot.processId;

	if (driverConnected) {
		vm.driverTotalPct = driver.process.cpuPctOneCore;
		vm.driverUnattributedPct = UnattributedPct(driver);
		vm.driverWorkingSetMb = static_cast<double>(driver.process.snapshot.workingSetBytes) / (1024.0 * 1024.0);
		vm.driverThreadCount = driver.processThreadCount;
		vm.driverHandleCount = driver.process.snapshot.handleCount;
		vm.driverPid = driver.process.snapshot.processId;
	}

	for (uint32_t slot = 0; slot < moduleperf::kSlotCount; ++slot) {
		const moduleperf::ModuleSample& o = overlay.modules[slot];
		const moduleperf::ModuleSample& d = driver.modules[slot];
		const bool driverSide = driverConnected && d.active;
		if (!o.active && !driverSide) continue;

		PerfModuleRow row;
		row.id = static_cast<moduleperf::ModuleId>(slot);
		row.overlayActive = o.active;
		row.driverActive = driverSide;
		row.overlayPct = ModuleInProcessPct(o);
		row.overlayThreads = o.threadCount;
		if (driverSide) {
			row.driverPct = ModuleInProcessPct(d);
			row.driverThreads = d.threadCount;
			row.sidecarPresent = d.sidecarValid;
			row.sidecarPct = d.sidecarCpuPctOneCore;
			row.sidecarThreads = d.sidecarThreadCount;
			row.sidecarPid = d.sidecarPid;
			row.sidecarProcessCount = d.sidecarProcessCount;
			row.sidecarWorkingSetBytes = d.sidecarWorkingSetBytes;
			vm.sidecarTotalPct += d.sidecarCpuPctOneCore;
		}
		row.totalPct = row.overlayPct + row.driverPct + row.sidecarPct;
		vm.rows.push_back(row);
	}

	return vm;
}

// Exponential moving average for the table's displayed percentages; the
// graphs plot raw values. ~5-sample horizon at the 1 Hz cadence.
inline double EmaUpdate(double previous, double raw, bool hasPrevious, double alpha = 0.35)
{
	if (!hasPrevious) return raw;
	return previous + alpha * (raw - previous);
}

} // namespace openvr_pair::overlay
