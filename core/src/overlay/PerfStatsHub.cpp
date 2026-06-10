#include "PerfStatsHub.h"

#include "DebugLogging.h"
#include "DiagnosticsLog.h"
#include "Protocol.h"
#include "ProtocolNames.h"
#include "RuntimeHealthSummary.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace openvr_pair::overlay {

namespace {

// Driver samples older than this are treated as "driver gone": the segment
// outlives vrserver, so a mapped segment alone proves nothing.
constexpr uint64_t kDriverStaleAfterMs = 5000;

// Mirrors one published wire snapshot back into the sampler's result shape
// so the history/view-model layer handles both processes identically.
moduleperf::PerfSampleResult ConvertDriverSnapshot(const ::protocol::PerfStatsSnapshot& snap)
{
	moduleperf::PerfSampleResult result{};
	result.process.snapshot.processId = snap.process.pid;
	result.process.snapshot.logicalProcessors = snap.process.logicalProcessors;
	result.process.snapshot.cpuTime100ns = snap.process.cpuTimeMs * 10000ULL;
	result.process.snapshot.cpuTimeValid = snap.process.cpuValid != 0;
	result.process.snapshot.memoryValid = snap.process.memoryValid != 0;
	result.process.snapshot.workingSetBytes = snap.process.workingSetBytes;
	result.process.snapshot.privateBytes = snap.process.privateBytes;
	result.process.snapshot.peakWorkingSetBytes = snap.process.peakWorkingSetBytes;
	result.process.snapshot.handleCountValid = snap.process.handleValid != 0;
	result.process.snapshot.handleCount = snap.process.handleCount;
	result.process.cpuValid = snap.process.cpuValid != 0;
	result.process.intervalMs = snap.intervalMs;
	result.process.cpuPctOneCore = snap.process.cpuPctOneCore;
	result.process.cpuPctTotal = snap.process.cpuPctTotal;
	result.processThreadCount = snap.process.threadCount;

	for (uint32_t slot = 0; slot < moduleperf::kSlotCount && slot < ::protocol::PERF_STATS_MODULE_SLOTS; ++slot) {
		const ::protocol::PerfStatsModuleSlot& in = snap.modules[slot];
		moduleperf::ModuleSample& out = result.modules[slot];
		out.active = in.active != 0;
		out.threadCount = in.threadCount;
		out.sectionCpuPctOneCore = in.sectionCpuPctOneCore;
		out.threadCpuPctOneCore = in.threadCpuPctOneCore;
		out.sidecarValid = in.hasSidecar != 0;
		out.sidecarPid = in.sidecarPid;
		out.sidecarProcessCount = in.sidecarProcessCount;
		out.sidecarCpuPctOneCore = in.sidecarCpuPctOneCore;
		out.sidecarCpuPctTotal = in.sidecarCpuPctTotal;
		out.sidecarWorkingSetBytes = in.sidecarWorkingSetBytes;
		out.sidecarPrivateBytes = in.sidecarPrivateBytes;
		out.sidecarThreadCount = in.sidecarThreadCount;
		out.sidecarHandleCount = in.sidecarHandleCount;
	}
	return result;
}

} // namespace

PerfStatsHub& GetPerfStatsHub()
{
	static PerfStatsHub hub;
	return hub;
}

PerfStatsHub::~PerfStatsHub() = default;

bool PerfStatsHub::TryOpenDriverSegment()
{
	if (shmem_) return true;
	try {
		auto next = std::make_unique<::protocol::PerfStatsShmem>();
		next->Open(OPENVR_PAIRDRIVER_PERFSTATS_SHMEM_NAME);
		shmem_ = std::move(next);
		common::DiagnosticLog("perf", "driver perf stats segment mapped");
		return true;
	}
	catch (...) {
		shmem_.reset();
		// Expected while SteamVR is not running (or the installed driver
		// predates the segment); retried on the next 1 Hz tick.
		return false;
	}
}

void PerfStatsHub::EmitLogLines(const moduleperf::PerfSampleResult& overlaySample)
{
	if (!common::IsDebugLoggingEnabled()) {
		lastLogWallMs_ = 0;
		return;
	}
	const uint64_t nowMs = GetTickCount64();
	if (lastLogWallMs_ != 0 && nowMs - lastLogWallMs_ < 10000) return;
	lastLogWallMs_ = nowMs;

	common::DiagnosticLog("perf", "%s", moduleperf::FormatPerfProcessLine("overlay", overlaySample).c_str());
	size_t moduleCount = 0;
	const common::modules::ModuleInfo* infos = common::modules::All(&moduleCount);
	for (size_t i = 0; i < moduleCount; ++i) {
		const uint32_t slot = moduleperf::SlotIndex(infos[i].id);
		if (overlaySample.modules[slot].active) {
			common::DiagnosticLog(
			    "perf", "%s",
			    moduleperf::FormatPerfModuleLine("overlay", infos[i].id, overlaySample.modules[slot]).c_str());
		}
		// The driver host logs its own [perf] lines into the vrserver log;
		// mirroring the freshest driver sample here keeps a single-file view
		// in the diagnostics log for bug reports.
		if (viewModel_.driverConnected && lastDriverSample_.modules[slot].active) {
			common::DiagnosticLog(
			    "perf", "%s",
			    moduleperf::FormatPerfModuleLine("driver-host", infos[i].id, lastDriverSample_.modules[slot]).c_str());
		}
	}
	if (viewModel_.driverConnected) {
		common::DiagnosticLog("perf", "%s",
		                      moduleperf::FormatPerfProcessLine("driver-host", lastDriverSample_).c_str());
	}
}

void PerfStatsHub::Tick(double nowSeconds)
{
	moduleperf::PerfSampleResult overlaySample{};
	if (!sampler_.MaybeSample(overlaySample)) return;
	lastOverlaySample_ = overlaySample;

	bool driverFresh = false;
	double driverAgeSec = 0.0;
	if (TryOpenDriverSegment()) {
		::protocol::PerfStatsSnapshot snap{};
		if (shmem_->TryRead(snap) && snap.sampleIndex != 0) {
			const uint64_t nowMs = GetTickCount64();
			const uint64_t ageMs = nowMs >= snap.sampleTickMs ? nowMs - snap.sampleTickMs : 0;
			driverAgeSec = static_cast<double>(ageMs) / 1000.0;
			driverFresh = ageMs < kDriverStaleAfterMs;
			if (driverFresh && snap.sampleIndex != lastDriverSampleIndex_) {
				lastDriverSampleIndex_ = snap.sampleIndex;
				lastDriverSample_ = ConvertDriverSnapshot(snap);
			}
		}
	}

	PushPerfHistory(history_, nowSeconds, overlaySample, driverFresh, lastDriverSample_);
	viewModel_ = BuildPerfViewModel(overlaySample, driverFresh, lastDriverSample_, driverAgeSec);

	bool seen[moduleperf::kSlotCount]{};
	for (PerfModuleRow& row : viewModel_.rows) {
		const uint32_t slot = moduleperf::SlotIndex(row.id);
		smoothedTotalPct_[slot] = EmaUpdate(smoothedTotalPct_[slot], row.totalPct, smoothedValid_[slot]);
		smoothedValid_[slot] = true;
		seen[slot] = true;
	}
	for (uint32_t slot = 0; slot < moduleperf::kSlotCount; ++slot) {
		if (!seen[slot]) {
			smoothedTotalPct_[slot] = 0.0;
			smoothedValid_[slot] = false;
		}
	}

	common::RecordRuntimeProcessSample("overlay", overlaySample.process);
	EmitLogLines(overlaySample);
	common::MaybeWriteRuntimeHealthSummary();
}

} // namespace openvr_pair::overlay
