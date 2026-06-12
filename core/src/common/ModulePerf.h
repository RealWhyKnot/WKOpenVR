#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "ModuleRegistry.h"

namespace openvr_pair::common {

// Snapshot of one process's CPU/memory/handle counters from the Win32
// process-information APIs. CPU time is cumulative since process start;
// usable rates come from diffing two snapshots.
struct ProcessPerfSnapshot
{
	uint32_t processId = 0;
	uint32_t logicalProcessors = 1;
	uint64_t wallMs = 0;
	uint64_t cpuTime100ns = 0;
	bool cpuTimeValid = false;

	bool memoryValid = false;
	uint64_t workingSetBytes = 0;
	uint64_t privateBytes = 0;
	uint64_t peakWorkingSetBytes = 0;

	bool handleCountValid = false;
	uint32_t handleCount = 0;
};

struct ProcessPerfSample
{
	ProcessPerfSnapshot snapshot;
	bool cpuValid = false;
	uint64_t intervalMs = 0;
	uint64_t processCpuMs = 0;
	double cpuPctTotal = 0.0;
	double cpuPctOneCore = 0.0;
};

double CalculateProcessCpuPercentOneCore(uint64_t processDelta100ns, uint64_t wallDeltaMs);
double CalculateProcessCpuPercentTotal(uint64_t processDelta100ns, uint64_t wallDeltaMs, uint32_t logicalProcessors);
bool ShouldTakeProcessPerfSample(uint64_t lastSampleWallMs, uint64_t nowWallMs, uint64_t intervalMs);
bool CollectProcessPerfSnapshot(ProcessPerfSnapshot& out);
// Same counters for another process (a module's sidecar). The handle needs
// PROCESS_QUERY_LIMITED_INFORMATION access or better.
bool CollectProcessPerfSnapshotForHandle(void* processHandle, ProcessPerfSnapshot& out);

// Per-module CPU attribution. Each process keeps one Registry; modules feed
// it three ways:
//  - ScopedSection wraps module work that runs on shared threads (overlay
//    tick/draw, driver pose-path hooks) and accumulates wall time.
//  - ScopedThreadRegistration marks a worker thread as owned by a module so
//    its full CPU time is attributed via GetThreadTimes deltas.
//  - RegisterChildProcess attributes a spawned sidecar's whole-process CPU
//    and memory to the owning module.
// A Sampler periodically drains the registry alongside whole-process
// counters; section + thread + sidecar percentages never overlap, so their
// sum stays comparable against the process total.
namespace moduleperf {

using modules::ModuleId;

// Attribution slots indexed by ModuleId value. Spare slots past the last
// enum entry stay zeroed so the enum can grow without relayout; the shmem
// mirror in Protocol.h uses the same count.
inline constexpr uint32_t kSlotCount = 16;

uint32_t SlotIndex(ModuleId id);

// One module's share of one sampling interval.
struct ModuleSample
{
	// Sticky: true once the module has shown any activity this session
	// (a registered thread or sidecar, or nonzero scoped-section time).
	bool active = false;
	uint32_t threadCount = 0;          // currently registered worker threads
	double sectionCpuPctOneCore = 0.0; // scoped-section wall time, % of one core
	double threadCpuPctOneCore = 0.0;  // registered-thread CPU time, % of one core

	bool sidecarValid = false;
	uint32_t sidecarPid = 0;
	uint32_t sidecarProcessCount = 0;
	double sidecarCpuPctOneCore = 0.0;
	double sidecarCpuPctTotal = 0.0;
	uint64_t sidecarWorkingSetBytes = 0;
	uint64_t sidecarPrivateBytes = 0;
	uint32_t sidecarThreadCount = 0;
	uint32_t sidecarHandleCount = 0;
};

struct PerfSampleResult
{
	ProcessPerfSample process;
	ModuleSample modules[kSlotCount];
	uint32_t processThreadCount = 0;
};

double ModuleInProcessCpuPercentOneCore(const ModuleSample& sample);
double AttributedProcessCpuPercentOneCore(const PerfSampleResult& result);
double UnattributedProcessCpuPercentOneCore(const PerfSampleResult& result);

class Registry
{
public:
	static Registry& Instance();

	// Duplicates a handle to the calling thread and marks the thread so
	// ScopedSection no-ops on it (its CPU time is already fully counted).
	void RegisterCurrentThread(ModuleId id, const char* label);
	// Folds the thread's final CPU delta into the slot before dropping it,
	// so short-lived workers still show up in the next sample.
	void UnregisterCurrentThread();

	// Duplicates the given process handle; safe to call from supervisor
	// threads right after a successful spawn.
	void RegisterChildProcess(ModuleId id, void* processHandle, const char* label);
	void UnregisterChildProcess(ModuleId id, uint32_t pid);

	// Lock-free accumulation target for ScopedSection.
	void AddSectionQpc(uint32_t slot, int64_t qpcDelta);

	// Drains section accumulators and diffs thread/child CPU counters over
	// wallDeltaMs. A zero delta establishes baselines and reports zero
	// percentages. processThreadCount is the calling process's own thread
	// count from the same system snapshot used for sidecar thread counts.
	void Sample(uint64_t wallDeltaMs, ModuleSample (&out)[kSlotCount], uint32_t& processThreadCount);

	void ResetForTests();

private:
	Registry() = default;

	struct ThreadEntry
	{
		void* handle = nullptr;
		uint32_t threadId = 0;
		uint32_t slot = 0;
		std::string label;
		uint64_t prevCpu100ns = 0;
		bool havePrev = false;
	};

	struct ChildEntry
	{
		void* handle = nullptr;
		uint32_t pid = 0;
		uint32_t slot = 0;
		std::string label;
		uint64_t prevCpu100ns = 0;
		bool havePrev = false;
	};

	std::mutex mutex_;
	std::vector<ThreadEntry> threads_;
	std::vector<ChildEntry> children_;
	std::atomic<int64_t> sectionQpc_[kSlotCount]{};
	std::atomic<uint64_t> retiredThread100ns_[kSlotCount]{};
	std::atomic<uint64_t> retiredChild100ns_[kSlotCount]{};
	std::atomic<bool> everActive_[kSlotCount]{};
};

// Accumulates wall time spent inside a scope into the owning module's slot.
// No-ops on registered worker threads (their CPU time is already counted)
// and when nested inside another section (the outermost section wins), so
// attribution never double-counts. Cheap enough for per-pose-update paths:
// two QueryPerformanceCounter reads and one relaxed fetch_add.
class ScopedSection
{
public:
	explicit ScopedSection(ModuleId id);
	~ScopedSection();

	ScopedSection(const ScopedSection&) = delete;
	ScopedSection& operator=(const ScopedSection&) = delete;

private:
	int64_t startQpc_ = 0;
	uint32_t slot_ = 0;
	bool enteredDepth_ = false;
	bool active_ = false;
};

// RAII registration for a module-owned worker thread; place at the top of
// the thread function.
struct ScopedThreadRegistration
{
	ScopedThreadRegistration(ModuleId id, const char* label) { Registry::Instance().RegisterCurrentThread(id, label); }
	~ScopedThreadRegistration() { Registry::Instance().UnregisterCurrentThread(); }

	ScopedThreadRegistration(const ScopedThreadRegistration&) = delete;
	ScopedThreadRegistration& operator=(const ScopedThreadRegistration&) = delete;
};

class Sampler
{
public:
	explicit Sampler(uint64_t intervalMs = 1000);

	void Reset();
	bool MaybeSample(PerfSampleResult& out);

private:
	uint64_t intervalMs_ = 1000;
	uint64_t lastSampleWallMs_ = 0;
	bool havePrevious_ = false;
	ProcessPerfSnapshot previous_{};
};

std::string FormatPerfProcessLine(const char* role, const PerfSampleResult& result);
std::string FormatPerfModuleLine(const char* role, ModuleId id, const ModuleSample& m);

} // namespace moduleperf
} // namespace openvr_pair::common
