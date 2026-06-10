#include "ModulePerf.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <psapi.h>
#include <tlhelp32.h>

namespace openvr_pair::common {
namespace {

uint64_t FileTimeToUint64(const FILETIME& ft)
{
	ULARGE_INTEGER value{};
	value.LowPart = ft.dwLowDateTime;
	value.HighPart = ft.dwHighDateTime;
	return value.QuadPart;
}

double BytesToMb(uint64_t bytes)
{
	return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

uint32_t LogicalProcessorCount()
{
	SYSTEM_INFO sys{};
	GetSystemInfo(&sys);
	return std::max<uint32_t>(1, sys.dwNumberOfProcessors);
}

int64_t QpcFrequency()
{
	static const int64_t freq = [] {
		LARGE_INTEGER f{};
		QueryPerformanceFrequency(&f);
		return f.QuadPart > 0 ? f.QuadPart : 1;
	}();
	return freq;
}

int64_t QpcNow()
{
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	return now.QuadPart;
}

// Thread counts come from one system process walk per sample rather than a
// per-process API; the same snapshot serves the host process and every
// registered sidecar pid.
void CollectThreadCounts(std::unordered_map<uint32_t, uint32_t>& countsByPid)
{
	HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (snap == INVALID_HANDLE_VALUE) return;

	PROCESSENTRY32W entry{};
	entry.dwSize = sizeof(entry);
	if (Process32FirstW(snap, &entry)) {
		do {
			auto it = countsByPid.find(static_cast<uint32_t>(entry.th32ProcessID));
			if (it != countsByPid.end()) {
				it->second = static_cast<uint32_t>(entry.cntThreads);
			}
		} while (Process32NextW(snap, &entry));
	}
	CloseHandle(snap);
}

bool CollectSnapshotForProcess(HANDLE process, ProcessPerfSnapshot& out)
{
	out = ProcessPerfSnapshot{};
	out.processId = GetProcessId(process);
	out.logicalProcessors = LogicalProcessorCount();
	out.wallMs = GetTickCount64();

	FILETIME creation{};
	FILETIME exit{};
	FILETIME kernel{};
	FILETIME user{};
	if (GetProcessTimes(process, &creation, &exit, &kernel, &user)) {
		out.cpuTime100ns = FileTimeToUint64(kernel) + FileTimeToUint64(user);
		out.cpuTimeValid = true;
	}

	PROCESS_MEMORY_COUNTERS_EX mem{};
	mem.cb = sizeof(mem);
	if (GetProcessMemoryInfo(process, reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&mem), sizeof(mem))) {
		out.workingSetBytes = static_cast<uint64_t>(mem.WorkingSetSize);
		out.privateBytes = static_cast<uint64_t>(mem.PrivateUsage);
		out.peakWorkingSetBytes = static_cast<uint64_t>(mem.PeakWorkingSetSize);
		out.memoryValid = true;
	}

	DWORD handles = 0;
	if (GetProcessHandleCount(process, &handles)) {
		out.handleCount = handles;
		out.handleCountValid = true;
	}

	return out.cpuTimeValid || out.memoryValid || out.handleCountValid;
}

} // namespace

double CalculateProcessCpuPercentOneCore(uint64_t processDelta100ns, uint64_t wallDeltaMs)
{
	if (wallDeltaMs == 0) return 0.0;

	const double processMs = static_cast<double>(processDelta100ns) / 10000.0;
	const double pct = (processMs / static_cast<double>(wallDeltaMs)) * 100.0;
	if (!std::isfinite(pct) || pct < 0.0) return 0.0;
	return pct;
}

double CalculateProcessCpuPercentTotal(uint64_t processDelta100ns, uint64_t wallDeltaMs, uint32_t logicalProcessors)
{
	const uint32_t cpus = std::max<uint32_t>(1, logicalProcessors);
	const double pct = CalculateProcessCpuPercentOneCore(processDelta100ns, wallDeltaMs) / static_cast<double>(cpus);
	if (!std::isfinite(pct) || pct < 0.0) return 0.0;
	return std::min(100.0, pct);
}

bool ShouldTakeProcessPerfSample(uint64_t lastSampleWallMs, uint64_t nowWallMs, uint64_t intervalMs)
{
	if (lastSampleWallMs == 0) return true;
	if (nowWallMs < lastSampleWallMs) return true;
	return nowWallMs - lastSampleWallMs >= intervalMs;
}

bool CollectProcessPerfSnapshot(ProcessPerfSnapshot& out)
{
	return CollectSnapshotForProcess(GetCurrentProcess(), out);
}

bool CollectProcessPerfSnapshotForHandle(void* processHandle, ProcessPerfSnapshot& out)
{
	if (!processHandle || processHandle == INVALID_HANDLE_VALUE) {
		out = ProcessPerfSnapshot{};
		return false;
	}
	return CollectSnapshotForProcess(static_cast<HANDLE>(processHandle), out);
}

namespace moduleperf {
namespace {

// Slot index of the module owning the current thread, or -1 when the thread
// is unregistered. Lets ScopedSection skip threads whose CPU time is already
// fully attributed via GetThreadTimes.
thread_local int t_registeredSlot = -1;
// Section nesting depth on this thread; only the outermost section records.
thread_local int t_sectionDepth = 0;

} // namespace

uint32_t SlotIndex(ModuleId id)
{
	const uint32_t index = static_cast<uint32_t>(id);
	return index < kSlotCount ? index : kSlotCount - 1;
}

Registry& Registry::Instance()
{
	static Registry instance;
	return instance;
}

void Registry::RegisterCurrentThread(ModuleId id, const char* label)
{
	HANDLE dup = nullptr;
	if (!DuplicateHandle(GetCurrentProcess(), GetCurrentThread(), GetCurrentProcess(), &dup, 0, FALSE,
	                     DUPLICATE_SAME_ACCESS)) {
		return;
	}

	ThreadEntry entry{};
	entry.handle = dup;
	entry.threadId = GetCurrentThreadId();
	entry.slot = SlotIndex(id);
	entry.label = label ? label : "";

	FILETIME creation{};
	FILETIME exit{};
	FILETIME kernel{};
	FILETIME user{};
	if (GetThreadTimes(dup, &creation, &exit, &kernel, &user)) {
		entry.prevCpu100ns = FileTimeToUint64(kernel) + FileTimeToUint64(user);
		entry.havePrev = true;
	}

	{
		std::lock_guard<std::mutex> lock(mutex_);
		threads_.push_back(std::move(entry));
	}
	everActive_[SlotIndex(id)].store(true, std::memory_order_relaxed);
	t_registeredSlot = static_cast<int>(SlotIndex(id));
}

void Registry::UnregisterCurrentThread()
{
	const uint32_t tid = GetCurrentThreadId();
	std::lock_guard<std::mutex> lock(mutex_);
	for (auto it = threads_.begin(); it != threads_.end(); ++it) {
		if (it->threadId != tid) continue;

		// Bank the time spent since the last sample so a worker that exits
		// mid-interval still shows up in the next one.
		FILETIME creation{};
		FILETIME exit{};
		FILETIME kernel{};
		FILETIME user{};
		if (it->havePrev && GetThreadTimes(static_cast<HANDLE>(it->handle), &creation, &exit, &kernel, &user)) {
			const uint64_t now100ns = FileTimeToUint64(kernel) + FileTimeToUint64(user);
			if (now100ns > it->prevCpu100ns) {
				retiredThread100ns_[it->slot].fetch_add(now100ns - it->prevCpu100ns, std::memory_order_relaxed);
			}
		}
		CloseHandle(static_cast<HANDLE>(it->handle));
		threads_.erase(it);
		break;
	}
	t_registeredSlot = -1;
}

void Registry::RegisterChildProcess(ModuleId id, void* processHandle, const char* label)
{
	if (!processHandle || processHandle == INVALID_HANDLE_VALUE) return;

	HANDLE dup = nullptr;
	if (!DuplicateHandle(GetCurrentProcess(), static_cast<HANDLE>(processHandle), GetCurrentProcess(), &dup, 0, FALSE,
	                     DUPLICATE_SAME_ACCESS)) {
		return;
	}

	const uint32_t slot = SlotIndex(id);
	ChildEntry entry{};
	entry.handle = dup;
	entry.pid = GetProcessId(dup);
	entry.slot = slot;
	entry.label = label ? label : "";

	FILETIME creation{};
	FILETIME exit{};
	FILETIME kernel{};
	FILETIME user{};
	if (GetProcessTimes(dup, &creation, &exit, &kernel, &user)) {
		entry.prevCpu100ns = FileTimeToUint64(kernel) + FileTimeToUint64(user);
		entry.havePrev = true;
	}

	std::lock_guard<std::mutex> lock(mutex_);
	// A respawn can race the unregister of the previous incarnation; replace
	// any stale entry with the same pid in the same slot.
	for (auto it = children_.begin(); it != children_.end(); ++it) {
		if (it->slot == entry.slot && it->pid == entry.pid) {
			CloseHandle(static_cast<HANDLE>(it->handle));
			children_.erase(it);
			break;
		}
	}
	children_.push_back(std::move(entry));
	everActive_[slot].store(true, std::memory_order_relaxed);
}

void Registry::UnregisterChildProcess(ModuleId id, uint32_t pid)
{
	const uint32_t slot = SlotIndex(id);
	std::lock_guard<std::mutex> lock(mutex_);
	for (auto it = children_.begin(); it != children_.end(); ++it) {
		if (it->slot != slot || it->pid != pid) continue;

		// Bank the final delta; the handle stays valid after process exit so
		// GetProcessTimes still reports the closing totals.
		FILETIME creation{};
		FILETIME exit{};
		FILETIME kernel{};
		FILETIME user{};
		if (it->havePrev && GetProcessTimes(static_cast<HANDLE>(it->handle), &creation, &exit, &kernel, &user)) {
			const uint64_t now100ns = FileTimeToUint64(kernel) + FileTimeToUint64(user);
			if (now100ns > it->prevCpu100ns) {
				retiredChild100ns_[slot].fetch_add(now100ns - it->prevCpu100ns, std::memory_order_relaxed);
			}
		}
		CloseHandle(static_cast<HANDLE>(it->handle));
		children_.erase(it);
		break;
	}
}

void Registry::AddSectionQpc(uint32_t slot, int64_t qpcDelta)
{
	if (slot >= kSlotCount || qpcDelta <= 0) return;
	sectionQpc_[slot].fetch_add(qpcDelta, std::memory_order_relaxed);
	everActive_[slot].store(true, std::memory_order_relaxed);
}

void Registry::Sample(uint64_t wallDeltaMs, ModuleSample (&out)[kSlotCount], uint32_t& processThreadCount)
{
	for (uint32_t slot = 0; slot < kSlotCount; ++slot) {
		out[slot] = ModuleSample{};
	}
	processThreadCount = 0;

	uint64_t threadDelta100ns[kSlotCount]{};
	uint64_t childDelta100ns[kSlotCount]{};
	const uint32_t logicalCpus = LogicalProcessorCount();

	std::unordered_map<uint32_t, uint32_t> threadCounts;
	threadCounts[GetCurrentProcessId()] = 0;

	{
		std::lock_guard<std::mutex> lock(mutex_);

		for (auto it = threads_.begin(); it != threads_.end();) {
			FILETIME creation{};
			FILETIME exit{};
			FILETIME kernel{};
			FILETIME user{};
			if (!GetThreadTimes(static_cast<HANDLE>(it->handle), &creation, &exit, &kernel, &user)) {
				CloseHandle(static_cast<HANDLE>(it->handle));
				it = threads_.erase(it);
				continue;
			}
			const uint64_t now100ns = FileTimeToUint64(kernel) + FileTimeToUint64(user);
			if (it->havePrev && now100ns > it->prevCpu100ns) {
				threadDelta100ns[it->slot] += now100ns - it->prevCpu100ns;
			}
			it->prevCpu100ns = now100ns;
			it->havePrev = true;
			out[it->slot].threadCount++;
			++it;
		}

		for (auto& child : children_) {
			ProcessPerfSnapshot snap{};
			if (!CollectProcessPerfSnapshotForHandle(child.handle, snap)) continue;

			ModuleSample& m = out[child.slot];
			if (snap.cpuTimeValid) {
				if (child.havePrev && snap.cpuTime100ns > child.prevCpu100ns) {
					childDelta100ns[child.slot] += snap.cpuTime100ns - child.prevCpu100ns;
				}
				child.prevCpu100ns = snap.cpuTime100ns;
				child.havePrev = true;
			}
			if (!m.sidecarValid) {
				m.sidecarPid = child.pid;
			}
			m.sidecarValid = true;
			m.sidecarProcessCount++;
			if (snap.memoryValid) {
				m.sidecarWorkingSetBytes += snap.workingSetBytes;
				m.sidecarPrivateBytes += snap.privateBytes;
			}
			if (snap.handleCountValid) {
				m.sidecarHandleCount += snap.handleCount;
			}
			threadCounts[child.pid] = 0;
		}
	}

	CollectThreadCounts(threadCounts);
	processThreadCount = threadCounts[GetCurrentProcessId()];

	{
		std::lock_guard<std::mutex> lock(mutex_);
		for (const auto& child : children_) {
			const auto it = threadCounts.find(child.pid);
			if (it != threadCounts.end()) {
				out[child.slot].sidecarThreadCount += it->second;
			}
		}
	}

	const double qpcToMs = 1000.0 / static_cast<double>(QpcFrequency());
	for (uint32_t slot = 0; slot < kSlotCount; ++slot) {
		ModuleSample& m = out[slot];
		m.active = everActive_[slot].load(std::memory_order_relaxed);

		const int64_t sectionQpc = sectionQpc_[slot].exchange(0, std::memory_order_relaxed);
		const uint64_t threadTotal =
		    threadDelta100ns[slot] + retiredThread100ns_[slot].exchange(0, std::memory_order_relaxed);
		const uint64_t childTotal =
		    childDelta100ns[slot] + retiredChild100ns_[slot].exchange(0, std::memory_order_relaxed);

		if (wallDeltaMs == 0) continue;

		const double sectionMs = static_cast<double>(sectionQpc) * qpcToMs;
		const double sectionPct = (sectionMs / static_cast<double>(wallDeltaMs)) * 100.0;
		m.sectionCpuPctOneCore = (std::isfinite(sectionPct) && sectionPct > 0.0) ? sectionPct : 0.0;
		m.threadCpuPctOneCore = CalculateProcessCpuPercentOneCore(threadTotal, wallDeltaMs);
		m.sidecarCpuPctOneCore = CalculateProcessCpuPercentOneCore(childTotal, wallDeltaMs);
		m.sidecarCpuPctTotal = CalculateProcessCpuPercentTotal(childTotal, wallDeltaMs, logicalCpus);
	}
}

void Registry::ResetForTests()
{
	std::lock_guard<std::mutex> lock(mutex_);
	for (auto& entry : threads_) {
		CloseHandle(static_cast<HANDLE>(entry.handle));
	}
	threads_.clear();
	for (auto& entry : children_) {
		CloseHandle(static_cast<HANDLE>(entry.handle));
	}
	children_.clear();
	for (uint32_t slot = 0; slot < kSlotCount; ++slot) {
		sectionQpc_[slot].store(0, std::memory_order_relaxed);
		retiredThread100ns_[slot].store(0, std::memory_order_relaxed);
		retiredChild100ns_[slot].store(0, std::memory_order_relaxed);
		everActive_[slot].store(false, std::memory_order_relaxed);
	}
	t_registeredSlot = -1;
	t_sectionDepth = 0;
}

ScopedSection::ScopedSection(ModuleId id)
{
	if (t_registeredSlot >= 0) return;
	enteredDepth_ = true;
	if (++t_sectionDepth != 1) return;
	slot_ = SlotIndex(id);
	startQpc_ = QpcNow();
	active_ = true;
}

ScopedSection::~ScopedSection()
{
	if (!enteredDepth_) return;
	if (active_) {
		Registry::Instance().AddSectionQpc(slot_, QpcNow() - startQpc_);
	}
	--t_sectionDepth;
}

Sampler::Sampler(uint64_t intervalMs) : intervalMs_(intervalMs) {}

void Sampler::Reset()
{
	lastSampleWallMs_ = 0;
	havePrevious_ = false;
	previous_ = ProcessPerfSnapshot{};
}

bool Sampler::MaybeSample(PerfSampleResult& out)
{
	const uint64_t nowMs = GetTickCount64();
	if (!ShouldTakeProcessPerfSample(lastSampleWallMs_, nowMs, intervalMs_)) {
		return false;
	}

	ProcessPerfSnapshot current{};
	if (!CollectProcessPerfSnapshot(current)) {
		return false;
	}

	out = PerfSampleResult{};
	out.process.snapshot = current;

	uint64_t wallDeltaMs = 0;
	if (havePrevious_ && current.wallMs > previous_.wallMs) {
		wallDeltaMs = current.wallMs - previous_.wallMs;
	}

	if (havePrevious_ && previous_.cpuTimeValid && current.cpuTimeValid && wallDeltaMs > 0 &&
	    current.cpuTime100ns >= previous_.cpuTime100ns) {
		const uint64_t processDelta100ns = current.cpuTime100ns - previous_.cpuTime100ns;
		out.process.intervalMs = wallDeltaMs;
		out.process.processCpuMs = processDelta100ns / 10000ULL;
		out.process.cpuPctOneCore = CalculateProcessCpuPercentOneCore(processDelta100ns, wallDeltaMs);
		out.process.cpuPctTotal =
		    CalculateProcessCpuPercentTotal(processDelta100ns, wallDeltaMs, current.logicalProcessors);
		out.process.cpuValid = true;
	}

	Registry::Instance().Sample(wallDeltaMs, out.modules, out.processThreadCount);

	previous_ = current;
	havePrevious_ = true;
	lastSampleWallMs_ = current.wallMs;
	return true;
}

std::string FormatPerfProcessLine(const char* role, const PerfSampleResult& result)
{
	char buffer[768]{};
	const ProcessPerfSample& sample = result.process;
	const ProcessPerfSnapshot& s = sample.snapshot;
	std::snprintf(buffer, sizeof(buffer),
	              "role=%s pid=%lu interval_ms=%llu cpu_valid=%d cpu_pct_total=%.2f "
	              "cpu_pct_one_core=%.2f cpu_ms=%llu logical_cpus=%lu memory_valid=%d "
	              "working_set_mb=%.2f private_mb=%.2f peak_working_set_mb=%.2f "
	              "handle_valid=%d handles=%lu threads=%lu",
	              role ? role : "process", static_cast<unsigned long>(s.processId),
	              static_cast<unsigned long long>(sample.intervalMs), sample.cpuValid ? 1 : 0, sample.cpuPctTotal,
	              sample.cpuPctOneCore, static_cast<unsigned long long>(sample.processCpuMs),
	              static_cast<unsigned long>(s.logicalProcessors), s.memoryValid ? 1 : 0, BytesToMb(s.workingSetBytes),
	              BytesToMb(s.privateBytes), BytesToMb(s.peakWorkingSetBytes), s.handleCountValid ? 1 : 0,
	              static_cast<unsigned long>(s.handleCount), static_cast<unsigned long>(result.processThreadCount));
	return std::string(buffer);
}

std::string FormatPerfModuleLine(const char* role, ModuleId id, const ModuleSample& m)
{
	char buffer[768]{};
	std::snprintf(buffer, sizeof(buffer),
	              "role=%s module=%s active=%d section_pct_one_core=%.2f thread_pct_one_core=%.2f "
	              "threads=%lu sidecar=%d sidecar_pid=%lu sidecar_cpu_pct_one_core=%.2f "
	              "sidecar_cpu_pct_total=%.2f sidecar_ws_mb=%.2f sidecar_private_mb=%.2f "
	              "sidecar_threads=%lu sidecar_handles=%lu",
	              role ? role : "process", modules::Slug(id), m.active ? 1 : 0, m.sectionCpuPctOneCore,
	              m.threadCpuPctOneCore, static_cast<unsigned long>(m.threadCount), m.sidecarValid ? 1 : 0,
	              static_cast<unsigned long>(m.sidecarPid), m.sidecarCpuPctOneCore, m.sidecarCpuPctTotal,
	              BytesToMb(m.sidecarWorkingSetBytes), BytesToMb(m.sidecarPrivateBytes),
	              static_cast<unsigned long>(m.sidecarThreadCount), static_cast<unsigned long>(m.sidecarHandleCount));
	return std::string(buffer);
}

} // namespace moduleperf
} // namespace openvr_pair::common
