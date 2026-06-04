#include "ProcessPerfLog.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <psapi.h>

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
	out = ProcessPerfSnapshot{};
	out.processId = GetCurrentProcessId();
	out.logicalProcessors = LogicalProcessorCount();
	out.wallMs = GetTickCount64();

	FILETIME creation{};
	FILETIME exit{};
	FILETIME kernel{};
	FILETIME user{};
	if (GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user)) {
		out.cpuTime100ns = FileTimeToUint64(kernel) + FileTimeToUint64(user);
		out.cpuTimeValid = true;
	}

	PROCESS_MEMORY_COUNTERS_EX mem{};
	mem.cb = sizeof(mem);
	if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&mem), sizeof(mem))) {
		out.workingSetBytes = static_cast<uint64_t>(mem.WorkingSetSize);
		out.privateBytes = static_cast<uint64_t>(mem.PrivateUsage);
		out.peakWorkingSetBytes = static_cast<uint64_t>(mem.PeakWorkingSetSize);
		out.memoryValid = true;
	}

	DWORD handles = 0;
	if (GetProcessHandleCount(GetCurrentProcess(), &handles)) {
		out.handleCount = handles;
		out.handleCountValid = true;
	}

	return out.cpuTimeValid || out.memoryValid || out.handleCountValid;
}

std::string FormatProcessPerfSample(const char* role, const ProcessPerfSample& sample)
{
	char buffer[768]{};
	const ProcessPerfSnapshot& s = sample.snapshot;
	std::snprintf(buffer, sizeof(buffer),
	              "role=%s pid=%lu interval_ms=%llu cpu_valid=%d cpu_pct_total=%.2f "
	              "cpu_pct_one_core=%.2f cpu_ms=%llu logical_cpus=%lu memory_valid=%d "
	              "working_set_mb=%.2f private_mb=%.2f peak_working_set_mb=%.2f "
	              "handle_valid=%d handles=%lu",
	              role ? role : "process", static_cast<unsigned long>(s.processId),
	              static_cast<unsigned long long>(sample.intervalMs), sample.cpuValid ? 1 : 0, sample.cpuPctTotal,
	              sample.cpuPctOneCore, static_cast<unsigned long long>(sample.processCpuMs),
	              static_cast<unsigned long>(s.logicalProcessors), s.memoryValid ? 1 : 0, BytesToMb(s.workingSetBytes),
	              BytesToMb(s.privateBytes), BytesToMb(s.peakWorkingSetBytes), s.handleCountValid ? 1 : 0,
	              static_cast<unsigned long>(s.handleCount));
	return std::string(buffer);
}

ProcessPerfSampler::ProcessPerfSampler(uint64_t intervalMs) : intervalMs_(intervalMs) {}

void ProcessPerfSampler::Reset()
{
	lastSampleWallMs_ = 0;
	havePrevious_ = false;
	previous_ = ProcessPerfSnapshot{};
}

bool ProcessPerfSampler::MaybeSample(ProcessPerfSample& out)
{
	const uint64_t nowMs = GetTickCount64();
	if (!ShouldTakeProcessPerfSample(lastSampleWallMs_, nowMs, intervalMs_)) {
		return false;
	}

	ProcessPerfSnapshot current{};
	if (!CollectProcessPerfSnapshot(current)) {
		return false;
	}

	out = ProcessPerfSample{};
	out.snapshot = current;

	if (havePrevious_ && previous_.cpuTimeValid && current.cpuTimeValid && current.wallMs > previous_.wallMs &&
	    current.cpuTime100ns >= previous_.cpuTime100ns) {
		const uint64_t processDelta100ns = current.cpuTime100ns - previous_.cpuTime100ns;
		out.intervalMs = current.wallMs - previous_.wallMs;
		out.processCpuMs = processDelta100ns / 10000ULL;
		out.cpuPctOneCore = CalculateProcessCpuPercentOneCore(processDelta100ns, out.intervalMs);
		out.cpuPctTotal = CalculateProcessCpuPercentTotal(processDelta100ns, out.intervalMs, current.logicalProcessors);
		out.cpuValid = true;
	}

	previous_ = current;
	havePrevious_ = true;
	lastSampleWallMs_ = current.wallMs;
	return true;
}

} // namespace openvr_pair::common
