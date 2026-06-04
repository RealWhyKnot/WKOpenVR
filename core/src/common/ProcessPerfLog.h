#pragma once

#include <cstdint>
#include <string>

namespace openvr_pair::common {

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
std::string FormatProcessPerfSample(const char* role, const ProcessPerfSample& sample);

class ProcessPerfSampler
{
public:
	explicit ProcessPerfSampler(uint64_t intervalMs = 10000);

	void Reset();
	bool MaybeSample(ProcessPerfSample& out);

private:
	uint64_t intervalMs_ = 10000;
	uint64_t lastSampleWallMs_ = 0;
	bool havePrevious_ = false;
	ProcessPerfSnapshot previous_{};
};

} // namespace openvr_pair::common
