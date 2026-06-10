#pragma once

#include "ModulePerf.h"

#include <cstdint>
#include <string>

namespace openvr_pair::common {

struct RuntimeCompositorTimingSample
{
	uint32_t frameIndex = 0;
	uint32_t framePresents = 0;
	uint32_t droppedFrames = 0;
	uint32_t mispresentedFrames = 0;
	uint32_t reprojectionFlags = 0;
	double clientFrameIntervalMs = 0.0;
	double totalRenderGpuMs = 0.0;
	double compositorRenderGpuMs = 0.0;
	double compositorRenderCpuMs = 0.0;
	double submitFrameMs = 0.0;
	bool hmdPoseValid = true;
	int hmdTrackingResult = 0;
};

struct RuntimePoseHealthSample
{
	double refPoseAgeMs = 0.0;
	double targetPoseAgeMs = 0.0;
	double refPoseGapMs = 0.0;
	double targetPoseGapMs = 0.0;
	bool invalid = false;
	bool stale = false;
	bool jump = false;
	int refTrackingResult = 0;
	int targetTrackingResult = 0;
};

struct RuntimeCalibrationHealthSample
{
	bool valid = false;
	int sampleCount = 0;
	int validSampleCount = 0;
	int pairedSampleCount = 0;
	bool trackingHealthPass = true;
	bool shadowDynamicPass = false;
	double residualRmsMm = 0.0;
	double residualP95Mm = 0.0;
	double holdoutRmsMm = 0.0;
};

void RecordRuntimeProcessSample(const char* role, const ProcessPerfSample& sample);
void RecordRuntimeCompositorTiming(const RuntimeCompositorTimingSample& sample);
void RecordRuntimePoseHealth(const RuntimePoseHealthSample& sample);
void RecordRuntimeCalibrationHealth(const RuntimeCalibrationHealthSample& sample);

bool WriteRuntimeHealthSummary(const wchar_t* fileName = L"runtime_health_summary.json");
bool MaybeWriteRuntimeHealthSummary(uint64_t intervalMs = 10000,
                                    const wchar_t* fileName = L"runtime_health_summary.json");

void ResetRuntimeHealthSummaryForTests();
std::string FormatRuntimeHealthSummaryForTests();

} // namespace openvr_pair::common
