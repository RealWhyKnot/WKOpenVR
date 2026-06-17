#pragma once

#include <cstdint>
#include <deque>
#include <string>

namespace wkopenvr::dynamicres {

enum class ResolutionPressure
{
	Waiting,
	GpuBound,
	CpuBound,
	Headroom,
};

enum class ResolutionAction
{
	None,
	Lower,
	Raise,
	Restore,
	ExternalOverride,
	NoEffect,
};

struct DynamicResolutionSettings
{
	double minScaleFraction = 0.60;
	double streamingMinScaleFraction = 0.75;
	double stepFraction = 0.05;
	double lowerGpuBudgetFraction = 0.88;
	double cpuGpuBudgetFraction = 0.70;
	double headroomGpuBudgetFraction = 0.72;
	int windowSize = 8;
	int lowerRequiredTicks = 4;
	int raiseRequiredTicks = 12;
	int settleTicks = 2;
	int noEffectLimit = 3;
	bool allowRaiseBack = true;
	bool actUnderStreaming = false;
	bool conservativeStreaming = true;
};

struct DynamicResolutionTiming
{
	double frameBudgetMs = 1000.0 / 90.0;
	double preSubmitGpuMs = 0.0;
	double postSubmitGpuMs = 0.0;
	double totalRenderGpuMs = 0.0;
	double compositorRenderGpuMs = 0.0;
	uint32_t framePresents = 1;
	uint32_t droppedFrames = 0;
	uint32_t mispresentedFrames = 0;
	uint32_t reprojectionFlags = 0;
	bool valid = true;
};

struct DynamicResolutionClassification
{
	ResolutionPressure pressure = ResolutionPressure::Waiting;
	double appGpuMs = 0.0;
	double medianAppGpuMs = 0.0;
	double frameBudgetMs = 1000.0 / 90.0;
	int sampleCount = 0;
	int unstableSamples = 0;
	int cpuReasonSamples = 0;
	int gpuReasonSamples = 0;
	bool unstable = false;
};

struct DynamicResolutionControllerInput
{
	DynamicResolutionTiming timing;
	double baselineScale = 1.0;
	double currentScale = 1.0;
	bool sceneRunning = true;
	bool streamingDetected = false;
	bool streamingCodecAllowsAction = false;
	bool externalOverride = false;
};

struct DynamicResolutionControllerOutput
{
	ResolutionAction action = ResolutionAction::None;
	double targetScale = 1.0;
	std::string reason;
	DynamicResolutionClassification classification;
	bool actingBlocked = false;
};

double ClampScaleFraction(double value);
double ComputeAppGpuMs(const DynamicResolutionTiming& sample);
bool IsUnstableTiming(const DynamicResolutionTiming& sample);
const char* ResolutionPressureLabel(ResolutionPressure pressure);
const char* ResolutionActionLabel(ResolutionAction action);
bool ScaleDiffers(double a, double b, double tolerance = 0.005);

class DynamicResolutionController
{
public:
	void Reset();
	DynamicResolutionControllerOutput Evaluate(const DynamicResolutionControllerInput& input,
	                                           const DynamicResolutionSettings& settings);
	void NoteWrite(ResolutionAction action, double writtenScale, const DynamicResolutionClassification& classification,
	               const DynamicResolutionSettings& settings);

private:
	DynamicResolutionClassification Classify(const DynamicResolutionTiming& timing,
	                                         const DynamicResolutionSettings& settings);
	double FloorFor(const DynamicResolutionSettings& settings, double baselineScale, bool streamingDetected) const;
	double StepFor(const DynamicResolutionSettings& settings, bool streamingDetected) const;

	std::deque<DynamicResolutionTiming> samples_;
	ResolutionPressure lastPressure_ = ResolutionPressure::Waiting;
	int pressureTicks_ = 0;
	int settleTicksRemaining_ = 0;
	int noEffectCount_ = 0;
	bool effectCheckPending_ = false;
	double lastLowerAppGpuMs_ = 0.0;
	double lastWrittenScale_ = 0.0;
};

} // namespace wkopenvr::dynamicres
