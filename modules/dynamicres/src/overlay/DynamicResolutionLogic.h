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
	double stepFraction = 0.15;
	double lowerGpuBudgetFraction = 0.88;
	double cpuGpuBudgetFraction = 0.70;
	double headroomGpuBudgetFraction = 0.70;
	int windowSize = 6;
	int lowerRequiredTicks = 3;
	int raiseRequiredTicks = 3;
	int settleTicks = 0;
	int noEffectLimit = 3;
	bool allowRaiseBack = true;
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
	bool externalOverride = false;
};

struct DynamicResolutionControllerOutput
{
	ResolutionAction action = ResolutionAction::None;
	double targetScale = 1.0;
	std::string reason;
	DynamicResolutionClassification classification;
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
	double FloorFor(const DynamicResolutionSettings& settings, double baselineScale) const;
	double RaiseStepFor(const DynamicResolutionSettings& settings,
	                    const DynamicResolutionClassification& classification) const;
	double LowerStepFor(const DynamicResolutionSettings& settings,
	                    const DynamicResolutionClassification& classification) const;

	std::deque<DynamicResolutionTiming> samples_;
	ResolutionPressure lastPressure_ = ResolutionPressure::Waiting;
	int pressureTicks_ = 0;
	int settleTicksRemaining_ = 0;
	int noEffectCount_ = 0;
	bool effectCheckPending_ = false;
	double lastLowerAppGpuMs_ = 0.0;
	double lastWrittenScale_ = 0.0;
	ResolutionAction activeDirection_ = ResolutionAction::None;
};

} // namespace wkopenvr::dynamicres
