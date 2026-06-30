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

// Quality vs FPS bias presets. Custom means the user edited a raw knob and the
// preset no longer matches a named point on the spectrum.
enum class QualityPreset
{
	MaxFps,
	FpsFirst,
	Balanced,
	Quality,
	Custom,
};

struct DynamicResolutionSettings
{
	// Defaults match the FpsFirst preset (see ApplyQualityPreset).
	QualityPreset qualityPreset = QualityPreset::FpsFirst;
	double minScaleFraction = 0.60;           // quality floor, as a fraction of baseline
	double maxScaleFraction = 1.50;           // quality ceiling; >1 supersamples above baseline
	double stepFraction = 0.15;               // max adaptive scale change per step
	double lowerGpuBudgetFraction = 0.85;     // median app GPU >= this*budget -> GPU-bound
	double gpuSafetyMarginFraction = 0.90;    // peak app GPU >= this*budget -> proactive lower
	double overBudgetFraction = 0.15;         // share of frames over budget that forces a lower
	double headroomGpuBudgetFraction = 0.70;  // median app GPU <= this*budget -> headroom
	double raiseAboveBaselineFraction = 0.65; // peak app GPU <= this*budget gates >baseline raises
	double cpuStallFraction = 1.05;           // frame interval > this*budget (GPU idle) -> CPU-bound
	int windowSize = 6;
	int lowerRequiredTicks = 2;
	int raiseRequiredTicks = 4;
	int cpuReleaseTicks = 4;
	int raiseAboveBaselineTicks = 8;
	int settleTicks = 0;
	int noEffectLimit = 3;
	bool allowRaiseBack = true;
	bool releaseOnCpuBound = true;
};

struct DynamicResolutionTiming
{
	double frameBudgetMs = 1000.0 / 90.0;
	double preSubmitGpuMs = 0.0;
	double postSubmitGpuMs = 0.0;
	double totalRenderGpuMs = 0.0;
	double compositorRenderGpuMs = 0.0;
	double peakAppGpuMs = 0.0;          // worst per-frame app GPU ms in the batch (tail signal)
	double clientFrameIntervalMs = 0.0; // mean CPU cadence between WaitGetPoses calls
	int framesOverBudget = 0;           // per-frame app GPU >= budget (would-drop frames)
	int framesConsidered = 0;           // frames aggregated into this sample
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
	double peakAppGpuMs = 0.0;
	double clientFrameIntervalMs = 0.0;
	double frameBudgetMs = 1000.0 / 90.0;
	int sampleCount = 0;
	int unstableSamples = 0;
	int cpuReasonSamples = 0;
	int gpuReasonSamples = 0;
	int framesOverBudgetTotal = 0;
	int framesConsideredTotal = 0;
	bool unstable = false;
	bool gpuHasHeadroom = false;
	bool deepHeadroom = false;
	bool gpuOverMargin = false;
	bool cpuStalled = false;
	bool motionSmoothingActive = false;
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
const char* QualityPresetLabel(QualityPreset preset);
bool ScaleDiffers(double a, double b, double tolerance = 0.005);

// Highest scale the controller may write, given a captured baseline. >baseline when
// maxScaleFraction > 1 (supersampling above the user's SteamVR scale).
double CeilingScale(const DynamicResolutionSettings& settings, double baselineScale);

// Seeds settings to a named quality/FPS bias. Does not touch persistence-only fields.
void ApplyQualityPreset(QualityPreset preset, DynamicResolutionSettings& settings);

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
