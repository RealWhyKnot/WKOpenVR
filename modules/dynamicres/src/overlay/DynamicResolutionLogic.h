#pragma once

#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace wkopenvr::dynamicres {

enum class ResolutionPressure
{
	Waiting,
	Steady,
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
	double stepFraction = 0.15;               // max scale change per write
	double gpuHarmRateFraction = 0.015;       // share of a tick's frames GPU-harmed before the tick counts as harmed
	double raiseHarmRateFraction = 0.002;     // max window harmed-frame rate a raise tolerates
	double motionRateFraction = 0.05;         // recent motion-smoothing frame rate that blocks raises
	double lowerTargetFraction = 0.90;        // lowering solves p95 GPU time toward this fraction of budget
	double raiseSafetyFraction = 0.90;        // predicted p95 at a raise target must fit this fraction of budget
	double raiseAboveBaselineFraction = 0.65; // window p95 <= this*budget gates >baseline raises
	double cpuStallFraction = 1.05;           // frame interval > this*budget (GPU idle) -> CPU-bound
	int windowSize = 6;
	int lowerRequiredTicks = 2; // consecutive GPU-harmed ticks before lowering (min 2 enforced)
	int raiseRequiredTicks = 4; // consecutive clean ticks before recovering toward baseline
	int cpuReleaseTicks = 4;
	int raiseAboveBaselineTicks = 8;
	int settleTicks = 2;
	int noEffectLimit = 3;
	int burnedDecayTicks = 120; // ticks a regressed raise target stays off-limits
	bool allowRaiseBack = true;
	bool releaseOnCpuBound = true;
};

// One 1 Hz sample: per-frame distributions and harm counts across the tick's frames.
struct DynamicResolutionTiming
{
	double frameBudgetMs = 1000.0 / 90.0;
	int framesConsidered = 0; // deduped frames in this tick; 0 => not usable
	double appGpuP50Ms = 0.0;
	double appGpuP95Ms = 0.0;
	double appGpuMaxMs = 0.0;
	double clientFrameIntervalMs = 0.0; // median CPU cadence between WaitGetPoses calls
	double compositorIdleCpuMs = 0.0;   // median compositor idle (spare CPU the app could use)
	int framesWithGpuReproj = 0;        // reprojection reason: GPU
	int framesWithCpuReproj = 0;        // reprojection reason: CPU
	int framesWithMotion = 0;           // motion smoothing synthesized this frame
	int framesThrottled = 0;            // compositor throttled the app this frame
	int framesWithDrops = 0;
	int framesMispresented = 0;
	int framesMultiPresented = 0; // presented more than once (app below refresh)
	int framesOverBudget = 0;     // per-frame app GPU >= budget
	bool valid = true;
};

struct DynamicResolutionClassification
{
	ResolutionPressure pressure = ResolutionPressure::Waiting;
	double frameBudgetMs = 1000.0 / 90.0;
	int tickCount = 0;
	int framesTotal = 0;
	double appGpuP50Ms = 0.0;
	double appGpuP95Ms = 0.0; // control value for solves
	double appGpuPeakMs = 0.0;
	double clientFrameIntervalMs = 0.0;
	double compositorIdleCpuMs = 0.0;
	// Window frame rates in [0,1]: harmed frames / framesTotal.
	double gpuHarmRate = 0.0; // GPU-reproj + throttled + GPU-attributed drops
	double cpuHarmRate = 0.0;
	double motionRate = 0.0;       // whole window
	double recentMotionRate = 0.0; // last few ticks; gates raises
	double dropRate = 0.0;
	double multiPresentRate = 0.0;
	double overBudgetRate = 0.0;
	double harmRate = 0.0; // all non-motion harm conditions
	int consecutiveHarmTicks = 0;
	int consecutiveCleanTicks = 0;
	bool tickHarmed = false;             // most recent tick GPU-harmed
	bool motionSmoothingEngaged = false; // sustained motion smoothing (half-rate)
	bool appPaced = false;               // app below refresh by its own cap; not a GPU problem
	bool cpuStalled = false;
	bool gpuHasHeadroom = false;
	double costBeta = 1.0;
	double predictedRaiseP95Ms = 0.0; // model output for the last evaluated raise target
};

struct DynamicResolutionControllerInput
{
	DynamicResolutionTiming timing;
	double baselineScale = 1.0;
	double currentScale = 1.0;
	bool sceneRunning = true;
	bool externalOverride = false;
};

// Populated when the post-write effect check ran this tick, so the decision is auditable.
struct DynamicResolutionEffectCheck
{
	bool ran = false;
	ResolutionAction after = ResolutionAction::None;
	double fromScale = 0.0;
	double toScale = 0.0;
	double preP95Ms = 0.0;
	double postP95Ms = 0.0;
	double betaObserved = 0.0;
	bool betaMeasured = false;
	const char* verdict = ""; // effective | no_effect | regressed | unmeasured
};

struct DynamicResolutionControllerOutput
{
	ResolutionAction action = ResolutionAction::None;
	double targetScale = 1.0;
	std::string reason;
	DynamicResolutionClassification classification;
	DynamicResolutionEffectCheck effectCheck;
	// First blocking gate of the most relevant withheld action, for audit logging.
	const char* withheldGate = nullptr; // lower | raise | supersample | cpu_release
	const char* withheldCause =
	    nullptr; // dwell | harm_rate | motion_rate | predicted_fit | burned | floor | ceiling | app_paced
	double withheldValue = 0.0;
	double withheldLimit = 0.0;
};

double ClampScaleFraction(double value);
// Per-frame app GPU ms: pre+post submit work, falling back to total minus compositor.
double PerFrameAppGpuMs(double preSubmitGpuMs, double postSubmitGpuMs, double totalRenderGpuMs,
                        double compositorRenderGpuMs);
// Exact percentile of a small sample set (sorts a copy; intended for <=128 values).
double PercentileSorted(std::vector<double> values, double fraction);
const char* ResolutionPressureLabel(ResolutionPressure pressure);
const char* ResolutionActionLabel(ResolutionAction action);
const char* QualityPresetLabel(QualityPreset preset);
bool ScaleDiffers(double a, double b, double tolerance = 0.005);

// Highest scale the controller may write, given a captured baseline. >baseline when
// maxScaleFraction > 1 (supersampling above the user's SteamVR scale).
double CeilingScale(const DynamicResolutionSettings& settings, double baselineScale);

// Seeds settings to a named quality/FPS bias. Does not touch persistence-only fields.
void ApplyQualityPreset(QualityPreset preset, DynamicResolutionSettings& settings);

// Re-derives preset-owned knobs when the profile is pinned to a built-in preset (not Custom), so
// named-preset users track the current preset definition instead of stale saved values.
void ReconcilePresetSettings(DynamicResolutionSettings& settings);

class DynamicResolutionController
{
public:
	void Reset();
	DynamicResolutionControllerOutput Evaluate(const DynamicResolutionControllerInput& input,
	                                           const DynamicResolutionSettings& settings);
	void NoteWrite(ResolutionAction action, double writtenScale, const DynamicResolutionClassification& classification,
	               const DynamicResolutionSettings& settings);

private:
	struct PendingWrite
	{
		ResolutionAction action = ResolutionAction::None;
		double fromScale = 0.0;
		double toScale = 0.0;
		double preP95Ms = 0.0;
	};

	DynamicResolutionClassification Classify(const DynamicResolutionTiming& timing,
	                                         const DynamicResolutionSettings& settings);
	void RunEffectCheck(const DynamicResolutionClassification& cls, double baselineScale,
	                    const DynamicResolutionSettings& settings, DynamicResolutionControllerOutput& out);
	double FloorFor(const DynamicResolutionSettings& settings, double baselineScale) const;
	double PredictP95Ms(const DynamicResolutionClassification& cls, double currentScale, double targetScale) const;
	double SolveScaleForP95(const DynamicResolutionClassification& cls, double currentScale, double targetP95Ms) const;

	std::deque<DynamicResolutionTiming> samples_;
	ResolutionPressure lastPressure_ = ResolutionPressure::Waiting;
	int pressureTicks_ = 0;
	int settleTicksRemaining_ = 0;
	int noEffectCount_ = 0;
	int consecutiveHarmTicks_ = 0;
	int consecutiveCleanTicks_ = 0;
	bool harmSinceWrite_ = false;
	bool raiseStreak_ = false; // last raise passed its effect check; follow up quickly
	double costBeta_ = 1.0;
	double burnedScale_ = 0.0; // raise target that regressed; off-limits while decaying
	int burnedTicksRemaining_ = 0;
	std::optional<PendingWrite> pendingWrite_;
	double lastEvaluatedScale_ = 0.0;
};

} // namespace wkopenvr::dynamicres
