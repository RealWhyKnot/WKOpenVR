#include "DynamicResolutionLogic.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace wkopenvr::dynamicres {
namespace {

constexpr uint32_t kReprojectionReasonCpu = 0x01;
constexpr uint32_t kReprojectionReasonGpu = 0x02;
constexpr uint32_t kReprojectionMotion = 0x08;
constexpr uint32_t kActualReprojectionMask = kReprojectionReasonCpu | kReprojectionReasonGpu | kReprojectionMotion;
constexpr int kMinActSamples = 3;
constexpr double kRaiseTargetUtil = 0.78;
constexpr double kLowerTargetUtil = 0.80;
constexpr double kRaiseGain = 0.60;
constexpr double kLowerGain = 0.80;
constexpr double kMinStep = 0.03;
constexpr double kMaxAdaptiveStep = 0.25;

double Median(std::vector<double> values)
{
	if (values.empty()) return 0.0;
	std::sort(values.begin(), values.end());
	const size_t mid = values.size() / 2;
	if ((values.size() % 2) == 1) return values[mid];
	return (values[mid - 1] + values[mid]) * 0.5;
}

int MinWindowSamples(const DynamicResolutionSettings& settings)
{
	return std::max(1, std::min(settings.windowSize, kMinActSamples));
}

int RaiseUnstableTolerance(const DynamicResolutionSettings& settings)
{
	return std::max(1, std::max(1, settings.windowSize) / 4);
}

double Utilization(const DynamicResolutionClassification& classification)
{
	const double budget = std::max(1.0, classification.frameBudgetMs);
	return std::max(0.0, classification.medianAppGpuMs) / budget;
}

double StepCap(const DynamicResolutionSettings& settings)
{
	return std::clamp(settings.stepFraction, 0.01, kMaxAdaptiveStep);
}

double ClampAdaptiveStep(double rawStep, double cap)
{
	const double minStep = std::min(kMinStep, cap);
	return std::clamp(std::isfinite(rawStep) ? rawStep : minStep, minStep, cap);
}

} // namespace

double ClampScaleFraction(double value)
{
	if (!std::isfinite(value)) return 1.0;
	return std::clamp(value, 0.10, 1.50);
}

double ComputeAppGpuMs(const DynamicResolutionTiming& sample)
{
	if (!sample.valid) return 0.0;
	const double preSubmit = std::isfinite(sample.preSubmitGpuMs) ? std::max(0.0, sample.preSubmitGpuMs) : 0.0;
	const double postSubmit = std::isfinite(sample.postSubmitGpuMs) ? std::max(0.0, sample.postSubmitGpuMs) : 0.0;
	if (preSubmit > 0.0 || postSubmit > 0.0) {
		return preSubmit + postSubmit;
	}
	const double total = std::max(0.0, sample.totalRenderGpuMs);
	const double compositor = std::max(0.0, sample.compositorRenderGpuMs);
	return std::max(0.0, total - compositor);
}

bool IsUnstableTiming(const DynamicResolutionTiming& sample)
{
	return sample.droppedFrames > 0 || sample.mispresentedFrames > 0 ||
	       (sample.reprojectionFlags & kActualReprojectionMask) != 0 || sample.framePresents > 1;
}

const char* ResolutionPressureLabel(ResolutionPressure pressure)
{
	switch (pressure) {
		case ResolutionPressure::GpuBound:
			return "GPU-bound";
		case ResolutionPressure::CpuBound:
			return "CPU-bound";
		case ResolutionPressure::Headroom:
			return "Headroom";
		case ResolutionPressure::Waiting:
		default:
			return "Waiting";
	}
}

const char* ResolutionActionLabel(ResolutionAction action)
{
	switch (action) {
		case ResolutionAction::Lower:
			return "Lower";
		case ResolutionAction::Raise:
			return "Raise";
		case ResolutionAction::Restore:
			return "Restore";
		case ResolutionAction::ExternalOverride:
			return "External override";
		case ResolutionAction::NoEffect:
			return "No effect";
		case ResolutionAction::None:
		default:
			return "None";
	}
}

bool ScaleDiffers(double a, double b, double tolerance)
{
	if (!std::isfinite(a) || !std::isfinite(b)) return false;
	return std::abs(a - b) > std::max(0.0, tolerance);
}

void DynamicResolutionController::Reset()
{
	samples_.clear();
	lastPressure_ = ResolutionPressure::Waiting;
	pressureTicks_ = 0;
	settleTicksRemaining_ = 0;
	noEffectCount_ = 0;
	effectCheckPending_ = false;
	lastLowerAppGpuMs_ = 0.0;
	lastWrittenScale_ = 0.0;
	activeDirection_ = ResolutionAction::None;
}

DynamicResolutionControllerOutput DynamicResolutionController::Evaluate(const DynamicResolutionControllerInput& input,
                                                                        const DynamicResolutionSettings& settings)
{
	DynamicResolutionControllerOutput out;
	out.targetScale = input.currentScale;

	if (!input.sceneRunning) {
		out.action = ResolutionAction::Restore;
		out.targetScale = input.baselineScale;
		out.reason = "scene stopped";
		return out;
	}

	if (input.externalOverride) {
		out.action = ResolutionAction::ExternalOverride;
		out.targetScale = input.currentScale;
		out.reason = "SteamVR value changed";
		return out;
	}

	out.classification = Classify(input.timing, settings);
	if (out.classification.pressure == lastPressure_) {
		++pressureTicks_;
	}
	else {
		lastPressure_ = out.classification.pressure;
		pressureTicks_ = 1;
	}

	if (settleTicksRemaining_ > 0) {
		--settleTicksRemaining_;
		out.reason = "settling";
		return out;
	}

	if (effectCheckPending_ && out.classification.sampleCount >= MinWindowSamples(settings)) {
		effectCheckPending_ = false;
		if (lastLowerAppGpuMs_ > 0.0 && out.classification.medianAppGpuMs >= lastLowerAppGpuMs_ * 0.97) {
			++noEffectCount_;
			if (noEffectCount_ >= std::max(1, settings.noEffectLimit)) {
				out.action = ResolutionAction::NoEffect;
				out.targetScale = input.baselineScale;
				out.reason = "lowering did not reduce GPU time";
				return out;
			}
		}
		else {
			noEffectCount_ = 0;
		}
	}

	const double baseline = std::max(0.1, input.baselineScale);
	const double current = std::clamp(input.currentScale, 0.1, std::max(2.0, baseline));
	const double floor = FloorFor(settings, baseline);
	if (out.classification.sampleCount >= MinWindowSamples(settings) &&
	    out.classification.pressure != ResolutionPressure::GpuBound &&
	    out.classification.pressure != ResolutionPressure::Headroom) {
		activeDirection_ = ResolutionAction::None;
	}

	if (out.classification.pressure == ResolutionPressure::GpuBound &&
	    pressureTicks_ >=
	        (activeDirection_ == ResolutionAction::Lower ? 1 : std::max(1, settings.lowerRequiredTicks)) &&
	    current > floor + 0.005) {
		out.action = ResolutionAction::Lower;
		out.targetScale = std::max(floor, current * (1.0 - LowerStepFor(settings, out.classification)));
		out.reason = "GPU-bound";
		return out;
	}

	if (settings.releaseOnCpuBound && settings.allowRaiseBack &&
	    out.classification.pressure == ResolutionPressure::CpuBound && out.classification.gpuHasHeadroom &&
	    pressureTicks_ >= std::max(1, settings.cpuReleaseTicks) && current < baseline - 0.005) {
		out.action = ResolutionAction::Raise;
		out.targetScale = std::min(baseline, current * (1.0 + RaiseStepFor(settings, out.classification)));
		out.reason = "CPU-bound; GPU headroom";
		return out;
	}

	if (settings.allowRaiseBack && out.classification.pressure == ResolutionPressure::Headroom &&
	    pressureTicks_ >=
	        (activeDirection_ == ResolutionAction::Raise ? 1 : std::max(1, settings.raiseRequiredTicks)) &&
	    current < baseline - 0.005) {
		out.action = ResolutionAction::Raise;
		out.targetScale = std::min(baseline, current * (1.0 + RaiseStepFor(settings, out.classification)));
		out.reason = "GPU headroom";
		return out;
	}

	return out;
}

void DynamicResolutionController::NoteWrite(ResolutionAction action, double writtenScale,
                                            const DynamicResolutionClassification& classification,
                                            const DynamicResolutionSettings& settings)
{
	lastWrittenScale_ = writtenScale;
	samples_.clear();
	settleTicksRemaining_ = std::max(0, settings.settleTicks);
	pressureTicks_ = 0;
	lastPressure_ = ResolutionPressure::Waiting;
	activeDirection_ =
	    (action == ResolutionAction::Lower || action == ResolutionAction::Raise) ? action : ResolutionAction::None;
	if (action == ResolutionAction::Lower) {
		lastLowerAppGpuMs_ =
		    classification.medianAppGpuMs > 0.0 ? classification.medianAppGpuMs : classification.appGpuMs;
		effectCheckPending_ = true;
	}
}

DynamicResolutionClassification DynamicResolutionController::Classify(const DynamicResolutionTiming& timing,
                                                                      const DynamicResolutionSettings& settings)
{
	DynamicResolutionClassification out;
	out.appGpuMs = ComputeAppGpuMs(timing);
	out.frameBudgetMs = timing.frameBudgetMs > 1.0 ? timing.frameBudgetMs : 1000.0 / 90.0;
	out.unstable = IsUnstableTiming(timing);

	if (timing.valid) {
		samples_.push_back(timing);
		const size_t window = static_cast<size_t>(std::max(1, settings.windowSize));
		while (samples_.size() > window) {
			samples_.pop_front();
		}
	}

	std::vector<double> appGpu;
	appGpu.reserve(samples_.size());
	for (const DynamicResolutionTiming& sample : samples_) {
		appGpu.push_back(ComputeAppGpuMs(sample));
		if (IsUnstableTiming(sample)) ++out.unstableSamples;
		if ((sample.reprojectionFlags & kReprojectionReasonCpu) != 0) ++out.cpuReasonSamples;
		if ((sample.reprojectionFlags & kReprojectionReasonGpu) != 0) ++out.gpuReasonSamples;
		if ((sample.reprojectionFlags & kReprojectionMotion) != 0) out.motionSmoothingActive = true;
	}

	out.sampleCount = static_cast<int>(samples_.size());
	out.medianAppGpuMs = Median(appGpu);
	if (out.sampleCount < MinWindowSamples(settings)) {
		out.pressure = ResolutionPressure::Waiting;
		return out;
	}

	const double budget = std::max(1.0, out.frameBudgetMs);
	const bool highGpu = out.medianAppGpuMs >= budget * settings.lowerGpuBudgetFraction;
	const bool lowGpu = out.medianAppGpuMs <= budget * settings.headroomGpuBudgetFraction;
	const bool explicitGpuReason = out.gpuReasonSamples > 0;
	const int raiseUnstableTolerance = RaiseUnstableTolerance(settings);
	out.gpuHasHeadroom = lowGpu;

	if (out.unstableSamples > 0 && explicitGpuReason && highGpu) {
		out.pressure = ResolutionPressure::GpuBound;
	}
	else if (lowGpu && out.gpuReasonSamples == 0 && out.unstableSamples <= raiseUnstableTolerance) {
		out.pressure = ResolutionPressure::Headroom;
	}
	else if (out.unstableSamples > 0) {
		out.pressure = ResolutionPressure::CpuBound;
	}
	else {
		out.pressure = ResolutionPressure::Waiting;
	}
	return out;
}

double DynamicResolutionController::FloorFor(const DynamicResolutionSettings& settings, double baselineScale) const
{
	const double fraction = ClampScaleFraction(settings.minScaleFraction);
	return std::max(0.1, baselineScale * fraction);
}

double DynamicResolutionController::RaiseStepFor(const DynamicResolutionSettings& settings,
                                                 const DynamicResolutionClassification& classification) const
{
	return ClampAdaptiveStep(kRaiseGain * (kRaiseTargetUtil - Utilization(classification)), StepCap(settings));
}

double DynamicResolutionController::LowerStepFor(const DynamicResolutionSettings& settings,
                                                 const DynamicResolutionClassification& classification) const
{
	return ClampAdaptiveStep(kLowerGain * (Utilization(classification) - kLowerTargetUtil), StepCap(settings));
}

} // namespace wkopenvr::dynamicres
