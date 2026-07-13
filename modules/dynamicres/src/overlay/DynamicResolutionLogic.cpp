#include "DynamicResolutionLogic.h"

#include <algorithm>
#include <cmath>

namespace wkopenvr::dynamicres {
namespace {

constexpr int kMinActSamples = 3;
constexpr int kRecentTicks = 3;                     // sub-window for raise-gating motion rate
constexpr double kMinStep = 0.03;
constexpr double kMaxAdaptiveStep = 0.25;
constexpr double kAboveBaselineStep = 0.05;         // gentle additive nudge when supersampling above baseline
constexpr double kBetaMin = 0.4;
constexpr double kBetaMax = 1.6;
constexpr double kBetaEmaAlpha = 0.5;
constexpr double kBetaMinScaleLogDelta = 0.05;      // writes closer than this cannot measure cost sensitivity
constexpr double kBetaMinP95Ms = 0.5;
constexpr double kMotionSustainedRate = 0.30;       // motion-smoothing frame share meaning "engaged"
constexpr double kMotionHarmP95Fraction = 0.90;     // engaged smoothing counts as GPU harm above this p95
constexpr double kDropAttributionP95Fraction = 0.95;// drops attribute to GPU only with p95 at budget, no CPU bits
constexpr double kAppPacedIntervalFraction = 1.15;
constexpr double kSuperOverBudgetRateMax = 0.005;
constexpr double kBurnedScaleMargin = 0.98;
constexpr double kNoEffectBetaFloor = 0.10;         // observed sensitivity below this means lowering did nothing

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

double StepCap(const DynamicResolutionSettings& settings)
{
	return std::clamp(settings.stepFraction, 0.01, kMaxAdaptiveStep);
}

// Frames in this tick harmed for a GPU-attributable reason: a GPU-reason reprojection, compositor
// throttling, or dropped frames while the GPU tail sits at budget and the CPU shows nothing.
int GpuHarmFrames(const DynamicResolutionTiming& tick)
{
	const double budget = std::max(1.0, tick.frameBudgetMs);
	int harmed = tick.framesWithGpuReproj + tick.framesThrottled;
	if (tick.appGpuP95Ms >= budget * kDropAttributionP95Fraction && tick.framesWithCpuReproj == 0) {
		harmed += tick.framesWithDrops;
	}
	return harmed;
}

// Frames harmed by any non-motion condition. Motion smoothing is gated separately by rate so a
// sustained-but-recoverable half-rate state does not read as damage.
int HarmFrames(const DynamicResolutionTiming& tick)
{
	return tick.framesWithGpuReproj + tick.framesThrottled + tick.framesWithCpuReproj + tick.framesWithDrops +
	       tick.framesMispresented;
}

bool TickGpuHarmed(const DynamicResolutionTiming& tick, const DynamicResolutionSettings& settings)
{
	const double frames = std::max(1, tick.framesConsidered);
	const double budget = std::max(1.0, tick.frameBudgetMs);
	if (static_cast<double>(GpuHarmFrames(tick)) / frames >= settings.gpuHarmRateFraction) return true;
	return static_cast<double>(tick.framesWithMotion) / frames >= kMotionSustainedRate &&
	       tick.appGpuP95Ms >= budget * kMotionHarmP95Fraction;
}

} // namespace

double ClampScaleFraction(double value)
{
	if (!std::isfinite(value)) return 1.0;
	return std::clamp(value, 0.10, 1.50);
}

double PerFrameAppGpuMs(double preSubmitGpuMs, double postSubmitGpuMs, double totalRenderGpuMs,
                        double compositorRenderGpuMs)
{
	const double pre = std::isfinite(preSubmitGpuMs) ? std::max(0.0, preSubmitGpuMs) : 0.0;
	const double post = std::isfinite(postSubmitGpuMs) ? std::max(0.0, postSubmitGpuMs) : 0.0;
	if (pre > 0.0 || post > 0.0) {
		return pre + post;
	}
	const double total = std::isfinite(totalRenderGpuMs) ? std::max(0.0, totalRenderGpuMs) : 0.0;
	const double compositor = std::isfinite(compositorRenderGpuMs) ? std::max(0.0, compositorRenderGpuMs) : 0.0;
	return std::max(0.0, total - compositor);
}

double PercentileSorted(std::vector<double> values, double fraction)
{
	if (values.empty()) return 0.0;
	std::sort(values.begin(), values.end());
	const double clamped = std::clamp(fraction, 0.0, 1.0);
	const size_t index = static_cast<size_t>(std::max(
	    0.0, std::ceil(clamped * static_cast<double>(values.size())) - 1.0));
	return values[std::min(index, values.size() - 1)];
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
		case ResolutionPressure::Steady:
			return "Steady";
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

const char* QualityPresetLabel(QualityPreset preset)
{
	switch (preset) {
		case QualityPreset::MaxFps:
			return "Max FPS";
		case QualityPreset::FpsFirst:
			return "FPS-first";
		case QualityPreset::Balanced:
			return "Balanced";
		case QualityPreset::Quality:
			return "Quality";
		case QualityPreset::Custom:
		default:
			return "Custom";
	}
}

double CeilingScale(const DynamicResolutionSettings& settings, double baselineScale)
{
	const double baseline = std::max(0.1, baselineScale);
	const double fraction = std::isfinite(settings.maxScaleFraction) ? std::max(1.0, settings.maxScaleFraction) : 1.0;
	return std::max(baseline, ClampScaleFraction(baseline * fraction));
}

void ApplyQualityPreset(QualityPreset preset, DynamicResolutionSettings& settings)
{
	settings.qualityPreset = preset;
	switch (preset) {
		case QualityPreset::MaxFps:
			settings.minScaleFraction = 0.50;
			settings.maxScaleFraction = 1.00;
			settings.gpuHarmRateFraction = 0.010;
			settings.raiseHarmRateFraction = 0.000;
			settings.motionRateFraction = 0.02;
			settings.lowerTargetFraction = 0.85;
			settings.raiseSafetyFraction = 0.85;
			settings.lowerRequiredTicks = 2;
			settings.raiseRequiredTicks = 5;
			settings.burnedDecayTicks = 180;
			settings.settleTicks = 1;
			break;
		case QualityPreset::FpsFirst:
			settings.minScaleFraction = 0.60;
			settings.maxScaleFraction = 1.50;
			settings.gpuHarmRateFraction = 0.015;
			settings.raiseHarmRateFraction = 0.002;
			settings.motionRateFraction = 0.05;
			settings.lowerTargetFraction = 0.90;
			settings.raiseSafetyFraction = 0.90;
			settings.lowerRequiredTicks = 2;
			settings.raiseRequiredTicks = 4;
			settings.burnedDecayTicks = 120;
			settings.settleTicks = 2;
			break;
		case QualityPreset::Balanced:
			settings.minScaleFraction = 0.70;
			settings.maxScaleFraction = 1.50;
			settings.gpuHarmRateFraction = 0.020;
			settings.raiseHarmRateFraction = 0.002;
			settings.motionRateFraction = 0.05;
			settings.lowerTargetFraction = 0.92;
			settings.raiseSafetyFraction = 0.90;
			settings.lowerRequiredTicks = 3;
			settings.raiseRequiredTicks = 3;
			settings.burnedDecayTicks = 120;
			settings.settleTicks = 2;
			break;
		case QualityPreset::Quality:
			settings.minScaleFraction = 0.80;
			settings.maxScaleFraction = 1.50;
			settings.gpuHarmRateFraction = 0.030;
			settings.raiseHarmRateFraction = 0.005;
			settings.motionRateFraction = 0.10;
			settings.lowerTargetFraction = 0.94;
			settings.raiseSafetyFraction = 0.92;
			settings.lowerRequiredTicks = 4;
			settings.raiseRequiredTicks = 3;
			settings.burnedDecayTicks = 90;
			settings.settleTicks = 3;
			break;
		case QualityPreset::Custom:
		default:
			break;
	}
}

void ReconcilePresetSettings(DynamicResolutionSettings& settings)
{
	// A profile pinned to a built-in preset should track the current preset definition, not stale
	// knob values saved by an older build. Editing any knob in the UI moves the profile to Custom,
	// so a named preset here means the user never customized -- safe to re-derive.
	if (settings.qualityPreset != QualityPreset::Custom) {
		ApplyQualityPreset(settings.qualityPreset, settings);
	}
}

void DynamicResolutionController::Reset()
{
	samples_.clear();
	lastPressure_ = ResolutionPressure::Waiting;
	pressureTicks_ = 0;
	settleTicksRemaining_ = 0;
	noEffectCount_ = 0;
	consecutiveHarmTicks_ = 0;
	consecutiveCleanTicks_ = 0;
	harmSinceWrite_ = false;
	raiseStreak_ = false;
	costBeta_ = 1.0;
	burnedScale_ = 0.0;
	burnedTicksRemaining_ = 0;
	pendingWrite_.reset();
	lastEvaluatedScale_ = 0.0;
}

double DynamicResolutionController::PredictP95Ms(const DynamicResolutionClassification& cls, double currentScale,
                                                 double targetScale) const
{
	const double p95 = std::max(0.0, cls.appGpuP95Ms);
	const double beta = std::clamp(costBeta_, kBetaMin, kBetaMax);
	const double ratio = targetScale / std::max(0.1, currentScale);
	const double predicted = p95 * std::pow(ratio, beta);
	return std::isfinite(predicted) ? predicted : p95;
}

double DynamicResolutionController::SolveScaleForP95(const DynamicResolutionClassification& cls, double currentScale,
                                                     double targetP95Ms) const
{
	const double p95 = std::max(cls.appGpuP95Ms, 0.1);
	const double beta = std::clamp(costBeta_, kBetaMin, kBetaMax);
	const double solved = currentScale * std::pow(targetP95Ms / p95, 1.0 / beta);
	return std::isfinite(solved) ? solved : currentScale;
}

void DynamicResolutionController::RunEffectCheck(const DynamicResolutionClassification& cls, double baselineScale,
                                                 const DynamicResolutionSettings& settings,
                                                 DynamicResolutionControllerOutput& out)
{
	const PendingWrite pending = *pendingWrite_;
	pendingWrite_.reset();

	DynamicResolutionEffectCheck& check = out.effectCheck;
	check.ran = true;
	check.after = pending.action;
	check.fromScale = pending.fromScale;
	check.toScale = pending.toScale;
	check.preP95Ms = pending.preP95Ms;
	check.postP95Ms = cls.appGpuP95Ms;

	// A raise that brought harm back gets reverted exactly, and its target is off-limits until the
	// burn decays; otherwise a marginal scene oscillates between two scales forever.
	if (pending.action == ResolutionAction::Raise && harmSinceWrite_) {
		check.verdict = "regressed";
		burnedScale_ = (burnedTicksRemaining_ > 0 && burnedScale_ > 0.0) ? std::min(burnedScale_, pending.toScale)
		                                                                 : pending.toScale;
		burnedTicksRemaining_ = std::max(1, settings.burnedDecayTicks);
		raiseStreak_ = false;
		out.action = ResolutionAction::Lower;
		out.targetScale = std::min(pending.fromScale, pending.toScale);
		out.reason = "raise regressed";
		return;
	}

	const double logScale =
	    (pending.fromScale > 0.0 && pending.toScale > 0.0) ? std::log(pending.toScale / pending.fromScale) : 0.0;
	const bool measurable = std::abs(logScale) >= kBetaMinScaleLogDelta && check.preP95Ms >= kBetaMinP95Ms &&
	                        check.postP95Ms >= kBetaMinP95Ms;
	if (!measurable) {
		check.verdict = "unmeasured";
		if (pending.action == ResolutionAction::Raise) raiseStreak_ = true;
		return;
	}

	const double betaObserved = std::log(check.postP95Ms / check.preP95Ms) / logScale;
	check.betaObserved = betaObserved;
	check.betaMeasured = std::isfinite(betaObserved);
	// A measurement window that itself contained harm confounds the sensitivity estimate.
	if (check.betaMeasured && !harmSinceWrite_) {
		costBeta_ = std::clamp(costBeta_ * (1.0 - kBetaEmaAlpha) + betaObserved * kBetaEmaAlpha, kBetaMin, kBetaMax);
	}

	if (pending.action == ResolutionAction::Lower) {
		if (check.betaMeasured && betaObserved < kNoEffectBetaFloor) {
			check.verdict = "no_effect";
			++noEffectCount_;
			if (noEffectCount_ >= std::max(1, settings.noEffectLimit)) {
				out.action = ResolutionAction::NoEffect;
				out.targetScale = baselineScale;
				out.reason = "lowering did not reduce GPU time";
			}
			return;
		}
		noEffectCount_ = 0;
		check.verdict = "effective";
		return;
	}

	check.verdict = "effective";
	raiseStreak_ = true;
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

	lastEvaluatedScale_ = input.currentScale;
	out.classification = Classify(input.timing, settings);
	if (out.classification.pressure == lastPressure_) {
		++pressureTicks_;
	}
	else {
		lastPressure_ = out.classification.pressure;
		pressureTicks_ = 1;
	}

	if (burnedTicksRemaining_ > 0) {
		--burnedTicksRemaining_;
		if (burnedTicksRemaining_ == 0) burnedScale_ = 0.0;
	}

	if (settleTicksRemaining_ > 0) {
		--settleTicksRemaining_;
		out.reason = "settling";
		return out;
	}

	if (pendingWrite_.has_value() && out.classification.tickCount >= MinWindowSamples(settings)) {
		RunEffectCheck(out.classification, input.baselineScale, settings, out);
		if (out.action != ResolutionAction::None) return out;
	}

	const double baseline = std::max(0.1, input.baselineScale);
	const double ceiling = CeilingScale(settings, baseline);
	const double current = std::clamp(input.currentScale, 0.1, std::max(2.0, ceiling));
	const double floor = FloorFor(settings, baseline);
	DynamicResolutionClassification& cls = out.classification;
	const double budget = std::max(1.0, cls.frameBudgetMs);

	if (cls.tickCount < MinWindowSamples(settings)) return out;

	auto withhold = [&out](const char* gate, const char* cause, double value, double limit) {
		if (out.withheldGate != nullptr) return;
		out.withheldGate = gate;
		out.withheldCause = cause;
		out.withheldValue = value;
		out.withheldLimit = limit;
	};

	// -- Lower: only proven, sustained GPU harm; one solve-sized cut. --
	if (cls.pressure == ResolutionPressure::GpuBound) {
		const int requiredHarmTicks = std::max(2, settings.lowerRequiredTicks);
		if (cls.consecutiveHarmTicks < requiredHarmTicks) {
			withhold("lower", "dwell", static_cast<double>(cls.consecutiveHarmTicks),
			         static_cast<double>(requiredHarmTicks));
		}
		else if (current <= floor + 0.005) {
			withhold("lower", "floor", current, floor);
		}
		else {
			double target = SolveScaleForP95(cls, current, settings.lowerTargetFraction * budget);
			if (!(target < current - kMinStep)) {
				// Harm is proven while p95 hides under the target; the solve cannot size the cut.
				target = current * (1.0 - std::max(0.05, StepCap(settings) * 0.5));
			}
			target = std::max(target, current * (1.0 - StepCap(settings)));
			target = std::min(target, current - kMinStep);
			out.action = ResolutionAction::Lower;
			out.targetScale = std::max(floor, target);
			out.reason = "GPU-bound";
			return out;
		}
	}

	// Shared raise-target sizing: capped by the per-write step, the burn, and the predicted fit.
	auto raiseTarget = [&](double cap, bool& fitCapped, bool& burnCapped) {
		double candidate = std::min(cap, current * (1.0 + StepCap(settings)));
		burnCapped = false;
		fitCapped = false;
		if (burnedTicksRemaining_ > 0 && burnedScale_ > 0.0 && candidate >= burnedScale_ * kBurnedScaleMargin) {
			candidate = burnedScale_ * kBurnedScaleMargin;
			burnCapped = true;
		}
		if (cls.appGpuP95Ms >= 0.1) {
			const double fit = SolveScaleForP95(cls, current, settings.raiseSafetyFraction * budget);
			if (fit < candidate) {
				candidate = fit;
				fitCapped = true;
			}
		}
		cls.predictedRaiseP95Ms = PredictP95Ms(cls, current, candidate);
		return candidate;
	};

	// -- CPU-bound with GPU headroom: recover toward baseline (resolution cannot fix CPU). --
	if (settings.releaseOnCpuBound && settings.allowRaiseBack && cls.pressure == ResolutionPressure::CpuBound &&
	    cls.gpuHasHeadroom && current < baseline - 0.005) {
		if (pressureTicks_ < std::max(1, settings.cpuReleaseTicks)) {
			withhold("cpu_release", "dwell", static_cast<double>(pressureTicks_),
			         static_cast<double>(std::max(1, settings.cpuReleaseTicks)));
		}
		else {
			bool fitCapped = false;
			bool burnCapped = false;
			const double target = raiseTarget(baseline, fitCapped, burnCapped);
			if (target - current >= kMinStep) {
				out.action = ResolutionAction::Raise;
				out.targetScale = target;
				out.reason = "CPU-bound; GPU headroom";
				return out;
			}
			if (fitCapped) {
				withhold("cpu_release", "predicted_fit", cls.predictedRaiseP95Ms,
				         settings.raiseSafetyFraction * budget);
			}
			else if (burnCapped) {
				withhold("cpu_release", "burned", target, burnedScale_ * kBurnedScaleMargin);
			}
		}
	}

	// -- Recover toward baseline: harm-free window plus a predicted fit at the target. CPU-bound
	// recovery goes exclusively through the release path above, honoring its own toggle. --
	if (settings.allowRaiseBack && cls.pressure != ResolutionPressure::CpuBound && current < baseline - 0.005) {
		const int requiredCleanTicks = raiseStreak_ ? 1 : std::max(1, settings.raiseRequiredTicks);
		const bool motionOk = cls.recentMotionRate < settings.motionRateFraction ||
		                      (cls.motionSmoothingEngaged && cls.gpuHasHeadroom);
		if (cls.harmRate > settings.raiseHarmRateFraction) {
			withhold("raise", "harm_rate", cls.harmRate, settings.raiseHarmRateFraction);
		}
		else if (!motionOk) {
			withhold("raise", "motion_rate", cls.recentMotionRate, settings.motionRateFraction);
		}
		else if (cls.consecutiveCleanTicks < requiredCleanTicks) {
			withhold("raise", "dwell", static_cast<double>(cls.consecutiveCleanTicks),
			         static_cast<double>(requiredCleanTicks));
		}
		else {
			bool fitCapped = false;
			bool burnCapped = false;
			const double target = raiseTarget(baseline, fitCapped, burnCapped);
			if (target - current >= kMinStep) {
				out.action = ResolutionAction::Raise;
				out.targetScale = target;
				out.reason = "GPU headroom";
				return out;
			}
			if (fitCapped) {
				withhold("raise", "predicted_fit", cls.predictedRaiseP95Ms, settings.raiseSafetyFraction * budget);
			}
			else if (burnCapped) {
				withhold("raise", "burned", target, burnedScale_ * kBurnedScaleMargin);
			}
		}
		return out;
	}

	// -- Supersample above baseline: a spotless window, a reachable p95 gate, and a predicted fit.
	// The Lower path stays above this one, so any proven harm falls straight back. --
	if (settings.allowRaiseBack && current >= baseline - 0.005 && current < ceiling - 0.005) {
		const double superBudgetMs = settings.raiseAboveBaselineFraction * budget;
		if (cls.appPaced) {
			withhold("supersample", "app_paced", cls.clientFrameIntervalMs, kAppPacedIntervalFraction * budget);
		}
		else if (cls.harmRate > 0.0) {
			withhold("supersample", "harm_rate", cls.harmRate, 0.0);
		}
		else if (cls.recentMotionRate > 0.0) {
			withhold("supersample", "motion_rate", cls.recentMotionRate, 0.0);
		}
		else if (cls.overBudgetRate > kSuperOverBudgetRateMax) {
			withhold("supersample", "harm_rate", cls.overBudgetRate, kSuperOverBudgetRateMax);
		}
		else if (cls.appGpuP95Ms > superBudgetMs) {
			withhold("supersample", "predicted_fit", cls.appGpuP95Ms, superBudgetMs);
		}
		else if (cls.consecutiveCleanTicks < std::max(1, settings.raiseAboveBaselineTicks)) {
			withhold("supersample", "dwell", static_cast<double>(cls.consecutiveCleanTicks),
			         static_cast<double>(std::max(1, settings.raiseAboveBaselineTicks)));
		}
		else {
			double target = std::min(ceiling, current + std::min(kAboveBaselineStep, StepCap(settings)));
			if (burnedTicksRemaining_ > 0 && burnedScale_ > 0.0 && target >= burnedScale_ * kBurnedScaleMargin) {
				withhold("supersample", "burned", target, burnedScale_ * kBurnedScaleMargin);
			}
			else {
				cls.predictedRaiseP95Ms = PredictP95Ms(cls, current, target);
				if (cls.predictedRaiseP95Ms > superBudgetMs) {
					withhold("supersample", "predicted_fit", cls.predictedRaiseP95Ms, superBudgetMs);
				}
				else if (target - current > 0.005) {
					out.action = ResolutionAction::Raise;
					out.targetScale = target;
					out.reason = "GPU headroom (supersample)";
					return out;
				}
			}
		}
	}

	return out;
}

void DynamicResolutionController::NoteWrite(ResolutionAction action, double writtenScale,
                                            const DynamicResolutionClassification& classification,
                                            const DynamicResolutionSettings& settings)
{
	samples_.clear();
	settleTicksRemaining_ = std::max(0, settings.settleTicks);
	pressureTicks_ = 0;
	lastPressure_ = ResolutionPressure::Waiting;
	consecutiveHarmTicks_ = 0;
	consecutiveCleanTicks_ = 0;
	harmSinceWrite_ = false;
	if (action == ResolutionAction::Lower || action == ResolutionAction::Raise) {
		PendingWrite pending;
		pending.action = action;
		pending.fromScale = lastEvaluatedScale_ > 0.0 ? lastEvaluatedScale_ : writtenScale;
		pending.toScale = writtenScale;
		pending.preP95Ms = classification.appGpuP95Ms;
		pendingWrite_ = pending;
		if (action == ResolutionAction::Lower) raiseStreak_ = false;
	}
	else {
		pendingWrite_.reset();
	}
}

DynamicResolutionClassification DynamicResolutionController::Classify(const DynamicResolutionTiming& timing,
                                                                      const DynamicResolutionSettings& settings)
{
	DynamicResolutionClassification out;
	out.frameBudgetMs = timing.frameBudgetMs > 1.0 ? timing.frameBudgetMs : 1000.0 / 90.0;

	if (timing.valid && timing.framesConsidered > 0) {
		samples_.push_back(timing);
		const size_t window = static_cast<size_t>(std::max(1, settings.windowSize));
		while (samples_.size() > window) {
			samples_.pop_front();
		}
		if (TickGpuHarmed(timing, settings)) {
			++consecutiveHarmTicks_;
			harmSinceWrite_ = true;
			raiseStreak_ = false;
		}
		else {
			consecutiveHarmTicks_ = 0;
		}
		if (HarmFrames(timing) == 0) {
			++consecutiveCleanTicks_;
		}
		else {
			consecutiveCleanTicks_ = 0;
		}
	}

	std::vector<double> p50s;
	std::vector<double> p95s;
	std::vector<double> intervals;
	std::vector<double> idles;
	p50s.reserve(samples_.size());
	p95s.reserve(samples_.size());
	long long framesTotal = 0;
	long long gpuHarmFrames = 0;
	long long cpuHarmFrames = 0;
	long long motionFrames = 0;
	long long dropFrames = 0;
	long long multiPresentFrames = 0;
	long long overBudgetFrames = 0;
	long long harmFrames = 0;
	long long recentFrames = 0;
	long long recentMotionFrames = 0;
	double peak = 0.0;
	const size_t recentStart =
	    samples_.size() > static_cast<size_t>(kRecentTicks) ? samples_.size() - static_cast<size_t>(kRecentTicks) : 0;
	for (size_t i = 0; i < samples_.size(); ++i) {
		const DynamicResolutionTiming& sample = samples_[i];
		p50s.push_back(sample.appGpuP50Ms);
		p95s.push_back(sample.appGpuP95Ms);
		if (sample.clientFrameIntervalMs > 0.0) intervals.push_back(sample.clientFrameIntervalMs);
		if (sample.compositorIdleCpuMs > 0.0) idles.push_back(sample.compositorIdleCpuMs);
		peak = std::max(peak, sample.appGpuMaxMs);
		framesTotal += sample.framesConsidered;
		gpuHarmFrames += GpuHarmFrames(sample);
		cpuHarmFrames += sample.framesWithCpuReproj;
		motionFrames += sample.framesWithMotion;
		dropFrames += sample.framesWithDrops;
		multiPresentFrames += sample.framesMultiPresented;
		overBudgetFrames += sample.framesOverBudget;
		harmFrames += HarmFrames(sample);
		if (i >= recentStart) {
			recentFrames += sample.framesConsidered;
			recentMotionFrames += sample.framesWithMotion;
		}
	}

	out.tickCount = static_cast<int>(samples_.size());
	out.framesTotal = static_cast<int>(framesTotal);
	out.appGpuP50Ms = Median(p50s);
	out.appGpuP95Ms = Median(p95s);
	out.appGpuPeakMs = peak;
	out.clientFrameIntervalMs = Median(intervals);
	out.compositorIdleCpuMs = Median(idles);
	const double frameDenominator = std::max(1.0, static_cast<double>(framesTotal));
	out.gpuHarmRate = static_cast<double>(gpuHarmFrames) / frameDenominator;
	out.cpuHarmRate = static_cast<double>(cpuHarmFrames) / frameDenominator;
	out.motionRate = static_cast<double>(motionFrames) / frameDenominator;
	out.recentMotionRate = static_cast<double>(recentMotionFrames) / std::max(1.0, static_cast<double>(recentFrames));
	out.dropRate = static_cast<double>(dropFrames) / frameDenominator;
	out.multiPresentRate = static_cast<double>(multiPresentFrames) / frameDenominator;
	out.overBudgetRate = static_cast<double>(overBudgetFrames) / frameDenominator;
	out.harmRate = static_cast<double>(harmFrames) / frameDenominator;
	out.consecutiveHarmTicks = consecutiveHarmTicks_;
	out.consecutiveCleanTicks = consecutiveCleanTicks_;
	out.tickHarmed = consecutiveHarmTicks_ > 0;
	out.costBeta = costBeta_;

	if (out.tickCount < MinWindowSamples(settings)) {
		out.pressure = ResolutionPressure::Waiting;
		return out;
	}

	const double budget = std::max(1.0, out.frameBudgetMs);
	out.gpuHasHeadroom = out.appGpuP95Ms <= budget * settings.raiseSafetyFraction;
	out.motionSmoothingEngaged = out.motionRate >= kMotionSustainedRate;
	out.appPaced = harmFrames == 0 && out.clientFrameIntervalMs > budget * kAppPacedIntervalFraction &&
	               out.recentMotionRate < settings.motionRateFraction && out.multiPresentRate >= 0.5;
	out.cpuStalled = out.clientFrameIntervalMs > budget * settings.cpuStallFraction && out.gpuHasHeadroom &&
	                 !out.appPaced && !out.motionSmoothingEngaged;

	// Pressure names the evidence; dwell requirements live in Evaluate. GPU-bound means the most
	// recent tick carried proven GPU-attributed harm, not that utilization looked high.
	if (out.consecutiveHarmTicks >= 1) {
		out.pressure = ResolutionPressure::GpuBound;
	}
	else if (out.cpuHarmRate >= settings.gpuHarmRateFraction || out.cpuStalled) {
		out.pressure = ResolutionPressure::CpuBound;
	}
	else if (out.harmRate <= settings.raiseHarmRateFraction && out.recentMotionRate < settings.motionRateFraction &&
	         out.gpuHasHeadroom) {
		out.pressure = ResolutionPressure::Headroom;
	}
	else {
		out.pressure = ResolutionPressure::Steady;
	}
	return out;
}

double DynamicResolutionController::FloorFor(const DynamicResolutionSettings& settings, double baselineScale) const
{
	const double fraction = ClampScaleFraction(settings.minScaleFraction);
	return std::max(0.1, baselineScale * fraction);
}

} // namespace wkopenvr::dynamicres
