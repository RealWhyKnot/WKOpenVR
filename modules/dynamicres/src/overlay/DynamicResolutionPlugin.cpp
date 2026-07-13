#include "DynamicResolutionPlugin.h"

#include "DiagnosticsLog.h"
#include "ShellContext.h"
#include "UiCore.h"
#include "UiTables.h"

#include <openvr.h>
#include <imgui.h>

#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <cctype>
#include <memory>
#include <optional>
#include <vector>

namespace wkopenvr::dynamicres {
namespace {

using openvr_pair::overlay::ui::StatusTone;

StatusTone ToneForPressure(ResolutionPressure pressure)
{
	switch (pressure) {
		case ResolutionPressure::GpuBound:
			return StatusTone::Warn;
		case ResolutionPressure::CpuBound:
			return StatusTone::Info;
		case ResolutionPressure::Headroom:
			return StatusTone::Ok;
		case ResolutionPressure::Steady:
		case ResolutionPressure::Waiting:
		default:
			return StatusTone::Idle;
	}
}

StatusTone ToneForAction(ResolutionAction action)
{
	switch (action) {
		case ResolutionAction::Lower:
			return StatusTone::Warn;
		case ResolutionAction::Raise:
		case ResolutionAction::Restore:
			return StatusTone::Ok;
		case ResolutionAction::ExternalOverride:
		case ResolutionAction::NoEffect:
			return StatusTone::Info;
		case ResolutionAction::None:
		default:
			return StatusTone::Idle;
	}
}

void LogDynamicRes(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	openvr_pair::common::DiagnosticLogV("dynamicres", fmt, args);
	va_end(args);
}

double FrameAppGpuMs(const vr::Compositor_FrameTiming& timing)
{
	return PerFrameAppGpuMs(timing.m_flPreSubmitGpuMs, timing.m_flPostSubmitGpuMs, timing.m_flTotalRenderGpuMs,
	                        timing.m_flCompositorRenderGpuMs);
}

// One line carrying every input a scale decision was made from, so a wrong decision can be
// audited from the log alone.
void LogEvidence(const DynamicResolutionClassification& c, double scale)
{
	LogDynamicRes("evidence frames=%d ticks=%d p50=%.2f p95=%.2f peak=%.2f budget=%.2f gpu_harm=%.4f "
	              "cpu_harm=%.4f motion=%.4f drops=%.4f over=%.4f interval=%.2f idle_cpu=%.2f harm_ticks=%d "
	              "clean_ticks=%d beta=%.2f scale=%.3f",
	              c.framesTotal, c.tickCount, c.appGpuP50Ms, c.appGpuP95Ms, c.appGpuPeakMs, c.frameBudgetMs,
	              c.gpuHarmRate, c.cpuHarmRate, c.motionRate, c.dropRate, c.overBudgetRate, c.clientFrameIntervalMs,
	              c.compositorIdleCpuMs, c.consecutiveHarmTicks, c.consecutiveCleanTicks, c.costBeta, scale);
}

} // namespace

bool DynamicResolutionPlugin::IsInstalled(openvr_pair::overlay::ShellContext& context) const
{
	return context.IsFlagPresent(FlagFileName()) || profile_.restore.restorePending;
}

void DynamicResolutionPlugin::OnStart(openvr_pair::overlay::ShellContext&)
{
	profile_ = LoadDynamicResolutionProfile();
	if (profile_.restore.restorePending) {
		RestoreBaseline("startup restore");
	}
}

void DynamicResolutionPlugin::OnShutdown(openvr_pair::overlay::ShellContext&)
{
	if (profile_.restore.restorePending) {
		RestoreBaseline("overlay shutdown");
	}
}

void DynamicResolutionPlugin::Tick(openvr_pair::overlay::ShellContext& context)
{
	const auto now = std::chrono::steady_clock::now();
	if (nextTick_ != std::chrono::steady_clock::time_point{} && now < nextTick_) return;
	nextTick_ = now + std::chrono::seconds(1);

	const bool moduleEnabled = context.IsFlagPresent(FlagFileName());
	if (!moduleEnabled) {
		if (profile_.restore.restorePending) {
			RestoreBaseline("module disabled");
		}
		return;
	}

	const SceneState scene = ReadSceneState();
	if (!scene.running || scene.processId == 0) {
		if (profile_.restore.restorePending) {
			RestoreBaseline("scene stopped");
		}
		controller_.Reset();
		externalOverride_ = false;
		disabledForScene_ = false;
		lastReason_ = "Waiting for scene app";
		return;
	}

	if (profile_.restore.restorePending && profile_.restore.sceneProcessId != 0 &&
	    profile_.restore.sceneProcessId != scene.processId) {
		RestoreBaseline("scene changed");
		controller_.Reset();
		externalOverride_ = false;
		disabledForScene_ = false;
	}

	if (disabledForScene_) {
		lastReason_ = "Stopped for this scene";
		return;
	}

	if (!profile_.restore.restorePending && !CaptureBaseline(scene.processId)) {
		lastReason_ = "SteamVR settings unavailable";
		return;
	}

	DynamicResolutionTiming timing;
	if (!CollectTiming(timing)) {
		lastReason_ = "Waiting for timing";
		return;
	}
	RefreshMotionSmoothingState();

	double liveScale =
	    profile_.restore.lastWrittenScale > 0.0 ? profile_.restore.lastWrittenScale : profile_.restore.baselineScale;
	if (ReadSteamVrScale(liveScale)) {
		lastLiveScale_ = liveScale;
	}

	const bool external =
	    profile_.restore.lastWrittenScale > 0.0 && ScaleDiffers(liveScale, profile_.restore.lastWrittenScale, 0.01);
	DynamicResolutionControllerInput input;
	input.timing = timing;
	input.baselineScale = profile_.restore.baselineScale;
	input.currentScale = liveScale;
	input.sceneRunning = true;
	input.externalOverride = external || externalOverride_;

	const DynamicResolutionControllerOutput output = controller_.Evaluate(input, profile_.settings);
	lastClassification_ = output.classification;
	UpdateStatus(output);

	if (output.classification.pressure != lastLoggedPressure_) {
		LogDynamicRes("classify pressure=%s->%s", ResolutionPressureLabel(lastLoggedPressure_),
		              ResolutionPressureLabel(output.classification.pressure));
		lastLoggedPressure_ = output.classification.pressure;
		LogEvidence(output.classification, lastLiveScale_);
	}

	if (output.effectCheck.ran) {
		const DynamicResolutionEffectCheck& check = output.effectCheck;
		LogDynamicRes("effect_check after=%s scale=%.3f->%.3f p95=%.2f->%.2f beta_obs=%.2f beta=%.2f verdict=%s",
		              ResolutionActionLabel(check.after), check.fromScale, check.toScale, check.preP95Ms,
		              check.postP95Ms, check.betaObserved, output.classification.costBeta, check.verdict);
	}

	// A withheld action is logged once per blocking cause, when the cause changes.
	if (output.withheldGate != nullptr) {
		std::string withheldKey = output.withheldGate;
		withheldKey += ':';
		withheldKey += output.withheldCause;
		if (withheldKey != lastWithheldKey_) {
			lastWithheldKey_ = withheldKey;
			LogDynamicRes("withheld gate=%s cause=%s value=%.4f limit=%.4f predicted_p95=%.2f burned=%.3f",
			              output.withheldGate, output.withheldCause, output.withheldValue, output.withheldLimit,
			              output.classification.predictedRaiseP95Ms, 0.0);
			LogEvidence(output.classification, lastLiveScale_);
		}
	}
	else {
		lastWithheldKey_.clear();
	}

	switch (output.action) {
		case ResolutionAction::Lower:
		case ResolutionAction::Raise:
			if (WriteSteamVrScale(output.targetScale)) {
				double writtenScale = output.targetScale;
				if (ReadSteamVrScale(writtenScale)) {
					lastLiveScale_ = writtenScale;
				}
				profile_.restore.lastWrittenScale = writtenScale;
				SaveProfile();
				controller_.NoteWrite(output.action, writtenScale, output.classification, profile_.settings);
				const DynamicResolutionClassification& c = output.classification;
				LogDynamicRes("scale_change action=%s from=%.3f to=%.3f reason='%s' p95=%.2f budget=%.2f beta=%.2f "
				              "harm_ticks=%d clean_ticks=%d",
				              ResolutionActionLabel(output.action), liveScale, writtenScale, output.reason.c_str(),
				              c.appGpuP95Ms, c.frameBudgetMs, c.costBeta, c.consecutiveHarmTicks,
				              c.consecutiveCleanTicks);
				LogEvidence(c, writtenScale);
				LogRenderTargetSize(writtenScale);
			}
			break;
		case ResolutionAction::NoEffect:
			if (RestoreBaseline("no effect")) {
				disabledForScene_ = true;
			}
			break;
		case ResolutionAction::ExternalOverride:
			ClearExternalOverride();
			disabledForScene_ = true;
			break;
		case ResolutionAction::Restore:
			RestoreBaseline(output.reason.c_str());
			break;
		case ResolutionAction::None:
		default:
			break;
	}
}

void DynamicResolutionPlugin::DrawTab(openvr_pair::overlay::ShellContext&)
{
	openvr_pair::overlay::ui::TabBarScope tabs("dynamicres_tabs");
	if (tabs) {
		openvr_pair::overlay::ui::DrawTabItem("Status", [&] { DrawStatus(); });
		openvr_pair::overlay::ui::DrawTabItem("Settings", [&] { DrawSettings(); });
		openvr_pair::overlay::ui::DrawTabItem("Advanced", [&] { DrawAdvanced(); });
	}
}

void DynamicResolutionPlugin::DrawLogsSection(openvr_pair::overlay::ShellContext&)
{
	openvr_pair::overlay::ui::DrawSectionHeading("File locations");
	ImGui::TextWrapped("Overlay:  %%LocalAppDataLow%%\\WKOpenVR\\Logs\\overlay_log.<ts>.txt");
	ImGui::TextWrapped("Settings: %%LocalAppDataLow%%\\WKOpenVR\\profiles\\dynamicres.txt");
}

bool DynamicResolutionPlugin::CaptureBaseline(uint32_t sceneProcessId)
{
	double scale = 1.0;
	bool manual = false;
	if (!ReadSteamVrScale(scale)) return false;
	ReadSteamVrManualOverride(manual);

	profile_.restore.restorePending = true;
	profile_.restore.baselineScale = std::max(0.1, scale);
	profile_.restore.baselineManualOverride = manual;
	profile_.restore.sceneProcessId = sceneProcessId;
	profile_.restore.lastWrittenScale = scale;
	SaveProfile();

	if (!WriteSteamVrManualOverride(true)) {
		profile_.restore = {};
		SaveProfile();
		lastError_ = "manual override write failed";
		return false;
	}

	controller_.Reset();
	externalOverride_ = false;
	disabledForScene_ = false;
	LogDynamicRes("baseline_captured scene_pid=%u baseline_scale=%.3f manual_override=%d", sceneProcessId,
	              profile_.restore.baselineScale, profile_.restore.baselineManualOverride ? 1 : 0);
	return true;
}

bool DynamicResolutionPlugin::RestoreBaseline(const char* reason)
{
	if (!profile_.restore.restorePending) return true;
	bool ok = true;
	if (!WriteSteamVrScale(profile_.restore.baselineScale)) ok = false;
	if (!WriteSteamVrManualOverride(profile_.restore.baselineManualOverride)) ok = false;
	if (ok) {
		LogDynamicRes("baseline_restored reason='%s' scale=%.3f manual_override=%d", reason ? reason : "",
		              profile_.restore.baselineScale, profile_.restore.baselineManualOverride ? 1 : 0);
		profile_.restore = {};
		SaveProfile();
		controller_.Reset();
		externalOverride_ = false;
		disabledForScene_ = false;
		lastAction_ = "Restore";
		lastReason_ = reason ? reason : "Restored";
	}
	return ok;
}

void DynamicResolutionPlugin::ClearExternalOverride()
{
	LogDynamicRes("external_override live_scale=%.3f last_written=%.3f", lastLiveScale_,
	              profile_.restore.lastWrittenScale);
	if (profile_.restore.restorePending && !WriteSteamVrManualOverride(profile_.restore.baselineManualOverride)) {
		LogDynamicRes("external_override_manual_restore_failed baseline_manual_override=%d",
		              profile_.restore.baselineManualOverride ? 1 : 0);
	}
	profile_.restore = {};
	SaveProfile();
	controller_.Reset();
	externalOverride_ = true;
	lastAction_ = "External override";
	lastReason_ = "SteamVR value changed";
}

bool DynamicResolutionPlugin::ReadSteamVrScale(double& outScale) const
{
	vr::IVRSettings* settings = vr::VRSettings();
	if (!settings) return false;
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	const float value = settings->GetFloat(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_SupersampleScale_Float, &err);
	if (err != vr::VRSettingsError_None || !std::isfinite(value) || value <= 0.0f) return false;
	outScale = value;
	return true;
}

bool DynamicResolutionPlugin::ReadSteamVrManualOverride(bool& outManual) const
{
	vr::IVRSettings* settings = vr::VRSettings();
	if (!settings) return false;
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	const bool value =
	    settings->GetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_SupersampleManualOverride_Bool, &err);
	if (err != vr::VRSettingsError_None) return false;
	outManual = value;
	return true;
}

bool DynamicResolutionPlugin::WriteSteamVrScale(double scale)
{
	vr::IVRSettings* settings = vr::VRSettings();
	if (!settings) return false;
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	settings->SetFloat(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_SupersampleScale_Float,
	                   static_cast<float>(std::max(0.1, scale)), &err);
	if (err != vr::VRSettingsError_None) {
		lastError_ = "scale write failed";
		return false;
	}
	lastError_.clear();
	return true;
}

bool DynamicResolutionPlugin::WriteSteamVrManualOverride(bool value)
{
	vr::IVRSettings* settings = vr::VRSettings();
	if (!settings) return false;
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	settings->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_SupersampleManualOverride_Bool, value, &err);
	if (err != vr::VRSettingsError_None) {
		lastError_ = "manual override write failed";
		return false;
	}
	lastError_.clear();
	return true;
}

bool DynamicResolutionPlugin::CollectTiming(DynamicResolutionTiming& outTiming)
{
	vr::IVRCompositor* compositor = vr::VRCompositor();
	if (!compositor) return false;

	std::array<vr::Compositor_FrameTiming, 128> timings{};
	for (auto& timing : timings) {
		timing.m_nSize = sizeof(vr::Compositor_FrameTiming);
	}

	const uint32_t count = compositor->GetFrameTimings(timings.data(), static_cast<uint32_t>(timings.size()));
	if (count == 0) return false;

	uint32_t used = 0;
	uint32_t maxFrameIndex = lastFrameIndex_;
	DynamicResolutionTiming sample;
	sample.frameBudgetMs = ReadFrameBudgetMs();
	sample.valid = true;
	std::vector<double> appGpu;
	std::vector<double> intervals;
	std::vector<double> idleCpu;
	appGpu.reserve(timings.size());
	intervals.reserve(timings.size());
	idleCpu.reserve(timings.size());

	for (uint32_t i = 0; i < count && i < timings.size(); ++i) {
		const vr::Compositor_FrameTiming& timing = timings[i];
		if (haveLastFrameIndex_ && timing.m_nFrameIndex <= lastFrameIndex_) continue;
		const double perFrame = FrameAppGpuMs(timing);
		appGpu.push_back(perFrame);
		if (timing.m_flClientFrameIntervalMs > 0.0f) intervals.push_back(timing.m_flClientFrameIntervalMs);
		if (timing.m_flCompositorIdleCpuMs > 0.0f) idleCpu.push_back(timing.m_flCompositorIdleCpuMs);
		if ((timing.m_nReprojectionFlags & vr::VRCompositor_ReprojectionReason_Gpu) != 0) ++sample.framesWithGpuReproj;
		if ((timing.m_nReprojectionFlags & vr::VRCompositor_ReprojectionReason_Cpu) != 0) ++sample.framesWithCpuReproj;
		if ((timing.m_nReprojectionFlags & vr::VRCompositor_ReprojectionMotion) != 0) ++sample.framesWithMotion;
		if (VR_COMPOSITOR_NUMBER_OF_THROTTLED_FRAMES(timing) > 0) ++sample.framesThrottled;
		if (timing.m_nNumDroppedFrames > 0) ++sample.framesWithDrops;
		if (timing.m_nNumMisPresented > 0) ++sample.framesMispresented;
		if (timing.m_nNumFramePresents > 1) ++sample.framesMultiPresented;
		if (perFrame >= sample.frameBudgetMs) ++sample.framesOverBudget;
		maxFrameIndex = std::max(maxFrameIndex, timing.m_nFrameIndex);
		++used;
	}

	if (used == 0) return false;
	sample.framesConsidered = static_cast<int>(used);
	sample.appGpuP50Ms = PercentileSorted(appGpu, 0.50);
	sample.appGpuP95Ms = PercentileSorted(appGpu, 0.95);
	sample.appGpuMaxMs = PercentileSorted(appGpu, 1.0);
	sample.clientFrameIntervalMs = PercentileSorted(intervals, 0.50);
	sample.compositorIdleCpuMs = PercentileSorted(idleCpu, 0.50);
	lastFrameIndex_ = maxFrameIndex;
	haveLastFrameIndex_ = true;
	lastFrameBudgetMs_ = sample.frameBudgetMs;
	outTiming = sample;
	return true;
}

void DynamicResolutionPlugin::LogRenderTargetSize(double scale)
{
	// The recommended render-target size reflects the written supersample scale, so the actual
	// pixel-count response to each write is verifiable from the log.
	vr::IVRSystem* system = vr::VRSystem();
	if (!system) return;
	uint32_t width = 0;
	uint32_t height = 0;
	system->GetRecommendedRenderTargetSize(&width, &height);
	lastRtWidth_ = width;
	lastRtHeight_ = height;
	LogDynamicRes("rt_size scale=%.3f width=%u height=%u", scale, width, height);
}

void DynamicResolutionPlugin::RefreshMotionSmoothingState()
{
	vr::IVRCompositor* compositor = vr::VRCompositor();
	if (!compositor) {
		haveMotionSmoothingState_ = false;
		motionSmoothingSupported_ = false;
		motionSmoothingEnabled_ = false;
		return;
	}
	motionSmoothingSupported_ = compositor->IsMotionSmoothingSupported();
	motionSmoothingEnabled_ = compositor->IsMotionSmoothingEnabled();
	haveMotionSmoothingState_ = true;
}

double DynamicResolutionPlugin::ReadFrameBudgetMs()
{
	vr::IVRSystem* system = vr::VRSystem();
	if (!system) {
		displayFrequencyFallback_ = true;
		return 1000.0 / 90.0;
	}
	vr::ETrackedPropertyError err = vr::TrackedProp_Success;
	const float hz =
	    system->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float, &err);
	if (err != vr::TrackedProp_Success || !std::isfinite(hz) || hz <= 1.0f) {
		displayFrequencyFallback_ = true;
		return 1000.0 / 90.0;
	}
	displayFrequencyFallback_ = false;
	return 1000.0 / hz;
}

DynamicResolutionPlugin::SceneState DynamicResolutionPlugin::ReadSceneState() const
{
	SceneState scene;
	vr::IVRApplications* apps = vr::VRApplications();
	if (!apps) return scene;
	const vr::EVRSceneApplicationState state = apps->GetSceneApplicationState();
	scene.running = state == vr::EVRSceneApplicationState_Running;
	scene.processId = apps->GetCurrentSceneProcessId();
	return scene;
}

void DynamicResolutionPlugin::UpdateStatus(const DynamicResolutionControllerOutput& output)
{
	lastActionCode_ = output.action;
	lastAction_ = ResolutionActionLabel(output.action);
	lastTargetScale_ = output.targetScale;
	if (!output.reason.empty()) {
		lastReason_ = output.reason;
	}
	else {
		lastReason_ = ResolutionPressureLabel(output.classification.pressure);
	}
}

void DynamicResolutionPlugin::SaveProfile()
{
	SaveDynamicResolutionProfile(profile_);
}

void DynamicResolutionPlugin::DrawSettings()
{
	openvr_pair::overlay::ui::DrawSettingTable(
	    "dynamicres_settings", 190.0f, [&](openvr_pair::overlay::ui::SettingTableScope& table) {
		    openvr_pair::overlay::ui::SettingRow(table, "Quality preset", [&] {
			    static const char* kPresetItems[] = {"Max FPS", "FPS-first", "Balanced", "Quality", "Custom"};
			    int current = std::clamp(static_cast<int>(profile_.settings.qualityPreset), 0,
			                             static_cast<int>(QualityPreset::Custom));
			    ImGui::SetNextItemWidth(-1.0f);
			    if (ImGui::Combo("##dynamicres_preset", &current, kPresetItems, IM_ARRAYSIZE(kPresetItems))) {
				    ApplyQualityPreset(static_cast<QualityPreset>(current), profile_.settings);
				    SaveProfile();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Raise back", [&] {
			    bool value = profile_.settings.allowRaiseBack;
			    if (openvr_pair::overlay::ui::CheckboxWithTooltip(
			            "##dynamicres_raise_back", &value,
			            "Allows recovering quality (including above your scale) once GPU headroom is solid.")) {
				    profile_.settings.allowRaiseBack = value;
				    SaveProfile();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Release on CPU-bound", [&] {
			    bool value = profile_.settings.releaseOnCpuBound;
			    if (openvr_pair::overlay::ui::CheckboxWithTooltip(
			            "##dynamicres_release_cpu", &value,
			            "Raises back toward the original scale when misses are CPU-bound and GPU time is low.")) {
				    profile_.settings.releaseOnCpuBound = value;
				    SaveProfile();
			    }
		    });
	    });

	openvr_pair::overlay::ui::DrawTextWrapped(
	    "FPS-first holds your headset's native refresh by lowering render resolution under GPU load, then "
	    "spends genuine spare GPU on extra sharpness. CPU bottlenecks are detected and left alone. Use the "
	    "Advanced tab for raw thresholds.");
}

void DynamicResolutionPlugin::DrawAdvanced()
{
	// Any change here means the knobs no longer match a named preset.
	auto markCustom = [&] {
		profile_.settings.qualityPreset = QualityPreset::Custom;
		SaveProfile();
	};
	openvr_pair::overlay::ui::DrawSettingTable(
	    "dynamicres_advanced", 200.0f, [&](openvr_pair::overlay::ui::SettingTableScope& table) {
		    openvr_pair::overlay::ui::SettingRow(table, "Minimum scale", [&] {
			    float value = static_cast<float>(profile_.settings.minScaleFraction * 100.0);
			    if (openvr_pair::overlay::ui::SliderFloatWithTooltip(
			            "##dynamicres_min_scale", &value, 40.0f, 100.0f, "%.0f%%",
			            "Quality floor: lowest render scale this module may write, as a % of your SteamVR scale.")) {
				    profile_.settings.minScaleFraction = ClampScaleFraction(value / 100.0);
				    markCustom();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Maximum scale", [&] {
			    float value = static_cast<float>(profile_.settings.maxScaleFraction * 100.0);
			    if (openvr_pair::overlay::ui::SliderFloatWithTooltip(
			            "##dynamicres_max_scale", &value, 100.0f, 150.0f, "%.0f%%",
			            "Quality ceiling: highest render scale, as a % of your SteamVR scale. >100% supersamples "
			            "above your scale when the GPU is idle.")) {
				    profile_.settings.maxScaleFraction = std::clamp(value / 100.0, 1.0, 2.0);
				    markCustom();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Step size", [&] {
			    float value = static_cast<float>(profile_.settings.stepFraction * 100.0);
			    if (openvr_pair::overlay::ui::SliderFloatWithTooltip(
			            "##dynamicres_step", &value, 1.0f, 25.0f, "%.0f%%",
			            "Maximum adaptive scale change per controller step.")) {
				    profile_.settings.stepFraction = std::clamp(value / 100.0, 0.01, 0.25);
				    markCustom();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Harm tolerance", [&] {
			    float value = static_cast<float>(profile_.settings.gpuHarmRateFraction * 100.0);
			    if (openvr_pair::overlay::ui::SliderFloatWithTooltip(
			            "##dynamicres_harm_rate", &value, 0.1f, 10.0f, "%.1f%%",
			            "Share of a second's frames that must miss for a GPU reason before that second counts "
			            "toward lowering.")) {
				    profile_.settings.gpuHarmRateFraction = std::clamp(value / 100.0, 0.001, 0.20);
				    markCustom();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Lower target", [&] {
			    float value = static_cast<float>(profile_.settings.lowerTargetFraction * 100.0);
			    if (openvr_pair::overlay::ui::SliderFloatWithTooltip(
			            "##dynamicres_lower_target", &value, 70.0f, 98.0f, "%.0f%%",
			            "Lowering sizes one cut to land GPU frame time at this % of the frame budget.")) {
				    profile_.settings.lowerTargetFraction = std::clamp(value / 100.0, 0.70, 0.98);
				    markCustom();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Raise safety", [&] {
			    float value = static_cast<float>(profile_.settings.raiseSafetyFraction * 100.0);
			    if (openvr_pair::overlay::ui::SliderFloatWithTooltip(
			            "##dynamicres_raise_safety", &value, 70.0f, 98.0f, "%.0f%%",
			            "A raise must be predicted to keep GPU frame time within this % of the budget.")) {
				    profile_.settings.raiseSafetyFraction = std::clamp(value / 100.0, 0.70, 0.98);
				    markCustom();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Supersample gate", [&] {
			    float value = static_cast<float>(profile_.settings.raiseAboveBaselineFraction * 100.0);
			    if (openvr_pair::overlay::ui::SliderFloatWithTooltip(
			            "##dynamicres_super_gate", &value, 30.0f, 95.0f, "%.0f%%",
			            "GPU frame time (p95) must stay at or below this % of the budget before raising above "
			            "your scale.")) {
				    profile_.settings.raiseAboveBaselineFraction = std::clamp(value / 100.0, 0.30, 0.95);
				    markCustom();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Lower dwell", [&] {
			    int value = profile_.settings.lowerRequiredTicks;
			    if (openvr_pair::overlay::ui::SliderIntWithTooltip(
			            "##dynamicres_lower_ticks", &value, 2, 10, "%d s",
			            "Consecutive seconds of proven GPU misses before lowering.")) {
				    profile_.settings.lowerRequiredTicks = std::clamp(value, 2, 30);
				    markCustom();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Raise dwell", [&] {
			    int value = profile_.settings.raiseRequiredTicks;
			    if (openvr_pair::overlay::ui::SliderIntWithTooltip(
			            "##dynamicres_raise_ticks", &value, 1, 15, "%d s",
			            "Seconds of headroom before recovering quality toward your scale.")) {
				    profile_.settings.raiseRequiredTicks = std::clamp(value, 1, 30);
				    markCustom();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Supersample dwell", [&] {
			    int value = profile_.settings.raiseAboveBaselineTicks;
			    if (openvr_pair::overlay::ui::SliderIntWithTooltip(
			            "##dynamicres_super_ticks", &value, 1, 30, "%d s",
			            "Seconds of clean deep headroom before raising above your scale.")) {
				    profile_.settings.raiseAboveBaselineTicks = std::clamp(value, 1, 60);
				    markCustom();
			    }
		    });
	    });
}

void DynamicResolutionPlugin::DrawStatus()
{
	openvr_pair::overlay::ui::DrawSettingTable(
	    "dynamicres_status", 180.0f, [&](openvr_pair::overlay::ui::SettingTableScope& table) {
		    openvr_pair::overlay::ui::SettingRow(table, "State", [&] {
			    openvr_pair::overlay::ui::DrawStatusCell(ResolutionPressureLabel(lastClassification_.pressure),
			                                             ToneForPressure(lastClassification_.pressure), false);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Action", [&] {
			    openvr_pair::overlay::ui::DrawStatusCell(lastAction_.c_str(), ToneForAction(lastActionCode_), false);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Decision",
		                                         [&] { openvr_pair::overlay::ui::DrawTextWrapped(lastReason_); });
		    openvr_pair::overlay::ui::SettingRow(table, "Hold target", [&] {
			    const double hz = lastFrameBudgetMs_ > 0.0 ? 1000.0 / lastFrameBudgetMs_ : 0.0;
			    ImGui::Text("%.0f Hz (no reprojection)", hz);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Scale", [&] {
			    const double ceiling = CeilingScale(profile_.settings, profile_.restore.baselineScale);
			    ImGui::Text("%.3f current / %.3f baseline / %.3f ceiling", lastLiveScale_,
			                profile_.restore.baselineScale, ceiling);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Next scale", [&] {
			    if (lastActionCode_ == ResolutionAction::Lower || lastActionCode_ == ResolutionAction::Raise) {
				    ImGui::Text("%.3f", lastTargetScale_);
			    }
			    else {
				    ImGui::TextUnformatted("-");
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "GPU frame time", [&] {
			    ImGui::Text("%.2f p50 / %.2f p95 / %.2f peak of %.2f ms", lastClassification_.appGpuP50Ms,
			                lastClassification_.appGpuP95Ms, lastClassification_.appGpuPeakMs, lastFrameBudgetMs_);
			    if (displayFrequencyFallback_) {
				    ImGui::SameLine();
				    openvr_pair::overlay::ui::StatusBadge("90Hz fallback", StatusTone::Warn);
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Harmed frames", [&] {
			    ImGui::Text("GPU %.2f%% / CPU %.2f%% / smoothing %.1f%% of %d", 100.0 * lastClassification_.gpuHarmRate,
			                100.0 * lastClassification_.cpuHarmRate, 100.0 * lastClassification_.motionRate,
			                lastClassification_.framesTotal);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Streak", [&] {
			    ImGui::Text("%d s clean / %d s GPU-harmed", lastClassification_.consecutiveCleanTicks,
			                lastClassification_.consecutiveHarmTicks);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Cost model", [&] {
			    ImGui::Text("beta %.2f", lastClassification_.costBeta);
			    if (lastClassification_.predictedRaiseP95Ms > 0.0) {
				    ImGui::SameLine();
				    ImGui::Text("/ next step p95 %.2f ms", lastClassification_.predictedRaiseP95Ms);
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Render target", [&] {
			    if (lastRtWidth_ > 0 && lastRtHeight_ > 0) {
				    ImGui::Text("%u x %u", lastRtWidth_, lastRtHeight_);
			    }
			    else {
				    ImGui::TextUnformatted("-");
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "CPU cadence", [&] {
			    const double interval = lastClassification_.clientFrameIntervalMs;
			    const double hz = interval > 0.0 ? 1000.0 / interval : 0.0;
			    ImGui::Text("%.2f ms (%.0f Hz)", interval, hz);
			    if (lastClassification_.cpuStalled) {
				    ImGui::SameLine();
				    openvr_pair::overlay::ui::StatusBadge("CPU-limited", StatusTone::Warn);
			    }
			    if (lastClassification_.appPaced) {
				    ImGui::SameLine();
				    openvr_pair::overlay::ui::StatusBadge("App-paced", StatusTone::Info);
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Motion smoothing", [&] {
			    if (!haveMotionSmoothingState_) {
				    openvr_pair::overlay::ui::DrawStatusCell("Unknown", StatusTone::Idle, false);
			    }
			    else {
				    openvr_pair::overlay::ui::DrawStatusCell(
				        motionSmoothingSupported_ ? "Supported" : "Unsupported",
				        motionSmoothingSupported_ ? StatusTone::Ok : StatusTone::Idle, false);
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Smoothing setting", [&] {
			    if (!haveMotionSmoothingState_) {
				    ImGui::TextUnformatted("-");
			    }
			    else {
				    openvr_pair::overlay::ui::DrawStatusCell(
				        motionSmoothingEnabled_ ? "Enabled" : "Off",
				        motionSmoothingEnabled_ ? StatusTone::Ok : StatusTone::Idle, false);
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Smoothing active", [&] {
			    openvr_pair::overlay::ui::DrawStatusCell(
			        lastClassification_.motionSmoothingEngaged ? "Active" : "Idle",
			        lastClassification_.motionSmoothingEngaged ? StatusTone::Warn : StatusTone::Idle, false);
		    });
		    if (lastClassification_.pressure == ResolutionPressure::CpuBound) {
			    openvr_pair::overlay::ui::SettingRow(table, "CPU-bound", [&] {
				    if (lastClassification_.gpuHasHeadroom) {
					    openvr_pair::overlay::ui::DrawTextWrapped(
					        "GPU has headroom; lowering scale will not help this miss.");
				    }
				    else {
					    openvr_pair::overlay::ui::DrawTextWrapped("Holding scale until GPU pressure is clear.");
				    }
			    });
		    }
		    openvr_pair::overlay::ui::SettingRow(table, "Restore", [&] {
			    const bool pending = profile_.restore.restorePending;
			    if (pending) {
				    openvr_pair::overlay::ui::StatusBadge("Pending", StatusTone::Pending);
			    }
			    else {
				    ImGui::TextUnformatted("None");
			    }
			    ImGui::SameLine();
			    ImGui::BeginDisabled(!pending);
			    if (ImGui::SmallButton("Restore Now")) {
				    RestoreBaseline("manual restore");
			    }
			    ImGui::EndDisabled();
		    });
	    });

	if (!lastError_.empty()) {
		openvr_pair::overlay::ui::DrawErrorBanner("SteamVR settings", lastError_.c_str());
	}
}

} // namespace wkopenvr::dynamicres

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateDynamicResolutionPlugin()
{
	return std::make_unique<wkopenvr::dynamicres::DynamicResolutionPlugin>();
}

} // namespace openvr_pair::overlay
