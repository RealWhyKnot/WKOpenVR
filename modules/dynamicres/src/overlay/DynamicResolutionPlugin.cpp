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
				LogDynamicRes("scale_change action=%s from=%.3f to=%.3f reason='%s' app_gpu_ms=%.3f budget_ms=%.3f",
				              ResolutionActionLabel(output.action), liveScale, writtenScale, output.reason.c_str(),
				              output.classification.medianAppGpuMs, output.classification.frameBudgetMs);
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
	DynamicResolutionTiming aggregate;
	aggregate.frameBudgetMs = ReadFrameBudgetMs();
	aggregate.valid = true;

	for (uint32_t i = 0; i < count && i < timings.size(); ++i) {
		const vr::Compositor_FrameTiming& timing = timings[i];
		if (haveLastFrameIndex_ && timing.m_nFrameIndex <= lastFrameIndex_) continue;
		aggregate.preSubmitGpuMs += timing.m_flPreSubmitGpuMs;
		aggregate.postSubmitGpuMs += timing.m_flPostSubmitGpuMs;
		aggregate.totalRenderGpuMs += timing.m_flTotalRenderGpuMs;
		aggregate.compositorRenderGpuMs += timing.m_flCompositorRenderGpuMs;
		aggregate.droppedFrames += timing.m_nNumDroppedFrames;
		aggregate.mispresentedFrames += timing.m_nNumMisPresented;
		aggregate.reprojectionFlags |= timing.m_nReprojectionFlags;
		aggregate.framePresents = std::max(aggregate.framePresents, timing.m_nNumFramePresents);
		maxFrameIndex = std::max(maxFrameIndex, timing.m_nFrameIndex);
		++used;
	}

	if (used == 0) return false;
	aggregate.preSubmitGpuMs /= used;
	aggregate.postSubmitGpuMs /= used;
	aggregate.totalRenderGpuMs /= used;
	aggregate.compositorRenderGpuMs /= used;
	lastFrameIndex_ = maxFrameIndex;
	haveLastFrameIndex_ = true;
	lastFrameBudgetMs_ = aggregate.frameBudgetMs;
	outTiming = aggregate;
	return true;
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
		    openvr_pair::overlay::ui::SettingRow(table, "Minimum scale", [&] {
			    float value = static_cast<float>(profile_.settings.minScaleFraction * 100.0);
			    if (openvr_pair::overlay::ui::SliderFloatWithTooltip(
			            "##dynamicres_min_scale", &value, 40.0f, 100.0f, "%.0f%%",
			            "Lowest SteamVR supersample scale this module may write.")) {
				    profile_.settings.minScaleFraction = ClampScaleFraction(value / 100.0);
				    SaveProfile();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Step size", [&] {
			    float value = static_cast<float>(profile_.settings.stepFraction * 100.0);
			    if (openvr_pair::overlay::ui::SliderFloatWithTooltip(
			            "##dynamicres_step", &value, 1.0f, 25.0f, "%.0f%%",
			            "Maximum adaptive scale change per controller step.")) {
				    profile_.settings.stepFraction = std::clamp(value / 100.0, 0.01, 0.25);
				    SaveProfile();
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Raise back", [&] {
			    bool value = profile_.settings.allowRaiseBack;
			    if (openvr_pair::overlay::ui::CheckboxWithTooltip(
			            "##dynamicres_raise_back", &value,
			            "Allows return toward the original SteamVR scale after GPU headroom.")) {
				    profile_.settings.allowRaiseBack = value;
				    SaveProfile();
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
		    openvr_pair::overlay::ui::SettingRow(table, "Scale", [&] {
			    ImGui::Text("%.3f current / %.3f baseline", lastLiveScale_, profile_.restore.baselineScale);
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Next scale", [&] {
			    if (lastActionCode_ == ResolutionAction::Lower || lastActionCode_ == ResolutionAction::Raise) {
				    ImGui::Text("%.3f", lastTargetScale_);
			    }
			    else {
				    ImGui::TextUnformatted("-");
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "GPU budget", [&] {
			    ImGui::Text("%.2f / %.2f ms", lastClassification_.medianAppGpuMs, lastFrameBudgetMs_);
			    if (displayFrequencyFallback_) {
				    ImGui::SameLine();
				    openvr_pair::overlay::ui::StatusBadge("90Hz fallback", StatusTone::Warn);
			    }
		    });
		    openvr_pair::overlay::ui::SettingRow(table, "Missed frames", [&] {
			    ImGui::Text("%d of %d samples", lastClassification_.unstableSamples, lastClassification_.sampleCount);
		    });
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
