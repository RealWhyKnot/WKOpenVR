#include "Calibration.h"
#include "CalibrationHeadMountShadow.h" // CCal_SeedHeadMountProxyRelativeLock
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "HeadMountOffsetModal.h"     // wkopenvr::headmount::OpenOffsetModal
#include "HeadMountOffsetPreflight.h" // offset-calibration preflight for the action button
#include "HeadMountTargetBinding.h"   // wkopenvr::headmount::HeadMountMatchesContinuousTarget
#include "TrackingStyle.h"
#include "TrackingStyleActions.h"
#include "UserInterface.h" // IsVRReady

#include <imgui/imgui.h>
#include "imgui_extensions.h" // BeginGroupPanel

#include <cfloat>
#include <cstdio>
#include <string>

// Defined in UserInterface.cpp; shared with the main-window flow there.
void SendHeadMountConfigFromCalCtx();
std::string CalibrationBlockedMessage();

static bool ContinuousCalibrationIsRunning()
{
	return wkopenvr::tracking_style_ui::ContinuousCalibrationIsRunning(CalCtx.state);
}

static void ApplySelectedTrackingStyle(TrackingStyle style)
{
	const TrackingStyle prev = CalCtx.trackingStyle;
	const bool shouldEndContinuous = style == TrackingStyle::Manual && ContinuousCalibrationIsRunning();
	if (prev == style && !shouldEndContinuous) return;
	if (shouldEndContinuous) {
		EndContinuousCalibration();
	}
	ApplyTrackingStylePreset(CalCtx, style);
	CalCtx.ResolveLockMode();
	SaveProfile(CalCtx);
	SendHeadMountConfigFromCalCtx();
	char buf[160];
	snprintf(buf, sizeof buf, "tracking_style_ui_write: prev=%d now=%d lockMode=%d headMountMode=%d fallback=%d",
	         (int)prev, (int)CalCtx.trackingStyle, (int)CalCtx.lockRelativePositionMode, (int)CalCtx.headMount.mode,
	         (int)CalCtx.headMount.allowRawHmdFallback);
	Metrics::WriteLogAnnotation(buf);
}

static void SetLockRelativePositionModeFromUi(bool enabled)
{
	const auto prev = CalCtx.lockRelativePositionMode;
	const auto next = LockRelativeModeFromEnabled(enabled);
	if (prev == next) return;

	CalCtx.lockRelativePositionMode = next;
	CalCtx.ResolveLockMode();
	SaveProfile(CalCtx);

	char buf[120];
	snprintf(buf, sizeof buf, "lock_relative_position_ui_write: prev=%d now=%d effective=%d", (int)prev, (int)next,
	         (int)CalCtx.lockRelativePosition);
	Metrics::WriteLogAnnotation(buf);
}

static void SeedSavedHeadsetTrackerLockFromUi(const char* reason)
{
	if (CCal_SeedHeadMountProxyRelativeLock(reason)) {
		SaveProfile(CalCtx);
		SendHeadMountConfigFromCalCtx();
	}
	else {
		Metrics::WriteLogAnnotation("head_mount_relative_lock_seed_failed: reason=ui_requested");
	}
}

static bool FinishHardTrackerLockSetup()
{
	if (!CalCtx.headMount.offsetCalibrated) return false;
	if (!CalCtx.relativePosCalibrated && !CCal_SeedHeadMountProxyRelativeLock("ui_finish_hard_lock")) {
		Metrics::WriteLogAnnotation("head_mount_relative_lock_seed_failed: reason=ui_finish_hard_lock");
		return false;
	}
	if (ContinuousCalibrationIsRunning()) {
		EndContinuousCalibration();
	}
	ApplyTrackingStylePresetPreservingLockMode(CalCtx, TrackingStyle::HardTrackerLock);
	if (!CalCtx.relativePosCalibrated && !CCal_SeedHeadMountProxyRelativeLock("ui_finish_hard_lock_after_end")) {
		Metrics::WriteLogAnnotation("head_mount_relative_lock_seed_failed: reason=ui_finish_hard_lock_after_end");
		return false;
	}
	CalCtx.ResolveLockMode();
	SaveProfile(CalCtx);
	SendHeadMountConfigFromCalCtx();
	return true;
}

static wkopenvr::tracking_style_ui::ActionInputs BuildTrackingStyleActionInputs()
{
	wkopenvr::tracking_style_ui::ActionInputs in;
	in.style = CalCtx.trackingStyle;
	in.state = CalCtx.state;
	in.vrReady = IsVRReady();
	in.validProfile = CalCtx.validProfile;
	in.offsetCalibrated = CalCtx.headMount.offsetCalibrated;
	in.relativePosCalibrated = CalCtx.relativePosCalibrated;
	in.targetMatches = wkopenvr::headmount::HeadMountMatchesContinuousTarget(CalCtx);
	in.vrBlockedMessage = CalibrationBlockedMessage();

	if (TrackingStyleUsesHeadsetSynthesis(CalCtx.trackingStyle) && ContinuousCalibrationIsRunning()) {
		const auto preflight = wkopenvr::headmount::EvaluateOffsetCalibrationPreflight(CalCtx);
		in.offsetPreflightReady = preflight.ready;
		in.offsetPreflightMessage = preflight.message;
	}
	return in;
}

static void RunPrimaryTrackingAction()
{
	ApplyTrackingStylePresetPreservingLockMode(CalCtx, CalCtx.trackingStyle);
	CalCtx.ResolveLockMode();
	SaveProfile(CalCtx);
	SendHeadMountConfigFromCalCtx();

	switch (CalCtx.trackingStyle) {
		case TrackingStyle::Manual:
			if (ContinuousCalibrationIsRunning()) {
				EndContinuousCalibration();
				ApplyTrackingStylePresetPreservingLockMode(CalCtx, TrackingStyle::Manual);
				CalCtx.ResolveLockMode();
				SaveProfile(CalCtx);
				SendHeadMountConfigFromCalCtx();
			}
			ImGui::OpenPopup("Calibration Progress");
			StartCalibration("ui_tracking_style_manual");
			break;
		case TrackingStyle::Continuous:
			StartContinuousCalibration("ui_tracking_style_continuous");
			break;
		case TrackingStyle::LockedWithRecovery:
			if (!ContinuousCalibrationIsRunning()) {
				StartContinuousCalibration("ui_tracking_style_locked_recovery");
			}
			else if (!CalCtx.headMount.offsetCalibrated) {
				wkopenvr::headmount::OpenOffsetModal();
			}
			else if (!CalCtx.relativePosCalibrated) {
				SeedSavedHeadsetTrackerLockFromUi("ui_lock_recovery");
			}
			else {
				wkopenvr::headmount::OpenOffsetModal();
			}
			break;
		case TrackingStyle::HardTrackerLock:
			if (!ContinuousCalibrationIsRunning()) {
				StartContinuousCalibration("ui_tracking_style_hard_setup");
			}
			else if (!CalCtx.headMount.offsetCalibrated) {
				wkopenvr::headmount::OpenOffsetModal();
			}
			else if (!CalCtx.relativePosCalibrated) {
				SeedSavedHeadsetTrackerLockFromUi("ui_lock_hard");
			}
			else {
				FinishHardTrackerLockSetup();
			}
			break;
	}
}

void BuildTrackingStyleSetup(bool continuousCalibration)
{
	ImVec2 panelSize{ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x, 0};
	ImGui::BeginGroupPanel("Tracking style", panelSize);

	const TrackingStyle styles[] = {TrackingStyle::Manual, TrackingStyle::Continuous, TrackingStyle::LockedWithRecovery,
	                                TrackingStyle::HardTrackerLock};
	for (const TrackingStyle style : styles) {
		ImGui::PushID((int)style);
		if (ImGui::RadioButton(TrackingStyleLabel(style), CalCtx.trackingStyle == style)) {
			ApplySelectedTrackingStyle(style);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("%s", TrackingStyleSummary(style));
		}
		ImGui::PopID();
	}

	ImGui::Spacing();
	ImGui::TextUnformatted("Lock relative position");
	const bool lockRelativeOn = LockRelativeModeEnabled(CalCtx.lockRelativePositionMode);
	if (ImGui::RadioButton("Off##lock_relative_position", !lockRelativeOn)) {
		SetLockRelativePositionModeFromUi(false);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Continuous calibration may re-solve the relative pose between reference and target.");
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("On##lock_relative_position", lockRelativeOn)) {
		SetLockRelativePositionModeFromUi(true);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Freeze the calibrated relative pose. Useful when the target is rigidly attached.");
	}

	ImGui::Spacing();
	const bool headsetStyle = TrackingStyleUsesHeadsetSynthesis(CalCtx.trackingStyle);
	const bool needsOffset = headsetStyle && !CalCtx.headMount.offsetCalibrated;
	const bool needsRelativeLock = headsetStyle && CalCtx.headMount.offsetCalibrated && !CalCtx.relativePosCalibrated;
	const char* setupSuffix = needsOffset ? " | offset needed" : (needsRelativeLock ? " | relative pose needed" : "");
	ImGui::TextDisabled("Status: %s | %s%s", continuousCalibration ? "running" : "stopped",
	                    TrackingStyleSummary(CalCtx.trackingStyle), setupSuffix);

	ImGui::Spacing();
	std::string disabledReason;
	const auto actionInputs = BuildTrackingStyleActionInputs();
	const bool actionEnabled = wkopenvr::tracking_style_ui::PrimaryActionEnabled(actionInputs, &disabledReason);
	const char* actionLabel = wkopenvr::tracking_style_ui::PrimaryActionLabel(actionInputs);
	ImGui::BeginDisabled(!actionEnabled);
	if (ImGui::Button(actionLabel, ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * 2.0f))) {
		RunPrimaryTrackingAction();
	}
	ImGui::EndDisabled();
	if (!disabledReason.empty()) {
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("%s", disabledReason.c_str());
		}
		ImGui::TextDisabled("%s", disabledReason.c_str());
	}

	ImGui::EndGroupPanel();
}
