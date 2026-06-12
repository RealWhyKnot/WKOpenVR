#include "UserInterfaceHeadMount.h"

#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "HeadMountOffsetModal.h"
#include "HeadMountOffsetPreflight.h"
#include "HeadMountPreview.h"
#include "HeadMountTargetBinding.h"
#include "IPCClient.h"
#include "Protocol.h"
#include "TrackingStyle.h"
#include "UiHelpers.h"

#include <openvr.h>
#include <Eigen/Geometry>

#include <cstdio>
#include <cstring>
#include <exception>
#include <string>

extern SCIPCClient Driver;
extern CalibrationContext CalCtx;

void SaveProfile(CalibrationContext& ctx);
std::string LabelString(const StandbyDevice& device);

void CCal_SendHeadMountConfig()
{
	const auto& hm = CalCtx.headMount;
	protocol::Request req(protocol::RequestSetHeadMountConfig);
	auto& p = req.setHeadMountConfig;
	p.mode = static_cast<uint32_t>(hm.mode);
	p.deviceId = hm.deviceID;
	p.hideTracker = hm.hideTracker;
	p.offsetCalibrated = hm.offsetCalibrated;
	p.allowRawHmdFallback = hm.allowRawHmdFallback;
	p.lockedHeadsetSmoothing = hm.lockedHeadsetSmoothing;
	const auto timing = wkopenvr::headmount::ClampDriverSynthTimingConfig(hm.driverSynthTiming);
	p.driverSynthStaleLimitMs = static_cast<uint16_t>(timing.staleLimitMs);
	p.driverSynthGraceHoldMs = static_cast<uint16_t>(timing.graceHoldMs);
	p.driverSynthBlendToFallbackMs = static_cast<uint16_t>(timing.blendToFallbackMs);
	p.driverSynthStableBeforeSynthMs = static_cast<uint16_t>(timing.stableBeforeSynthMs);
	p.driverSynthBlendToSynthMs = static_cast<uint16_t>(timing.blendToSynthMs);

	size_t len = hm.trackerSerial.size();
	if (len >= sizeof p.trackerSerial) len = sizeof p.trackerSerial - 1;
	std::memcpy(p.trackerSerial, hm.trackerSerial.data(), len);
	p.trackerSerial[len] = '\0';

	len = hm.trackerTrackingSystem.size();
	if (len >= sizeof p.trackerTrackingSystem) len = sizeof p.trackerTrackingSystem - 1;
	std::memcpy(p.trackerTrackingSystem, hm.trackerTrackingSystem.data(), len);
	p.trackerTrackingSystem[len] = '\0';

	Eigen::Quaterniond q(hm.headFromTracker.linear());
	q.normalize();
	const Eigen::Vector3d t = hm.headFromTracker.translation();
	p.headFromTrackerTrans[0] = t.x();
	p.headFromTrackerTrans[1] = t.y();
	p.headFromTrackerTrans[2] = t.z();
	p.headFromTrackerRot[0] = q.x();
	p.headFromTrackerRot[1] = q.y();
	p.headFromTrackerRot[2] = q.z();
	p.headFromTrackerRot[3] = q.w();

	try {
		Driver.SendBlocking(req);
	}
	catch (const std::exception& e) {
		char buf[240];
		std::snprintf(buf, sizeof buf, "[head-mount] config push failed: %s", e.what());
		Metrics::WriteLogAnnotation(buf);
	}
}

namespace {

bool s_offsetSlidersOpen = false;

bool DrawDriverSynthTimingControl(const char* label, int& value, int minValue, int maxValue, const char* tooltip)
{
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(0);
	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted(label);
	ImGui::TableSetColumnIndex(1);
	ImGui::PushItemWidth(-1.0f);
	const bool changed = ImGui::SliderInt("##value", &value, minValue, maxValue, "%d ms");
	ImGui::PopItemWidth();
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("%s", tooltip);
	}
	return changed;
}

void DrawDriverSynthTimingPanel(HeadMountConfig& hm)
{
	if (hm.mode != HeadMountMode::DriverSynth) return;

	ImGui::Spacing();
	ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
	if (!ImGui::CollapsingHeader("Advanced fallback timing")) return;

	ImGui::Indent();
	if (ImGui::BeginTable("driver_synth_timing", 2,
	                      ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoSavedSettings)) {
		auto timing = wkopenvr::headmount::ClampDriverSynthTimingConfig(hm.driverSynthTiming);
		bool changed = false;
		ImGui::PushID("stale_limit");
		changed |= DrawDriverSynthTimingControl(
		    "Tracker stale limit", timing.staleLimitMs, wkopenvr::headmount::kDriverSynthStaleLimitMsMin,
		    wkopenvr::headmount::kDriverSynthStaleLimitMsMax,
		    "How old the last tracker pose can be before it is treated as missing. Raise this so brief\n"
		    "tracker dropouts do not fall back to the raw headset (which can move with inside-out relocalization).");
		ImGui::PopID();
		ImGui::PushID("grace_hold");
		changed |= DrawDriverSynthTimingControl(
		    "Grace hold", timing.graceHoldMs, wkopenvr::headmount::kDriverSynthTransitionMsMin,
		    wkopenvr::headmount::kDriverSynthTransitionMsMax,
		    "How long to keep the last tracker-synth pose before fading to headset tracking.");
		ImGui::PopID();
		ImGui::PushID("blend_out");
		changed |= DrawDriverSynthTimingControl(
		    "Blend to fallback", timing.blendToFallbackMs, wkopenvr::headmount::kDriverSynthTransitionMsMin,
		    wkopenvr::headmount::kDriverSynthTransitionMsMax,
		    "Fade duration from tracker-synth pose to headset tracking after grace expires.");
		ImGui::PopID();
		ImGui::PushID("stable_return");
		changed |= DrawDriverSynthTimingControl(
		    "Stable before return", timing.stableBeforeSynthMs, wkopenvr::headmount::kDriverSynthTransitionMsMin,
		    wkopenvr::headmount::kDriverSynthTransitionMsMax,
		    "How long the tracker must be good again before WKOpenVR blends back to it.");
		ImGui::PopID();
		ImGui::PushID("blend_in");
		changed |= DrawDriverSynthTimingControl("Blend back to tracker", timing.blendToSynthMs,
		                                        wkopenvr::headmount::kDriverSynthTransitionMsMin,
		                                        wkopenvr::headmount::kDriverSynthTransitionMsMax,
		                                        "Fade duration from headset tracking back to tracker-synth pose.");
		ImGui::PopID();
		ImGui::EndTable();

		if (changed) {
			hm.driverSynthTiming = timing;
			SaveProfile(CalCtx);
			CCal_SendHeadMountConfig();
		}
	}
	if (ImGui::Button("Reset fallback timing")) {
		hm.driverSynthTiming = {};
		SaveProfile(CalCtx);
		CCal_SendHeadMountConfig();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Restore the default fallback timing values.");
	}
	ImGui::Unindent();
}

} // namespace

void CCal_DrawHeadMountSection(const ImVec2& panelSize)
{
	const auto& pal = openvr_pair::overlay::ui::GetPalette();
	auto& hm = CalCtx.headMount;
	const bool bindingChanged = wkopenvr::headmount::BindHeadMountToContinuousTarget(CalCtx);
	if (bindingChanged) {
		SaveProfile(CalCtx);
		CCal_SendHeadMountConfig();
	}
	const bool hasContinuousTarget = wkopenvr::headmount::HasContinuousTargetIdentity(CalCtx);

	{
		openvr_pair::overlay::ui::PanelScope panel("Step 1: Head-mounted tracker", panelSize);

		ImGui::TextWrapped("Uses the active continuous target as the lighthouse tracker attached to your headset.");
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip(
			    "Start continuous calibration with the headset-mounted lighthouse tracker selected as the target.\n"
			    "This tab binds to that target automatically, so you do not have to pick the same tracker twice.");
		}

		ImGui::Spacing();

		ImGui::TextUnformatted("1. Continuous target");
		if (hasContinuousTarget) {
			const std::string label = LabelString(CalCtx.targetStandby);
			ImGui::TextWrapped("%s", label.c_str());
			if (CalCtx.targetID < 0) {
				ImGui::SameLine();
				ImGui::TextDisabled("(waiting for device)");
			}
		}
		else {
			ImGui::TextDisabled("Start continuous calibration with the headset-mounted tracker as the target.");
		}

		const bool hasTracker = hasContinuousTarget && !hm.trackerSerial.empty();
		const bool offsetOk = hm.offsetCalibrated;
		if ((!hasTracker || !offsetOk) && s_offsetSlidersOpen) {
			s_offsetSlidersOpen = false;
			wkopenvr::headmount::TickPreview(false, Eigen::Affine3d::Identity(), Eigen::AffineCompact3d::Identity(),
			                                 wkopenvr::headmount::HeadMountPreviewTrackingOrigin(),
			                                 "fine_tune_unavailable");
		}
		const wkopenvr::headmount::OffsetCalibrationPreflight offsetPreflight =
		    wkopenvr::headmount::EvaluateOffsetCalibrationPreflight(CalCtx);

		ImGui::Spacing();
		ImGui::TextUnformatted("2. Calibrate the tracker-to-headset offset");
		{
			openvr_pair::overlay::ui::DisabledSection ds(!offsetPreflight.ready, offsetPreflight.message);
			const char* btnLabel = offsetOk ? "Re-calibrate offset" : "Calibrate offset";
			if (ImGui::Button(btnLabel)) {
				wkopenvr::headmount::OpenOffsetModal();
			}
			ds.AttachReasonTooltip();
		}
		if (ImGui::IsItemHovered() && offsetPreflight.ready) {
			ImGui::SetTooltip("Solves the rigid offset from the tracker to the headset.\n"
			                  "Move your head slowly through pitch, yaw, and roll for ~10 seconds.");
		}
		ImGui::SameLine();
		{
			openvr_pair::overlay::ui::DisabledSection ds(
			    !hasTracker || !offsetOk,
			    !hasTracker ? "Start continuous calibration with the headset-mounted tracker as the target first."
			                : "Calibrate offset first.");
			if (ImGui::Button(s_offsetSlidersOpen ? "Hide fine-tune" : "Fine-tune offset")) {
				s_offsetSlidersOpen = !s_offsetSlidersOpen;
				if (!s_offsetSlidersOpen) {
					wkopenvr::headmount::TickPreview(
					    false, Eigen::Affine3d::Identity(), Eigen::AffineCompact3d::Identity(),
					    wkopenvr::headmount::HeadMountPreviewTrackingOrigin(), "fine_tune_closed");
				}
			}
			ds.AttachReasonTooltip();
		}
		if (ImGui::IsItemHovered() && offsetOk) {
			ImGui::SetTooltip("Nudge the solved offset by hand. Useful if the auto-solve\n"
			                  "got close but the in-headset preview marker doesn't stay\n"
			                  "centered in front of your view.");
		}
		if (!offsetPreflight.ready) {
			ImGui::TextDisabled("%s", offsetPreflight.message);
		}

		if (s_offsetSlidersOpen && offsetOk) {
			ImGui::Spacing();
			wkopenvr::headmount::DrawOffsetInlinePanel();
		}

		ImGui::Spacing();
		ImGui::TextUnformatted("3. Tracking style");
		ImGui::TextDisabled("%s", TrackingStyleLabel(CalCtx.trackingStyle));
		const bool headsetStyle = TrackingStyleUsesHeadsetSynthesis(CalCtx.trackingStyle);
		ImGui::TextDisabled("Headset synthesis: %s", headsetStyle ? "on" : "off");
		if (headsetStyle) {
			ImGui::TextDisabled("Raw HMD fallback: %s", hm.allowRawHmdFallback ? "on" : "off");
			int lockedSmoothing = (int)hm.lockedHeadsetSmoothing;
			ImGui::SetNextItemWidth(200.0f);
			const char* smoothFmt = lockedSmoothing > 0 ? "%d" : "off";
			if (ImGui::SliderInt("Smooth locked headset", &lockedSmoothing, 0, 100, smoothFmt)) {
				if (lockedSmoothing < 0) lockedSmoothing = 0;
				if (lockedSmoothing > 100) lockedSmoothing = 100;
				hm.lockedHeadsetSmoothing = (uint8_t)lockedSmoothing;
				SaveProfile(CalCtx);
				CCal_SendHeadMountConfig();
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Speed-adaptive low-pass on the headset pose while the head-mounted tracker\n"
				                  "drives it. Higher = steadier at rest; real head motion stays responsive. 0 = off.");
			}
			DrawDriverSynthTimingPanel(hm);
		}

		// Auto-correct headset tracker offset is demoted to shadow-log-only: the
		// continuous-cal offset check is logged (shadow_offset_would_apply) but never
		// changes the saved offset, so the toggle is intentionally hidden. The offset
		// is set via the manual offset calibration.

		ImGui::Spacing();
		{
			static int32_t s_lastDeviceID = -2;
			static std::string s_lastSerial, s_lastModel, s_lastSystem;
			static int s_lastBits = -1;
			const int32_t curDev = hm.deviceID;
			const bool inRange = curDev >= 0 && (uint32_t)curDev < vr::k_unMaxTrackedDeviceCount;
			const bool poseValid = inRange && CalCtx.devicePoses[curDev].poseIsValid;
			const int curBits = (inRange ? 1 : 0) | (poseValid ? 2 : 0);
			if (curDev != s_lastDeviceID || hm.trackerSerial != s_lastSerial || hm.trackerModel != s_lastModel ||
			    hm.trackerTrackingSystem != s_lastSystem || curBits != s_lastBits) {
				s_lastDeviceID = curDev;
				s_lastSerial = hm.trackerSerial;
				s_lastModel = hm.trackerModel;
				s_lastSystem = hm.trackerTrackingSystem;
				s_lastBits = curBits;
				char lbuf[320];
				std::snprintf(lbuf, sizeof lbuf,
				              "[head-mount-status] deviceID=%d inRange=%d poseIsValid=%d"
				              " serial='%s' model='%s' system='%s' targetID=%d state=%d",
				              (int)curDev, (int)inRange, (int)poseValid, hm.trackerSerial.c_str(),
				              hm.trackerModel.c_str(), hm.trackerTrackingSystem.c_str(), (int)CalCtx.targetID,
				              (int)CalCtx.state);
				Metrics::WriteLogAnnotation(lbuf);
			}

			const bool trackerValid = hm.deviceID >= 0 && (uint32_t)hm.deviceID < vr::k_unMaxTrackedDeviceCount &&
			                          CalCtx.devicePoses[hm.deviceID].poseIsValid;

			if (!hasTracker) {
				ImGui::TextDisabled("No tracker selected.");
			}
			else if (!trackerValid) {
				openvr_pair::overlay::ui::DrawStatusDot(pal.dotError);
				ImGui::TextColored(pal.statusError, "Tracker not reporting a valid pose.");
			}
			else if (!offsetOk) {
				openvr_pair::overlay::ui::DrawStatusDot(pal.dotPending);
				ImGui::TextColored(pal.statusWarn, "Offset uncalibrated.");
			}
			else if (hm.mode == HeadMountMode::Off) {
				openvr_pair::overlay::ui::DrawStatusDot(pal.dotPending);
				ImGui::TextColored(pal.statusWarn, "Headset synthesis is off for this style.");
			}
			else {
				const double residualMm =
				    (Metrics::headMountResidualMm.size() > 0) ? Metrics::headMountResidualMm.last() : 0.0;
				char buf[128];
				if (residualMm > 0.0) {
					std::snprintf(buf, sizeof buf, "Active. Offset residual %.2f mm.", residualMm);
				}
				else {
					std::snprintf(buf, sizeof buf, "Active.");
				}
				openvr_pair::overlay::ui::DrawStatusDot(pal.dotOk);
				ImGui::TextColored(pal.statusOk, "%s", buf);
			}
		}
	}
}
