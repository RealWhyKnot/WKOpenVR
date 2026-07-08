#include "Calibration.h"
#include "CalibrationAutoSpeed.h"
#include "CalibrationExperimentFlags.h"
#include "Configuration.h"
#include "CalibrationMetrics.h"
#include "Wizard.h"
#include "UiHelpers.h"

#include <openvr.h>
#include <string>
#include <imgui/imgui.h>
#include "imgui_extensions.h"

// Forward decl for DrawVectorElement defined in UserInterface.cpp
void DrawVectorElement(const std::string id, const char* text, double* value, int defaultValue = 0,
                       const char* defaultValueStr = " 0 ");

// Forward decl (defined in Calibration.cpp). Declared locally rather than via
// CalibrationInternal.h because that header pulls in <openvr_driver.h>, which
// conflicts with the <openvr.h> this translation unit already includes.
void SendFreezeAllTracking();

static void ScaledDragFloat(const char* label, double& f, double scale, double min, double max,
                            int flags = ImGuiSliderFlags_AlwaysClamp)
{
	float v = (float)(f * scale);
	std::string labelStr = std::string(label);

	// If starts with ##, just do a normal SliderFloat
	if (labelStr.size() > 2 && labelStr[0] == '#' && labelStr[1] == '#') {
		ImGui::SliderFloat(label, &v, (float)min, (float)max, "%1.2f", flags);
	}
	else {
		// Otherwise do funny
		ImGui::Text("%s", label);
		ImGui::SameLine();
		ImGui::PushID((std::string(label) + "_id").c_str());
		// Line up to a column, multiples of 100
		constexpr uint32_t LABEL_CURSOR = 100;
		uint32_t cursorPosX = (int)ImGui::GetCursorPosX();
		uint32_t roundedPosition = ((cursorPosX + LABEL_CURSOR / 2) / LABEL_CURSOR) * LABEL_CURSOR;
		ImGui::SetCursorPosX((float)roundedPosition);
		ImGui::SliderFloat((std::string("##") + label).c_str(), &v, (float)min, (float)max, "%1.2f", flags);
		ImGui::PopID();
	}

	f = v / scale;
}

// Helper for the per-slider "Reset to default" right-click context menu added in
// the settings split. Always pops up on right-click of the slider that called
// AddDefaultsContextMenu() last (i.e. the previous widget in submission order).
// resetFn is run when the user picks "Reset to default".
template <typename Fn> static void AddResetContextMenu(const char* popupId, Fn resetFn)
{
	if (ImGui::BeginPopupContextItem(popupId)) {
		if (ImGui::MenuItem("Reset to default")) {
			resetFn();
		}
		ImGui::EndPopup();
	}
}

static void LogExperimentToggle(const char* optionName, bool enabled)
{
	const std::string annotation = spacecal::calibration_experiments::ToggleAnnotation(optionName, enabled);
	Metrics::WriteLogAnnotation(annotation.c_str());
}

static bool ExperimentCheckbox(const char* optionName, const char* id, bool* value, const char* tooltip)
{
	if (!openvr_pair::overlay::ui::CheckboxWithTooltip(id, value, tooltip)) return false;
	LogExperimentToggle(optionName, *value);
	return true;
}

// Persist a continuous-speed radio choice with a log annotation; the setting
// changes the solve window (and so the whole session's log shape), which made
// unannotated changes a blind spot when reading incident logs.
static void SetContinuousSpeed(CalibrationContext::Speed value)
{
	if (CalCtx.continuousCalibrationSpeed != value) {
		Metrics::LogAnnotationf("calibration_speed_changed: source=ui which=continuous prev=%s now=%s",
		                        CalibrationContext::SpeedName(CalCtx.continuousCalibrationSpeed),
		                        CalibrationContext::SpeedName(value));
	}
	CalCtx.continuousCalibrationSpeed = value;
	SaveProfile(CalCtx);
}

// Render the watchdog / HMD-stall diagnostic counters wrapped in a group panel.
// Lives in Advanced (not Basic) since these are bug-report breadcrumbs, not
// something a casual user needs to see while running.
static void DrawDiagnosticsPanel(ImVec2 panelSize)
{
	openvr_pair::overlay::ui::DrawPanel(
	    "Diagnostics",
	    [&] {
		    const auto& pal = openvr_pair::overlay::ui::GetPalette();

		    // Watchdog reset tracking. We reflect whether the count has changed
		    // recently (within ~15 s) by colouring the line amber, matching the
		    // continuous-recalibration banner pattern.
		    static int s_lastSeenWatchdog = -1;
		    static double s_lastWatchdogResetTime = 0.0;
		    static int s_lastSeenStallCount = 0;
		    static int s_stallPurgeCount = 0;
		    static double s_lastStallPurgeTime = 0.0;

		    const int wdResets = GetWatchdogResetCount();
		    const double now = ImGui::GetTime();
		    if (s_lastSeenWatchdog < 0) {
			    s_lastSeenWatchdog = wdResets;
		    }
		    else if (wdResets != s_lastSeenWatchdog) {
			    s_lastSeenWatchdog = wdResets;
			    s_lastWatchdogResetTime = now;
		    }

		    // Long-stall counter: HMD stalled for >=30 ticks (~1.5 s). Previously the
		    // calibration tick purged the sample buffer at this point; reverted
		    // 2026-05-04 because the purge + warm-start re-anchor caused cumulative
		    // drift on every HMD off/on cycle. The counter remains as a diagnostic
		    // -- frequent long-stalls still indicate a tracking-environment problem.
		    const int kHmdLongStallThreshold = 30;
		    const int curStalls = CalCtx.consecutiveHmdStalls;
		    if (curStalls >= kHmdLongStallThreshold && s_lastSeenStallCount < kHmdLongStallThreshold) {
			    s_stallPurgeCount++;
			    s_lastStallPurgeTime = now;
		    }
		    s_lastSeenStallCount = curStalls;

		    const bool wdRecent =
		        wdResets > 0 && (now - s_lastWatchdogResetTime) < 15.0 && s_lastWatchdogResetTime > 0.0;
		    if (wdRecent) {
			    ImGui::PushStyleColor(ImGuiCol_Text, pal.statusPending);
			    ImGui::Text("Watchdog reset %.0fs ago - recollecting samples (count: %d)",
			                now - s_lastWatchdogResetTime, wdResets);
			    ImGui::PopStyleColor();
		    }
		    else if (wdResets == 0) {
			    ImGui::TextDisabled("Watchdog resets: 0 (last: never)");
		    }
		    else {
			    ImGui::TextDisabled("Watchdog resets: %d (last: %.0fs ago)", wdResets,
			                        s_lastWatchdogResetTime > 0.0 ? (now - s_lastWatchdogResetTime) : 0.0);
		    }
		    if (ImGui::IsItemHovered()) {
			    ImGui::SetTooltip(
			        "Stuck-loop watchdog: fires when continuous calibration has been rejecting every new\n"
			        "sample for ~25 seconds. When it fires, the current estimate is discarded and\n"
			        "we recollect from scratch. A high count here usually means motion conditioning is\n"
			        "poor (move slower, rotate around more axes) or trackers are drifting against each\n"
			        "other faster than the solver can keep up.");
		    }

		    const bool stallRecent = s_stallPurgeCount > 0 && (now - s_lastStallPurgeTime) < 15.0;
		    if (stallRecent) {
			    ImGui::PushStyleColor(ImGuiCol_Text, pal.statusPending);
			    ImGui::Text("HMD long-stall %.0fs ago (count: %d)", now - s_lastStallPurgeTime, s_stallPurgeCount);
			    ImGui::PopStyleColor();
		    }
		    else if (s_stallPurgeCount == 0) {
			    ImGui::TextDisabled("HMD long-stalls: 0 (last: never)");
		    }
		    else {
			    ImGui::TextDisabled("HMD long-stalls: %d (last: %.0fs ago)", s_stallPurgeCount,
			                        now - s_lastStallPurgeTime);
		    }
		    if (ImGui::IsItemHovered()) {
			    ImGui::SetTooltip("HMD long-stall: the headset stopped reporting fresh poses for ~1.5 seconds or more\n"
			                      "(SteamVR hiccup, tracking loss, headset taken off). Diagnostic counter only --\n"
			                      "calibration is no longer disturbed during a stall (rolling sample buffer ages out\n"
			                      "stale samples naturally on recovery). Frequent long-stalls suggest a tracking-\n"
			                      "environment problem (lighting, USB bandwidth, etc).");
		    }
	    },
	    panelSize);
}

// Tip strip for both tabs. Reminds the user that hover = tooltip and
// right-click on sliders = reset to default. Light-weight, identical between
// Basic and Advanced so the hint is always reachable.
static void DrawTipPanel(ImVec2 panelSize)
{
	openvr_pair::overlay::ui::DrawPanel(
	    "Tip",
	    [&] {
		    ImGui::TextWrapped("Hover over any setting to learn more about it. Right-click any slider to reset it to "
		                       "its default value.");
	    },
	    panelSize);
}

void CCal_DrawSettings()
{
	// panel size for boxes
	ImVec2 panel_size{ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x, 0};

	// Non-continuous and continuous Advanced read as two different surfaces.
	// Continuous owns the long-session refinement knobs (recalibration
	// threshold, tracker offset, target-quash); one-shot only
	// uses a couple of always-applicable thresholds.
	// Everything that only does something while continuous calibration is
	// running is gated on this flag and hidden in one-shot.
	const bool kInContinuous =
	    CalCtx.state == CalibrationState::Continuous || CalCtx.state == CalibrationState::ContinuousStandby;

	// Diagnostics panel: continuous-only watchdog + HMD-stall counters. Both
	// only tick during the continuous-cal refiner; rendering them outside
	// continuous would always show 0 / never, which is more confusing than
	// informative.
	if (kInContinuous) {
		DrawDiagnosticsPanel(panel_size);
	}

	// Diagnostic sample rejection. Style presets own tracker hiding and
	// head-mount behavior; this panel keeps only the solver diagnostic toggle.
	if (kInContinuous) {
		ImGui::BeginGroupPanel("Diagnostics", panel_size);
		ImGui::Checkbox("Ignore outliers", &CalCtx.ignoreOutliers);
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip(
			    "Drop sample pairs whose rotation axis disagrees with the consensus before the LS solve.\n"
			    "Default on.  Turn off only if you suspect the outlier rejector is throwing out good samples\n"
			    "(e.g. genuinely jittery motion the cosine-similarity test mistakes for outliers).");
		}
		if (ExperimentCheckbox("precision_weighted_relpose", "Weight samples by distance##precision_weighted_relpose",
		                       &CalCtx.precisionWeightedRelPose,
		                       "Weight each sample by how far the devices are from the playspace origin\n"
		                       "(closer = more trustworthy) when the locked relative pose is averaged.\n"
		                       "Keeps a far-from-origin playspace from slowly dragging the calibration.\n"
		                       "Default on.  Turn off for the plain uniform average.")) {
			SaveProfile(CalCtx);
		}
		ImGui::EndGroupPanel();
		ImGui::Spacing();
	}

	// === ADVANCED SETTINGS ==================================================
	// All technical knobs.  No longer behind a CollapsingHeader: the tab
	// itself is now the gate.  A user clicking "Advanced" is signalling
	// they want to see everything at once, so flatten the hierarchy.
	{
		// Calibration speed radio + speed-threshold matrix. The whole
		// speed-tuning surface is continuous-only: it tunes the rate at which
		// continuous calibration drags the offset back into place. One-shot
		// runs the calibrator once and applies the result; speed thresholds
		// do nothing there.
		if (kInContinuous) {
			ImGui::BeginGroupPanel("Calibration speeds", panel_size);

			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
			ImGui::TextWrapped(
			    "SpaceCalibrator uses up to three different speeds at which it drags the calibration back into "
			    "position when drift occurs. These settings control how far off the calibration should be before going "
			    "back to low speed (for "
			    "Decel) or going to higher speeds (for Slow and Fast).");
			ImGui::PopStyleColor();

			// Calibration Speed radio. Also rendered in the one-shot Settings
			// tab; gate on continuous mode here to avoid the same radio appearing
			// twice when state == None (both tabs visible at once).
			if (kInContinuous) {
				ImGui::BeginGroupPanel("Calibration speed", panel_size);

				auto speed = CalCtx.continuousCalibrationSpeed;

				ImGui::Columns(4, nullptr, false);
				if (ImGui::RadioButton(" Auto          ", speed == CalibrationContext::AUTO)) {
					SetContinuousSpeed(CalibrationContext::AUTO);
				}
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("Pick the speed automatically from correction error and fresh-fit RMS.\n"
					                  "Uses Fast while the current calibration is fixably off, then sizes\n"
					                  "the buffer by the settled noise floor.\n"
					                  "Recommended for continuous mode. (One-shot mode hides Auto because\n"
					                  "it has no second chance to switch.)");
				}
				ImGui::NextColumn();
				if (ImGui::RadioButton(" Fast          ", speed == CalibrationContext::FAST)) {
					SetContinuousSpeed(CalibrationContext::FAST);
				}
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("30-sample buffer. Fastest convergence, most sensitive to noise.\n"
					                  "Good for clean lighthouse setups and tracker-on-HMD mounts.");
				}
				ImGui::NextColumn();
				if (ImGui::RadioButton(" Slow          ", speed == CalibrationContext::SLOW)) {
					SetContinuousSpeed(CalibrationContext::SLOW);
				}
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("100-sample buffer. Smoother result at the cost of slower response.\n"
					                  "Good for typical mixed setups.");
				}
				ImGui::NextColumn();
				if (ImGui::RadioButton(" Very Slow     ", speed == CalibrationContext::VERY_SLOW)) {
					SetContinuousSpeed(CalibrationContext::VERY_SLOW);
				}
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("200-sample buffer. Maximum smoothing, slowest convergence.\n"
					                  "For noisy / reflective rooms or drift-prone IMU trackers.");
				}
				ImGui::Columns(1);

				// Show the resolved speed when AUTO is on so the user understands
				// what the program decided. Faded text so it doesn't draw the eye.
				if (speed == CalibrationContext::AUTO) {
					namespace cs = spacecal::calibration_speed;
					const auto decision = CalCtx.lastAutoSpeedDecision;
					const char* resolvedName = cs::AutoSpeedBucketName(decision.bucket);
					const bool haveFresh = cs::IsUsableFitRmsMm(decision.freshFitMm);
					const bool haveCorrection = cs::IsUsableFitRmsMm(decision.correctionFitMm);
					ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
					if (haveFresh || haveCorrection) {
						ImGui::Text("    %s (%s): fresh %.2f mm, reducible %.2f mm", resolvedName,
						            cs::AutoSpeedPhaseName(decision.state.phase),
						            haveFresh ? decision.freshFitMm : -1.0, decision.reducibleMm);
					}
					else {
						ImGui::Text("    Currently resolved to: %s  (waiting for first fit)", resolvedName);
					}
					ImGui::PopStyleColor();
				}

				ImGui::EndGroupPanel();
			}

			if (ImGui::BeginTable("SpeedThresholds", 3, 0)) {
				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(1);
				ImGui::Text("Translation (mm)");
				ImGui::TableSetColumnIndex(2);
				ImGui::Text("Rotation (degrees)");

				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::Text("Decel");
				ImGui::TableSetColumnIndex(1);
				ScaledDragFloat("##TransDecel", CalCtx.alignmentSpeedParams.thr_trans_tiny, 1000.0, 0, 20.0);
				AddResetContextMenu("trans_decel_ctx",
				                    [] { CalCtx.alignmentSpeedParams.thr_trans_tiny = 0.98f / 1000.0f; });
				ImGui::TableSetColumnIndex(2);
				ScaledDragFloat("##RotDecel", CalCtx.alignmentSpeedParams.thr_rot_tiny, 180.0 / EIGEN_PI, 0, 5.0);
				AddResetContextMenu("rot_decel_ctx", [] {
					CalCtx.alignmentSpeedParams.thr_rot_tiny = 0.49f * (float)(EIGEN_PI / 180.0f);
				});

				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::Text("Slow");
				ImGui::TableSetColumnIndex(1);
				ScaledDragFloat("##TransSlow", CalCtx.alignmentSpeedParams.thr_trans_small, 1000.0,
				                CalCtx.alignmentSpeedParams.thr_trans_tiny * 1000.0, 20.0);
				AddResetContextMenu("trans_slow_ctx",
				                    [] { CalCtx.alignmentSpeedParams.thr_trans_small = 1.0f / 1000.0f; });
				ImGui::TableSetColumnIndex(2);
				ScaledDragFloat("##RotSlow", CalCtx.alignmentSpeedParams.thr_rot_small, 180.0 / EIGEN_PI,
				                CalCtx.alignmentSpeedParams.thr_rot_tiny * (180.0 / EIGEN_PI), 10.0);
				AddResetContextMenu("rot_slow_ctx", [] {
					CalCtx.alignmentSpeedParams.thr_rot_small = 0.5f * (float)(EIGEN_PI / 180.0f);
				});

				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::Text("Fast");
				ImGui::TableSetColumnIndex(1);
				ScaledDragFloat("##TransFast", CalCtx.alignmentSpeedParams.thr_trans_large, 1000.0,
				                CalCtx.alignmentSpeedParams.thr_trans_small * 1000.0, 50.0);
				AddResetContextMenu("trans_fast_ctx",
				                    [] { CalCtx.alignmentSpeedParams.thr_trans_large = 20.0f / 1000.0f; });
				ImGui::TableSetColumnIndex(2);
				ScaledDragFloat("##RotFast", CalCtx.alignmentSpeedParams.thr_rot_large, 180.0 / EIGEN_PI,
				                CalCtx.alignmentSpeedParams.thr_rot_small * (180.0 / EIGEN_PI), 20.0);
				AddResetContextMenu("rot_fast_ctx", [] {
					CalCtx.alignmentSpeedParams.thr_rot_large = 5.0f * (float)(EIGEN_PI / 180.0f);
				});

				ImGui::EndTable();
			}

			ImGui::EndGroupPanel();
		} // if (kInContinuous): Calibration speeds

		// Alignment speeds (Decel/Slow/Fast). Same continuous-only domain as
		// Calibration speeds above -- the alignment-rate caps feed
		// ComputeIncremental, which only runs during continuous calibration.
		if (kInContinuous) {
			ImGui::BeginGroupPanel("Alignment speeds", panel_size);

			ScaledDragFloat("Decel", CalCtx.alignmentSpeedParams.align_speed_tiny, 1.0, 0, 2.0, 0);
			AddResetContextMenu("align_decel_ctx", [] { CalCtx.alignmentSpeedParams.align_speed_tiny = 1.0f; });
			ScaledDragFloat("Slow", CalCtx.alignmentSpeedParams.align_speed_small, 1.0, 0, 2.0, 0);
			AddResetContextMenu("align_slow_ctx", [] { CalCtx.alignmentSpeedParams.align_speed_small = 1.0f; });
			ScaledDragFloat("Fast", CalCtx.alignmentSpeedParams.align_speed_large, 1.0, 0, 2.0, 0);
			AddResetContextMenu("align_fast_ctx", [] { CalCtx.alignmentSpeedParams.align_speed_large = 2.0f; });

			ImGui::EndGroupPanel();
		}

		// Other advanced sliders
		{
			ImGui::BeginGroupPanel("Thresholds", panel_size);

			// Jitter threshold (moved from Basic / one-shot Settings -- these
			// are rarely touched and were padding the Basic surfaces). Sized
			// label/control via a 2-col table so they line up cleanly with
			// the other thresholds.
			openvr_pair::overlay::ui::DrawSettingTable(
			    "##advanced_thresholds_grid", 230.0f, [&](openvr_pair::overlay::ui::SettingTableScope& table) {
				    openvr_pair::overlay::ui::SettingRow(table, "Jitter threshold", [&] {
					    ImGui::PushID("adv_jitter_threshold");
					    ImGui::SetNextItemWidth(-FLT_MIN);
					    openvr_pair::overlay::ui::SliderFloatWithTooltip(
					        "##adv_jitter_threshold_slider", &CalCtx.jitterThreshold, 0.1f, 10.0f, "%1.1f",
					        "Controls how much jitter will be allowed for calibration.\n"
					        "Higher values allow worse tracking to calibrate, but may result in poorer tracking.");
					    AddResetContextMenu("adv_jitter_threshold_ctx", [] { CalCtx.jitterThreshold = 3.0f; });
					    ImGui::PopID();
				    });

				    // Recalibration threshold + Max relative error threshold both
				    // feed ComputeIncremental, the continuous-mode refiner.
				    if (kInContinuous) {
					    openvr_pair::overlay::ui::SettingRow(table, "Recalibration threshold", [&] {
						    ImGui::PushID("adv_recalibration_threshold");
						    ImGui::SetNextItemWidth(-FLT_MIN);
						    openvr_pair::overlay::ui::SliderFloatWithTooltip(
						        "##adv_recalibration_threshold_slider", &CalCtx.continuousCalibrationThreshold, 1.01f,
						        10.0f, "%1.1f",
						        "Controls how good the calibration must be before realigning the trackers.\n"
						        "Higher values cause calibration to happen less often, and may be useful for "
						        "systems with lots of tracking drift.");
						    AddResetContextMenu("adv_recalibration_threshold_ctx",
						                        [] { CalCtx.continuousCalibrationThreshold = 1.5f; });
						    ImGui::PopID();
					    });

					    openvr_pair::overlay::ui::SettingRow(table, "Max relative error threshold", [&] {
						    ImGui::PushID("adv_max_relative_error_threshold");
						    ImGui::SetNextItemWidth(-FLT_MIN);
						    openvr_pair::overlay::ui::SliderFloatWithTooltip(
						        "##adv_max_relative_error_threshold_slider", &CalCtx.maxRelativeErrorThreshold, 0.01f,
						        1.0f, "%1.1f",
						        "Controls the maximum acceptable relative error. If the error from the "
						        "relative calibration is too poor, the calibration will be discarded.");
						    AddResetContextMenu("adv_max_rel_err_ctx",
						                        [] { CalCtx.maxRelativeErrorThreshold = 0.005f; });
						    ImGui::PopID();
					    });
				    }
			    });
			ImGui::EndGroupPanel();
		}

		// Tracker offset is continuous-only: only the Continuous path applies
		// it via continuousCalibrationOffset. Hide outside continuous so the
		// one-shot Advanced tab isn't padded with a knob that does nothing.
		if (kInContinuous) {
			ImVec2 panel_size_inner{panel_size.x - 11 * 2, 0};
			ImGui::BeginGroupPanel("Tracker offset", panel_size_inner);
			DrawVectorElement("cc_tracker_offset", "X", &CalCtx.continuousCalibrationOffset.x());
			DrawVectorElement("cc_tracker_offset", "Y", &CalCtx.continuousCalibrationOffset.y());
			DrawVectorElement("cc_tracker_offset", "Z", &CalCtx.continuousCalibrationOffset.z());
			ImGui::EndGroupPanel();
		}

		{
			ImVec2 panel_size_inner{panel_size.x - 11 * 2, 0};
			ImGui::BeginGroupPanel("Playspace scale", panel_size_inner);
			DrawVectorElement("cc_playspace_scale", "Playspace Scale", &CalCtx.calibratedScale, 1, " 1 ");
			ImGui::EndGroupPanel();
		}

		{
			ImGui::BeginGroupPanel("Freeze all tracking", panel_size);
			bool frozen = CalCtx.freezeAllTracking;
			if (openvr_pair::overlay::ui::CheckboxWithTooltip(
			        "Freeze all tracking##freeze_all_tracking", &frozen,
			        "Hold every tracker and controller perfectly still until you turn this off -- a time freeze.\n\n"
			        "While frozen your controllers can't aim the laser, so toggle it back off from this window on the "
			        "desktop, or with the Ctrl+Alt+F hotkey.")) {
				CalCtx.freezeAllTracking = frozen;
				SendFreezeAllTracking();
				Metrics::LogAnnotationf("freeze_all_tracking: source=ui frozen=%d include_hmd=%d",
				                        (int)CalCtx.freezeAllTracking, (int)CalCtx.freezeIncludeHmd);
			}
			ImGui::Indent();
			if (openvr_pair::overlay::ui::CheckboxWithTooltip(
			        "Include headset##freeze_include_hmd", &CalCtx.freezeIncludeHmd,
			        "Also freeze the headset view. This is the literal time freeze, but the rendered image locks to "
			        "your head (turning your head won't update the view), which can be disorienting. Default off.")) {
				if (CalCtx.freezeAllTracking) SendFreezeAllTracking();
				SaveProfile(CalCtx);
			}
			ImGui::Unindent();
			ImGui::EndGroupPanel();
		}

		{
			ImGui::BeginGroupPanel("Experimental", panel_size);
			openvr_pair::overlay::ui::DrawSettingTable(
			    "##advanced_experimental_grid", 230.0f, [&](openvr_pair::overlay::ui::SettingTableScope& table) {
				    openvr_pair::overlay::ui::SettingRow(table, "Confidence-weighted calibration", [&] {
					    if (ExperimentCheckbox(
					            "confidence_weighted_calibration", "##head_mount_experimental_confidence_fusion",
					            &CalCtx.headMount.experimentalConfidenceFusion,
					            "Fuse each accepted continuous re-solve into a running estimate weighted by its "
					            "geometry confidence, instead of overwriting the calibration outright (classic "
					            "behaviour). Sample-level distance weighting is its own setting under Diagnostics. "
					            "Experimental. Default off.")) {
						    SaveProfile(CalCtx);
					    }
				    });
			    });
			ImGui::EndGroupPanel();
		}
	}

	// Maintenance buttons grouped in their own panel so they don't read as
	// floating buttons under the speed/threshold matrix.
	ImGui::Spacing();
	ImGui::BeginGroupPanel("Maintenance", panel_size);
	if (ImGui::Button("Reset settings")) {
		CalCtx.ResetConfig();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Reset all settings (jitter / speed / lock / etc.) to defaults.\n"
		                  "Does NOT clear your calibrated profile -- only the tunables.");
	}
	ImGui::SameLine();
	if (ImGui::Button("Run setup wizard")) {
		spacecal::wizard::Open();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Re-run the first-run setup wizard. Useful after changing your hardware\n"
		                  "(adding/removing a tracking system) or if you want to start fresh.");
	}
	ImGui::SameLine();
	if (ImGui::Button("Clear head-tracker offset")) {
		CalCtx.headMount.offsetCalibrated = false;
		CalCtx.headMount.offsetWitnessAutoCaptured = false;
		CalCtx.headMount.headFromTracker = Eigen::AffineCompact3d::Identity();
		CalCtx.NoteHeadMountOffsetChanged();
		SaveProfile(CalCtx);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Clear the stored headset<->head-tracker offset (manual or auto-captured).\n"
		                  "Use if the offset was set wrong; re-run the offset wizard to set a new one.");
	}
	ImGui::EndGroupPanel();

	// Section: Contributors credits
	{
		ImGui::BeginGroupPanel("Credits", panel_size);

		ImGui::TextDisabled("pushrax");
		ImGui::TextDisabled("hyblocker");
		ImGui::TextDisabled("tach");
		ImGui::TextDisabled("bd_");
		ImGui::TextDisabled("ArcticFox");
		ImGui::TextDisabled("hekky");
		ImGui::TextDisabled("pimaker");

		// Current maintainer rendered in a brighter accent + label so it
		// reads as the active fork owner rather than another past contributor.
		ImGui::TextColored(ImVec4(0.55f, 0.75f, 0.95f, 1.0f), "WhyKnot");
		ImGui::SameLine();
		ImGui::TextDisabled("- current maintainer");

		ImGui::EndGroupPanel();
	}
}
