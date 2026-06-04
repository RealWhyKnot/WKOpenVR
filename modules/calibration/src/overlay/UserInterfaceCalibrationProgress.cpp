#include "UserInterfaceCalibrationProgress.h"

#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "CalibrationProgress.h"
#include "imgui_extensions.h"

#include <Eigen/Core>

#include <cstdio>

namespace spacecal::ui {
namespace {

bool IsOneShotCollecting()
{
	return CalCtx.state == CalibrationState::Begin || CalCtx.state == CalibrationState::Rotation ||
	       CalCtx.state == CalibrationState::Translation;
}

void DrawMessage(const CalibrationContext::Message& message, bool isCollecting)
{
	switch (message.type) {
		case CalibrationContext::Message::String:
			ImGui::TextWrapped("%s", message.str.c_str());
			break;
		case CalibrationContext::Message::Progress: {
			float fraction = 0.0f;
			if (isCollecting) {
				fraction = (float)spacecal::calibration_progress::OneShotReadyScore(
				    CalCtx.state, message.progress, message.target, Metrics::rotationDiversity.last(),
				    Metrics::translationDiversity.last());
			}
			else {
				fraction = (float)spacecal::calibration_progress::SampleFillScore(message.progress, message.target);
			}
			char readyLabel[32];
			snprintf(readyLabel, sizeof readyLabel, "Ready %d%%", (int)(fraction * 100.0f));
			ImGui::Text("");
			ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f), readyLabel);
			break;
		}
	}
}

void DrawPairedMotionWarning()
{
	const int pairedMotionWarn = (int)Metrics::pairedMotionWarningCount.last();
	if (pairedMotionWarn < 5) return;

	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.55f, 0.30f, 1.0f));
	ImGui::TextWrapped("Reference and target aren't moving together. If you're in passthrough or a desktop overlay, "
	                   "your headset pose may be frozen -- exit to VR view so the headset reports real motion.");
	ImGui::PopStyleColor();
	ImGui::Spacing();
}

void DrawPhaseBanner()
{
	if (CalCtx.state == CalibrationState::Rotation) {
		ImGui::TextDisabled("Phase 1 of 2: Rotation");
		ImGui::TextWrapped("Rotate the tracker through different orientations (>= 90 deg between some pair).");
	}
	else if (CalCtx.state == CalibrationState::Translation) {
		ImGui::TextDisabled("Phase 2 of 2: Translation");
		ImGui::TextWrapped("Wave the tracker through ~15 cm on each of left/right, up/down, and forward/back.");
	}
	else {
		ImGui::TextDisabled("Motion coverage");
	}
	ImGui::Spacing();
}

void DrawTranslationTooltip()
{
	const Eigen::Vector3d& r = Metrics::translationAxisRangesCm.last();
	int minIdx = 0;
	double minR = r(0);
	for (int i = 1; i < 3; ++i) {
		if (r(i) < minR) {
			minR = r(i);
			minIdx = i;
		}
	}
	static const char* kAxisName[] = {"X (left/right)", "Y (up/down)", "Z (forward/back)"};
	ImGui::SetTooltip("Translation coverage: how much you've moved the tracker along all three axes.\n"
	                  "Wave it ~15 cm in each of left/right, up/down, and forward/back to fill this bar.\n"
	                  "Green = enough variety for a clean calibration.\n"
	                  "\n"
	                  "Current ranges: X=%.0f cm, Y=%.0f cm, Z=%.0f cm\n"
	                  "Weakest axis: %s",
	                  r(0), r(1), r(2), kAxisName[minIdx]);
}

void DrawCoverageBars()
{
	const float trDiv = (float)Metrics::translationDiversity.last();
	const float rotDiv =
	    (CalCtx.state == CalibrationState::Translation) ? 1.0f : (float)Metrics::rotationDiversity.last();

	char trLabel[64], rotLabel[64];
	snprintf(trLabel, sizeof trLabel, "Translation %d%%", (int)(trDiv * 100.0f));
	snprintf(rotLabel, sizeof rotLabel, "Rotation %d%%", (int)(rotDiv * 100.0f));

	constexpr float kTrGoodThreshold = (float)spacecal::calibration_progress::kOneShotTranslationReadyDiversity;
	constexpr float kRotGoodThreshold = (float)spacecal::calibration_progress::kOneShotRotationReadyDiversity;
	const ImVec4 trColor =
	    trDiv >= kTrGoodThreshold ? ImVec4(0.40f, 0.85f, 0.40f, 1.0f) : ImVec4(0.95f, 0.70f, 0.20f, 1.0f);
	const ImVec4 rotColor =
	    rotDiv >= kRotGoodThreshold ? ImVec4(0.40f, 0.85f, 0.40f, 1.0f) : ImVec4(0.95f, 0.70f, 0.20f, 1.0f);

	ImGui::PushStyleColor(ImGuiCol_PlotHistogram, trColor);
	ImGui::ProgressBar(trDiv, ImVec2(-1.0f, 0.0f), trLabel);
	ImGui::PopStyleColor();
	if (ImGui::IsItemHovered()) {
		DrawTranslationTooltip();
	}

	ImGui::PushStyleColor(ImGuiCol_PlotHistogram, rotColor);
	ImGui::ProgressBar(rotDiv, ImVec2(-1.0f, 0.0f), rotLabel);
	ImGui::PopStyleColor();
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Rotation coverage: the widest angle between any two sampled tracker rotations.\n"
		                  "Twist the tracker through ~90 degrees at some point to fill this bar.\n"
		                  "Green = enough variety for a clean calibration.");
	}

	if (trDiv < kTrGoodThreshold || rotDiv < kRotGoodThreshold) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		if (trDiv < rotDiv) {
			ImGui::TextWrapped("Tip: try moving the tracker through wider distances on every axis.");
		}
		else {
			ImGui::TextWrapped("Tip: try rotating the tracker more (point it in different directions).");
		}
		ImGui::PopStyleColor();
	}
}

void DrawOneShotCoverage()
{
	ImGui::Spacing();
	ImGui::Separator();
	DrawPhaseBanner();
	DrawPairedMotionWarning();
	DrawCoverageBars();

	ImGui::Spacing();
	if (ImGui::Button("Cancel calibration", ImVec2(-1.0f, ImGui::GetTextLineHeight() * 1.6f))) {
		CancelCalibration("ui_oneshot_progress");
		ImGui::CloseCurrentPopup();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Stop this calibration run and discard the samples collected so far.");
	}
}

} // namespace

void DrawCalibrationProgressPopup(const ImVec2& displaySize, ImGuiWindowFlags windowFlags)
{
	ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(ImVec2(displaySize.x - 40.0f, displaySize.y - 40.0f), ImGuiCond_Always);
	if (!ImGui::BeginPopupModal("Calibration Progress", nullptr, windowFlags)) {
		return;
	}

	ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImVec4(0, 0, 0, 1));
	const bool isCollecting = IsOneShotCollecting();
	for (const auto& message : CalCtx.messages) {
		DrawMessage(message, isCollecting);
	}
	ImGui::PopStyleColor();

	if (isCollecting) {
		DrawOneShotCoverage();
	}

	if (CalCtx.state == CalibrationState::None) {
		ImGui::Text("");
		if (ImGui::Button("Close", ImVec2(ImGui::GetWindowContentRegionWidth(), ImGui::GetTextLineHeight() * 2)))
			ImGui::CloseCurrentPopup();
	}

	ImGui::EndPopup();
}

} // namespace spacecal::ui
