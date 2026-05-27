#include "HeadMountOffsetModal.h"

#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "HeadFromTrackerSolve.h"
#include "HeadMountPreview.h"

#include <imgui/imgui.h>
#include <Eigen/Geometry>

#include <chrono>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <string>

// SaveProfile is defined in Calibration.cpp; expose via forward declaration
// matching the pattern used by Wizard.cpp.
void SaveProfile(CalibrationContext& ctx);

namespace wkopenvr::headmount {

// ---------------------------------------------------------------------------
// Module-level state
// ---------------------------------------------------------------------------

namespace {

struct ModalState {
    bool        wantOpen    = false;
    bool        popupOpened = false;
    Solver      solver;
    SolveResult lastResult;     // copy of solver.result() at the moment Finish ran
    bool        showResult  = false;  // true while awaiting Save/Discard after Done/Failed

    bool            hasFrozenFrame = false;
    Eigen::Affine3d targetFromReferenceAtStart = Eigen::Affine3d::Identity();
    bool            loggedMissingCommonFrame = false;

    bool            hasPreviousMotionPose = false;
    Eigen::Affine3d previousMotionPose = Eigen::Affine3d::Identity();
    double          previousMotionTime = 0.0;

    double          lastCollectingLogTime = 0.0;
    int             acceptedInWindow = 0;
    int             rejectedInWindow = 0;
    double          lastReportedSpeedMps = 0.0;
    double          lastDerivedLinearSpeedMps = 0.0;
    double          lastDerivedAngularSpeedRadps = 0.0;
    double          lastEffectiveSpeedMps = 0.0;
};

ModalState& MS() {
    static ModalState s;
    return s;
}

constexpr const char* kPopupId = "##hmt_offset_modal";
constexpr double kAngularMotionToLinearMps = 0.50;

double NowSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void ResetCollectionFrame(ModalState& s) {
    s.hasFrozenFrame = false;
    s.targetFromReferenceAtStart = Eigen::Affine3d::Identity();
    s.loggedMissingCommonFrame = false;
    s.hasPreviousMotionPose = false;
    s.previousMotionPose = Eigen::Affine3d::Identity();
    s.previousMotionTime = 0.0;
    s.lastCollectingLogTime = 0.0;
    s.acceptedInWindow = 0;
    s.rejectedInWindow = 0;
    s.lastReportedSpeedMps = 0.0;
    s.lastDerivedLinearSpeedMps = 0.0;
    s.lastDerivedAngularSpeedRadps = 0.0;
    s.lastEffectiveSpeedMps = 0.0;
}

void CancelSolverAndFrame(ModalState& s) {
    s.solver.Cancel();
    ResetCollectionFrame(s);
}

double RotationDeltaRad(const Eigen::Affine3d& previous,
                        const Eigen::Affine3d& current)
{
    const Eigen::Matrix3d delta =
        previous.rotation().transpose() * current.rotation();
    const double cosAngle = std::clamp((delta.trace() - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosAngle);
}

void LogCollectingStatus(ModalState& s, double now) {
    if (s.lastCollectingLogTime != 0.0
        && now - s.lastCollectingLogTime < 1.0) {
        return;
    }

    const CollectionReadiness ready = s.solver.readiness();
    char cbuf[640];
    std::snprintf(cbuf, sizeof cbuf,
        "[head-mount-modal] collecting:"
        " frame_frozen=%d samples=%zu ready=%d overall=%.2f sample=%.2f motion=%.2f"
        " consistency=%.2f residual_mm=%.2f axes_deg=(%.1f,%.1f,%.1f)"
        " accepted=%d rejected=%d speed_reported=%.3f speed_derived=%.3f"
        " angular_radps=%.3f speed_effective=%.3f",
        s.hasFrozenFrame ? 1 : 0,
        ready.samplesUsed,
        ready.ready ? 1 : 0,
        ready.overallScore,
        ready.sampleScore,
        ready.motionScore,
        ready.residualScore,
        ready.residualMm,
        ready.axisRangeDeg[0],
        ready.axisRangeDeg[1],
        ready.axisRangeDeg[2],
        s.acceptedInWindow,
        s.rejectedInWindow,
        s.lastReportedSpeedMps,
        s.lastDerivedLinearSpeedMps,
        s.lastDerivedAngularSpeedRadps,
        s.lastEffectiveSpeedMps);
    Metrics::WriteLogAnnotation(cbuf);

    s.lastCollectingLogTime = now;
    s.acceptedInWindow = 0;
    s.rejectedInWindow = 0;
}

} // namespace

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void OpenOffsetModal() {
    auto& s = MS();
    // Reset state but keep the solver clean until the user hits Start.
    CancelSolverAndFrame(s);
    s.showResult  = false;
    s.popupOpened = false;
    s.wantOpen    = true;
}

bool OffsetModalIsOpen() {
    return MS().popupOpened || MS().wantOpen;
}

bool DrawOffsetModal() {
    auto& s = MS();

    if (s.wantOpen && !s.popupOpened) {
        ImGui::OpenPopup(kPopupId);
        s.popupOpened = true;
        s.wantOpen    = false;
    }

    if (!s.popupOpened) return false;

    ImVec2 vpSize = ImGui::GetMainViewport()->Size;
    ImGui::SetNextWindowPos(
        ImVec2((vpSize.x - 480.0f) * 0.5f, (vpSize.y - 340.0f) * 0.5f),
        ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(480.0f, 340.0f), ImGuiCond_Always);

    bool savedOffset = false;

    if (ImGui::BeginPopupModal(kPopupId, nullptr,
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize   |
            ImGuiWindowFlags_NoMove))
    {
        ImGui::TextUnformatted("Head-Tracker Offset Calibration");
        ImGui::Separator();
        ImGui::Spacing();

        const SolveState st = s.solver.state();

        if (!s.showResult) {
            // --- Idle / collecting phase ---
            if (st == SolveState::Idle) {
                ImGui::TextWrapped(
                    "Attach the tracker firmly to your headset. "
                    "Click Start, then move your head slowly through "
                    "pitch, yaw, and roll for ~10 seconds.");
                ImGui::Spacing();
                if (ImGui::Button("Start##hmt_start")) {
                    ResetCollectionFrame(s);
                    s.solver.Start();
                    Metrics::WriteLogAnnotation("[head-mount-modal] solver started");
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel##hmt_cancel_idle")) {
                    ImGui::CloseCurrentPopup();
                    s.popupOpened = false;
                }
            }
            else if (st == SolveState::Collecting) {
                const size_t collected = s.solver.sampleCount();
                const CollectionReadiness ready = s.solver.readiness();
                const float  fraction  = static_cast<float>(ready.overallScore);
                char progLabel[64];
                std::snprintf(progLabel, sizeof progLabel,
                    "Readiness %d%%", static_cast<int>(ready.overallScore * 100.0));
                ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f), progLabel);
                ImGui::Spacing();

                auto ReadinessBar = [](const char* label, double score) {
                    char text[48];
                    std::snprintf(text, sizeof text, "%s %d%%",
                        label, static_cast<int>(score * 100.0));
                    ImGui::ProgressBar(static_cast<float>(score),
                        ImVec2(-1.0f, 0.0f), text);
                };
                ReadinessBar("Pitch", ready.axisScore[0]);
                ReadinessBar("Yaw", ready.axisScore[1]);
                ReadinessBar("Roll", ready.axisScore[2]);
                ReadinessBar("Consistency", ready.residualScore);
                ImGui::Spacing();

                char statusBuf[160];
                std::snprintf(statusBuf, sizeof statusBuf,
                    "%zu samples, residual %.2f mm, frame %s",
                    collected, ready.residualMm,
                    s.hasFrozenFrame ? "locked" : "waiting");
                ImGui::TextDisabled("%s", statusBuf);
                if (!s.hasFrozenFrame) {
                    ImGui::TextDisabled("Waiting for a valid calibration profile.");
                }
                else if (collected >= 2 && !ready.residualGood) {
                    ImGui::TextDisabled(
                        "Consistency needs residual <= %.1f mm.",
                        Solver::kResidualThresholdMm);
                }
                ImGui::Spacing();

                bool canFinish = ready.ready;
                if (!canFinish) ImGui::BeginDisabled();
                if (ImGui::Button("Finish##hmt_finish")) {
                    s.solver.Finish();
                    s.lastResult  = s.solver.result();
                    s.showResult  = true;
                    // Push solve residual so the diagnostics graph shows per-solve
                    // accuracy. Only push on success; failure residual is noisy.
                    if (s.lastResult.failReason.empty()) {
                        Metrics::headMountResidualMm.Push(s.lastResult.residualMm);
                        char rbuf[128];
                        snprintf(rbuf, sizeof rbuf,
                            "[head-mount-modal] solved: residual=%.2fmm samples=%d",
                            s.lastResult.residualMm, s.lastResult.samplesUsed);
                        Metrics::WriteLogAnnotation(rbuf);
                    } else {
                        char rbuf[256];
                        snprintf(rbuf, sizeof rbuf,
                            "[head-mount-modal] failed: reason='%s' residual=%.2fmm samples=%d",
                            s.lastResult.failReason.c_str(),
                            s.lastResult.residualMm, s.lastResult.samplesUsed);
                        Metrics::WriteLogAnnotation(rbuf);
                    }
                    {
                        char fbuf[96];
                        snprintf(fbuf, sizeof fbuf,
                            "[head-mount-modal] finish requested: samples=%zu readiness=%.2f",
                            collected, ready.overallScore);
                        Metrics::WriteLogAnnotation(fbuf);
                    }
                }
                if (!canFinish) ImGui::EndDisabled();
                ImGui::SameLine();
                if (ImGui::Button("Cancel##hmt_cancel_coll")) {
                    CancelSolverAndFrame(s);
                    ImGui::CloseCurrentPopup();
                    s.popupOpened = false;
                }
            }
        }
        else {
            // --- Result phase (Done or Failed) ---
            const bool success = (s.lastResult.failReason.empty());
            if (success) {
                char buf[128];
                std::snprintf(buf, sizeof buf,
                    "Solved: residual %.2f mm  (%d samples)",
                    s.lastResult.residualMm,
                    s.lastResult.samplesUsed);
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
                ImGui::TextUnformatted(buf);
                ImGui::PopStyleColor();
                ImGui::Spacing();
                if (ImGui::Button("Save##hmt_save")) {
                    CalCtx.headMount.headFromTracker = s.lastResult.headFromTracker;
                    CalCtx.headMount.offsetCalibrated = true;
                    SaveProfile(CalCtx);
                    {
                        const Eigen::Vector3d tcm = s.lastResult.headFromTracker.translation() * 100.0;
                        const Eigen::Vector3d rpy = s.lastResult.headFromTracker.rotation()
                            .eulerAngles(2, 1, 0) * (180.0 / EIGEN_PI);
                        char sbuf[192];
                        snprintf(sbuf, sizeof sbuf,
                            "[head-mount-modal] offset saved:"
                            " trans=(%.2f,%.2f,%.2f)cm rpy=(%.2f,%.2f,%.2f)deg",
                            tcm.x(), tcm.y(), tcm.z(),
                            rpy(0), rpy(1), rpy(2));
                        Metrics::WriteLogAnnotation(sbuf);
                    }
                    savedOffset   = true;
                    s.showResult  = false;
                    s.solver.Cancel();
                    ImGui::CloseCurrentPopup();
                    s.popupOpened = false;
                }
                ImGui::SameLine();
            }
            else {
                char buf[256];
                std::snprintf(buf, sizeof buf,
                    "Solve failed: %s", s.lastResult.failReason.c_str());
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
                ImGui::TextUnformatted(buf);
                ImGui::PopStyleColor();
                ImGui::Spacing();
                // Let the user retry from scratch.
                if (ImGui::Button("Retry##hmt_retry")) {
                    CancelSolverAndFrame(s);
                    s.showResult = false;
                }
                ImGui::SameLine();
            }

            if (ImGui::Button("Discard##hmt_discard")) {
                s.showResult  = false;
                CancelSolverAndFrame(s);
                ImGui::CloseCurrentPopup();
                s.popupOpened = false;
            }
        }

        ImGui::EndPopup();
    }
    else {
        // ImGui auto-closed (Esc or outside click) -- sync state.
        if (s.popupOpened) {
            s.popupOpened = false;
            s.showResult  = false;
            CancelSolverAndFrame(s);
        }
    }

    // Drive the live preview marker while the modal is open.
    {
        const bool modalOpen = s.popupOpened;
        const auto& hm = CalCtx.headMount;
        bool trackerOk = hm.deviceID >= 0
            && (uint32_t)hm.deviceID < vr::k_unMaxTrackedDeviceCount
            && CalCtx.devicePoses[hm.deviceID].poseIsValid;

        if (modalOpen && trackerOk) {
            const vr::DriverPose_t& raw = CalCtx.devicePoses[hm.deviceID];
            // Build world pose from DriverPose fields.
            Eigen::Quaterniond wfd(
                raw.qWorldFromDriverRotation.w,
                raw.qWorldFromDriverRotation.x,
                raw.qWorldFromDriverRotation.y,
                raw.qWorldFromDriverRotation.z);
            Eigen::Quaterniond localRot(
                raw.qRotation.w, raw.qRotation.x,
                raw.qRotation.y, raw.qRotation.z);
            Eigen::Quaterniond worldRot = (wfd * localRot).normalized();
            Eigen::Vector3d wfdTrans(
                raw.vecWorldFromDriverTranslation[0],
                raw.vecWorldFromDriverTranslation[1],
                raw.vecWorldFromDriverTranslation[2]);
            Eigen::Vector3d localPos(
                raw.vecPosition[0], raw.vecPosition[1], raw.vecPosition[2]);
            Eigen::Affine3d trackerWorld =
                Eigen::Translation3d(wfdTrans + wfd * localPos) * worldRot;

            TickPreview(true, trackerWorld, hm.headFromTracker);
        }
        else {
            TickPreview(false, Eigen::Affine3d::Identity(),
                Eigen::AffineCompact3d::Identity());
        }
    }

    return savedOffset;
}

// ---------------------------------------------------------------------------
// Inline nudge-slider panel
// ---------------------------------------------------------------------------

void DrawOffsetInlinePanel() {
    auto& hm = CalCtx.headMount;

    // Decompose headFromTracker into XYZ (cm) + RPY (deg) for display.
    // The sliders write back into the transform each frame.
    Eigen::Vector3d t = hm.headFromTracker.translation() * 100.0; // m -> cm
    Eigen::Vector3d rpy = hm.headFromTracker.rotation()
        .eulerAngles(2, 1, 0) * (180.0 / EIGEN_PI);              // ZYX -> yaw/pitch/roll in deg

    bool changed = false;

    auto SliderCm = [&](const char* label, double& v) -> bool {
        float fv = static_cast<float>(v);
        if (ImGui::SliderFloat(label, &fv, -5.0f, 5.0f, "%.2f cm",
                ImGuiSliderFlags_AlwaysClamp)) {
            v = static_cast<double>(fv);
            return true;
        }
        return false;
    };
    auto SliderDeg = [&](const char* label, double& v) -> bool {
        float fv = static_cast<float>(v);
        if (ImGui::SliderFloat(label, &fv, -15.0f, 15.0f, "%.2f deg",
                ImGuiSliderFlags_AlwaysClamp)) {
            v = static_cast<double>(fv);
            return true;
        }
        return false;
    };

    if (hm.offsetCalibrated) {
        char buf[64];
        std::snprintf(buf, sizeof buf, "Residual from last solve: calibrated");
        ImGui::TextUnformatted(buf);
    }
    else {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.3f, 1.0f));
        ImGui::TextUnformatted("Offset not calibrated -- use the Calibrate button above.");
        ImGui::PopStyleColor();
    }

    ImGui::Spacing();
    ImGui::TextUnformatted("Fine adjustment (XYZ offset, RPY rotation):");
    ImGui::Spacing();

    changed |= SliderCm("X##hft_x", t(0));
    changed |= SliderCm("Y##hft_y", t(1));
    changed |= SliderCm("Z##hft_z", t(2));
    changed |= SliderDeg("Yaw##hft_yaw",   rpy(0));
    changed |= SliderDeg("Pitch##hft_pit", rpy(1));
    changed |= SliderDeg("Roll##hft_rol",  rpy(2));

    if (changed) {
        // Recompose. ZYX order matches eulerAngles(2,1,0) decomposition above.
        Eigen::Quaterniond q =
            Eigen::AngleAxisd(rpy(0) * EIGEN_PI / 180.0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy(1) * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy(2) * EIGEN_PI / 180.0, Eigen::Vector3d::UnitX());
        hm.headFromTracker.linear()      = q.toRotationMatrix();
        hm.headFromTracker.translation() = t * 0.01; // cm -> m

        // Drive preview while sliders are active.
        bool trackerOk = hm.deviceID >= 0
            && (uint32_t)hm.deviceID < vr::k_unMaxTrackedDeviceCount
            && CalCtx.devicePoses[hm.deviceID].poseIsValid;
        if (trackerOk) {
            const vr::DriverPose_t& raw = CalCtx.devicePoses[hm.deviceID];
            Eigen::Quaterniond wfd(
                raw.qWorldFromDriverRotation.w,
                raw.qWorldFromDriverRotation.x,
                raw.qWorldFromDriverRotation.y,
                raw.qWorldFromDriverRotation.z);
            Eigen::Quaterniond localRot(
                raw.qRotation.w, raw.qRotation.x,
                raw.qRotation.y, raw.qRotation.z);
            Eigen::Affine3d trackerWorld =
                Eigen::Translation3d(
                    Eigen::Vector3d(
                        raw.vecWorldFromDriverTranslation[0],
                        raw.vecWorldFromDriverTranslation[1],
                        raw.vecWorldFromDriverTranslation[2]) +
                    wfd *
                    Eigen::Vector3d(
                        raw.vecPosition[0],
                        raw.vecPosition[1],
                        raw.vecPosition[2])) *
                (wfd * localRot).normalized();
            TickPreview(true, trackerWorld, hm.headFromTracker);
        }
    }
}

// Called from CalibrationTick to feed the modal's Solver after building
// HMD reference-frame and tracker target-frame poses from DriverPose_t fields.
void FeedSolverTick(const Eigen::Affine3d& hmdPose,
                    const Eigen::Affine3d& trackerPose,
                    const Eigen::Affine3d& targetFromReference,
                    bool targetFromReferenceValid,
                    double hmdSpeedMps)
{
    auto& s = MS();
    if (s.solver.state() != SolveState::Collecting) return;

    const double now = NowSeconds();

    if (!targetFromReferenceValid || !targetFromReference.matrix().allFinite()) {
        if (!s.loggedMissingCommonFrame) {
            s.loggedMissingCommonFrame = true;
            Metrics::WriteLogAnnotation(
                "[head-mount-modal] collecting blocked: no valid calibration profile for common-frame conversion");
        }
        s.rejectedInWindow++;
        LogCollectingStatus(s, now);
        return;
    }

    if (!s.hasFrozenFrame) {
        s.targetFromReferenceAtStart = targetFromReference;
        s.hasFrozenFrame = true;
        s.hasPreviousMotionPose = false;

        const Eigen::Vector3d tcm =
            s.targetFromReferenceAtStart.translation() * 100.0;
        const Eigen::Vector3d rpy =
            s.targetFromReferenceAtStart.rotation().eulerAngles(2, 1, 0)
            * (180.0 / EIGEN_PI);
        char fbuf[240];
        std::snprintf(fbuf, sizeof fbuf,
            "[head-mount-modal] common frame frozen:"
            " target_from_reference_trans_cm=(%.2f,%.2f,%.2f)"
            " rpy_deg=(%.2f,%.2f,%.2f)",
            tcm.x(), tcm.y(), tcm.z(),
            rpy(0), rpy(1), rpy(2));
        Metrics::WriteLogAnnotation(fbuf);
    }

    const Eigen::Affine3d hmdTargetPose =
        s.targetFromReferenceAtStart * hmdPose;

    double derivedLinearSpeedMps = 0.0;
    double derivedAngularSpeedRadps = 0.0;
    if (s.hasPreviousMotionPose && now > s.previousMotionTime) {
        const double dt = now - s.previousMotionTime;
        derivedLinearSpeedMps =
            (hmdTargetPose.translation()
                - s.previousMotionPose.translation()).norm() / dt;
        derivedAngularSpeedRadps =
            RotationDeltaRad(s.previousMotionPose, hmdTargetPose) / dt;
    }
    s.previousMotionPose = hmdTargetPose;
    s.previousMotionTime = now;
    s.hasPreviousMotionPose = true;

    const double reportedSpeed =
        std::isfinite(hmdSpeedMps) && hmdSpeedMps > 0.0 ? hmdSpeedMps : 0.0;
    const double angularEquivalentSpeed =
        derivedAngularSpeedRadps * kAngularMotionToLinearMps;
    const double effectiveSpeed = std::max(
        reportedSpeed,
        std::max(derivedLinearSpeedMps, angularEquivalentSpeed));

    s.lastReportedSpeedMps = reportedSpeed;
    s.lastDerivedLinearSpeedMps = derivedLinearSpeedMps;
    s.lastDerivedAngularSpeedRadps = derivedAngularSpeedRadps;
    s.lastEffectiveSpeedMps = effectiveSpeed;

    const bool accepted = s.solver.Tick(hmdTargetPose, trackerPose, effectiveSpeed);
    if (accepted) {
        s.acceptedInWindow++;
    } else {
        s.rejectedInWindow++;
    }
    LogCollectingStatus(s, now);
}

} // namespace wkopenvr::headmount
