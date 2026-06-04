#pragma once

// Head-mount offset calibration modal and inline nudge-slider panel.
//
// DrawOffsetModal() renders the solver flow as an ImGui modal popup.
// OpenOffsetModal() signals that the modal should open on the next Draw call.
// DrawOffsetInlinePanel() renders the XYZ/RPY nudge sliders plus a live
// residual readout, inline in the head-mount group panel (Task 10).
// FeedSolverTick() routes live pose pairs into the modal's Solver; called from
// the calibration tick whenever the modal is in Collecting state.

#include <Eigen/Geometry>

namespace wkopenvr::headmount {

// Open the modal. Safe to call from within an ImGui frame; the actual
// OpenPopup fires from inside DrawOffsetModal() on the next iteration.
void OpenOffsetModal();

// Must be called every frame from the main UI render function (after all
// tab-content drawing). Returns true if a new offset was saved this frame.
bool DrawOffsetModal();

// Renders an inline status line and 6 nudge sliders (XYZ cm, RPY deg)
// bound to CalCtx.headMount.headFromTracker. Caller is the head-mount
// group panel in the Basic tab.
void DrawOffsetInlinePanel();

// Route live pose pairs into the Solver while the modal is in Collecting state.
// Called from CalibrationTick (or the head-mount pose-sampling path) each tick
// that the head-mount tracker is valid. The HMD pose arrives in the reference
// tracking frame and the tracker pose arrives in the target frame; the modal
// freezes targetFromReference for the collection so continuous calibration
// updates cannot mix coordinate frames inside one solve.
void FeedSolverTick(const Eigen::Affine3d& hmdReferencePose, const Eigen::Affine3d& trackerTargetPose,
                    const Eigen::Affine3d& targetFromReference, bool targetFromReferenceValid, double hmdSpeedMps);

// Returns true while the offset calibration popup is open (any phase: Idle,
// Collecting, or showing results). Used by the CalibrationTick diagnostic path
// to detect the "modal open but tracker not resolved" condition.
bool OffsetModalIsOpen();

} // namespace wkopenvr::headmount
