#pragma once

#include "TrackerLiveness.h"

#include <string>

void TickBaseStationDrift(double now);
void TickHmdRelocalizationDetector(double now);

void RecoverFromWedgedCalibration(const char* userFacingMessage, const char* recoverReason = "auto_recovery_snap");

struct CalibrationContext;

// Re-anchor to the saved profile: re-apply it on the next ScanAndApplyProfile
// cycle and open the warm-restart validation grace window. Shared by snap
// corroboration, relocalization re-anchor, and the warm-restart witness veto so
// all three drive the output identically (the apply switches from snap to a
// constant-velocity ramp in one place when that lands). Does NOT touch
// warmRestartReanchorCount -- callers manage retry accounting.
void ArmReanchorToProfile(CalibrationContext& ctx);

extern spacecal::liveness::TrackerLivenessState g_refLiveness;
extern spacecal::liveness::TrackerLivenessState g_tgtLiveness;
extern bool g_refWasOffline;
extern bool g_tgtWasOffline;
