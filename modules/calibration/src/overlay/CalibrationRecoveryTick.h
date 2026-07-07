#pragma once

#include "TrackerLiveness.h"
#include "WitnessHealth.h"

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

// Drop every sample collected before this instant from the primary and all
// additional calibration buffers. For disturbances that move the reference
// frame itself (a corroborated SLAM snap, an away gap long enough for the
// headset to re-anchor), pre-disturbance samples describe a dead coordinate
// frame: they cannot converge back and instead poison every solve until they
// age out of the window. Pair with ArmReanchorToProfile -- the re-applied
// profile carries tracking while the buffers refill. Returns samples dropped.
size_t EvictDeadFrameSamples(CalibrationContext& ctx, const char* reason);

// Witness-puck health rollup from the last relocalization-detector tick.
// Read by the [cal-heartbeat] emitter; updated inside TickHmdRelocalizationDetector.
const spacecal::witness_health::WitnessHealth& LastWitnessHealth();

extern spacecal::liveness::TrackerLivenessState g_refLiveness;
extern spacecal::liveness::TrackerLivenessState g_tgtLiveness;
extern bool g_refWasOffline;
extern bool g_tgtWasOffline;
