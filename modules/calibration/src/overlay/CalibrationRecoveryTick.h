#pragma once

#include "TrackerLiveness.h"

#include <string>

void TickBaseStationDrift(double now);
void TickHmdRelocalizationDetector(double now);

void RecoverFromWedgedCalibration(const char* userFacingMessage, const char* recoverReason = "auto_recovery_snap");

extern spacecal::liveness::TrackerLivenessState g_refLiveness;
extern spacecal::liveness::TrackerLivenessState g_tgtLiveness;
extern bool g_refWasOffline;
extern bool g_tgtWasOffline;
