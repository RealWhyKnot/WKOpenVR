#pragma once

#include "Calibration.h"

// Cross-tick watchdogs and log emitters that run at the top of every
// CalibrationTick pass. Each keeps its own cross-tick state as
// function-local statics, so the only inputs are the calibration context
// and the tick clock.
void TickWarmRestartDetection(CalibrationContext& ctx, double time);
void TraceRelPoseCalFlips(CalibrationContext& ctx);
void TickPoseFreshnessWatchdog(CalibrationContext& ctx, double time);
void TickStuckCalWatchdog(CalibrationContext& ctx, double time);
void EmitCalHeartbeat(CalibrationContext& ctx, double time);
void EmitSessionConfigDumpOnce(CalibrationContext& ctx);
