#pragma once

#include "Calibration.h"
#include "CalibrationCalc.h"
#include "GeometryShiftDetector.h"
#include "IPCClient.h"
#include "Protocol.h"

extern CalibrationCalc calibration;
extern SCIPCClient Driver;
extern protocol::DriverPoseShmem shmem;

extern bool g_snapNextProfileApply;
extern bool g_reanchorNextProfileApply;

// Auto-lock MAD + geometry-shift sustain counters. Owned and written by the
// detector blocks in Calibration.cpp; read by the [cal-heartbeat] emitter in
// CalibrationWatchdogs.cpp so each heartbeat can report the latest values
// from the previous tick.
extern double g_lastAutoLockTranslMad;
extern double g_lastAutoLockRotMad;
extern int g_geomShiftConsecutiveBadTicks;

// Shared transform + speed helpers defined in Calibration.cpp; used by the
// head-mount shadow unit and the tick helpers.
Eigen::AffineCompact3d ProfileTransform(Eigen::Vector3d eulerDeg, Eigen::Vector3d transCm);
Eigen::Affine3d CalibrationTransformFromContext(const CalibrationContext& ctx);
double ComputeHmdSpeedMps(const CalibrationContext& ctx);

// Push the current CalCtx freeze-all-tracking state (freezeAllTracking +
// freezeIncludeHmd) to the driver. Called on toggle, from the ~1 Hz heartbeat
// while frozen, and on IPC reconnect so the driver's view stays in sync.
void SendFreezeAllTracking();

// Zero the geometry-shift watchdog accumulator (consecutive-spike count).
// The state is file-scope in Calibration.cpp; exported so
// ResetCustomCheckState can drop it when the enhanced-tracking master switch
// flips at runtime.
void ResetGeometryShiftDetectorState();
