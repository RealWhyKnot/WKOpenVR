#pragma once

#include "Calibration.h"
#include "CalibrationCalc.h"
#include "IPCClient.h"
#include "Protocol.h"

extern CalibrationCalc calibration;
extern SCIPCClient Driver;
extern protocol::DriverPoseShmem shmem;

extern bool g_snapNextProfileApply;
extern bool g_reanchorNextProfileApply;

// Auto-lock MAD counters. Owned and written by the detector block in
// Calibration.cpp; read by the [cal-heartbeat] emitter in
// CalibrationWatchdogs.cpp so each heartbeat can report the latest values
// from the previous tick.
extern double g_lastAutoLockTranslMad;
extern double g_lastAutoLockRotMad;

// Profile euler (degrees, Z/Y/X compose order) + translation (cm) -> affine
// transform (metres). Header-inline so overlay TUs and the test binary share
// one definition (ProfileJson.cpp needs it in both).
inline Eigen::AffineCompact3d ProfileTransform(Eigen::Vector3d eulerDeg, Eigen::Vector3d transCm)
{
	auto euler = eulerDeg * EIGEN_PI / 180.0;
	Eigen::Quaterniond rotQuat = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
	                             Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
	                             Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

	Eigen::AffineCompact3d transform = Eigen::AffineCompact3d::Identity();
	transform.linear() = rotQuat.toRotationMatrix();
	transform.translation() = transCm * 0.01;
	return transform;
}

// Shared transform + speed helpers defined in Calibration.cpp; used by the
// head-mount shadow unit and the tick helpers.
Eigen::Affine3d CalibrationTransformFromContext(const CalibrationContext& ctx);
double ComputeHmdSpeedMps(const CalibrationContext& ctx);

// Push the current CalCtx freeze-all-tracking state (freezeAllTracking +
// freezeIncludeHmd) to the driver. Called on toggle, from the ~1 Hz heartbeat
// while frozen, and on IPC reconnect so the driver's view stays in sync.
void SendFreezeAllTracking();

// Drop every piece of rolling state behind the enhanced-tracking master
// switch. Called when the switch flips at runtime (UI toggle, profile load)
// so no armed check fires after opt-out and no stale window influences the
// first checks after re-enable.
void ResetCustomCheckState(CalibrationContext& ctx);
