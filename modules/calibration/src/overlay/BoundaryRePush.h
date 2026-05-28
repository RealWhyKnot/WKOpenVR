#pragma once

// Per-tick hook: if the safety boundary is enabled and the calibration
// transform has drifted enough since the last push, re-transform the stored
// lighthouse-space vertices into standing-universe coordinates and push them
// to SteamVR chaperone. Throttled to at most 1 Hz.
//
// Call once per CalibrationTick, AFTER the calibration transform is current
// (i.e. after ScanAndApplyProfile / ComputeIncremental have run). Accesses
// CalCtx directly.
//
// Also call ScheduleBoundaryStartupPush() once after profile load completes
// so a boundary that was enabled in the saved profile gets pushed after the
// calibration transform converges.

#include <Eigen/Geometry>

void TickBoundaryRePush(double time);
void ScheduleBoundaryStartupPush();
void NoteBoundaryPushedForTransform(const Eigen::AffineCompact3d& targetToStanding);
