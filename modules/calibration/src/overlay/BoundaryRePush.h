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

namespace boundary_repush {

// One step of the startup-push countdown, factored out for testing.
//
// The boundary is pushed once -- after `pending` counts down to 0 -- but only
// while it is actually pushable (enabled, has vertices, valid calibration). The
// critical property: while NOT pushable the countdown is HELD, not consumed.
// Consuming it while disabled / before the profile turned valid was the bug
// that let the one-shot push be swallowed before the boundary was ready, so the
// chaperone ended up pushed with a stale (identity) transform.
//
// Returns the new pending count and whether the push should fire this tick
// (fire can only be true when pushable).
struct StartupCountdown
{
	int pending = 0;
	bool fire = false;
};

inline StartupCountdown StepStartupCountdown(bool pushable, int pending)
{
	if (pending <= 0) return {0, false};
	if (!pushable) return {pending, false}; // hold until pushable
	const int next = pending - 1;
	return {next, next == 0};
}

} // namespace boundary_repush

void TickBoundaryRePush(double time);
void ScheduleBoundaryStartupPush();
void NoteBoundaryPushedForTransform(const Eigen::AffineCompact3d& targetToStanding);
