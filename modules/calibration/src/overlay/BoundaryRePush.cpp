#include "BoundaryRePush.h"

#include "Boundary.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

namespace {

// Wall-clock instant (steady_clock) of the last successful chaperone push.
// Zero until the first push fires.
static std::chrono::steady_clock::time_point g_lastPushAt;
static bool g_firstPush = true;

// Transform in effect when we last pushed. Used to skip re-pushes when the
// calibration hasn't moved enough to matter.
static Eigen::AffineCompact3d g_lastPushedTransform = Eigen::AffineCompact3d::Identity();

// Ticks elapsed since profile load. Used by the startup-push gate: we wait
// for kStartupGraceTicks continuous ticks before pushing so the transform has
// had a chance to converge from an empty sample buffer.
static int g_startupPushPending = 0;  // 0 = no pending push; >0 = countdown

// Minimum translation change (metres) between pushes to qualify as "meaningful".
static constexpr double kTranslationThresholdM = 0.005;  // 5 mm

// Minimum rotation change (radians) between pushes to qualify as "meaningful".
static constexpr double kRotationThresholdRad = 0.005236;  // ~0.3 deg

// Minimum time between pushes (seconds).
static constexpr double kMinPushIntervalSec = 1.0;

// Number of CalibrationTicks to skip after a profile load before pushing the
// startup boundary. At ~20 Hz this is ~1.5 s -- long enough for the first
// ComputeIncremental cycle to start filling the sample buffer and produce a
// reasonable transform estimate.
static constexpr int kStartupGraceTicks = 30;

// ---------------------------------------------------------------------------
// Build the SE(3) lighthouse-to-standing transform from CalCtx fields.
// ---------------------------------------------------------------------------

static Eigen::AffineCompact3d BuildCalibrationTransform(const CalibrationContext& ctx) {
    // calibratedRotation is degrees ZYX Euler (same convention as
    // CalibrationProfileApply.cpp lines 262-266).
    // calibratedTranslation is in centimetres; convert to metres.
    const Eigen::Vector3d euler = ctx.calibratedRotation * EIGEN_PI / 180.0;
    const Eigen::Quaterniond rot =
        Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
    const Eigen::Vector3d trans = ctx.calibratedTranslation * 0.01;

    Eigen::AffineCompact3d xf;
    xf.linear() = rot.toRotationMatrix();
    xf.translation() = trans;
    return xf;
}

// ---------------------------------------------------------------------------
// Determine whether the transform changed enough to warrant a re-push.
// ---------------------------------------------------------------------------

static bool TransformDeltaExceedsThreshold(const Eigen::AffineCompact3d& current,
                                            const Eigen::AffineCompact3d& last) {
    const double transDelta = (current.translation() - last.translation()).norm();
    if (transDelta >= kTranslationThresholdM) return true;

    const Eigen::Matrix3d dR = current.rotation() * last.rotation().transpose();
    const double trace = dR.trace();
    // clamp against floating-point drift outside [-1, 3]
    const double cosAngle = std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5));
    const double angle = std::acos(cosAngle);
    return angle >= kRotationThresholdRad;
}

// ---------------------------------------------------------------------------
// Core push: snapshot (once), transform vertices, call PushToChaperone.
// ---------------------------------------------------------------------------

static void DoPush(const Eigen::AffineCompact3d& xf) {
    auto& ctx = CalCtx;
    auto& bc = ctx.boundary;

    if (bc.vertices.empty()) return;

    // Snapshot the pre-existing chaperone exactly once, before our first push.
    if (!bc.priorChaperoneCaptured) {
        auto snap = wkopenvr::boundary::SnapshotCurrentChaperone();
        if (!snap.empty()) {
            bc.priorChaperone = std::move(snap);
            bc.priorChaperoneCaptured = true;
        }
        // Even if snapshot fails (SteamVR unavailable), still proceed with push.
    }

    auto standing = wkopenvr::boundary::TransformToStandingUniverse(bc.vertices, xf);
    const double standingFloorY =
        wkopenvr::boundary::TransformHeightToStandingUniverse(bc.vertices, bc.floorY, xf);
    const double standingCeilingY =
        wkopenvr::boundary::TransformHeightToStandingUniverse(bc.vertices, bc.ceilingY, xf);
    const bool ok = wkopenvr::boundary::PushToChaperone(
        standing,
        standingFloorY,
        standingCeilingY);
    if (ok) {
        if (g_firstPush) {
            // Initial push after profile load.
            char lbuf[80];
            snprintf(lbuf, sizeof lbuf,
                "[boundary-re-push] initial push: vertices=%zu",
                bc.vertices.size());
            Metrics::WriteLogAnnotation(lbuf);
        }
        g_lastPushedTransform = xf;
        g_lastPushAt = std::chrono::steady_clock::now();
        g_firstPush = false;
        Metrics::chaperoneRePushCount.Push(
            Metrics::chaperoneRePushCount.size() > 0
                ? static_cast<uint32_t>(Metrics::chaperoneRePushCount.last() + 1u)
                : 1u);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void ScheduleBoundaryStartupPush() {
    g_startupPushPending = kStartupGraceTicks;
    g_firstPush = true;
}

void TickBoundaryRePush(double /*time*/) {
    auto& ctx = CalCtx;
    auto& bc = ctx.boundary;

    // Publish the active metric regardless of whether a push fires.
    Metrics::boundaryActive.Push(bc.enabled);

    if (!bc.enabled || bc.vertices.empty() || !ctx.validProfile) {
        // Startup-push countdown: still tick down so it doesn't stall when
        // the profile flips valid mid-session.
        if (g_startupPushPending > 0) --g_startupPushPending;
        return;
    }

    const Eigen::AffineCompact3d xf = BuildCalibrationTransform(ctx);

    // Startup-push path: fire once after the grace window elapses.
    if (g_startupPushPending > 0) {
        --g_startupPushPending;
        if (g_startupPushPending == 0) {
            DoPush(xf);
            return;
        }
        return;
    }

    // Throttle to kMinPushIntervalSec.
    if (!g_firstPush) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsedSec =
            std::chrono::duration<double>(now - g_lastPushAt).count();
        if (elapsedSec < kMinPushIntervalSec) return;
    }

    // Skip if the transform hasn't moved enough.
    if (!g_firstPush && !TransformDeltaExceedsThreshold(xf, g_lastPushedTransform)) {
        // Still update the timestamp so we don't spin-check every tick when
        // the interval gate would have passed but the delta gate rejects.
        g_lastPushAt = std::chrono::steady_clock::now();
        return;
    } else if (!g_firstPush) {
        // Delta exceeded threshold -- log on edge (one annotation per push).
        const double transDelta = (xf.translation() - g_lastPushedTransform.translation()).norm();
        const Eigen::Matrix3d dR = xf.rotation() * g_lastPushedTransform.rotation().transpose();
        const double trace = dR.trace();
        const double cosAngle = std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5));
        const double angleDeg = std::acos(cosAngle) * (180.0 / EIGEN_PI);
        char lbuf[160];
        snprintf(lbuf, sizeof lbuf,
            "[boundary-re-push] transform delta exceeded threshold:"
            " trans=%.2fmm rot=%.2fdeg",
            transDelta * 1000.0, angleDeg);
        Metrics::WriteLogAnnotation(lbuf);
    }

    DoPush(xf);
}
