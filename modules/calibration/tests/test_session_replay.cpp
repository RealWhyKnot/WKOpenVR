#include <gtest/gtest.h>

#include "SessionReplay.h"

#include <cstdlib>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace replay = spacecal::replay;
namespace autolock = spacecal::autolock;

namespace {

bool EnvFlagLocal(const char* name, bool fallback)
{
	const char* raw = std::getenv(name);
	if (!raw) return fallback;
	std::string value = raw;
	for (char& c : value) {
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	if (value == "1" || value == "true" || value == "yes" || value == "on") return true;
	if (value == "0" || value == "false" || value == "no" || value == "off") return false;
	return fallback;
}

double EnvDoubleLocal(const char* name, double fallback)
{
	const char* raw = std::getenv(name);
	if (!raw) return fallback;
	char* end = nullptr;
	const double v = std::strtod(raw, &end);
	return end == raw ? fallback : v;
}

// Synthetic session for the AUTO Lock simulator: ref fixed, target rigidly
// offset with a per-phase alternating +/- amplitude that sets the relative
// pose MAD directly (alternating +/-a puts the window MAD at ~3a). HMD and
// head tracker are stationary so the commit gate never blocks.
replay::LoadedRecording MakeAutoLockRecording(const std::vector<double>& phaseAmplitudesM, int rowsPerPhase)
{
	replay::LoadedRecording rec;
	rec.formatVersion = 5;
	rec.hasLockedSnapColumns = true;
	int i = 0;
	for (double amp : phaseAmplitudesM) {
		for (int r = 0; r < rowsPerPhase; ++r, ++i) {
			replay::ReplayRow row;
			row.timestamp = static_cast<double>(i) / 90.0;
			row.ref.rot.setIdentity();
			row.ref.trans = Eigen::Vector3d(0.0, 1.60, 0.0);
			row.target.rot.setIdentity();
			const double wobble = (r % 2 == 0) ? amp : -amp;
			row.target.trans = Eigen::Vector3d(0.15 + wobble, 1.60, 0.0);
			row.sample = Sample(row.ref, row.target, row.timestamp);
			row.hasHmdPose = true;
			row.hmd.rot.setIdentity();
			row.hmd.trans = Eigen::Vector3d(0.0, 1.60, 0.0);
			row.headTrackerValid = true;
			row.headTracker.rot.setIdentity();
			row.headTracker.trans = Eigen::Vector3d(0.0, 1.70, -0.1);
			rec.rows.push_back(std::move(row));
		}
	}
	return rec;
}

// Synthetic session for RunSessionReplay: rows all agree on calibration
// cTrue = +0.20 m X (relpose identity trick), HMD/witness stationary. An
// optional persistent HMD frame jump at `relocRow` (reloc_detected stamped),
// and an optional physical witness-tracker shift at `trackerShiftRow` (the
// remounted-puck shape that poisons a captured baseline).
struct SessionSpec
{
	int rows = 600;
	int relocRow = -1;
	double relocJumpM = 0.0;
	int trackerShiftRow = -1;
	double trackerShiftM = 0.0;
};

replay::LoadedRecording MakeSessionRecording(const SessionSpec& spec)
{
	Eigen::AffineCompact3d cTrue = Eigen::AffineCompact3d::Identity();
	cTrue.translation() = Eigen::Vector3d(0.20, 0.0, 0.0);

	replay::LoadedRecording rec;
	rec.formatVersion = 5;
	rec.hasLockedSnapColumns = true;
	rec.hasRelocDetectedColumn = true;
	rec.rows.reserve(spec.rows);
	for (int i = 0; i < spec.rows; ++i) {
		const double yaw = 0.03 * static_cast<double>(i % 40);
		const Eigen::Vector3d trans(0.05 * std::sin(0.7 * i), 1.60 + 0.05 * std::cos(0.5 * i),
		                            0.05 * std::sin(0.3 * i));
		Eigen::AffineCompact3d ref(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY())));
		ref.pretranslate(trans);
		const Eigen::AffineCompact3d target = cTrue.inverse() * ref;

		replay::ReplayRow row;
		row.timestamp = static_cast<double>(i) / 90.0;
		row.ref.rot = ref.rotation();
		row.ref.trans = ref.translation();
		row.target.rot = target.rotation();
		row.target.trans = target.translation();
		row.sample = Sample(row.ref, row.target, row.timestamp);
		row.hasHmdPose = true;
		row.hmd.rot.setIdentity();
		row.hmd.trans = Eigen::Vector3d(0.0, 1.60, 0.0);
		if (spec.relocRow >= 0 && i >= spec.relocRow) row.hmd.trans.x() += spec.relocJumpM;
		row.relocDetected = (spec.relocRow >= 0 && i == spec.relocRow);
		row.headTrackerValid = true;
		row.headTracker.rot.setIdentity();
		row.headTracker.trans = Eigen::Vector3d(0.0, 1.70, -0.10);
		if (spec.trackerShiftRow >= 0 && i >= spec.trackerShiftRow) row.headTracker.trans.x() += spec.trackerShiftM;
		rec.rows.push_back(std::move(row));
	}
	return rec;
}

} // namespace

// The parameterized overloads with default HysteresisParams must decide
// exactly like the historical constant-based entry points.
TEST(AutoLockHysteresisParamsTest, DefaultsMatchConstantEntryPoints)
{
	const autolock::HysteresisParams p;

	for (double floorM : {0.0, 0.002, 0.004, 0.0055, 0.010, 0.050}) {
		EXPECT_DOUBLE_EQ(autolock::EnterThresholdFor(floorM), autolock::EnterThresholdFor(floorM, p)) << floorM;
	}
	for (double t : {0.001, 0.004, 0.010, 0.016, 0.039, 0.041}) {
		for (double r : {0.001, 0.02, 0.09}) {
			EXPECT_EQ(autolock::IsPanicLevelDeviation(t, r), autolock::IsPanicLevelDeviation(t, r, p));
			for (bool prev : {false, true}) {
				EXPECT_EQ(autolock::VerdictWithHysteresis(t, r, prev),
				          autolock::VerdictWithHysteresis(t, r, prev, autolock::kEnterTranslM, p))
				    << "t=" << t << " r=" << r << " prev=" << prev;
			}
		}
	}
	for (double speed : {0.01, 0.049, 0.051, 0.3}) {
		EXPECT_EQ(autolock::HmdIsStationary(speed), autolock::HmdIsStationary(speed, p));
		for (bool to : {false, true}) {
			for (double held : {0.0, 4.9, 5.1}) {
				const auto a = autolock::EvaluateCommitGate(to, speed, 100.0, held);
				const auto b = autolock::EvaluateCommitGate(to, speed, 100.0, held, p);
				EXPECT_EQ(a.commit, b.commit);
				EXPECT_STREQ(a.mode, b.mode);
			}
		}
	}
	for (bool locked : {false, true}) {
		for (double held : {1.0, 2.9, 3.1}) {
			EXPECT_EQ(autolock::IsSettled(locked, 0.002, 0.001, held),
			          autolock::IsSettled(locked, 0.002, 0.001, held, p));
		}
	}
}

// A rig whose relative-pose MAD cycles through the 15-40 mm band flaps with
// the default deadband; widening the leave threshold above the excursion
// keeps the lock held and the session settled. This is the offline shape of
// the live 9-second flip cadence.
TEST(AutoLockSimTest, WiderDeadbandStopsFlapping)
{
	// quiet -> 30 mm MAD excursion -> quiet -> excursion -> quiet.
	const auto rec = MakeAutoLockRecording({0.0, 0.010, 0.0, 0.010, 0.0}, /*rowsPerPhase=*/400);

	const auto defaults = replay::RunAutoLockSim(rec, autolock::HysteresisParams{});
	ASSERT_TRUE(defaults.succeeded) << defaults.error;
	EXPECT_GE(defaults.lockFlips, 5) << "default deadband should flap across the excursion phases";
	EXPECT_EQ(defaults.panicUnlocks, 0);

	autolock::HysteresisParams widened;
	widened.leaveTranslM = 0.035; // above the ~30 mm excursion MAD
	const auto held = replay::RunAutoLockSim(rec, widened);
	ASSERT_TRUE(held.succeeded) << held.error;
	EXPECT_EQ(held.lockFlips, 1) << "widened deadband should lock once and hold";
	EXPECT_EQ(held.panicUnlocks, 0);
	EXPECT_GT(held.settledPct, defaults.settledPct);
	EXPECT_GT(held.lockedPct, 90.0);
}

// Panic-level deviation must bypass the commit gate and unlock immediately,
// exactly like the live path.
TEST(AutoLockSimTest, PanicLevelDeviationUnlocksImmediately)
{
	// quiet (locks) -> +/-40 mm alternation (~118 mm MAD, past the 40 mm panic).
	const auto rec = MakeAutoLockRecording({0.0, 0.040}, /*rowsPerPhase=*/400);

	const auto res = replay::RunAutoLockSim(rec, autolock::HysteresisParams{});
	ASSERT_TRUE(res.succeeded) << res.error;
	EXPECT_EQ(res.lockFlips, 1);
	EXPECT_GE(res.panicUnlocks, 1);
}

// Recorded auto-lock annotations feed the parity counters that the golden
// baseline uses to detect simulator/live divergence.
TEST(AutoLockSimTest, CountsRecordedDecisionAnnotations)
{
	auto rec = MakeAutoLockRecording({0.0}, /*rowsPerPhase=*/60);
	rec.annotations.push_back({0.3, "auto_lock_flip_pending", "# [0.3] auto_lock_flip_pending: target=1"});
	rec.annotations.push_back({0.4, "auto_lock_flip", "# [0.4] auto_lock_flip: previous=0 now=1"});
	rec.annotations.push_back({0.5, "auto_lock_panic_unlock", "# [0.5] auto_lock_panic_unlock: translMad=0.05m"});

	const auto res = replay::RunAutoLockSim(rec, autolock::HysteresisParams{});
	ASSERT_TRUE(res.succeeded) << res.error;
	EXPECT_EQ(res.recordedPendings, 1);
	EXPECT_EQ(res.recordedFlips, 1);
	EXPECT_EQ(res.recordedPanics, 1);
	// One clean quiet phase locks exactly once.
	EXPECT_EQ(res.lockFlips, 1);
}

TEST(AutoLockSimTest, RequiresV4PoseColumns)
{
	auto rec = MakeAutoLockRecording({0.0}, /*rowsPerPhase=*/30);
	rec.hasLockedSnapColumns = false;
	const auto res = replay::RunAutoLockSim(rec, autolock::HysteresisParams{});
	EXPECT_FALSE(res.succeeded);
	EXPECT_EQ(res.error, "autolock_sim_requires_v4");
}

// A corroborated universe flip (HMD jumps, witness still) takes the snap
// fast-reanchor path -- never a destructive clear.
TEST(SessionReplayTest, CorroboratedFlipIsSnapSuppressed)
{
	SessionSpec spec;
	spec.rows = 600;
	spec.relocRow = 300;
	spec.relocJumpM = 0.50;
	const auto rec = MakeSessionRecording(spec);

	replay::SessionReplayOptions opts;
	opts.seedMode = replay::ReplaySeedMode::None;
	const auto res = replay::RunSessionReplay(rec, opts);
	ASSERT_TRUE(res.succeeded) << res.error;
	EXPECT_EQ(res.relocsSeen, 1);
	EXPECT_EQ(res.snapSuppressed, 1);
	EXPECT_EQ(res.destructiveClears, 0);
	EXPECT_EQ(res.subThresholdRelocs, 0);
}

// A 6 cm frame jump under the 30 cm recovery gate: today it leaves a
// residual world offset; the micro-reanchor option absorbs it immediately.
TEST(SessionReplayTest, MicroReanchorRemovesSubThresholdResidual)
{
	SessionSpec spec;
	spec.rows = 600;
	spec.relocRow = 300;
	spec.relocJumpM = 0.06;
	const auto rec = MakeSessionRecording(spec);

	replay::SessionReplayOptions opts;
	opts.seedMode = replay::ReplaySeedMode::None;
	const auto baseline = replay::RunSessionReplay(rec, opts);
	ASSERT_TRUE(baseline.succeeded) << baseline.error;
	EXPECT_EQ(baseline.subThresholdRelocs, 1);
	EXPECT_NEAR(baseline.subThresholdResidualCm, 6.0, 0.5);
	EXPECT_EQ(baseline.microReanchors, 0);

	opts.microReanchor = true;
	const auto fixed = replay::RunSessionReplay(rec, opts);
	ASSERT_TRUE(fixed.succeeded) << fixed.error;
	EXPECT_EQ(fixed.subThresholdRelocs, 1);
	EXPECT_EQ(fixed.microReanchors, 1);
	EXPECT_NEAR(fixed.subThresholdResidualCm, 0.0, 1e-9);
	// The absorb lands as a ~6 cm applied step at the reloc row; the baseline
	// run's applied trajectory stays flat. (This synthetic keeps the ref
	// poses unshifted, so the solver later overwrites the absorb -- live, the
	// ref IS the HMD and the post-jump solve agrees with the absorb.)
	EXPECT_GT(fixed.peakAppliedStepCm, 5.0);
	EXPECT_LT(baseline.peakAppliedStepCm, 1.0);
}

// Env-driven session replay over retained/pinned recordings; the E3/E5
// harness. Prints one [session-replay] line per recording.
TEST(SessionReplayTest, ReplaySessionsWhenRequested)
{
	if (!EnvFlagLocal("WKOPENVR_REPLAY_SESSION", false)) {
		GTEST_SKIP() << "Set WKOPENVR_REPLAY_SESSION=1 (with WKOPENVR_REPLAY_PATHS) to session-replay recordings.";
	}
	const char* rawPaths = std::getenv("WKOPENVR_REPLAY_PATHS");
	if (!rawPaths || !*rawPaths) {
		GTEST_SKIP() << "WKOPENVR_REPLAY_PATHS not set.";
	}

	replay::SessionReplayOptions opts;
	opts.relocRecoverThresholdM =
	    EnvDoubleLocal("WKOPENVR_REPLAY_SESSION_RELOC_RECOVER_CM", opts.relocRecoverThresholdM * 100.0) / 100.0;
	opts.microReanchor = EnvFlagLocal("WKOPENVR_REPLAY_SESSION_MICRO_REANCHOR", false);
	opts.precisionWeightedRelPose = EnvFlagLocal("WKOPENVR_REPLAY_PRECISION_WEIGHT", opts.precisionWeightedRelPose);

	std::string paths = rawPaths;
	std::size_t start = 0;
	int replayed = 0;
	while (start <= paths.size()) {
		std::size_t end = paths.find(';', start);
		if (end == std::string::npos) end = paths.size();
		const std::string path = paths.substr(start, end - start);
		start = end + 1;
		if (path.empty()) continue;

		const auto rec = replay::LoadRecording(path);
		ASSERT_TRUE(rec.error.empty()) << rec.error;
		const auto res = replay::RunSessionReplay(rec, opts);
		const std::string name = std::filesystem::path(path).filename().string();
		if (!res.succeeded) {
			std::cout << "[session-replay] " << name << " skipped=" << res.error << "\n";
			continue;
		}
		std::cout << "[session-replay] " << name << " reloc_recover_cm=" << opts.relocRecoverThresholdM * 100.0
		          << " micro_reanchor=" << (opts.microReanchor ? 1 : 0)
		          << " precision_weight=" << (opts.precisionWeightedRelPose ? 1 : 0)
		          << " seed_applied=" << (res.seedApplied ? 1 : 0) << " rows=" << res.rowsProcessed
		          << " accepts=" << res.accepts << " flips=" << res.lockFlips << " panics=" << res.panicUnlocks
		          << " settled_pct=" << res.settledPct << " relocs=" << res.relocsSeen
		          << " snap_suppressed=" << res.snapSuppressed << " holds=" << res.recoveryHolds
		          << " reanchors=" << res.recoveryReanchors << " destructive_clears=" << res.destructiveClears
		          << " sub_threshold_relocs=" << res.subThresholdRelocs
		          << " sub_threshold_residual_cm=" << res.subThresholdResidualCm
		          << " micro_reanchors=" << res.microReanchors << " applied_path_cm=" << res.totalAppliedPathCm
		          << " peak_step_cm=" << res.peakAppliedStepCm << " net_drift_mag_cm=" << res.netAppliedDriftCm.norm()
		          << "\n";
		++replayed;
	}
	EXPECT_GT(replayed, 0);
}

// Env-driven sweep entry point over retained/pinned recordings; the E4/E5
// harness. Prints one [autolock-sim] line per recording.
TEST(AutoLockSimTest, SimulateLocalRecordingsWhenRequested)
{
	if (!EnvFlagLocal("WKOPENVR_REPLAY_AUTOLOCK_SIM", false)) {
		GTEST_SKIP() << "Set WKOPENVR_REPLAY_AUTOLOCK_SIM=1 (with WKOPENVR_REPLAY_PATHS) to simulate recordings.";
	}
	const char* rawPaths = std::getenv("WKOPENVR_REPLAY_PATHS");
	if (!rawPaths || !*rawPaths) {
		GTEST_SKIP() << "WKOPENVR_REPLAY_PATHS not set.";
	}

	autolock::HysteresisParams p;
	p.enterTranslM = EnvDoubleLocal("WKOPENVR_REPLAY_AUTOLOCK_ENTER_MM", p.enterTranslM * 1000.0) / 1000.0;
	p.leaveTranslM = EnvDoubleLocal("WKOPENVR_REPLAY_AUTOLOCK_LEAVE_MM", p.leaveTranslM * 1000.0) / 1000.0;
	p.enterAdaptiveScale = EnvDoubleLocal("WKOPENVR_REPLAY_AUTOLOCK_SCALE", p.enterAdaptiveScale);
	p.panicTranslM = EnvDoubleLocal("WKOPENVR_REPLAY_AUTOLOCK_PANIC_MM", p.panicTranslM * 1000.0) / 1000.0;
	p.settledMinHoldSec = EnvDoubleLocal("WKOPENVR_REPLAY_AUTOLOCK_SETTLED_HOLD_SEC", p.settledMinHoldSec);
	p.stationaryHmdMps = EnvDoubleLocal("WKOPENVR_REPLAY_AUTOLOCK_STATIONARY_MPS", p.stationaryHmdMps);
	p.unlockMaxWaitSec = EnvDoubleLocal("WKOPENVR_REPLAY_AUTOLOCK_UNLOCK_WAIT_SEC", p.unlockMaxWaitSec);
	p.madFloorWindowSec = EnvDoubleLocal("WKOPENVR_REPLAY_AUTOLOCK_FLOOR_WINDOW_SEC", p.madFloorWindowSec);
	const double window = EnvDoubleLocal("WKOPENVR_REPLAY_AUTOLOCK_WINDOW", static_cast<double>(p.historyMax));
	if (window >= 2.0) {
		p.historyMax = static_cast<std::size_t>(window);
		p.samplesNeeded = static_cast<std::size_t>(window);
	}

	std::string paths = rawPaths;
	std::size_t start = 0;
	int simulated = 0;
	while (start <= paths.size()) {
		std::size_t end = paths.find(';', start);
		if (end == std::string::npos) end = paths.size();
		const std::string path = paths.substr(start, end - start);
		start = end + 1;
		if (path.empty()) continue;

		const auto rec = replay::LoadRecording(path);
		ASSERT_TRUE(rec.error.empty()) << rec.error;
		const auto res = replay::RunAutoLockSim(rec, p);
		const std::string name = std::filesystem::path(path).filename().string();
		if (!res.succeeded) {
			std::cout << "[autolock-sim] " << name << " skipped=" << res.error << "\n";
			continue;
		}
		std::cout << "[autolock-sim] " << name << " enter_mm=" << p.enterTranslM * 1000.0
		          << " leave_mm=" << p.leaveTranslM * 1000.0 << " scale=" << p.enterAdaptiveScale
		          << " panic_mm=" << p.panicTranslM * 1000.0 << " settled_hold_sec=" << p.settledMinHoldSec
		          << " stationary_mps=" << p.stationaryHmdMps << " window=" << p.historyMax
		          << " floor_window_sec=" << p.madFloorWindowSec << " rows=" << res.rowsProcessed
		          << " detector_updates=" << res.detectorUpdates << " flips=" << res.lockFlips
		          << " panics=" << res.panicUnlocks << " pendings=" << res.pendingQueued
		          << " flips_per_min=" << res.flipsPerMin << " settled_pct=" << res.settledPct
		          << " locked_pct=" << res.lockedPct << " median_mad_mm=" << res.medianTranslMadMm
		          << " median_enter_mm=" << res.medianEnterThresholdMm << " recorded_flips=" << res.recordedFlips
		          << " recorded_panics=" << res.recordedPanics << " recorded_pendings=" << res.recordedPendings << "\n";
		++simulated;
	}
	EXPECT_GT(simulated, 0);
}
