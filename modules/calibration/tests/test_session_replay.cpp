#include <gtest/gtest.h>

#include "AutoLockHysteresis.h"
#include "SessionReplay.h"

#include <cstdlib>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string>

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

// Synthetic session for RunSessionReplay: rows all agree on calibration
// cTrue = +0.20 m X (relpose identity trick). An optional mid-recording shift
// of the true calibration models a silent reference-frame re-anchor.
struct SessionSpec
{
	int rows = 600;
	int calShiftRow = -1;
	double calShiftM = 0.0;
};

replay::LoadedRecording MakeSessionRecording(const SessionSpec& spec)
{
	Eigen::AffineCompact3d cTrue = Eigen::AffineCompact3d::Identity();
	cTrue.translation() = Eigen::Vector3d(0.20, 0.0, 0.0);

	replay::LoadedRecording rec;
	rec.formatVersion = 5;
	rec.hasLockedSnapColumns = true;
	rec.rows.reserve(spec.rows);
	for (int i = 0; i < spec.rows; ++i) {
		// Multi-axis rotation sweep: the full-solve conditioning gate refuses
		// single-axis motion, so the synthetic session must vary yaw and pitch.
		const double yaw = 0.8 * std::sin(2.0 * EIGEN_PI * i / 41.0);
		const double pitch = 0.6 * std::sin(2.0 * EIGEN_PI * i / 57.0 + 1.0);
		const Eigen::Vector3d trans(0.05 * std::sin(0.7 * i), 1.60 + 0.05 * std::cos(0.5 * i),
		                            0.05 * std::sin(0.3 * i));
		Eigen::AffineCompact3d ref(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
		                                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX())));
		ref.pretranslate(trans);
		Eigen::AffineCompact3d cRow = cTrue;
		if (spec.calShiftRow >= 0 && i >= spec.calShiftRow) cRow.translation().x() += spec.calShiftM;
		const Eigen::AffineCompact3d target = cRow.inverse() * ref;

		replay::ReplayRow row;
		// 10 Hz cadence so a multi-minute session fits in a small row count;
		// the frame-jump test must outlive the 60 s validation-history expiry.
		row.timestamp = static_cast<double>(i) / 10.0;
		row.ref.rot = ref.rotation();
		row.ref.trans = ref.translation();
		row.target.rot = target.rotation();
		row.target.trans = target.translation();
		row.sample = Sample(row.ref, row.target, row.timestamp);
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

// A metres-wrong seed corrected by the session's first accepted candidate is
// a classified step (the live first-candidate motion gate): it must count in
// the applied path but stay out of the wander/unclassified metrics.
TEST(SessionReplayTest, WanderMetricExcludesClassifiedReanchorSteps)
{
	SessionSpec spec;
	spec.rows = 600;
	const auto rec = MakeSessionRecording(spec); // truth: +20 cm X

	replay::SessionReplayOptions opts;
	opts.seedMode = replay::ReplaySeedMode::Explicit;
	opts.seedTransCm = Eigen::Vector3d(120.0, 0.0, 0.0); // 1 m wrong
	const auto res = replay::RunSessionReplay(rec, opts);
	ASSERT_TRUE(res.succeeded) << res.error;
	ASSERT_GT(res.accepts, 1);
	EXPECT_GT(res.peakAppliedStepCm, 50.0) << "first candidate must correct the metre-wrong seed";
	EXPECT_LT(res.maxUnclassifiedStepCm, 5.0) << "the correction step is classified, not wander";
	EXPECT_LT(res.unclassifiedPathCm, res.totalAppliedPathCm - 50.0);
}

// A mid-session frame re-anchor with no observable event (the asleep-HMD
// shape) surfaces as one giant unclassified applied step once the stale
// validation history expires -- the metric that gates this incident class.
TEST(SessionReplayTest, UnclassifiedFrameJumpRaisesMaxStep)
{
	SessionSpec spec;
	spec.rows = 1200; // 120 s at 10 Hz: jump at 30 s, history expiry at 90 s
	spec.calShiftRow = 300;
	spec.calShiftM = 4.0;
	const auto rec = MakeSessionRecording(spec);

	replay::SessionReplayOptions opts;
	opts.seedMode = replay::ReplaySeedMode::None;
	const auto res = replay::RunSessionReplay(rec, opts);
	ASSERT_TRUE(res.succeeded) << res.error;
	EXPECT_GT(res.maxUnclassifiedStepCm, 300.0) << "the silent re-anchor lands as unclassified movement";
	EXPECT_GT(res.wanderPer10MinCm, 0.0);
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
	opts.lockRelativePosition = EnvFlagLocal("WKOPENVR_REPLAY_LOCK_REL", opts.lockRelativePosition);
	// Quick-gate row cap (Run-SessionReplayGate.ps1 -Quick): replay only the
	// first N rows; capped runs gate against their own quick baselines.
	const int maxRows = static_cast<int>(EnvDoubleLocal("WKOPENVR_REPLAY_MAX_ROWS", 0.0));

	std::string paths = rawPaths;
	std::size_t start = 0;
	int replayed = 0;
	while (start <= paths.size()) {
		std::size_t end = paths.find(';', start);
		if (end == std::string::npos) end = paths.size();
		const std::string path = paths.substr(start, end - start);
		start = end + 1;
		if (path.empty()) continue;

		auto rec = replay::LoadRecording(path);
		ASSERT_TRUE(rec.error.empty()) << rec.error;
		if (maxRows > 0 && rec.rows.size() > static_cast<std::size_t>(maxRows)) {
			rec.rows.resize(static_cast<std::size_t>(maxRows));
		}
		const auto res = replay::RunSessionReplay(rec, opts);
		const std::string name = std::filesystem::path(path).filename().string();
		if (!res.succeeded) {
			std::cout << "[session-replay] " << name << " skipped=" << res.error << "\n";
			continue;
		}
		std::cout << "[session-replay] " << name << " lock_rel=" << (opts.lockRelativePosition ? 1 : 0)
		          << " seed_applied=" << (res.seedApplied ? 1 : 0) << " rows=" << res.rowsProcessed
		          << " accepts=" << res.accepts << " applied_path_cm=" << res.totalAppliedPathCm
		          << " peak_step_cm=" << res.peakAppliedStepCm << " wander_per_10min_cm=" << res.wanderPer10MinCm
		          << " max_unclassified_step_cm=" << res.maxUnclassifiedStepCm
		          << " rot_wander_per_10min_deg=" << res.rotWanderPer10MinDeg
		          << " max_unclassified_rot_step_deg=" << res.maxUnclassifiedRotStepDeg
		          << " max_applied_tilt_deg=" << res.maxAppliedTiltDeg
		          << " final_applied_tilt_deg=" << res.finalAppliedTiltDeg
		          << " net_drift_mag_cm=" << res.netAppliedDriftCm.norm() << "\n";
		++replayed;
	}
	EXPECT_GT(replayed, 0);
}
