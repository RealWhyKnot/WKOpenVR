// Unit tests for facetracking::CalibrationEngine (bounded rest-anchored
// envelope normalizer).
//
// Coverage:
//   - Exact identity at cold start (conf = 0) and while not Load()ed.
//   - Deadband: an idle shape outputs exactly zero once rest is learned.
//   - Gain caps per tier: aggressive 3x, conservative 1.5x, excluded identity.
//   - Confidence blend is continuous -- no warm-up snap.
//   - Paired L/R shapes share learned state; winks still pass through.
//   - Eye openness is calibrated on the inverted signal (partial blinks
//     learn to reach fully closed; idle-open stays open).
//   - A single-frame spike does not change the learned mapping.
//   - Sustained new peaks DO grow the envelope (hold gate).
//   - Lower-face rest learning freezes while the face is moving.
//   - Reset op scoping (All / Eye / Expr) and user exclusion.
//   - Persistence v2 round-trip with the confidence cap on load; stale
//     schema files are discarded and deleted.

#include "CalibrationEngine.h"
#include "Protocol.h"
#include "facetracking/CalibrationShapeTable.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

constexpr int kJaw = 26;         // aggressive tier (cap 3.0)
constexpr int kBrowInnerL = 14;  // conservative tier (cap 1.5), upper face, paired with 15
constexpr int kMouthUpperL = 41; // conservative tier, lower face
constexpr int kSmileL = 45;      // aggressive tier, paired with 46
constexpr int kSmileR = 46;
constexpr int kTongueOut = 59; // excluded tier (cap 1.0)

// A frame with every field at rest except explicit overrides. Stamps
// qpc_sample_time to now so the 33 ms frame-age gate never trips.
protocol::FaceTrackingFrameBody BaseFrame()
{
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	protocol::FaceTrackingFrameBody f{};
	f.qpc_sample_time = (uint64_t)now.QuadPart;
	f.source_module_uuid_hash = 0xDEADBEEFCAFEBABEull;
	f.eye_openness_l = 1.f; // eyes open at rest
	f.eye_openness_r = 1.f;
	f.pupil_dilation_l = 0.5f;
	f.pupil_dilation_r = 0.5f;
	f.flags = 0x3; // eye + expression valid
	return f;
}

protocol::FaceTrackingFrameBody ExprFrame(int index, float value)
{
	protocol::FaceTrackingFrameBody f = BaseFrame();
	f.expressions[index] = value;
	return f;
}

protocol::FaceTrackingFrameBody EyeFrame(float openL, float openR)
{
	protocol::FaceTrackingFrameBody f = BaseFrame();
	f.eye_openness_l = openL;
	f.eye_openness_r = openR;
	return f;
}

// Fresh engine bound to `uuid` with any stale on-disk state removed.
void FreshEngine(facetracking::CalibrationEngine& eng, const std::string& uuid)
{
	eng.Load(uuid);
	eng.Reset(protocol::FaceCalibResetAll); // also deletes the persisted file
}

float NormalizedExpr(facetracking::CalibrationEngine& eng, int index, float value)
{
	protocol::FaceTrackingFrameBody f = ExprFrame(index, value);
	eng.Normalize(f);
	return f.expressions[index];
}

} // namespace

// Redirects the engine's persistence root to a per-run temp directory so
// tests never touch real learned state and leave nothing behind.
class CalibrationEngineTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		wchar_t tempBuf[MAX_PATH] = {};
		GetTempPathW(MAX_PATH, tempBuf);
		const auto stamp = std::to_wstring(GetCurrentProcessId()) + L"_" +
		                   std::to_wstring(std::chrono::steady_clock::now().time_since_epoch().count());
		dir_ = std::filesystem::path(tempBuf) / (L"WKOpenVR_CalibTests_" + stamp);
		std::filesystem::create_directories(dir_);

		DWORD needed = GetEnvironmentVariableW(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", nullptr, 0);
		if (needed > 0) {
			previous_.resize(needed, L'\0');
			DWORD written = GetEnvironmentVariableW(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", previous_.data(), needed);
			had_previous_ = written > 0 && written < needed;
			if (had_previous_) previous_.resize(written);
		}
		SetEnvironmentVariableW(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", dir_.c_str());
	}

	void TearDown() override
	{
		SetEnvironmentVariableW(L"WKOPENVR_LOCALAPPDATA_OVERRIDE", had_previous_ ? previous_.c_str() : nullptr);
		std::error_code ec;
		std::filesystem::remove_all(dir_, ec);
	}

	std::wstring CalibFilePathForUuid(const std::string& uuid) const
	{
		std::wstring path = (dir_ / L"WKOpenVR" / L"profiles" / L"facetracking_calib_").wstring();
		path.append(uuid.begin(), uuid.end());
		path += L".json";
		return path;
	}

private:
	std::filesystem::path dir_;
	std::wstring previous_;
	bool had_previous_ = false;
};

TEST_F(CalibrationEngineTest, IdentityAtColdStart)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-cold");

	protocol::FaceTrackingFrameBody f = ExprFrame(kJaw, 0.42f);
	f.eye_openness_l = 0.42f;
	f.eye_openness_r = 0.42f;
	eng.Normalize(f);

	// conf = 0 before any learning: output is exactly the raw value.
	EXPECT_FLOAT_EQ(f.expressions[kJaw], 0.42f);
	EXPECT_FLOAT_EQ(f.eye_openness_l, 0.42f);
	EXPECT_FLOAT_EQ(f.eye_openness_r, 0.42f);
}

TEST_F(CalibrationEngineTest, InertUntilLoaded)
{
	facetracking::CalibrationEngine eng; // no Load()

	for (int i = 0; i < 3000; ++i) {
		eng.IngestFrame(ExprFrame(kJaw, 0.2f));
	}
	protocol::FaceTrackingFrameBody f = ExprFrame(kJaw, 0.2f);
	eng.Normalize(f);
	EXPECT_FLOAT_EQ(f.expressions[kJaw], 0.2f);
	EXPECT_FALSE(eng.Snapshot().loaded);
}

TEST_F(CalibrationEngineTest, DeadbandZeroAtLearnedRest)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-deadband");

	// Jaw idling slightly above zero (the "slack jaw" case). Constant input
	// keeps the face-still detector satisfied so the lower-face rest gate
	// stays open.
	for (int i = 0; i < 6000; ++i) {
		eng.IngestFrame(ExprFrame(kJaw, 0.05f));
	}

	// At full confidence an idle-level sample lands inside the deadband and
	// must output exactly zero.
	EXPECT_FLOAT_EQ(NormalizedExpr(eng, kJaw, 0.05f), 0.0f);
	EXPECT_FLOAT_EQ(NormalizedExpr(eng, kJaw, 0.0f), 0.0f);
}

TEST_F(CalibrationEngineTest, GainCapsPerTier)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-gaincap");

	// Drive three shapes to a persistent maximum of 0.2. The envelope decays
	// toward the observed peak over thousands of active samples; the learned
	// gain must respect each tier's cap even though the observed span (0.2)
	// would justify a 5x stretch.
	for (int i = 0; i < 12000; ++i) {
		protocol::FaceTrackingFrameBody f = BaseFrame();
		f.expressions[kJaw] = 0.2f;
		f.expressions[kBrowInnerL] = 0.2f;
		f.expressions[kTongueOut] = 0.2f;
		eng.IngestFrame(f);
	}

	// rest = 0, sigma stays at its 0.02 seed (0.2 is never rest-eligible), so
	// x = 0.2 - 3*0.02 = 0.14 and the capped outputs are deterministic.
	EXPECT_NEAR(NormalizedExpr(eng, kJaw, 0.2f), 0.14f * 3.0f, 0.02f);
	EXPECT_NEAR(NormalizedExpr(eng, kBrowInnerL, 0.2f), 0.14f * 1.5f, 0.02f);
	// Excluded tier: exact identity, always.
	EXPECT_FLOAT_EQ(NormalizedExpr(eng, kTongueOut, 0.2f), 0.2f);
}

TEST_F(CalibrationEngineTest, ConfidenceBlendHasNoSnap)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-blend");

	// Sample the output for a fixed probe while confidence ramps. The output
	// must move smoothly from identity toward the learned mapping -- no
	// single step may jump.
	float prev = -1.f;
	float first = 0.f;
	float maxStep = 0.f;
	for (int block = 0; block < 30; ++block) {
		for (int i = 0; i < 100; ++i) {
			eng.IngestFrame(ExprFrame(kBrowInnerL, 0.5f));
		}
		const float out = NormalizedExpr(eng, kBrowInnerL, 0.5f);
		if (prev >= 0.f) {
			maxStep = std::max(maxStep, std::abs(out - prev));
		}
		else {
			first = out;
		}
		prev = out;
	}
	EXPECT_LT(maxStep, 0.05f);      // continuous -- no warm-up snap
	EXPECT_GT(prev, first + 0.02f); // and it did move toward the learned mapping
}

TEST_F(CalibrationEngineTest, PairedShapesShareLearnedState)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-pair");

	// Smile trained asymmetrically: left side strong, right side barely
	// moves. Independent learning would give the right side a huge gain.
	for (int i = 0; i < 6000; ++i) {
		protocol::FaceTrackingFrameBody f = BaseFrame();
		f.expressions[kSmileL] = 0.6f;
		f.expressions[kSmileR] = 0.1f;
		eng.IngestFrame(f);
	}

	// Equal raw inputs must produce equal outputs: one shared mapping.
	protocol::FaceTrackingFrameBody f = BaseFrame();
	f.expressions[kSmileL] = 0.3f;
	f.expressions[kSmileR] = 0.3f;
	eng.Normalize(f);
	EXPECT_FLOAT_EQ(f.expressions[kSmileL], f.expressions[kSmileR]);
}

TEST_F(CalibrationEngineTest, WinksSurviveSharedEyeCalibration)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-wink");

	// Symmetric blink cycles teach the shared openness range.
	for (int cycle = 0; cycle < 120; ++cycle) {
		for (int i = 0; i < 20; ++i)
			eng.IngestFrame(EyeFrame(0.95f, 0.95f));
		for (int i = 0; i < 12; ++i)
			eng.IngestFrame(EyeFrame(0.1f, 0.1f));
	}

	// A wink: one eye closed, one open. Shared calibration must not
	// symmetrize the OUTPUT -- each side is mapped from its own raw value.
	protocol::FaceTrackingFrameBody f = EyeFrame(0.0f, 1.0f);
	eng.Normalize(f);
	EXPECT_LT(f.eye_openness_l, 0.2f);
	EXPECT_GT(f.eye_openness_r, 0.8f);
}

TEST_F(CalibrationEngineTest, PartialBlinksLearnToReachFullyClosed)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-invert");

	// A tracker whose blinks only reach 0.4 openness (0.6 closedness).
	// Openness is calibrated on the inverted signal, so the learned range
	// should map that partial blink close to fully shut -- while idle-open
	// stays open.
	for (int cycle = 0; cycle < 200; ++cycle) {
		for (int i = 0; i < 30; ++i)
			eng.IngestFrame(EyeFrame(0.95f, 0.95f));
		for (int i = 0; i < 30; ++i)
			eng.IngestFrame(EyeFrame(0.4f, 0.4f));
	}

	protocol::FaceTrackingFrameBody blink = EyeFrame(0.4f, 0.4f);
	eng.Normalize(blink);
	EXPECT_LT(blink.eye_openness_l, 0.25f); // partial blink reads nearly closed

	protocol::FaceTrackingFrameBody open = EyeFrame(0.95f, 0.95f);
	eng.Normalize(open);
	EXPECT_GT(open.eye_openness_l, 0.85f); // idle-open stays open
}

TEST_F(CalibrationEngineTest, SingleFrameSpikeDoesNotMoveTheMapping)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-spike");

	for (int i = 0; i < 6000; ++i) {
		eng.IngestFrame(ExprFrame(kJaw, 0.2f));
	}
	const float before = NormalizedExpr(eng, kJaw, 0.2f);

	// One full-range glitch frame, then back to normal.
	eng.IngestFrame(ExprFrame(kJaw, 1.0f));
	eng.IngestFrame(ExprFrame(kJaw, 0.2f));

	const float after = NormalizedExpr(eng, kJaw, 0.2f);
	EXPECT_NEAR(after, before, 0.005f);
}

TEST_F(CalibrationEngineTest, SustainedPeakGrowsTheEnvelope)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-hold");

	for (int i = 0; i < 12000; ++i) {
		eng.IngestFrame(ExprFrame(kJaw, 0.2f));
	}
	const float before = NormalizedExpr(eng, kJaw, 0.2f);
	EXPECT_GT(before, 0.3f); // learned gain amplifies the 0.2 peak

	// A genuinely sustained wider movement (the hold gate needs >= 6 frames
	// spanning >= 80 ms of wall time) must widen the envelope, which lowers
	// the gain applied to the old peak.
	for (int i = 0; i < 150; ++i) {
		eng.IngestFrame(ExprFrame(kJaw, 0.9f));
		if (i == 0) Sleep(90);
	}
	const float after = NormalizedExpr(eng, kJaw, 0.2f);
	EXPECT_LT(after, before - 0.05f);
}

TEST_F(CalibrationEngineTest, LowerFaceRestFreezesWhileFaceMoves)
{
	// Engine A: face perfectly still -- the mouth shape's rest baseline is
	// learned and its idle value dead-bands to zero.
	facetracking::CalibrationEngine still;
	FreshEngine(still, "test-v2-rest-still");
	for (int i = 0; i < 6000; ++i) {
		still.IngestFrame(ExprFrame(kMouthUpperL, 0.08f));
	}
	EXPECT_FLOAT_EQ(NormalizedExpr(still, kMouthUpperL, 0.08f), 0.0f);

	// Engine B: same mouth signal but the jaw is talking the whole time, so
	// lower-face rest learning stays frozen and the idle value leaks through.
	facetracking::CalibrationEngine talking;
	FreshEngine(talking, "test-v2-rest-talk");
	for (int i = 0; i < 6000; ++i) {
		protocol::FaceTrackingFrameBody f = ExprFrame(kMouthUpperL, 0.08f);
		f.expressions[kJaw] = (i & 1) ? 0.3f : 0.0f;
		talking.IngestFrame(f);
	}
	EXPECT_GT(NormalizedExpr(talking, kMouthUpperL, 0.08f), 0.005f);
}

TEST_F(CalibrationEngineTest, ResetOpsScopeCorrectly)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-resets");

	auto learn = [&eng] {
		for (int cycle = 0; cycle < 100; ++cycle) {
			for (int i = 0; i < 30; ++i) {
				protocol::FaceTrackingFrameBody f = EyeFrame(0.95f, 0.95f);
				f.expressions[kJaw] = 0.2f;
				eng.IngestFrame(f);
			}
			for (int i = 0; i < 30; ++i) {
				protocol::FaceTrackingFrameBody f = EyeFrame(0.4f, 0.4f);
				f.expressions[kJaw] = 0.2f;
				eng.IngestFrame(f);
			}
		}
	};
	learn();
	ASSERT_NE(NormalizedExpr(eng, kJaw, 0.2f), 0.2f); // jaw mapping learned

	// ResetEye: openness back to identity, jaw mapping survives.
	eng.Reset(protocol::FaceCalibResetEye);
	protocol::FaceTrackingFrameBody f = EyeFrame(0.4f, 0.4f);
	eng.Normalize(f);
	EXPECT_FLOAT_EQ(f.eye_openness_l, 0.4f);
	EXPECT_NE(NormalizedExpr(eng, kJaw, 0.2f), 0.2f);

	// ResetExpr: jaw back to identity.
	eng.Reset(protocol::FaceCalibResetExpr);
	EXPECT_FLOAT_EQ(NormalizedExpr(eng, kJaw, 0.2f), 0.2f);
}

TEST_F(CalibrationEngineTest, UserExclusionForcesIdentity)
{
	facetracking::CalibrationEngine eng;
	FreshEngine(eng, "test-v2-exclude");

	for (int i = 0; i < 6000; ++i) {
		eng.IngestFrame(ExprFrame(kJaw, 0.2f));
	}
	ASSERT_NE(NormalizedExpr(eng, kJaw, 0.2f), 0.2f);

	eng.SetUserExcluded(kJaw, true);
	EXPECT_FLOAT_EQ(NormalizedExpr(eng, kJaw, 0.2f), 0.2f);

	// Learning continued while excluded; re-enabling restores the mapping.
	eng.SetUserExcluded(kJaw, false);
	EXPECT_NE(NormalizedExpr(eng, kJaw, 0.2f), 0.2f);

	eng.ClearUserExclusions();
	EXPECT_NE(NormalizedExpr(eng, kJaw, 0.2f), 0.2f);
}

TEST_F(CalibrationEngineTest, PersistenceRoundTripCapsConfidence)
{
	const std::string uuid = "test-v2-roundtrip";
	float learnedOut = 0.f;
	{
		facetracking::CalibrationEngine eng;
		FreshEngine(eng, uuid);
		for (int i = 0; i < 6000; ++i) {
			eng.IngestFrame(ExprFrame(kJaw, 0.2f));
		}
		learnedOut = NormalizedExpr(eng, kJaw, 0.2f);
		ASSERT_NE(learnedOut, 0.2f);
		EXPECT_NEAR(eng.Snapshot().avg_conf, 1.f, 0.01f);
		eng.Save();
	}

	facetracking::CalibrationEngine fresh;
	fresh.Load(uuid);

	// Ranges restored but confidence re-opened to at most 0.5, so the output
	// sits between identity and the fully-learned mapping.
	const auto snap = fresh.Snapshot();
	EXPECT_TRUE(snap.loaded);
	EXPECT_GT(snap.avg_conf, 0.f);
	EXPECT_LE(snap.avg_conf, 0.5f);

	const float out = NormalizedExpr(fresh, kJaw, 0.2f);
	EXPECT_NE(out, 0.2f);                                         // learned range is in effect...
	EXPECT_LT(std::abs(out - 0.2f), std::abs(learnedOut - 0.2f)); // ...but blended

	fresh.Reset(protocol::FaceCalibResetAll); // clean up the on-disk file
}

TEST_F(CalibrationEngineTest, StaleSchemaFileIsDiscardedAndDeleted)
{
	const std::string uuid = "test-v2-stale-schema";
	const std::wstring path = CalibFilePathForUuid(uuid);
	ASSERT_FALSE(path.empty());

	{
		std::filesystem::create_directories(std::filesystem::path(path).parent_path());
		std::ofstream out(path);
		ASSERT_TRUE(out.is_open());
		out << "{\"schema\": 1, \"shapes\": [{\"p02\": 0.4, \"p98\": 0.6}]}";
	}

	facetracking::CalibrationEngine eng;
	eng.Load(uuid);

	// State stays default (identity at conf 0) and the stale file is gone.
	EXPECT_FLOAT_EQ(NormalizedExpr(eng, kJaw, 0.42f), 0.42f);
	EXPECT_FLOAT_EQ(eng.Snapshot().avg_conf, 0.f);
	EXPECT_EQ(GetFileAttributesW(path.c_str()), INVALID_FILE_ATTRIBUTES);
}
