#pragma once

#include "Protocol.h"
#include "facetracking/CalibrationShapeTable.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <array>
#include <cstdint>
#include <mutex>
#include <string>

namespace facetracking {

// Per-shape continuous calibration state, second generation.
//
// The model is a bounded, rest-anchored envelope normalizer:
//
//   r    = inverted ? 1 - raw : raw          (openness rests high -> invert)
//   span = max(hi - rest, kMinSpan)
//   gain = min(1 / span, cap)                (cap from calib_table::kGainCap)
//   x    = max(0, r - rest - deadband)       (deadband = 3 * sigma)
//   norm = clamp01(x * gain)
//   out  = lerp(clamp01(r), norm, conf)      (identity until evidence earned)
//
// Three properties make this safe where a plain observed-range stretch is
// not:
//   1. Gain is CAPPED per shape -- a shape the user barely moves keeps a
//      near-1 gain instead of having its noise stretched to full output.
//   2. Output is anchored at a learned rest baseline with a noise-scaled
//      deadband above it, so an idle face produces exactly zero.
//   3. Output blends continuously from identity toward the learned mapping
//      as per-shape confidence accumulates -- there is no warm-up snap.
//
// Learning per shape:
//   - `hi` (max envelope) grows at kAlphaUp once a value has stayed above it
//     for >= kHoldCountMin frames spanning >= kHoldTimeMs, and shrinks at
//     kAlphaDown otherwise (asymmetric so blinks don't deflate the max).
//   - `rest`/`var` update only when the value is slow-moving and inside the
//     rest window; lower-face shapes additionally require the whole face to
//     be still so speech can't drag the mouth baseline upward.
//   - An isolated single-frame discontinuity is rejected outright (spike
//     gate against the previous raw sample -- the previous raw is always
//     recorded, so a genuine sustained step is only skipped for one frame
//     and can never be locked out).
//
// Left/right mirror pairs share one ShapeCalib (calib_table::kPairCanonical),
// learned from the more active side and applied to each side's own raw, so
// the two sides cannot learn diverging ranges but winks pass through.
struct ShapeCalib
{
	// Learned state (persisted).
	float rest; // idle baseline in the calibration domain
	float var;  // rest-noise variance (EMA)
	float hi;   // max envelope
	float conf; // 0..1 identity->calibrated blend weight

	// Cached derivatives (recomputed, not persisted).
	float sigma; // sqrt(var); refreshed only when var updates
	float gain;  // min(1/span, cap); refreshed on ingest

	// Transient gate state.
	float prev_raw; // last raw sample in the calibration domain
	bool prev_valid;
	uint32_t hold_count;     // consecutive frames above hi
	uint64_t hold_first_qpc; // QPC tick of the first frame in this hold run

	ShapeCalib();
	void RefreshCaches(float cap);
};

// Top-level per-module calibration engine. Thread-safe: all public methods
// take an internal mutex (the worker thread ingests/normalizes while the
// request thread can reset/save/load concurrently). Inert -- exact identity,
// no learning -- until Load() has been called with the active module uuid.
class CalibrationEngine
{
public:
	// Aggregate health snapshot for telemetry/UI. Confidence figures cover
	// the calibratable canonical slots only.
	struct Telemetry
	{
		bool loaded = false;
		float avg_conf = 0.f;
		float min_conf = 0.f;
		int capped_shapes = 0;       // canonical slots whose gain sits at its cap
		uint32_t idle_frames = 0;    // period: slot-frames classified idle
		uint32_t idle_false_act = 0; // period: idle slot-frames with output > 0.1
		float open_lr_div_avg = 0.f; // period: mean |outL - outR| eye openness
	};

	// Bind persistence to the active module and enable the engine. Loads any
	// schema-2 state on disk (confidence capped at 0.5 so a headset refit
	// between sessions re-learns quickly); discards and deletes stale-schema
	// files.
	void Load(const std::string& module_uuid);

	// Persist learned state to disk. No-op until Load() has run.
	void Save();

	// Learn from one raw frame (call BEFORE Normalize with the same frame).
	// Hot path -- no allocations, no logging.
	void IngestFrame(const protocol::FaceTrackingFrameBody& in);

	// Apply the learned mapping in-place. Slots with a 1.0 gain cap and
	// expressions excluded by the user are left untouched. Hot path.
	void Normalize(protocol::FaceTrackingFrameBody& inout);

	// Execute a calibration command from the overlay.
	//   FaceCalibSave      -- flush to disk immediately.
	//   FaceCalibResetAll  -- zero all learned state and delete the file.
	//   FaceCalibResetEye  -- zero eye openness + pupil state.
	//   FaceCalibResetExpr -- zero the 63 expression slots.
	void Reset(protocol::FaceCalibrationOp op);

	// Per-expression user opt-out (from the tuning table's exclude flag).
	// Excluded shapes keep learning but are never modified on output.
	void SetUserExcluded(uint32_t expression_index, bool excluded);
	void ClearUserExclusions();

	Telemetry Snapshot() const;
	void ResetPeriodCounters();

	// Compact diagnostic of the learned mappings for the signals most likely
	// to look wrong on the avatar. Not hot-path.
	std::string DebugSummary() const;

private:
	static constexpr int kTotalShapes = calib_table::kTotalShapes;

	ShapeCalib shapes_[kTotalShapes];
	std::array<bool, protocol::FACETRACKING_EXPRESSION_COUNT> user_excluded_{};
	bool loaded_ = false;
	std::string module_uuid_;

	// Face-still detector: consecutive expression frames where the jaw and
	// mouth-close signals both moved less than the still threshold.
	uint32_t still_streak_ = 0;

	// Period telemetry accumulators (reset by ResetPeriodCounters).
	uint32_t idle_frames_ = 0;
	uint32_t idle_false_act_ = 0;
	float open_lr_div_sum_ = 0.f;
	uint32_t open_lr_div_count_ = 0;

	mutable std::mutex mutex_;

	// QPC frequency cached at first use.
	mutable LARGE_INTEGER qpc_freq_{0};
	LARGE_INTEGER QpcFreq() const;

	// Update one canonical slot with a new raw value (calibration domain).
	// rest_frozen suppresses the rest/var update (lower face during speech).
	void UpdateShape(ShapeCalib& s, float raw, uint64_t now_qpc, float cap, bool rest_frozen) const;

	// Normalize one value through one slot's learned mapping; also feeds the
	// idle telemetry counters.
	float NormalizeOne(const ShapeCalib& s, float raw_domain);

	Telemetry SnapshotLocked() const;

	// Persistence helpers (callers hold mutex_).
	std::wstring CalibFilePath() const;
	void SaveLocked() const;
	void LoadLocked(const std::string& uuid);
};

} // namespace facetracking
