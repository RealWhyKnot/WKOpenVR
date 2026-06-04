#pragma once

#include "Protocol.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstdint>
#include <mutex>
#include <string>

namespace facetracking {

// Per-shape continuous calibration state.
//
// Maintains a P-square percentile estimator (P02 / P98) plus an EMA min/max
// envelope with asymmetric decay: the envelope grows quickly (alpha_up=0.02)
// so new peaks are captured within seconds, but it shrinks only very slowly
// (alpha_down=0.0005) so a momentary wide opening does not immediately deflate
// the max and produce runaway normalized values.  The P02/P98 values give the
// stable long-run bounds used for normalization; the EMA envelope acts as a
// fast-tracking guard so outliers from sensor noise don't corrupt the
// percentile estimators.
//
// Three outlier gates reject bad samples before they reach the estimators:
//   1. Frame-age gate  -- drop if the sample's QPC timestamp is > 33 ms stale.
//   2. Velocity gate   -- reject if |raw - ema_value| > 4 * sqrt(ema_var).
//                         4-sigma gives a false-rejection rate < 0.007% on a
//                         Gaussian signal, which is negligible at 120 Hz.
//   3. Hold-time gate  -- a new EMA max only commits after the input has
//                         stayed above the prior max for >= 6 consecutive
//                         samples spanning >= 80 ms.  Prevents a single-frame
//                         blink spike from immediately widening the calibrated
//                         range.
//
// Cold-start: raw passthrough until a shape accumulates >= 200 samples
// (roughly 1.7 s at 120 Hz).
struct ShapeCalib
{
	// P-square 5-marker state for P02 and P98.
	// Standard Jain-Chlamtac markers: q[0..4], n[0..4] (integer counts),
	// dn[0..4] (desired increments).
	float q[5];
	float n[5];
	float dn[5]; // desired marker positions
	float p02;   // current P02 estimate
	float p98;   // current P98 estimate

	// EMA min / max envelope.
	float ema_min;
	float ema_max;

	// EMA value + variance for the velocity gate.
	// ema_var is the rolling variance E[(x - mu)^2] maintained as an EMA;
	// sqrt(ema_var) is the expected instantaneous std-dev for this shape.
	float ema_value;
	float ema_var;

	// Hold-time gate state for the max side.
	uint32_t hold_count;     // consecutive frames above current ema_max
	uint64_t hold_first_qpc; // QPC tick of the first frame in this hold run

	// Warmup counter.
	uint32_t samples;
	bool warm; // true once samples >= 200

	// Initialised to passthrough defaults.
	ShapeCalib();
};

// Top-level per-instance calibration engine.
// One instance lives inside FacetrackingDriverModule; it is only ever accessed
// from the worker thread (IngestFrame) or from the HandleRequest path (Reset,
// Save, Load) -- both paths hold the module's config mutex before calling here,
// so no additional locking is needed inside.
class CalibrationEngine
{
public:
	// Called at Init() with the active module uuid so persistence uses the
	// right filename.
	void Load(const std::string& module_uuid);

	// Persist learned state to disk.  Called on clean Shutdown() and on every
	// FaceCalibSave command.
	void Save();

	// Ingest one raw frame.  Updates per-shape state for all shapes that pass
	// the outlier gates.  Hot path -- no allocations, no logging.
	void IngestFrame(const protocol::FaceTrackingFrameBody& in);

	// Apply learned normalization to a mutable frame in-place.  Every shape
	// (and both eye openness + pupil dilation values) is mapped through the
	// P02/P98 estimators and clamped to [0,1].  Warm-up passthrough applies
	// per shape: shapes not yet warm are left untouched.  Hot path.
	void Normalize(protocol::FaceTrackingFrameBody& inout) const;

	// Execute a calibration command from the overlay.
	//   FaceCalibResetAll  -- zero all shape state and delete the persisted file.
	//   FaceCalibResetEye  -- zero only eye openness + pupil dilation shapes.
	//   FaceCalibResetExpr -- zero only the 63 expression shapes.
	//   FaceCalibSave      -- flush to disk immediately.
	void Reset(protocol::FaceCalibrationOp op);

	// Query warmup status.  Returns the number of shapes that have reached the
	// warm threshold (>= 200 samples).  Exposed so the overlay can render the
	// readiness dot grid.
	int WarmShapeCount() const;
	int TotalShapeCount() const;

	// Per-shape warmth query for the telemetry sidecar.  idx must be in
	// [0, kTotalShapes).  Returns false for out-of-range indices.
	bool IsShapeWarm(int idx) const;

private:
	// Layout:
	//   [0..62]  expressions[0..62]
	//   [63]     eye_openness_l
	//   [64]     eye_openness_r
	//   [65]     pupil_dilation_l
	//   [66]     pupil_dilation_r
	static constexpr int kTotalShapes = protocol::FACETRACKING_EXPRESSION_COUNT + 4;
	static constexpr int kIdxOpenL = protocol::FACETRACKING_EXPRESSION_COUNT;
	static constexpr int kIdxOpenR = kIdxOpenL + 1;
	static constexpr int kIdxPupilL = kIdxOpenL + 2;
	static constexpr int kIdxPupilR = kIdxOpenL + 3;

	ShapeCalib shapes_[kTotalShapes];
	std::string module_uuid_;

	// QPC frequency cached at first use.
	mutable LARGE_INTEGER qpc_freq_{0};

	LARGE_INTEGER QpcFreq() const;

	// Update one ShapeCalib with a new raw value and the current QPC time.
	void UpdateShape(ShapeCalib& s, float raw, uint64_t now_qpc) const;

	// Normalize one float through one shape's P02/P98 window.
	static float NormalizeOne(const ShapeCalib& s, float raw);

	// Persistence helpers.
	std::wstring CalibFilePath() const;
	void SaveLocked() const;
	void LoadLocked(const std::string& uuid);
};

} // namespace facetracking
