#define _CRT_SECURE_NO_DEPRECATE
#include "CalibrationEngine.h"
#include "Logging.h"
#include "Win32Paths.h"

#include "picojson.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

namespace facetracking {

namespace {

using calib_table::kGainCap;
using calib_table::kInverted;
using calib_table::kLowerFace;
using calib_table::kPairCanonical;

// Envelope speeds. The max envelope grows in ~50 accepted frames (~0.4 s)
// once the hold gate opens, and contracts ~40x slower -- and ONLY from
// active samples. Idle samples must not shrink the envelope: decaying `hi`
// toward the rest level during a quiet minute would collapse the learned
// span, pin the gain at its cap, and over-amplify the first real expression
// afterwards.
constexpr float kAlphaUp = 0.02f;
constexpr float kAlphaDown = 0.0005f;

// Rest baseline adaptation. Slow (~10 s time constant at 120 Hz) so held
// expressions cannot drag the floor up before the activity gates release.
constexpr float kRestAlpha = 0.0008f;

// Output deadband above rest, in learned noise sigmas. Inside the deadband
// the calibrated output is exactly zero -- this is what keeps idle-face
// noise from becoming a visible expression.
constexpr float kDeadbandSigmas = 3.f;

// The rest window also accepts samples this far above the baseline even when
// sigma is tiny, so a baseline that idles above the initial window (e.g. a
// jaw signal resting at 0.08) can still be reached.
constexpr float kRestWindowMin = 0.1f;

// Never treat a learned span smaller than this as a real range.
constexpr float kMinSpan = 0.05f;

// Rest-noise variance bounds: sigma in [0.005, 0.1] keeps the deadband
// meaningful (never zero, never eating a third of the range).
constexpr float kVarMin = 2.5e-5f;
constexpr float kVarMax = 0.01f;

// Identity->calibrated blend ramp: full confidence after ~15 s at 120 Hz.
// Persisted confidence is capped on load so a headset refit re-learns.
constexpr float kConfPerSample = 1.f / 1800.f;
constexpr float kConfLoadCap = 0.5f;

// Hold gate: a value must stay above the current envelope for this many
// frames spanning this much time before the envelope starts growing, so a
// single-frame spike never widens the range.
constexpr uint32_t kHoldCountMin = 6;
constexpr float kHoldTimeMs = 80.f;

// Frame-age gate: learn only from fresh samples.
constexpr float kFrameAgeMaxMs = 33.f;

// Spike gate: reject an isolated single-frame discontinuity. The previous
// raw sample is recorded whether or not the sample is accepted, so a genuine
// sustained step is only skipped for one frame and can never be locked out.
constexpr float kSpikeGate = 0.75f;

// A slot is "slow-moving" (eligible for rest learning) when its frame-to-
// frame step is below this.
constexpr float kSlowDelta = 0.05f;

// Face-still detector: jaw + mouth-close both stepping less than this for
// this many consecutive expression frames.
constexpr float kStillDelta = 0.02f;
constexpr uint32_t kStillFrames = 30;

// Idle output above this counts as a false activation in telemetry.
constexpr float kIdleActThreshold = 0.1f;

constexpr int kIdxJawOpen = 26;
constexpr int kIdxMouthClose = 40;

inline float Clamp01(float v)
{
	return std::max(0.f, std::min(1.f, v));
}

} // namespace

// -----------------------------------------------------------------------
// ShapeCalib
// -----------------------------------------------------------------------

ShapeCalib::ShapeCalib()
{
	rest = 0.f;
	var = 4e-4f; // sigma seed 0.02
	hi = 1.f;    // identity gain until real peaks are observed
	conf = 0.f;
	sigma = 0.02f;
	gain = 1.f;
	prev_raw = 0.f;
	prev_valid = false;
	hold_count = 0;
	hold_first_qpc = 0;
}

void ShapeCalib::RefreshCaches(float cap)
{
	var = std::max(kVarMin, std::min(kVarMax, var));
	sigma = std::sqrt(var);
	if (rest > hi) rest = hi;
	const float span = std::max(hi - rest, kMinSpan);
	gain = std::min(1.f / span, cap);
}

// -----------------------------------------------------------------------
// CalibrationEngine -- learning
// -----------------------------------------------------------------------

LARGE_INTEGER CalibrationEngine::QpcFreq() const
{
	if (qpc_freq_.QuadPart == 0) {
		QueryPerformanceFrequency(&qpc_freq_);
	}
	return qpc_freq_;
}

void CalibrationEngine::UpdateShape(ShapeCalib& s, float raw, uint64_t now_qpc, float cap, bool rest_frozen) const
{
	if (s.prev_valid && std::abs(raw - s.prev_raw) > kSpikeGate) {
		s.prev_raw = raw;
		return;
	}
	const float step = s.prev_valid ? std::abs(raw - s.prev_raw) : 1.f;
	s.prev_raw = raw;
	s.prev_valid = true;

	const float rest_window = s.rest + std::max(kDeadbandSigmas * s.sigma, kRestWindowMin);

	// Max envelope: hold-gated growth, active-only contraction.
	if (raw > s.hi) {
		if (s.hold_count == 0) s.hold_first_qpc = now_qpc;
		s.hold_count++;
		const float held_ms = (now_qpc > s.hold_first_qpc)
		                          ? (float)(now_qpc - s.hold_first_qpc) * 1000.f / (float)QpcFreq().QuadPart
		                          : 0.f;
		if (s.hold_count >= kHoldCountMin && held_ms >= kHoldTimeMs) {
			s.hi += kAlphaUp * (raw - s.hi);
		}
	}
	else {
		s.hold_count = 0;
		s.hold_first_qpc = 0;
		if (raw > rest_window) {
			s.hi += kAlphaDown * (raw - s.hi);
		}
	}

	// Rest baseline + noise variance: only slow samples inside the window.
	if (!rest_frozen && step < kSlowDelta && raw < rest_window) {
		s.rest += kRestAlpha * (raw - s.rest);
		const float d = raw - s.rest;
		s.var += kRestAlpha * (d * d - s.var);
		s.var = std::max(kVarMin, std::min(kVarMax, s.var));
		s.sigma = std::sqrt(s.var);
	}
	if (s.rest > s.hi) s.rest = s.hi;

	s.conf = std::min(1.f, s.conf + kConfPerSample);

	const float span = std::max(s.hi - s.rest, kMinSpan);
	s.gain = std::min(1.f / span, cap);
}

namespace {

// Slot value in the calibration domain (openness inverted to closedness).
inline float SlotDomainValue(const protocol::FaceTrackingFrameBody& f, int idx)
{
	switch (idx) {
		case calib_table::kIdxOpenL:
			return 1.f - f.eye_openness_l;
		case calib_table::kIdxOpenR:
			return 1.f - f.eye_openness_r;
		case calib_table::kIdxPupilL:
			return f.pupil_dilation_l;
		case calib_table::kIdxPupilR:
			return f.pupil_dilation_r;
		default:
			return f.expressions[idx];
	}
}

inline bool SlotFamilyValid(uint32_t flags, int idx)
{
	return (idx >= calib_table::kIdxOpenL) ? (flags & 1u) != 0 : (flags & 2u) != 0;
}

} // namespace

void CalibrationEngine::IngestFrame(const protocol::FaceTrackingFrameBody& in)
{
	std::lock_guard<std::mutex> lk(mutex_);
	if (!loaded_) return;

	LARGE_INTEGER nowQpc{};
	QueryPerformanceCounter(&nowQpc);
	const float age_ms = (nowQpc.QuadPart > (int64_t)in.qpc_sample_time)
	                         ? (float)(nowQpc.QuadPart - in.qpc_sample_time) * 1000.f / (float)QpcFreq().QuadPart
	                         : 0.f;
	if (age_ms > kFrameAgeMaxMs) return;

	const uint64_t now_qpc = (uint64_t)nowQpc.QuadPart;

	// Face-still detector feeds the lower-face rest gate. Uses the jaw and
	// mouth-close slots' previous samples before this frame updates them.
	if (in.flags & 2u) {
		const ShapeCalib& jaw = shapes_[kIdxJawOpen];
		const ShapeCalib& mc = shapes_[kIdxMouthClose];
		const bool both_prev = jaw.prev_valid && mc.prev_valid;
		const bool still = both_prev && std::abs(in.expressions[kIdxJawOpen] - jaw.prev_raw) < kStillDelta &&
		                   std::abs(in.expressions[kIdxMouthClose] - mc.prev_raw) < kStillDelta;
		still_streak_ = still ? still_streak_ + 1 : 0;
	}
	const bool face_still = still_streak_ >= kStillFrames;

	for (int i = 0; i < kTotalShapes; ++i) {
		if (kPairCanonical[i] != i) continue; // mirrors feed their canonical
		if (kGainCap[i] <= 1.f) continue;     // excluded tier never learns
		if (!SlotFamilyValid(in.flags, i)) continue;

		float v = SlotDomainValue(in, i);
		const int other = calib_table::PairOther(i);
		if (other != i) v = std::max(v, SlotDomainValue(in, other));

		const bool rest_frozen = kLowerFace[i] && !face_still;
		UpdateShape(shapes_[i], v, now_qpc, kGainCap[i], rest_frozen);
	}
}

// -----------------------------------------------------------------------
// CalibrationEngine -- application
// -----------------------------------------------------------------------

float CalibrationEngine::NormalizeOne(const ShapeCalib& s, float raw_domain)
{
	const float rc = Clamp01(raw_domain);
	const float x = rc - s.rest - kDeadbandSigmas * s.sigma;
	const float norm = (x > 0.f) ? Clamp01(x * s.gain) : 0.f;
	const float out = rc + s.conf * (norm - rc);
	if (x <= 0.f) {
		++idle_frames_;
		if (out > kIdleActThreshold) ++idle_false_act_;
	}
	return out;
}

void CalibrationEngine::Normalize(protocol::FaceTrackingFrameBody& inout)
{
	std::lock_guard<std::mutex> lk(mutex_);
	if (!loaded_) return;

	if (inout.flags & 1u) {
		const ShapeCalib& open = shapes_[calib_table::kIdxOpenL]; // shared pair state
		const float outL = NormalizeOne(open, 1.f - inout.eye_openness_l);
		const float outR = NormalizeOne(open, 1.f - inout.eye_openness_r);
		inout.eye_openness_l = 1.f - outL;
		inout.eye_openness_r = 1.f - outR;
		open_lr_div_sum_ += std::abs(outL - outR);
		++open_lr_div_count_;
		// Pupil slots are excluded by tier -- untouched.
	}
	if (inout.flags & 2u) {
		for (int i = 0; i < (int)protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
			if (kGainCap[i] <= 1.f) continue;
			if (user_excluded_[i]) continue;
			inout.expressions[i] = NormalizeOne(shapes_[kPairCanonical[i]], inout.expressions[i]);
		}
	}
}

// -----------------------------------------------------------------------
// CalibrationEngine -- control
// -----------------------------------------------------------------------

void CalibrationEngine::Reset(protocol::FaceCalibrationOp op)
{
	std::lock_guard<std::mutex> lk(mutex_);
	switch (op) {
		case protocol::FaceCalibResetAll:
			for (auto& s : shapes_)
				s = ShapeCalib{};
			still_streak_ = 0;
			{
				std::wstring path = CalibFilePath();
				if (!path.empty()) DeleteFileW(path.c_str());
			}
			FT_LOG_DRV("[calib] reset all shapes", 0);
			break;
		case protocol::FaceCalibResetEye:
			shapes_[calib_table::kIdxOpenL] = ShapeCalib{};
			shapes_[calib_table::kIdxOpenR] = ShapeCalib{};
			shapes_[calib_table::kIdxPupilL] = ShapeCalib{};
			shapes_[calib_table::kIdxPupilR] = ShapeCalib{};
			FT_LOG_DRV("[calib] reset eye shapes", 0);
			break;
		case protocol::FaceCalibResetExpr:
			for (int i = 0; i < (int)protocol::FACETRACKING_EXPRESSION_COUNT; ++i)
				shapes_[i] = ShapeCalib{};
			still_streak_ = 0;
			FT_LOG_DRV("[calib] reset expression shapes", 0);
			break;
		case protocol::FaceCalibSave:
			SaveLocked();
			break;
		default:
			// Begin/End: always-learning while enabled; nothing to do.
			break;
	}
}

void CalibrationEngine::SetUserExcluded(uint32_t expression_index, bool excluded)
{
	std::lock_guard<std::mutex> lk(mutex_);
	if (expression_index < protocol::FACETRACKING_EXPRESSION_COUNT) {
		user_excluded_[expression_index] = excluded;
	}
}

void CalibrationEngine::ClearUserExclusions()
{
	std::lock_guard<std::mutex> lk(mutex_);
	user_excluded_.fill(false);
}

CalibrationEngine::Telemetry CalibrationEngine::SnapshotLocked() const
{
	Telemetry t;
	t.loaded = loaded_;
	float sum = 0.f;
	float mn = 1.f;
	int n = 0;
	for (int i = 0; i < kTotalShapes; ++i) {
		if (kPairCanonical[i] != i || kGainCap[i] <= 1.f) continue;
		const ShapeCalib& s = shapes_[i];
		sum += s.conf;
		mn = std::min(mn, s.conf);
		if (std::max(s.hi - s.rest, kMinSpan) < 1.f / kGainCap[i]) ++t.capped_shapes;
		++n;
	}
	t.avg_conf = (n > 0) ? sum / (float)n : 0.f;
	t.min_conf = (n > 0) ? mn : 0.f;
	t.idle_frames = idle_frames_;
	t.idle_false_act = idle_false_act_;
	t.open_lr_div_avg = (open_lr_div_count_ > 0) ? open_lr_div_sum_ / (float)open_lr_div_count_ : 0.f;
	return t;
}

CalibrationEngine::Telemetry CalibrationEngine::Snapshot() const
{
	std::lock_guard<std::mutex> lk(mutex_);
	return SnapshotLocked();
}

void CalibrationEngine::ResetPeriodCounters()
{
	std::lock_guard<std::mutex> lk(mutex_);
	idle_frames_ = 0;
	idle_false_act_ = 0;
	open_lr_div_sum_ = 0.f;
	open_lr_div_count_ = 0;
}

std::string CalibrationEngine::DebugSummary() const
{
	std::lock_guard<std::mutex> lk(mutex_);
	const Telemetry t = SnapshotLocked();
	auto shape = [&](int i) {
		return &shapes_[kPairCanonical[i]];
	};
	const ShapeCalib* open = shape(calib_table::kIdxOpenL);
	const ShapeCalib* jaw = shape(kIdxJawOpen);
	const ShapeCalib* smile = shape(45); // MouthSmileLeft
	const ShapeCalib* brow = shape(14);  // BrowInnerUpLeft
	char buf[512];
	std::snprintf(buf, sizeof buf,
	              "loaded=%d conf avg=%.2f min=%.2f capped=%d idle_act=%u/%u lr_div=%.3f | "
	              "open[r=%.2f hi=%.2f g=%.1f c=%.2f] jaw[r=%.2f hi=%.2f g=%.1f c=%.2f] "
	              "smileL[r=%.2f hi=%.2f g=%.1f c=%.2f] browIL[r=%.2f hi=%.2f g=%.1f c=%.2f]",
	              (int)t.loaded, t.avg_conf, t.min_conf, t.capped_shapes, t.idle_false_act, t.idle_frames,
	              t.open_lr_div_avg, open->rest, open->hi, open->gain, open->conf, jaw->rest, jaw->hi, jaw->gain,
	              jaw->conf, smile->rest, smile->hi, smile->gain, smile->conf, brow->rest, brow->hi, brow->gain,
	              brow->conf);
	return std::string(buf);
}

// -----------------------------------------------------------------------
// Persistence
// -----------------------------------------------------------------------

namespace {

std::wstring ProfilesDir()
{
	return openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
}

std::wstring Utf8ToWide(const std::string& s)
{
	if (s.empty()) return {};
	int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), nullptr, 0);
	std::wstring out(n, L'\0');
	MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), out.data(), n);
	return out;
}

constexpr int kSchemaVersion = 2;
constexpr int kPairLayoutVersion = 1;

} // namespace

std::wstring CalibrationEngine::CalibFilePath() const
{
	if (module_uuid_.empty()) return {};
	std::wstring dir = ProfilesDir();
	if (dir.empty()) return {};
	return dir + L"\\facetracking_calib_" + Utf8ToWide(module_uuid_) + L".json";
}

void CalibrationEngine::SaveLocked() const
{
	std::wstring path = CalibFilePath();
	if (path.empty()) return;
	std::wstring tmp = path + L".tmp";

	picojson::object root;
	root["schema"] = picojson::value((double)kSchemaVersion);
	root["pair_layout_version"] = picojson::value((double)kPairLayoutVersion);
	root["module_uuid"] = picojson::value(module_uuid_);

	picojson::array arr;
	arr.reserve(kTotalShapes);
	for (int i = 0; i < kTotalShapes; ++i) {
		picojson::object obj;
		// Only canonical calibratable slots carry learned state.
		if (kPairCanonical[i] == i && kGainCap[i] > 1.f) {
			const ShapeCalib& s = shapes_[i];
			obj["rest"] = picojson::value((double)s.rest);
			obj["var"] = picojson::value((double)s.var);
			obj["hi"] = picojson::value((double)s.hi);
			obj["conf"] = picojson::value((double)s.conf);
		}
		arr.push_back(picojson::value(obj));
	}
	root["shapes"] = picojson::value(arr);

	std::string body = picojson::value(root).serialize(true);

	HANDLE hFile = CreateFileW(tmp.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile == INVALID_HANDLE_VALUE) {
		FT_LOG_DRV("[calib] save: cannot open tmp file (err=%lu)", GetLastError());
		return;
	}
	DWORD written = 0;
	BOOL ok = WriteFile(hFile, body.data(), (DWORD)body.size(), &written, nullptr);
	if (ok) ok = FlushFileBuffers(hFile);
	CloseHandle(hFile);

	if (!ok || written != (DWORD)body.size()) {
		FT_LOG_DRV("[calib] save: write failed (err=%lu)", GetLastError());
		DeleteFileW(tmp.c_str());
		return;
	}
	if (!MoveFileExW(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
		FT_LOG_DRV("[calib] save: atomic rename failed (err=%lu)", GetLastError());
		DeleteFileW(tmp.c_str());
	}
}

void CalibrationEngine::LoadLocked(const std::string& uuid)
{
	module_uuid_ = uuid;
	for (auto& s : shapes_)
		s = ShapeCalib{};
	still_streak_ = 0;

	std::wstring path = CalibFilePath();
	if (path.empty()) return;

	std::ifstream in(path);
	if (!in.is_open()) return;

	std::stringstream ss;
	ss << in.rdbuf();
	in.close();
	picojson::value v;
	std::string err = picojson::parse(v, ss.str());
	if (!err.empty() || !v.is<picojson::object>()) {
		FT_LOG_DRV("[calib] load: parse error, discarding file: %s", err.c_str());
		DeleteFileW(path.c_str());
		return;
	}
	const auto& root = v.get<picojson::object>();

	auto getInt = [&](const char* k) -> int {
		auto it = root.find(k);
		return (it != root.end() && it->second.is<double>()) ? (int)it->second.get<double>() : -1;
	};
	if (getInt("schema") != kSchemaVersion || getInt("pair_layout_version") != kPairLayoutVersion) {
		FT_LOG_DRV("[calib] load: stale schema (%d), discarding file", getInt("schema"));
		DeleteFileW(path.c_str());
		return;
	}

	auto shapeIt = root.find("shapes");
	if (shapeIt == root.end() || !shapeIt->second.is<picojson::array>()) return;
	const auto& arr = shapeIt->second.get<picojson::array>();

	int loaded = 0;
	const int count = (int)std::min(arr.size(), (size_t)kTotalShapes);
	for (int i = 0; i < count; ++i) {
		if (kPairCanonical[i] != i || kGainCap[i] <= 1.f) continue;
		if (!arr[i].is<picojson::object>()) continue;
		const auto& obj = arr[i].get<picojson::object>();
		auto getF = [&](const char* k, float& out) {
			auto it = obj.find(k);
			if (it != obj.end() && it->second.is<double>()) out = (float)it->second.get<double>();
		};
		ShapeCalib& s = shapes_[i];
		getF("rest", s.rest);
		getF("var", s.var);
		getF("hi", s.hi);
		getF("conf", s.conf);
		s.rest = Clamp01(s.rest);
		s.hi = Clamp01(s.hi);
		s.conf = std::min(Clamp01(s.conf), kConfLoadCap);
		s.RefreshCaches(kGainCap[i]);
		++loaded;
	}
	FT_LOG_DRV("[calib] loaded %d canonical shapes from disk", loaded);
}

void CalibrationEngine::Load(const std::string& module_uuid)
{
	std::lock_guard<std::mutex> lk(mutex_);
	LoadLocked(module_uuid);
	loaded_ = true;
}

void CalibrationEngine::Save()
{
	std::lock_guard<std::mutex> lk(mutex_);
	if (!loaded_) return;
	SaveLocked();
}

} // namespace facetracking
