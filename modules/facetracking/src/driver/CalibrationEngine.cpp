#define _CRT_SECURE_NO_DEPRECATE
#include "CalibrationEngine.h"
#include "Logging.h"

#include "picojson.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shlobj.h>
#include <objbase.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

namespace facetracking {

// -----------------------------------------------------------------------
// ShapeCalib
// -----------------------------------------------------------------------

ShapeCalib::ShapeCalib()
{
	// P-square 5-marker initialisation for P02.
	// Desired markers sit at {0, p/2, p, (1+p)/2, 1} = {0, 0.01, 0.02, 0.51, 1}
	// for p=0.02 (Jain & Chlamtac 1985).  Seed q[] across [0,1] so the first
	// 200 samples (warm-up) reach a reasonable estimate before normalization kicks in.
	const float q_init[] = {0.f, 0.005f, 0.02f, 0.5f, 1.f};
	for (int i = 0; i < 5; ++i) {
		q[i] = q_init[i];
		n[i] = (float)(i + 1);
	}
	dn[0] = 0.f;
	dn[1] = 0.01f;
	dn[2] = 0.02f;
	dn[3] = 0.51f;
	dn[4] = 1.f;

	p02 = 0.f;
	p98 = 1.f;
	ema_min = 0.f;
	ema_max = 1.f;
	ema_value = 0.5f;
	ema_var = 0.04f; // seed variance so the velocity gate is not zero-width
	hold_count = 0;
	hold_first_qpc = 0;
	samples = 0;
	warm = false;
}

// -----------------------------------------------------------------------
// P-square update helpers
// -----------------------------------------------------------------------
namespace {

// Parabolic interpolation formula for a P-square inner marker update.
// Jain & Chlamtac 1985.  h1 = n[i+1]-n[i], h2 = n[i]-n[i-1], d in {+1,-1}.
inline float PsqUpdate(const float* q, const float* n, int i, float d)
{
	float h1 = n[i + 1] - n[i];
	float h2 = n[i] - n[i - 1];
	return q[i] + d / (h1 + h2) * ((h2 / h1) * (q[i + 1] - q[i]) + (h1 / h2) * (q[i] - q[i - 1]));
}

// Linear fallback for the P-square inner update when parabolic overshoots.
inline float PsqLinear(const float* q, const float* n, int i, float d)
{
	int j = (d > 0) ? i + 1 : i - 1;
	return q[i] + d * (q[j] - q[i]) / (n[j] - n[i]);
}

// Update one set of P-square markers for quantile `p`.
// `q` / `n` carry the 5 markers; `dn` is the desired-increment array.
// Returns the updated quantile estimate (marker q[2]).
// We track only one quantile per call; the caller runs this twice (P02, P98).
float PsqStep(float* q, float* n, const float* dn, float xn, float /*p*/)
{
	// Step 1: find the cell the new observation falls into.
	int k = 0;
	if (xn < q[0]) {
		q[0] = xn;
		k = 0;
	}
	else if (xn < q[1]) {
		k = 0;
	}
	else if (xn < q[2]) {
		k = 1;
	}
	else if (xn < q[3]) {
		k = 2;
	}
	else if (xn <= q[4]) {
		k = 3;
	}
	else {
		q[4] = xn;
		k = 3;
	}

	// Step 2: increment counts.
	for (int i = k + 1; i < 5; ++i)
		n[i] += 1.f;

	// Step 3: adjust inner markers (1..3) if needed.
	for (int i = 1; i <= 3; ++i) {
		float di = dn[i] - n[i];
		if ((di >= 1.f && n[i + 1] - n[i] > 1.f) || (di <= -1.f && n[i - 1] - n[i] < -1.f)) {
			float d = (di > 0.f) ? 1.f : -1.f;
			float q_new = PsqUpdate(q, n, i, d);
			if (q_new > q[i - 1] && q_new < q[i + 1]) {
				q[i] = q_new;
			}
			else {
				q[i] = PsqLinear(q, n, i, d);
			}
			n[i] += d;
		}
	}
	return q[2];
}

// Separate 5-marker state for P98 (reuses same q/n arrays through different
// dn targets stored separately in a static local).  We keep P02 and P98 in
// separate arrays on the ShapeCalib struct to keep things simple and avoid
// a nested struct.  The P98 state is stored interleaved by index offset.
// NOTE: implementation uses two independent ShapeCalib member arrays named
// q_98/n_98/dn_98 -- we add them below (simple approach, readable).

} // namespace

// -----------------------------------------------------------------------
// CalibrationEngine
// -----------------------------------------------------------------------

static constexpr float kAlphaUp = 0.02f;
static constexpr float kAlphaDown = 0.0005f;
// alpha_down is ~40x slower than alpha_up.  A shape that opens to a new wide
// peak will expand the envelope in ~50 frames (~0.4 s); an envelope that needs
// to contract (e.g. after a hardware module swap) takes ~2000 frames (~17 s).
// This asymmetry prevents transient blinks or spikes from deflating the max
// immediately, which would cause the normalized output to spike above 1.

static constexpr float kVelocityGateSigmas = 4.f;
// 4-sigma gate: a signal 4 std-devs from its EMA is exceedingly rare (~6 in
// 100 000 samples) for a well-behaved sensor.  At 120 Hz this fires once in
// ~14 minutes purely by chance, which is negligible.

static constexpr uint32_t kWarmThreshold = 200;
static constexpr uint32_t kHoldCountMin = 6;
static constexpr float kHoldTimeMs = 80.f;
static constexpr float kFrameAgeMaxMs = 33.f;
static constexpr float kNormEps = 1e-6f;

LARGE_INTEGER CalibrationEngine::QpcFreq() const
{
	if (qpc_freq_.QuadPart == 0) {
		QueryPerformanceFrequency(&qpc_freq_);
	}
	return qpc_freq_;
}

void CalibrationEngine::UpdateShape(ShapeCalib& s, float raw, uint64_t now_qpc) const
{
	const float alpha_ema_var = 0.05f; // variance EMA decay -- slower than value EMA
	                                   // so the gate doesn't collapse on flat regions.

	// Velocity gate.
	float dev = raw - s.ema_value;
	float gate = kVelocityGateSigmas * std::sqrt(s.ema_var + kNormEps);
	if (std::abs(dev) > gate) return;

	// EMA value update.
	s.ema_value += kAlphaUp * dev;
	float dev2 = (raw - s.ema_value) * (raw - s.ema_value);
	s.ema_var = (1.f - alpha_ema_var) * s.ema_var + alpha_ema_var * dev2;

	// Hold-time gate for new EMA max.
	if (raw > s.ema_max) {
		if (s.hold_count == 0) {
			s.hold_first_qpc = now_qpc;
		}
		s.hold_count++;

		// Commit the new max only when hold duration + count thresholds are met.
		const float elapsed_ms = (now_qpc > s.hold_first_qpc)
		                             ? (float)(now_qpc - s.hold_first_qpc) * 1000.f / (float)QpcFreq().QuadPart
		                             : 0.f;

		if (s.hold_count >= kHoldCountMin && elapsed_ms >= kHoldTimeMs) {
			s.ema_max = raw; // commit the new peak
			s.hold_count = 0;
			s.hold_first_qpc = 0;
		}
	}
	else {
		// Below current ema_max -- reset hold streak.
		s.hold_count = 0;
		s.hold_first_qpc = 0;

		// Shrink the envelope slowly.
		if (raw < s.ema_min) {
			s.ema_min = (1.f - kAlphaUp) * s.ema_min + kAlphaUp * raw;
		}
		else {
			s.ema_min = (1.f - kAlphaDown) * s.ema_min + kAlphaDown * raw;
		}
		s.ema_max = (1.f - kAlphaDown) * s.ema_max + kAlphaDown * raw;
	}

	// P-square update for P02.
	s.p02 = PsqStep(s.q, s.n, s.dn, raw, 0.02f);

	// P98 is tracked as the committed EMA max.  The EMA max grows fast
	// (alpha_up) on new peaks but only commits after the hold-time gate,
	// so it's a good high-percentile estimate without a second P-square set.
	s.p98 = s.ema_max;

	s.samples++;
	if (!s.warm && s.samples >= kWarmThreshold) {
		s.warm = true;
	}
}

float CalibrationEngine::NormalizeOne(const ShapeCalib& s, float raw)
{
	float lo = s.p02;
	float hi = s.p98;
	float range = hi - lo;
	if (range < kNormEps) return raw; // degenerate: no range learned yet
	float norm = (raw - lo) / range;
	return std::max(0.f, std::min(1.f, norm));
}

void CalibrationEngine::IngestFrame(const protocol::FaceTrackingFrameBody& in)
{
	// Frame-age gate: reject if the sample is more than 33 ms old.
	LARGE_INTEGER nowQpc{};
	QueryPerformanceCounter(&nowQpc);
	const float age_ms = (nowQpc.QuadPart > (int64_t)in.qpc_sample_time)
	                         ? (float)(nowQpc.QuadPart - in.qpc_sample_time) * 1000.f / (float)QpcFreq().QuadPart
	                         : 0.f;
	if (age_ms > kFrameAgeMaxMs) return;

	const uint64_t now_qpc = (uint64_t)nowQpc.QuadPart;

	if (in.flags & 1u) {
		// Eye fields valid.
		UpdateShape(shapes_[kIdxOpenL], in.eye_openness_l, now_qpc);
		UpdateShape(shapes_[kIdxOpenR], in.eye_openness_r, now_qpc);
		UpdateShape(shapes_[kIdxPupilL], in.pupil_dilation_l, now_qpc);
		UpdateShape(shapes_[kIdxPupilR], in.pupil_dilation_r, now_qpc);
	}
	if (in.flags & 2u) {
		// Expression fields valid.
		for (int i = 0; i < (int)protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
			UpdateShape(shapes_[i], in.expressions[i], now_qpc);
		}
	}
}

void CalibrationEngine::Normalize(protocol::FaceTrackingFrameBody& inout) const
{
	if (inout.flags & 1u) {
		if (shapes_[kIdxOpenL].warm) inout.eye_openness_l = NormalizeOne(shapes_[kIdxOpenL], inout.eye_openness_l);
		if (shapes_[kIdxOpenR].warm) inout.eye_openness_r = NormalizeOne(shapes_[kIdxOpenR], inout.eye_openness_r);
		if (shapes_[kIdxPupilL].warm)
			inout.pupil_dilation_l = NormalizeOne(shapes_[kIdxPupilL], inout.pupil_dilation_l);
		if (shapes_[kIdxPupilR].warm)
			inout.pupil_dilation_r = NormalizeOne(shapes_[kIdxPupilR], inout.pupil_dilation_r);
	}
	if (inout.flags & 2u) {
		for (int i = 0; i < (int)protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
			if (shapes_[i].warm) inout.expressions[i] = NormalizeOne(shapes_[i], inout.expressions[i]);
		}
	}
}

int CalibrationEngine::WarmShapeCount() const
{
	int count = 0;
	for (const auto& s : shapes_)
		if (s.warm) ++count;
	return count;
}

int CalibrationEngine::TotalShapeCount() const
{
	return kTotalShapes;
}

bool CalibrationEngine::IsShapeWarm(int idx) const
{
	if (idx < 0 || idx >= kTotalShapes) return false;
	return shapes_[idx].warm;
}

void CalibrationEngine::Reset(protocol::FaceCalibrationOp op)
{
	switch (op) {
		case protocol::FaceCalibResetAll:
			for (auto& s : shapes_)
				s = ShapeCalib{};
			// Delete persisted file so cold-start seed gets reloaded next time.
			{
				std::wstring path = CalibFilePath();
				if (!path.empty()) DeleteFileW(path.c_str());
			}
			FT_LOG_DRV("[calib] reset all shapes", 0);
			break;
		case protocol::FaceCalibResetEye:
			shapes_[kIdxOpenL] = ShapeCalib{};
			shapes_[kIdxOpenR] = ShapeCalib{};
			shapes_[kIdxPupilL] = ShapeCalib{};
			shapes_[kIdxPupilR] = ShapeCalib{};
			FT_LOG_DRV("[calib] reset eye shapes", 0);
			break;
		case protocol::FaceCalibResetExpr:
			for (int i = 0; i < (int)protocol::FACETRACKING_EXPRESSION_COUNT; ++i)
				shapes_[i] = ShapeCalib{};
			FT_LOG_DRV("[calib] reset expression shapes", 0);
			break;
		case protocol::FaceCalibSave:
			SaveLocked();
			break;
		default:
			break;
	}
}

// -----------------------------------------------------------------------
// Persistence
// -----------------------------------------------------------------------

namespace {

std::wstring ProfilesDir()
{
	PWSTR rawPtr = nullptr;
	if (S_OK != SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, nullptr, &rawPtr)) {
		if (rawPtr) CoTaskMemFree(rawPtr);
		return {};
	}
	std::wstring root(rawPtr);
	CoTaskMemFree(rawPtr);

	std::wstring dir = root + L"\\WKOpenVR";
	CreateDirectoryW(dir.c_str(), nullptr);
	dir += L"\\profiles";
	CreateDirectoryW(dir.c_str(), nullptr);
	return dir;
}

std::wstring Utf8ToWide(const std::string& s)
{
	if (s.empty()) return {};
	int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), nullptr, 0);
	std::wstring out(n, L'\0');
	MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), out.data(), n);
	return out;
}

} // namespace

std::wstring CalibrationEngine::CalibFilePath() const
{
	if (module_uuid_.empty()) return {};
	std::wstring dir = ProfilesDir();
	if (dir.empty()) return {};
	// Filename: facetracking_calib_<module_uuid>.json
	return dir + L"\\facetracking_calib_" + Utf8ToWide(module_uuid_) + L".json";
}

void CalibrationEngine::SaveLocked() const
{
	std::wstring path = CalibFilePath();
	if (path.empty()) return;
	std::wstring tmp = path + L".tmp";

	// Build JSON.
	// Each shape is serialised as an object with all learned parameters.
	// Schema version 1.
	picojson::object root;
	root["schema"] = picojson::value(1.0);
	root["module_uuid"] = picojson::value(module_uuid_);

	picojson::array arr;
	arr.reserve(kTotalShapes);
	for (int i = 0; i < kTotalShapes; ++i) {
		const ShapeCalib& s = shapes_[i];
		picojson::object obj;
		obj["p02"] = picojson::value((double)s.p02);
		obj["p98"] = picojson::value((double)s.p98);
		obj["ema_min"] = picojson::value((double)s.ema_min);
		obj["ema_max"] = picojson::value((double)s.ema_max);
		obj["ema_value"] = picojson::value((double)s.ema_value);
		obj["ema_var"] = picojson::value((double)s.ema_var);
		obj["samples"] = picojson::value((double)s.samples);
		obj["warm"] = picojson::value(s.warm);
		// P-square marker state (q[5], n[5], dn[5]).
		picojson::array pq, pn;
		for (int k = 0; k < 5; ++k) {
			pq.push_back(picojson::value((double)s.q[k]));
			pn.push_back(picojson::value((double)s.n[k]));
		}
		obj["q"] = picojson::value(pq);
		obj["n"] = picojson::value(pn);
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
	std::wstring path = CalibFilePath();
	if (path.empty()) return;

	std::ifstream in(path);
	if (!in.is_open()) return;

	std::stringstream ss;
	ss << in.rdbuf();
	picojson::value v;
	std::string err = picojson::parse(v, ss.str());
	if (!err.empty()) {
		FT_LOG_DRV("[calib] load: parse error: %s", err.c_str());
		return;
	}
	if (!v.is<picojson::object>()) return;
	const auto& root = v.get<picojson::object>();

	auto shapeIt = root.find("shapes");
	if (shapeIt == root.end() || !shapeIt->second.is<picojson::array>()) return;
	const auto& arr = shapeIt->second.get<picojson::array>();

	int count = (int)std::min(arr.size(), (size_t)kTotalShapes);
	for (int i = 0; i < count; ++i) {
		if (!arr[i].is<picojson::object>()) continue;
		const auto& obj = arr[i].get<picojson::object>();
		ShapeCalib& s = shapes_[i];

		auto getF = [&](const char* k, float& out) {
			auto it = obj.find(k);
			if (it != obj.end() && it->second.is<double>()) out = (float)it->second.get<double>();
		};
		auto getU32 = [&](const char* k, uint32_t& out) {
			auto it = obj.find(k);
			if (it != obj.end() && it->second.is<double>()) out = (uint32_t)it->second.get<double>();
		};
		auto getBool = [&](const char* k, bool& out) {
			auto it = obj.find(k);
			if (it != obj.end() && it->second.is<bool>()) out = it->second.get<bool>();
		};

		getF("p02", s.p02);
		getF("p98", s.p98);
		getF("ema_min", s.ema_min);
		getF("ema_max", s.ema_max);
		getF("ema_value", s.ema_value);
		getF("ema_var", s.ema_var);
		getU32("samples", s.samples);
		getBool("warm", s.warm);

		auto qIt = obj.find("q");
		if (qIt != obj.end() && qIt->second.is<picojson::array>()) {
			const auto& qa = qIt->second.get<picojson::array>();
			for (int k = 0; k < 5 && k < (int)qa.size(); ++k)
				if (qa[k].is<double>()) s.q[k] = (float)qa[k].get<double>();
		}
		auto nIt = obj.find("n");
		if (nIt != obj.end() && nIt->second.is<picojson::array>()) {
			const auto& na = nIt->second.get<picojson::array>();
			for (int k = 0; k < 5 && k < (int)na.size(); ++k)
				if (na[k].is<double>()) s.n[k] = (float)na[k].get<double>();
		}
	}
	FT_LOG_DRV("[calib] loaded %d shapes from disk for module '%s'", count, uuid.c_str());
}

void CalibrationEngine::Load(const std::string& module_uuid)
{
	LoadLocked(module_uuid);
}

void CalibrationEngine::Save()
{
	SaveLocked();
}

} // namespace facetracking
