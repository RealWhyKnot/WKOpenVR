#include "FaceFrameReader.h"
#include "Logging.h"
#include "facetracking/UpstreamShapeMap.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <cstring>
#include <limits>

namespace facetracking {

namespace {

// Translate a wire-format frame (88 upstream shapes) into our consumer-
// facing frame (63 internal shapes). Every non-expression field is a
// direct copy; expressions are remapped via the static name-match table
// in UpstreamShapeMap.h.
void TranslateWireBody(const protocol::FaceTrackingFrameBodyWire& wire, protocol::FaceTrackingFrameBody& out)
{
	out.qpc_sample_time = wire.qpc_sample_time;
	out.source_module_uuid_hash = wire.source_module_uuid_hash;
	std::memcpy(out.eye_origin_l, wire.eye_origin_l, sizeof(out.eye_origin_l));
	std::memcpy(out.eye_origin_r, wire.eye_origin_r, sizeof(out.eye_origin_r));
	std::memcpy(out.eye_gaze_l, wire.eye_gaze_l, sizeof(out.eye_gaze_l));
	std::memcpy(out.eye_gaze_r, wire.eye_gaze_r, sizeof(out.eye_gaze_r));
	out.eye_openness_l = wire.eye_openness_l;
	out.eye_openness_r = wire.eye_openness_r;
	out.pupil_dilation_l = wire.pupil_dilation_l;
	out.pupil_dilation_r = wire.pupil_dilation_r;
	out.eye_confidence_l = wire.eye_confidence_l;
	out.eye_confidence_r = wire.eye_confidence_r;

	// Zero our 63-slot array first; the remap fills direct and aliased
	// upstream equivalents, and slots with no upstream source stay at 0.
	for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i)
		out.expressions[i] = 0.f;
	facetracking::RemapUpstreamShapes(wire.expressions, out.expressions);
	for (uint32_t i = 0; i < protocol::FACETRACKING_UPSTREAM_EXPRESSION_COUNT; ++i) {
		out.upstream_expressions[i] = facetracking::ClampUpstreamUnitSignal(wire.expressions[i]);
	}

	out.flags = wire.flags;
	out.head_yaw = wire.head_yaw;
	out.head_pitch = wire.head_pitch;
	out.head_roll = wire.head_roll;
	out.head_pos_x = wire.head_pos_x;
	out.head_pos_y = wire.head_pos_y;
	out.head_pos_z = wire.head_pos_z;
	out.head_flags = wire.head_flags;
}

} // namespace

namespace {

uint64_t QpcNow()
{
	LARGE_INTEGER c{};
	QueryPerformanceCounter(&c);
	return static_cast<uint64_t>(c.QuadPart);
}

uint64_t QpcFrequency()
{
	static uint64_t freq = []() -> uint64_t {
		LARGE_INTEGER f{};
		QueryPerformanceFrequency(&f);
		return static_cast<uint64_t>(f.QuadPart);
	}();
	return freq;
}

} // namespace

FaceFrameReader::FaceFrameReader() = default;
FaceFrameReader::~FaceFrameReader()
{
	Close();
}

bool FaceFrameReader::Create(LPCSTR name)
{
	if (!shmem_.Create(name)) {
		FT_LOG_DRV("[reader] failed to create shmem segment '%s' (err=%lu)", name, (unsigned long)GetLastError());
		return false;
	}
	FT_LOG_DRV("[reader] created shmem segment '%s'", name);
	return true;
}

void FaceFrameReader::Open(LPCSTR name)
{
	try {
		shmem_.Open(name);
		FT_LOG_DRV("[reader] opened shmem segment '%s'", name);
	}
	catch (const std::exception& ex) {
		FT_LOG_DRV("[reader] failed to open shmem '%s': %s", name, ex.what());
	}
}

void FaceFrameReader::Close()
{
	shmem_.Close();
	last_index_ = 0;
}

bool FaceFrameReader::IsOpen() const
{
	return (bool)shmem_;
}

bool FaceFrameReader::TryRead(protocol::FaceTrackingFrameBody& out)
{
	if (!shmem_) return false;
	protocol::FaceTrackingFrameBodyWire wire;
	if (!shmem_.TryReadLatestWire(wire)) return false;
	TranslateWireBody(wire, out);
	return true;
}

uint64_t FaceFrameReader::LastPublishIndex() const
{
	return shmem_.PublishIndex();
}

uint32_t FaceFrameReader::HostState() const
{
	if (!shmem_) return protocol::HostStateLegacy;
	return shmem_.HostStateField();
}

uint64_t FaceFrameReader::HeartbeatAgeMs() const
{
	if (!shmem_) return std::numeric_limits<uint64_t>::max();
	const uint64_t hb = shmem_.HostHeartbeatQpc();
	if (hb == 0) return std::numeric_limits<uint64_t>::max();
	const uint64_t now = QpcNow();
	if (now <= hb) return 0; // QPC went backwards or clock-sync edge; treat as fresh.
	const uint64_t freq = QpcFrequency();
	if (freq == 0) return std::numeric_limits<uint64_t>::max();
	return (now - hb) * 1000ULL / freq;
}

void FaceFrameReader::ResetHostLiveness()
{
	if (!shmem_) return;
	shmem_.ResetHostLiveness();
}

} // namespace facetracking
