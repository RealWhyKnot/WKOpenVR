#pragma once

#include "PhantomTypes.h"
#include "RoleCatalog.h"

#include <windows.h>

#include <atomic>
#include <cstdint>

namespace phantom {

// Driver -> sidecar inference inputs. Updated by the driver every time
// the HMD pose updates; the sidecar reads the latest slot, runs inference,
// and writes the result to PhantomInferenceOutShmem. Single writer
// (driver) / single reader (sidecar) with a seqlock-style epoch counter
// for tear-free reads.
//
// Phase 3 ships the IPC + sidecar process scaffold only; the sidecar is
// a passthrough stub until an ONNX Runtime + SparsePoser model land
// alongside the AMASS / SMPL licensing review noted in the plan.

constexpr uint32_t kPhantomInferenceShmemMagic = 0x49484850; // 'PHIN'
constexpr uint32_t kPhantomInferenceShmemVersion = 1;

struct PhantomTrackerInput
{
	uint8_t role;          // BodyRole value; None = slot unused
	uint8_t has_real_pose; // 1 if the real tracker is currently online
	uint8_t has_offset;    // 1 if a T-pose calibration is on file
	uint8_t _pad[5];
	// Live pose for the device when has_real_pose; otherwise the most
	// recent observation. Frame of reference is world (SteamVR
	// standing universe), same as the driver consumes.
	double position[3];
	double rotation[4]; // quaternion w, x, y, z
};

struct PhantomInferenceInLayout
{
	uint32_t magic;
	uint32_t version;
	uint32_t epoch;    // seqlock counter: even = stable, odd = writing
	uint32_t frame_id; // monotonic per write

	// HMD pose for the inference frame. Sidecar uses it as the input
	// reference frame.
	double hmd_position[3];
	double hmd_rotation[4];

	// Per-role tracker inputs. Indexed by BodyRole (size kBodyRoleCount).
	PhantomTrackerInput trackers[kBodyRoleCount];
};

// Sidecar -> driver inference outputs. Per-role completed pose +
// per-role confidence (0..1). The driver consults global_confidence
// before honouring the output; below threshold it falls through to IK
// (Phase 1.5) or dead reckoning (Phase 1).
struct PhantomTrackerOutput
{
	uint8_t role;
	uint8_t valid; // 1 = the sidecar produced a pose for this role
	uint8_t _pad[6];
	double position[3];
	double rotation[4];
	float confidence; // 0..1; per-role; sidecar fills in
	float _reserved;
};

struct PhantomInferenceOutLayout
{
	uint32_t magic;
	uint32_t version;
	uint32_t epoch;
	uint32_t frame_id;       // mirrors the input frame_id this output was
	                         // computed for; lets the driver match input
	                         // and output deterministically.
	float global_confidence; // 0..1; <threshold -> driver ignores output
	float _reserved;
	uint64_t sidecar_qpc_ns; // when the sidecar finished inference

	PhantomTrackerOutput trackers[kBodyRoleCount];
};

static_assert(sizeof(PhantomInferenceInLayout) < 8192,
              "PhantomInferenceInLayout should fit comfortably in one page family");
static_assert(sizeof(PhantomInferenceOutLayout) < 8192,
              "PhantomInferenceOutLayout should fit comfortably in one page family");

// RAII wrapper around the input segment (driver owns Create, sidecar Opens).
class PhantomInferenceInShmem
{
public:
	PhantomInferenceInShmem() = default;
	~PhantomInferenceInShmem() { Close(); }
	PhantomInferenceInShmem(const PhantomInferenceInShmem&) = delete;
	PhantomInferenceInShmem& operator=(const PhantomInferenceInShmem&) = delete;

	bool Create(const char* name);
	bool Open(const char* name);
	void Close();
	PhantomInferenceInLayout* layout() { return layout_; }
	const PhantomInferenceInLayout* layout() const { return layout_; }

private:
	HANDLE mapping_ = nullptr;
	PhantomInferenceInLayout* layout_ = nullptr;
};

// RAII wrapper around the output segment (sidecar owns Create, driver Opens).
class PhantomInferenceOutShmem
{
public:
	PhantomInferenceOutShmem() = default;
	~PhantomInferenceOutShmem() { Close(); }
	PhantomInferenceOutShmem(const PhantomInferenceOutShmem&) = delete;
	PhantomInferenceOutShmem& operator=(const PhantomInferenceOutShmem&) = delete;

	bool Create(const char* name);
	bool Open(const char* name);
	void Close();
	PhantomInferenceOutLayout* layout() { return layout_; }
	const PhantomInferenceOutLayout* layout() const { return layout_; }

private:
	HANDLE mapping_ = nullptr;
	PhantomInferenceOutLayout* layout_ = nullptr;
};

} // namespace phantom
