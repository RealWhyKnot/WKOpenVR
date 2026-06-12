#pragma once

#include "PhantomTypes.h"
#include "RoleCatalog.h"

#include <windows.h>

#include <atomic>
#include <cstdint>

namespace phantom {

// Per-device phantom state published from the driver at ~10 Hz so the
// overlay can render a status badge per tracker without re-deriving the
// ladder from raw poses. Single writer (driver) / single reader (overlay).
//
// k_unMaxTrackedDeviceCount is 64; per-device record is small so the entire
// table is < 4 KB. Each writer-update is a relaxed memcpy of the per-device
// slot, with a seqlock-style epoch counter so a reader mid-copy sees a
// stable snapshot or retries.

constexpr uint32_t kMaxPhantomDevices = 64;

struct PhantomDeviceState
{
	// Bumped before and after each slot write. Even = stable, odd = writing.
	// The reader copies, checks epoch matches even-before, retries on miss.
	uint32_t epoch;

	// Current ladder state.
	uint8_t state; // TrackerState

	// True if the user opted this device in for dropout bridging. Drives the
	// badge colour ("watched" vs "ignored").
	uint8_t opted_in;

	// Passive role inference output for this physical tracker: the BodyRole the
	// motion-based inference currently believes it sits on, and whether that
	// belief has been auto-applied to the live role. inferred_confidence (below)
	// carries the strength. inferred_role == None when there is no estimate yet.
	uint8_t inferred_role;    // BodyRole
	uint8_t inferred_applied; // 1 if the driver auto-adopted this role

	// Cumulative count of dropout events on this device since session start.
	// A "dropout event" is a REAL -> BLEND_OUT transition.
	uint32_t dropout_count;

	// Milliseconds since the most recent dropout started, while the ladder
	// is active. 0 when state == REAL. Resets at REAL re-entry.
	uint32_t dropout_age_ms;

	// Longest single dropout duration this session, in milliseconds. Lets
	// the overlay show "longest" without keeping its own history.
	uint32_t longest_dropout_ms;

	// Length of serial[] string, capped at kMaxSerialLen-1. 0 if device has
	// never been seen (slot is empty); reader uses this as the "valid" gate.
	uint32_t serial_len;

	static constexpr uint32_t kMaxSerialLen = 64;
	char serial[kMaxSerialLen];

	// Confidence in [0,1] backing inferred_role. Appended after serial[] to
	// keep the older fixed prefix byte-stable.
	float inferred_confidence;
};

struct PhantomRoleCompletionState
{
	// Bumped before and after each role write. Even = stable, odd = writing.
	uint32_t epoch;

	uint8_t role; // BodyRole
	uint8_t valid;
	uint8_t solver_mode; // BodyCompletionMode wire value
	uint8_t _pad0;

	uint16_t source_mask;
	uint16_t _pad1;

	float confidence;
	uint32_t age_ms;
	double position[3];
};

struct PhantomStateShmemLayout
{
	// Sanity guard for shape changes; reader verifies before trusting layout.
	uint32_t magic;        // 'PHST'
	uint32_t version;      // bump on layout change
	uint32_t device_count; // kMaxPhantomDevices
	uint32_t _reserved;

	PhantomDeviceState devices[kMaxPhantomDevices];
	PhantomRoleCompletionState roles[kBodyRoleCount];
};

static_assert(sizeof(PhantomStateShmemLayout) < 8192,
              "PhantomStateShmemLayout must fit comfortably in a single 4 KB page family");

constexpr uint32_t kPhantomStateShmemMagic = 0x54534850; // 'PHST' little-endian
constexpr uint32_t kPhantomStateShmemVersion = 3;        // v3 adds passive role inference fields

// Thin RAII wrapper around the named-shmem mapping. Driver calls Create,
// overlay calls Open; both call Close on shutdown.
class PhantomStateShmem
{
public:
	PhantomStateShmem() = default;
	~PhantomStateShmem() { Close(); }

	PhantomStateShmem(const PhantomStateShmem&) = delete;
	PhantomStateShmem& operator=(const PhantomStateShmem&) = delete;

	// Returns true on success. On failure, layout() returns nullptr and the
	// owner must treat phantom-state publishing as disabled (non-fatal).
	bool Create(const char* segmentName);
	bool Open(const char* segmentName);
	void Close();

	PhantomStateShmemLayout* layout() { return layout_; }
	const PhantomStateShmemLayout* layout() const { return layout_; }

private:
	HANDLE mapping_ = nullptr;
	PhantomStateShmemLayout* layout_ = nullptr;
};

} // namespace phantom
