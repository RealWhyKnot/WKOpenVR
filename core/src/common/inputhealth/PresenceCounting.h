#pragma once

// Pure-function counting helpers for the InputHealth status summary.
// Kept in core/src/common so the overlay plugin and the unit tests
// share one source of truth -- the counting rules are subtle (path
// classification + zero-hash slot guard + ph_triggered alarm count) and the
// bug history is real (inflated controller counts when these rules were
// ad-hoc inside the plugin).
//
// Header-only: no SteamVR, no UI, no IPC. The plugin instantiates a
// std::unordered_set<uint64_t> per call; the test suite synthesizes
// InputHealthSnapshotBody arrays directly.

#include "Protocol.h"
#include "inputhealth/PathClassifier.h"

#include <cstddef>
#include <string>
#include <unordered_set>
#include <utility>

namespace inputhealth {

struct PresenceCounts
{
	// Distinct device_serial_hash values seen across compensation-eligible
	// slots. Zero is excluded -- slots in that state are counted under
	// zero_hash_slots instead.
	int devices = 0;

	// Compensation-eligible paths: Trigger | StickAxis | ControllerButton.
	// These are the paths the InputHealth feature actually acts on.
	int compensation_paths = 0;

	// Eye / face diagnostic-only paths. Visible in the diagnostics UI but
	// not learned into compensation. Counted separately so the presence
	// string can mention them without inflating "devices".
	int diagnostic_paths = 0;

	// Number of compensation paths with the Page-Hinkley drift alarm fired.
	int warnings = 0;

	// Slots with handle != 0 but device_serial_hash == 0. Should be rare and
	// transient (driver hasn't resolved the serial yet); a steady non-zero
	// count signals a publisher-side bug worth investigating.
	int zero_hash_slots = 0;

	// Slots whose path classifies as Unsupported (/proximity, /pupil, /imu).
	// Tracked for diagnostics so we can confirm the driver is publishing
	// them and the overlay is correctly ignoring them in the count.
	int unsupported_slots = 0;
};

// Decode the NUL-terminated path field of a snapshot body into a std::string.
inline std::string PathFromSnapshotBody(const protocol::InputHealthSnapshotBody& b)
{
	size_t n = 0;
	while (n < protocol::INPUTHEALTH_PATH_LEN && b.path[n] != '\0')
		++n;
	return std::string(b.path, b.path + n);
}

namespace detail {

// Pluck the snapshot body out of an iteration value, supporting:
//   (a) a bare InputHealthSnapshotBody              -> body itself
//   (b) a std::pair<const K, V> where V has .body   -> kv.second.body
//   (c) any V with .body                            -> v.body
//
// Overloads are placed in a nested namespace so the unqualified-name lookup
// inside the CountInputHealthPresence template can find them via ADL on the
// inputhealth namespace.

inline const protocol::InputHealthSnapshotBody& GetSnapshotBody(const protocol::InputHealthSnapshotBody& b)
{
	return b;
}

template <typename K, typename V>
inline const protocol::InputHealthSnapshotBody& GetSnapshotBody(const std::pair<const K, V>& kv)
{
	return kv.second.body;
}

template <typename V> inline auto GetSnapshotBody(const V& v) -> decltype((v.body))
{
	return v.body;
}

} // namespace detail

// Walk the given range of entries and tally presence counts. `entries` is
// any iterable whose value type is either an InputHealthSnapshotBody, a
// `std::pair<const K, V>` where V has a `.body` field (e.g. an unordered_map
// value), or any value with a `.body` field directly.
//
// Slots with handle == 0 are skipped entirely (empty/retired slots).
template <typename Range> inline PresenceCounts CountInputHealthPresence(const Range& entries)
{
	PresenceCounts c{};
	std::unordered_set<uint64_t> distinct_serials;

	for (const auto& item : entries) {
		const protocol::InputHealthSnapshotBody& body = detail::GetSnapshotBody(item);
		if (body.handle == 0) continue;

		const std::string path = PathFromSnapshotBody(body);
		const PathClass cls = ClassifyInputPath(path);

		if (cls == PathClass::Unsupported) {
			++c.unsupported_slots;
			continue;
		}
		if (cls == PathClass::DiagnosticsOnly) {
			++c.diagnostic_paths;
			continue;
		}

		++c.compensation_paths;
		if (body.ph_triggered) ++c.warnings;

		if (body.device_serial_hash == 0) {
			++c.zero_hash_slots;
		}
		else {
			distinct_serials.insert(body.device_serial_hash);
		}
	}

	c.devices = static_cast<int>(distinct_serials.size());
	return c;
}

} // namespace inputhealth
