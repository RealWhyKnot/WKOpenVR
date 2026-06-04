#include "SnapshotReader.h"

#include "Logging.h"

#include <chrono>
#include <cstring>
#include <exception>
#include <string_view>

bool SnapshotReader::TryOpen()
{
	try {
		shmem_.Open(OPENVR_PAIRDRIVER_INPUTHEALTH_SHMEM_NAME);
		last_error_.clear();
		last_error_is_version_mismatch_ = false;
		entries_by_handle_.clear();
		last_publish_tick_ = shmem_.LoadPublishTick();
		last_publish_tick_change_ = std::chrono::steady_clock::now();
		LOG("[snapshot] opened '%s'", OPENVR_PAIRDRIVER_INPUTHEALTH_SHMEM_NAME);
		return true;
	}
	catch (const std::exception& e) {
		last_error_ = e.what();
		// Distinguish a driver/overlay version-skew from a transient "not
		// ready yet" state. The shmem Open() throws with "mismatch" in the
		// message for both magic and version failures.
		last_error_is_version_mismatch_ = std::string_view(last_error_).find("mismatch") != std::string_view::npos;
		return false;
	}
}

void SnapshotReader::Close()
{
	shmem_.Close();
	entries_by_handle_.clear();
	last_publish_tick_ = 0;
	last_publish_tick_change_ = {};
}

void SnapshotReader::Refresh()
{
	if (!shmem_) {
		// Try a transparent re-open: the driver may not have been ready
		// when the overlay started. TryOpen() updates last_error_ on
		// failure so the UI can surface it.
		TryOpen();
		if (!shmem_) return;
	}

	const uint64_t publish_tick = shmem_.LoadPublishTick();
	const auto now = std::chrono::steady_clock::now();
	if (publish_tick != last_publish_tick_) {
		last_publish_tick_ = publish_tick;
		last_publish_tick_change_ = now;
	}
	else if (last_publish_tick_change_ != std::chrono::steady_clock::time_point{} &&
	         now - last_publish_tick_change_ > std::chrono::seconds(2)) {
		LOG("[snapshot] publish tick stale for >2s; closing mapping so it can reopen after driver restart");
		last_error_ = "InputHealth shmem stale; waiting for driver restart";
		Close();
		return;
	}

	const uint32_t slot_count = protocol::INPUTHEALTH_SLOT_COUNT;
	for (uint32_t i = 0; i < slot_count; ++i) {
		protocol::InputHealthSnapshotBody body;
		if (!shmem_.TryReadSlot(i, body)) continue;
		if (body.handle == 0) continue;
		auto& entry = entries_by_handle_[body.handle];
		std::memcpy(&entry.body, &body, sizeof(body));
		entry.last_seen_publish_tick = last_publish_tick_;
	}
}
