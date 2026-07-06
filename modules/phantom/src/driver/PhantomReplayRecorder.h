#pragma once

#include "PhantomReplayBudget.h"
#include "RecordingEnvelope.h"
#include "RoleCatalog.h"

#include <openvr_driver.h>

#include <cstdint>
#include <string>
#include <unordered_map>

namespace phantom {

class PhantomReplayRecorder
{
public:
	void RecordPose(int64_t qpc_ticks, int64_t qpc_freq, uint32_t openvr_id, const std::string& serial,
	                vr::ETrackedDeviceClass device_class, vr::ETrackedControllerRole controller_role,
	                BodyRole body_role, bool dropout_enabled, const vr::DriverPose_t& pose);

	// Ages out old recordings by count and total size. Runs at module init,
	// independent of whether recording is enabled this session, so a machine
	// that stops recording still reclaims the disk.
	static void PruneOnInit();

private:
	bool ShouldRecord();
	bool OpenIfNeeded();

	bool checked_enabled_ = false;
	bool enabled_ = false;
	bool open_attempted_ = false;
	const char* full_rate_source_ = "off";
	openvr_pair::common::recording::RecordingEnvelope envelope_;
	ReplayBudgetConfig budget_;
	std::unordered_map<uint32_t, DeviceRecordState> device_state_;
	int64_t first_qpc_ = 0;
	uint64_t rows_written_ = 0;
	uint64_t rows_suppressed_ = 0;
	double last_counter_annotation_ms_ = 0.0;
};

} // namespace phantom
