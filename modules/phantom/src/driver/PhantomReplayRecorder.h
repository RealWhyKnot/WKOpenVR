#pragma once

#include "RoleCatalog.h"

#include <openvr_driver.h>

#include <cstdint>
#include <fstream>
#include <string>

namespace phantom {

class PhantomReplayRecorder
{
public:
	void RecordPose(int64_t qpc_ticks, int64_t qpc_freq, uint32_t openvr_id, const std::string& serial,
	                vr::ETrackedDeviceClass device_class, vr::ETrackedControllerRole controller_role,
	                BodyRole body_role, bool dropout_enabled, const vr::DriverPose_t& pose);

private:
	bool ShouldRecord();
	bool OpenIfNeeded();

	bool checked_enabled_ = false;
	bool enabled_ = false;
	bool open_attempted_ = false;
	std::ofstream out_;
	int64_t first_qpc_ = 0;
};

} // namespace phantom
