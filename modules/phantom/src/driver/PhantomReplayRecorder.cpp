#include "PhantomReplayRecorder.h"

#include "BuildChannel.h"
#include "DebugLogging.h"
#include "Logging.h"
#include "Win32Paths.h"

#include <windows.h>

#include <chrono>
#include <cstdio>
#include <cwchar>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <sstream>

namespace phantom {

namespace {

bool EnvFlagEnabled(const wchar_t* name)
{
	wchar_t buf[32]{};
	const DWORD n = GetEnvironmentVariableW(name, buf, (DWORD)std::size(buf));
	if (n == 0 || n >= std::size(buf)) return false;
	return buf[0] == L'1' || buf[0] == L'y' || buf[0] == L'Y' || buf[0] == L't' || buf[0] == L'T';
}

const char* DeviceClassKey(vr::ETrackedDeviceClass c)
{
	switch (c) {
		case vr::TrackedDeviceClass_HMD:
			return "hmd";
		case vr::TrackedDeviceClass_Controller:
			return "controller";
		case vr::TrackedDeviceClass_GenericTracker:
			return "tracker";
		case vr::TrackedDeviceClass_TrackingReference:
			return "tracking_reference";
		default:
			return "invalid";
	}
}

const char* ControllerRoleKey(vr::ETrackedControllerRole r)
{
	switch (r) {
		case vr::TrackedControllerRole_LeftHand:
			return "left_hand";
		case vr::TrackedControllerRole_RightHand:
			return "right_hand";
		case vr::TrackedControllerRole_OptOut:
			return "opt_out";
		default:
			return "invalid";
	}
}

const char* TrackingResultKey(vr::ETrackingResult r)
{
	switch (r) {
		case vr::TrackingResult_Running_OK:
			return "ok";
		case vr::TrackingResult_Running_OutOfRange:
			return "out_of_range";
		case vr::TrackingResult_Uninitialized:
			return "uninitialized";
		default:
			return "other";
	}
}

std::string SanitizeCsvField(const std::string& input)
{
	std::string out = input;
	for (char& ch : out) {
		if (ch == ',' || ch == '\r' || ch == '\n' || ch == '\t') {
			ch = '_';
		}
	}
	return out;
}

std::wstring Timestamp()
{
	SYSTEMTIME st{};
	GetLocalTime(&st);
	wchar_t buf[64]{};
	std::swprintf(buf, std::size(buf), L"%04u-%02u-%02uT%02u-%02u-%02u", st.wYear, st.wMonth, st.wDay, st.wHour,
	              st.wMinute, st.wSecond);
	return buf;
}

} // namespace

bool PhantomReplayRecorder::ShouldRecord()
{
	if (!checked_enabled_) {
		checked_enabled_ = true;
		if (EnvFlagEnabled(L"WKOPENVR_NO_PHANTOM_REPLAY_RECORD")) {
			enabled_ = false;
		}
		else if (EnvFlagEnabled(L"WKOPENVR_PHANTOM_REPLAY_RECORD")) {
			enabled_ = true;
		}
		else {
			enabled_ = openvr_pair::common::IsDebugLoggingEnabled();
		}
	}
	return enabled_;
}

bool PhantomReplayRecorder::OpenIfNeeded()
{
	if (out_.is_open()) return true;
	if (open_attempted_) return false;
	open_attempted_ = true;

	const std::wstring logs = openvr_pair::common::WkOpenVrLogsPath(true);
	if (logs.empty()) return false;
	std::filesystem::path path(logs);
	path /= L"phantom_replay.";
	path += Timestamp();
	path += L".csv";
	out_.open(path, std::ios::out | std::ios::trunc);
	if (!out_) {
		LOG("[phantom][replay] failed to open replay recording at %ls", path.c_str());
		return false;
	}
	out_ << "# phantom_replay_v1\n";
	out_ << "# build_stamp=" << WKOPENVR_BUILD_STAMP << "\n";
	out_ << "# build_channel=" << WKOPENVR_BUILD_CHANNEL << "\n";
	out_ << "# columns=time_ms,device_id,serial,class,controller_role,body_role,dropout_enabled,pose_valid,connected,"
	        "result,x,y,z,qw,qx,qy,qz,vx,vy,vz\n";
	out_
	    << "time_ms,device_id,serial,class,controller_role,body_role,dropout_enabled,pose_valid,connected,result,x,y,z,"
	       "qw,qx,qy,qz,vx,vy,vz\n";
	LOG("[phantom][replay] recording poses to %ls", path.c_str());
	return true;
}

void PhantomReplayRecorder::RecordPose(int64_t qpc_ticks, int64_t qpc_freq, uint32_t openvr_id,
                                       const std::string& serial, vr::ETrackedDeviceClass device_class,
                                       vr::ETrackedControllerRole controller_role, BodyRole body_role,
                                       bool dropout_enabled, const vr::DriverPose_t& pose)
{
	if (!ShouldRecord() || qpc_freq <= 0) return;
	if (!OpenIfNeeded()) return;
	if (first_qpc_ == 0) first_qpc_ = qpc_ticks;
	const double time_ms = static_cast<double>(qpc_ticks - first_qpc_) * 1000.0 / static_cast<double>(qpc_freq);
	out_ << std::fixed << std::setprecision(6) << time_ms << ',' << openvr_id << ',' << SanitizeCsvField(serial) << ','
	     << DeviceClassKey(device_class) << ',' << ControllerRoleKey(controller_role) << ',' << BodyRoleToKey(body_role)
	     << ',' << (dropout_enabled ? 1 : 0) << ',' << (pose.poseIsValid ? 1 : 0) << ','
	     << (pose.deviceIsConnected ? 1 : 0) << ',' << TrackingResultKey(pose.result) << ',' << pose.vecPosition[0]
	     << ',' << pose.vecPosition[1] << ',' << pose.vecPosition[2] << ',' << pose.qRotation.w << ','
	     << pose.qRotation.x << ',' << pose.qRotation.y << ',' << pose.qRotation.z << ',' << pose.vecVelocity[0] << ','
	     << pose.vecVelocity[1] << ',' << pose.vecVelocity[2] << '\n';
}

} // namespace phantom
