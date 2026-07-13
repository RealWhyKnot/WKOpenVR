#include "PhantomReplayRecorder.h"

#include "DebugLogging.h"
#include "Logging.h"
#include "Win32Paths.h"

#include <windows.h>

#include <cstdio>
#include <iomanip>
#include <iterator>
#include <sstream>

namespace phantom {

namespace {

// Retention for phantom_replay.*.csv in the logs directory. Matches the
// envelope defaults; named here so init-time pruning and the recorder agree.
constexpr std::size_t kReplayRetentionMaxFiles = 5;
constexpr uint64_t kReplayRetentionMaxBytes = 512ull * 1024ull * 1024ull;

constexpr double kCounterAnnotationIntervalMs = 5.0 * 60.0 * 1000.0;

bool EnvFlagEnabled(const wchar_t* name)
{
	wchar_t buf[32]{};
	const DWORD n = GetEnvironmentVariableW(name, buf, (DWORD)std::size(buf));
	if (n == 0 || n >= std::size(buf)) return false;
	return buf[0] == L'1' || buf[0] == L'y' || buf[0] == L'Y' || buf[0] == L't' || buf[0] == L'T';
}

// Full-rate capture can also be requested by a flag file in the WKOpenVR data
// root (same mechanism as debug_logging.enabled). Environment variables are
// unreliable here: vrserver inherits Steam's environment, not the shell that
// configured the capture, so an env-only gate silently records throttled.
constexpr wchar_t kFullRateFlagFileName[] = L"phantom_replay_fullrate.enabled";

bool FullRateFlagFileExists()
{
	std::wstring path = openvr_pair::common::WkOpenVrRootPath(false);
	if (path.empty()) return false;
	if (path.back() != L'\\' && path.back() != L'/') {
		path.push_back(L'\\');
	}
	path += kFullRateFlagFileName;
	const DWORD attr = GetFileAttributesW(path.c_str());
	return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
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

// v2: pose columns are world-space (worldFromDriver folded in before
// recording), so devices from different drivers share one frame. v1 recorded
// raw driver-local values; column names and order are unchanged.
constexpr const char* kColumns =
    "time_ms,device_id,serial,class,controller_role,body_role,dropout_enabled,pose_valid,connected,"
    "result,x,y,z,qw,qx,qy,qz,vx,vy,vz";

} // namespace

void PhantomReplayRecorder::PruneOnInit()
{
	namespace recording = openvr_pair::common::recording;
	recording::RetentionPolicy policy;
	policy.maxFiles = kReplayRetentionMaxFiles;
	policy.maxTotalBytes = kReplayRetentionMaxBytes;
	const recording::PruneResult result = recording::PruneRecordings(L"phantom_replay", L"csv", policy);
	if (result.totalFiles > 0) {
		LOG("[phantom][replay] retention: kept %llu of %llu recordings, freed %llu MB (failed_deletes=%llu)",
		    (unsigned long long)result.keptFiles, (unsigned long long)result.totalFiles,
		    (unsigned long long)(result.freedBytes / (1024ull * 1024ull)), (unsigned long long)result.failedDeletes);
	}
}

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
		const bool envFullRate = EnvFlagEnabled(L"WKOPENVR_PHANTOM_REPLAY_FULLRATE");
		const bool flagFullRate = !envFullRate && FullRateFlagFileExists();
		budget_.fullRate = envFullRate || flagFullRate;
		full_rate_source_ = envFullRate ? "env" : (flagFullRate ? "flagfile" : "off");
		// The flag file states an intent to capture; don't leave that capture
		// hostage to the debug-logging toggle. The explicit no-record env var
		// still wins.
		if (flagFullRate && !EnvFlagEnabled(L"WKOPENVR_NO_PHANTOM_REPLAY_RECORD")) {
			enabled_ = true;
		}
	}
	return enabled_;
}

bool PhantomReplayRecorder::OpenIfNeeded()
{
	if (envelope_.IsOpen()) return true;
	if (open_attempted_) return false;
	open_attempted_ = true;

	namespace recording = openvr_pair::common::recording;
	recording::EnvelopeOptions options;
	options.prefix = L"phantom_replay";
	options.extension = L"csv";
	options.schemaBanner = "phantom_replay_v2";
	options.headerKVs = {{"columns", kColumns}};
	options.retention.maxFiles = kReplayRetentionMaxFiles;
	options.retention.maxTotalBytes = kReplayRetentionMaxBytes;

	if (!envelope_.Open(options)) {
		LOG("[phantom][replay] failed to open replay recording");
		return false;
	}
	envelope_.WriteRow(kColumns);

	std::ostringstream budget;
	budget << "budget: max_hz_hmd=" << budget_.maxHzHmd << " max_hz_dev=" << budget_.maxHzDevice
	       << " keyframe_ms=" << budget_.keyframeIntervalMs << " pos_eps_m=" << budget_.posEpsilonM
	       << " quat_dot_eps=" << budget_.quatDotEpsilon << " full_rate=" << (budget_.fullRate ? 1 : 0)
	       << " full_rate_src=" << full_rate_source_;
	envelope_.WriteAnnotation(0.0, budget.str());

	LOG("[phantom][replay] recording poses to %ls (full_rate=%d src=%s)", envelope_.Path().c_str(),
	    budget_.fullRate ? 1 : 0, full_rate_source_);
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

	const double pos[3] = {pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]};
	const double quat[4] = {pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z};
	const uint32_t discreteBits =
	    PackDiscreteBits(dropout_enabled, pose.poseIsValid, pose.deviceIsConnected, static_cast<int>(pose.result),
	                     static_cast<int>(controller_role), static_cast<int>(body_role));

	DeviceRecordState& state = device_state_[openvr_id];
	const bool isHmd = device_class == vr::TrackedDeviceClass_HMD;
	const RecordDecision decision = ShouldWriteRow(state, budget_, isHmd, time_ms, pos, quat, discreteBits);
	if (!decision.write) {
		++rows_suppressed_;
		return;
	}

	std::ostringstream row;
	row << std::fixed << std::setprecision(6) << time_ms << ',' << openvr_id << ',' << SanitizeCsvField(serial) << ','
	    << DeviceClassKey(device_class) << ',' << ControllerRoleKey(controller_role) << ',' << BodyRoleToKey(body_role)
	    << ',' << (dropout_enabled ? 1 : 0) << ',' << (pose.poseIsValid ? 1 : 0) << ','
	    << (pose.deviceIsConnected ? 1 : 0) << ',' << TrackingResultKey(pose.result) << ',' << pose.vecPosition[0]
	    << ',' << pose.vecPosition[1] << ',' << pose.vecPosition[2] << ',' << pose.qRotation.w << ','
	    << pose.qRotation.x << ',' << pose.qRotation.y << ',' << pose.qRotation.z << ',' << pose.vecVelocity[0] << ','
	    << pose.vecVelocity[1] << ',' << pose.vecVelocity[2];
	if (envelope_.WriteRow(row.str())) {
		CommitWrite(state, time_ms, pos, quat, discreteBits);
		++rows_written_;
	}

	if (time_ms - last_counter_annotation_ms_ >= kCounterAnnotationIntervalMs) {
		last_counter_annotation_ms_ = time_ms;
		std::ostringstream counters;
		counters << "budget_counters: written=" << rows_written_ << " suppressed=" << rows_suppressed_
		         << " devices=" << device_state_.size();
		envelope_.WriteAnnotation(time_ms / 1000.0, counters.str());
	}
}

} // namespace phantom
