#include "PhantomReplay.h"

#if WKOPENVR_BUILD_IS_DEV

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string_view>
#include <unordered_map>

namespace openvr_pair::overlay::testharness {

namespace {

std::string Trim(std::string_view v)
{
	while (!v.empty() && (v.front() == ' ' || v.front() == '\t' || v.front() == '\r' || v.front() == '\n')) {
		v.remove_prefix(1);
	}
	while (!v.empty() && (v.back() == ' ' || v.back() == '\t' || v.back() == '\r' || v.back() == '\n')) {
		v.remove_suffix(1);
	}
	return std::string(v);
}

std::vector<std::string> SplitSimple(const std::string& line)
{
	std::vector<std::string> out;
	std::string cur;
	for (char ch : line) {
		if (ch == ',') {
			out.push_back(Trim(cur));
			cur.clear();
		}
		else {
			cur.push_back(ch);
		}
	}
	out.push_back(Trim(cur));
	return out;
}

bool ParseDouble(const std::string& s, double& out)
{
	const std::string t = Trim(s);
	if (t.empty()) return false;
	char* end = nullptr;
	out = std::strtod(t.c_str(), &end);
	return end && *end == '\0' && std::isfinite(out);
}

bool ParseU32(const std::string& s, uint32_t& out)
{
	const std::string t = Trim(s);
	if (t.empty()) return false;
	uint64_t value = 0;
	const auto* begin = t.data();
	const auto* end = t.data() + t.size();
	auto result = std::from_chars(begin, end, value);
	if (result.ec != std::errc{} || result.ptr != end || value > UINT32_MAX) return false;
	out = static_cast<uint32_t>(value);
	return true;
}

bool ParseBool(const std::string& s)
{
	const std::string t = Trim(s);
	return t == "1" || t == "true" || t == "TRUE" || t == "yes" || t == "on";
}

vr::ETrackedDeviceClass ParseDeviceClass(const std::string& s)
{
	const std::string t = Trim(s);
	if (t == "hmd" || t == "HMD") return vr::TrackedDeviceClass_HMD;
	if (t == "controller" || t == "Controller") return vr::TrackedDeviceClass_Controller;
	if (t == "tracker" || t == "generic_tracker" || t == "GenericTracker") {
		return vr::TrackedDeviceClass_GenericTracker;
	}
	if (t == "tracking_reference") return vr::TrackedDeviceClass_TrackingReference;
	return vr::TrackedDeviceClass_Invalid;
}

vr::ETrackedControllerRole ParseControllerRole(const std::string& s)
{
	const std::string t = Trim(s);
	if (t == "left_hand" || t == "left") return vr::TrackedControllerRole_LeftHand;
	if (t == "right_hand" || t == "right") return vr::TrackedControllerRole_RightHand;
	if (t == "opt_out" || t == "tracker") return vr::TrackedControllerRole_OptOut;
	return vr::TrackedControllerRole_Invalid;
}

vr::ETrackingResult ParseTrackingResult(const std::string& s)
{
	const std::string t = Trim(s);
	if (t == "ok" || t == "running_ok" || t == "Running_OK") return vr::TrackingResult_Running_OK;
	if (t == "out_of_range" || t == "Running_OutOfRange") return vr::TrackingResult_Running_OutOfRange;
	uint32_t value = 0;
	if (ParseU32(t, value)) return static_cast<vr::ETrackingResult>(value);
	return vr::TrackingResult_Running_OK;
}

vr::DriverPose_t MakePose(double x, double y, double z, double qw, double qx, double qy, double qz, bool valid,
                          bool connected, vr::ETrackingResult result, double vx = 0.0, double vy = 0.0, double vz = 0.0)
{
	vr::DriverPose_t pose{};
	pose.qWorldFromDriverRotation = {1, 0, 0, 0};
	pose.qDriverFromHeadRotation = {1, 0, 0, 0};
	pose.qRotation = {qw, qx, qy, qz};
	pose.vecPosition[0] = x;
	pose.vecPosition[1] = y;
	pose.vecPosition[2] = z;
	pose.vecVelocity[0] = vx;
	pose.vecVelocity[1] = vy;
	pose.vecVelocity[2] = vz;
	pose.poseIsValid = valid;
	pose.deviceIsConnected = connected;
	pose.result = result;
	return pose;
}

struct ColumnMap
{
	std::unordered_map<std::string, size_t> idx;

	explicit ColumnMap(const std::vector<std::string>& header)
	{
		for (size_t i = 0; i < header.size(); ++i) {
			idx.emplace(header[i], i);
		}
	}

	bool Has(const char* name) const { return idx.find(name) != idx.end(); }

	std::string Get(const std::vector<std::string>& row, const char* name) const
	{
		auto it = idx.find(name);
		if (it == idx.end() || it->second >= row.size()) return {};
		return row[it->second];
	}

	bool Double(const std::vector<std::string>& row, const char* name, double& out) const
	{
		return ParseDouble(Get(row, name), out);
	}
};

PhantomReplayLoadResult ParsePhantomReplayV1(const std::vector<std::string>& lines)
{
	PhantomReplayLoadResult result;
	result.recording.source_format = "phantom_replay_v1";

	size_t header_line = SIZE_MAX;
	for (size_t i = 0; i < lines.size(); ++i) {
		const auto t = Trim(lines[i]);
		if (t.empty() || t[0] == '#') continue;
		header_line = i;
		break;
	}
	if (header_line == SIZE_MAX) {
		result.error = "phantom_replay_v1 has no column header";
		return result;
	}

	const auto header = SplitSimple(lines[header_line]);
	const ColumnMap cols(header);
	for (const char* required : {"time_ms", "device_id", "serial", "class", "x", "y", "z"}) {
		if (!cols.Has(required)) {
			result.error = std::string("phantom_replay_v1 missing column ") + required;
			return result;
		}
	}

	std::unordered_map<uint32_t, size_t> device_index;
	for (size_t i = header_line + 1; i < lines.size(); ++i) {
		const auto t = Trim(lines[i]);
		if (t.empty() || t[0] == '#') continue;
		const auto row = SplitSimple(t);

		uint32_t replay_id = 0;
		double time_ms = 0.0;
		if (!ParseU32(cols.Get(row, "device_id"), replay_id) || !cols.Double(row, "time_ms", time_ms)) {
			result.error = "phantom_replay_v1 row has invalid time_ms or device_id";
			return result;
		}

		auto dev_it = device_index.find(replay_id);
		if (dev_it == device_index.end()) {
			PhantomReplayDevice device;
			device.replay_id = replay_id;
			device.serial = cols.Get(row, "serial");
			device.device_class = ParseDeviceClass(cols.Get(row, "class"));
			device.controller_role = ParseControllerRole(cols.Get(row, "controller_role"));
			device.body_role = phantom::BodyRoleFromKey(cols.Get(row, "body_role").c_str());
			device.dropout_enabled = ParseBool(cols.Get(row, "dropout_enabled"));
			if (device.serial.empty()) {
				std::ostringstream oss;
				oss << "phantom-replay-device-" << replay_id;
				device.serial = oss.str();
			}
			device_index.emplace(replay_id, result.recording.devices.size());
			result.recording.devices.push_back(std::move(device));
		}

		double x = 0.0, y = 0.0, z = 0.0;
		double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
		double vx = 0.0, vy = 0.0, vz = 0.0;
		if (!cols.Double(row, "x", x) || !cols.Double(row, "y", y) || !cols.Double(row, "z", z)) {
			result.error = "phantom_replay_v1 row has invalid position";
			return result;
		}
		cols.Double(row, "qw", qw);
		cols.Double(row, "qx", qx);
		cols.Double(row, "qy", qy);
		cols.Double(row, "qz", qz);
		cols.Double(row, "vx", vx);
		cols.Double(row, "vy", vy);
		cols.Double(row, "vz", vz);
		const bool valid = cols.Has("pose_valid") ? ParseBool(cols.Get(row, "pose_valid")) : true;
		const bool connected = cols.Has("connected") ? ParseBool(cols.Get(row, "connected")) : true;
		const auto tracking = ParseTrackingResult(cols.Get(row, "result"));

		PhantomReplaySample sample;
		sample.time_ms = time_ms;
		sample.replay_device_id = replay_id;
		sample.pose = MakePose(x, y, z, qw, qx, qy, qz, valid, connected, tracking, vx, vy, vz);
		result.recording.duration_ms = std::max(result.recording.duration_ms, time_ms);
		result.recording.samples.push_back(sample);
	}

	std::sort(result.recording.samples.begin(), result.recording.samples.end(), [](const auto& a, const auto& b) {
		if (a.time_ms == b.time_ms) return a.replay_device_id < b.replay_device_id;
		return a.time_ms < b.time_ms;
	});
	result.ok = !result.recording.samples.empty();
	if (!result.ok) result.error = "phantom_replay_v1 had no pose samples";
	return result;
}

void AddSpacecalDevice(PhantomReplayRecording& rec, uint32_t id, const char* serial, vr::ETrackedDeviceClass cls,
                       phantom::BodyRole role)
{
	PhantomReplayDevice d;
	d.replay_id = id;
	d.serial = serial;
	d.device_class = cls;
	d.controller_role = vr::TrackedControllerRole_Invalid;
	d.body_role = role;
	d.dropout_enabled = false;
	rec.devices.push_back(std::move(d));
}

bool AppendSpacecalPose(const ColumnMap& cols, const std::vector<std::string>& row, const char* prefix,
                        uint32_t replay_id, double time_ms, bool valid, PhantomReplayRecording& rec)
{
	double x = 0.0, y = 0.0, z = 0.0;
	double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
	std::string base(prefix);
	if (!cols.Double(row, (base + "_tx").c_str(), x) || !cols.Double(row, (base + "_ty").c_str(), y) ||
	    !cols.Double(row, (base + "_tz").c_str(), z)) {
		return false;
	}
	cols.Double(row, (base + "_qw").c_str(), qw);
	cols.Double(row, (base + "_qx").c_str(), qx);
	cols.Double(row, (base + "_qy").c_str(), qy);
	cols.Double(row, (base + "_qz").c_str(), qz);

	PhantomReplaySample s;
	s.time_ms = time_ms;
	s.replay_device_id = replay_id;
	s.pose = MakePose(x, y, z, qw, qx, qy, qz, valid, true,
	                  valid ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange);
	rec.samples.push_back(s);
	rec.duration_ms = std::max(rec.duration_ms, time_ms);
	return true;
}

PhantomReplayLoadResult ParseSpacecalV4(const std::vector<std::string>& lines)
{
	PhantomReplayLoadResult result;
	result.recording.source_format = "spacecal_log_v4";
	AddSpacecalDevice(result.recording, 0, "spacecal:hmd", vr::TrackedDeviceClass_HMD, phantom::BodyRole::Hmd);
	AddSpacecalDevice(result.recording, 1, "spacecal:ref", vr::TrackedDeviceClass_GenericTracker,
	                  phantom::BodyRole::None);
	AddSpacecalDevice(result.recording, 2, "spacecal:target", vr::TrackedDeviceClass_GenericTracker,
	                  phantom::BodyRole::None);
	AddSpacecalDevice(result.recording, 3, "spacecal:head_tracker", vr::TrackedDeviceClass_GenericTracker,
	                  phantom::BodyRole::None);

	size_t header_line = SIZE_MAX;
	for (size_t i = 0; i < lines.size(); ++i) {
		if (lines[i].rfind("Timestamp,", 0) == 0) {
			header_line = i;
			break;
		}
	}
	if (header_line == SIZE_MAX) {
		result.error = "spacecal_log_v4 has no Timestamp header";
		return result;
	}

	const auto header = SplitSimple(lines[header_line]);
	const ColumnMap cols(header);
	double first_timestamp_s = 0.0;
	bool have_first = false;

	for (size_t i = header_line + 1; i < lines.size(); ++i) {
		const auto t = Trim(lines[i]);
		if (t.empty() || t[0] == '#') continue;
		const auto row = SplitSimple(t);
		double timestamp_s = 0.0;
		if (!cols.Double(row, "Timestamp", timestamp_s)) continue;
		if (!have_first) {
			first_timestamp_s = timestamp_s;
			have_first = true;
		}
		const double time_ms = (timestamp_s - first_timestamp_s) * 1000.0;

		const bool ref_valid = ParseBool(cols.Get(row, "sample_ref_pose_valid"));
		const bool tgt_valid = ParseBool(cols.Get(row, "sample_tgt_pose_valid"));
		const bool head_valid = ParseBool(cols.Get(row, "head_tracker_valid"));
		AppendSpacecalPose(cols, row, "hmd", 0, time_ms, true, result.recording);
		AppendSpacecalPose(cols, row, "ref", 1, time_ms, ref_valid, result.recording);
		AppendSpacecalPose(cols, row, "tgt", 2, time_ms, tgt_valid, result.recording);
		AppendSpacecalPose(cols, row, "head_tracker", 3, time_ms, head_valid, result.recording);
	}

	std::sort(result.recording.samples.begin(), result.recording.samples.end(), [](const auto& a, const auto& b) {
		if (a.time_ms == b.time_ms) return a.replay_device_id < b.replay_device_id;
		return a.time_ms < b.time_ms;
	});
	result.ok = !result.recording.samples.empty();
	if (!result.ok) result.error = "spacecal_log_v4 had no pose samples";
	return result;
}

} // namespace

PhantomReplayLoadResult LoadPhantomReplay(const std::filesystem::path& path)
{
	std::ifstream in(path);
	PhantomReplayLoadResult result;
	if (!in) {
		result.error = "could not open replay file";
		return result;
	}

	std::vector<std::string> lines;
	std::string line;
	while (std::getline(in, line)) {
		lines.push_back(line);
	}
	for (const auto& raw : lines) {
		const auto t = Trim(raw);
		if (t.empty()) continue;
		// v2 shares the v1 column set; only the pose-space semantics differ
		// (v2 is world-space, v1 raw driver-local).
		if (t.find("phantom_replay_v1") != std::string::npos || t.find("phantom_replay_v2") != std::string::npos) {
			return ParsePhantomReplayV1(lines);
		}
		if (t.find("spacecal_log_v4") != std::string::npos) {
			return ParseSpacecalV4(lines);
		}
		if (t[0] != '#') {
			return ParsePhantomReplayV1(lines);
		}
	}
	result.error = "empty replay file";
	return result;
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
