#define _CRT_SECURE_NO_DEPRECATE
#include "DriverTelemetryPoller.h"

#include "JsonUtil.h"
#include "Logging.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

namespace facetracking {

namespace {

// If the driver hasn't refreshed within this window the snapshot is flagged
// stale. The driver writes every ~500 ms; 5 s leaves ample slack for transient
// FS lag and short driver pauses.
constexpr auto kStaleAfter = std::chrono::seconds(5);
constexpr auto kReadInterval = std::chrono::milliseconds(500);

std::wstring ResolveTelemetryPath()
{
	std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"facetracking", false);
	return dir.empty() ? std::wstring() : dir + L"\\driver_telemetry.json";
}

} // namespace

DriverTelemetryPoller::DriverTelemetryPoller()
{
	ResolvePath();
}

void DriverTelemetryPoller::ResolvePath()
{
	std::wstring wpath = ResolveTelemetryPath();
	path_utf8_ = openvr_pair::common::WideToUtf8(wpath);
}

void DriverTelemetryPoller::Tick()
{
	auto now = std::chrono::steady_clock::now();

	if (now - last_read_attempt_ < kReadInterval) {
		if (snapshot_.valid) {
			snapshot_.stale = (now - last_successful_read_) > kStaleAfter;
		}
		return;
	}
	last_read_attempt_ = now;

	if (path_utf8_.empty()) return;

	std::wstring wpath = openvr_pair::common::Utf8ToWide(path_utf8_);
	int64_t mtime = openvr_pair::common::FileLastWriteTime(wpath);
	if (mtime == 0) {
		// Driver not running yet.
		if (snapshot_.valid) {
			snapshot_.stale = (now - last_successful_read_) > kStaleAfter;
		}
		return;
	}

	if (mtime == last_observed_mtime_) {
		snapshot_.stale = (now - last_successful_read_) > kStaleAfter;
		return;
	}

	ReadFile();
	if (snapshot_.valid) {
		last_observed_mtime_ = mtime;
		last_successful_read_ = now;
		snapshot_.stale = false;
	}
}

void DriverTelemetryPoller::ReadFile()
{
	std::ifstream is(openvr_pair::common::Utf8ToWide(path_utf8_));
	if (!is) return;

	std::stringstream ss;
	ss << is.rdbuf();
	std::string body = ss.str();

	picojson::value root;
	if (!openvr_pair::common::json::ParseObject(root, body)) {
		// Caught mid-write; next tick retries.
		return;
	}

	DriverTelemetrySnapshot s;
	s.valid = true;
	s.driver_pid = openvr_pair::common::json::IntAt(root, "driver_pid");
	s.frames_processed = static_cast<uint64_t>(openvr_pair::common::json::NumberAt(root, "frames_processed"));
	s.frames_read = static_cast<uint64_t>(openvr_pair::common::json::NumberAt(root, "frames_read"));
	s.osc_messages_sent = static_cast<uint64_t>(openvr_pair::common::json::NumberAt(root, "osc_messages_sent"));
	s.osc_messages_dropped = static_cast<uint64_t>(openvr_pair::common::json::NumberAt(root, "osc_messages_dropped"));
	s.active_module_uuid = openvr_pair::common::json::StringAt(root, "active_module_uuid");

	if (const auto* verg = openvr_pair::common::json::ValueAt(root, "vergence"); verg && verg->is<picojson::object>()) {
		s.vergence_enabled = openvr_pair::common::json::BoolAt(*verg, "enabled");
		s.focus_distance_m = static_cast<float>(openvr_pair::common::json::NumberAt(*verg, "focus_distance_m"));
		s.ipd_m = static_cast<float>(openvr_pair::common::json::NumberAt(*verg, "ipd_m"));
	}

	FT_LOG_OVL("DriverTelemetryPoller: refreshed (pid=%d read=%llu processed=%llu osc_sent=%llu osc_drop=%llu verg=%s "
	           "focus=%.3fm)",
	           s.driver_pid, (unsigned long long)s.frames_read, (unsigned long long)s.frames_processed,
	           (unsigned long long)s.osc_messages_sent, (unsigned long long)s.osc_messages_dropped,
	           s.vergence_enabled ? "on" : "off", s.focus_distance_m);

	snapshot_ = std::move(s);
}

} // namespace facetracking
