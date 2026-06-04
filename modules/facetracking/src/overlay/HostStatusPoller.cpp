#define _CRT_SECURE_NO_DEPRECATE
#include "HostStatusPoller.h"

#include "JsonUtil.h"
#include "Logging.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

namespace facetracking {

namespace {

// File-stat freshness threshold: if the host hasn't refreshed within this
// window, the snapshot is flagged stale. The host writes once per second
// in steady state, so 10 s leaves plenty of slack for transient FS lag.
constexpr auto kStaleAfter = std::chrono::seconds(10);

// Minimum interval between disk reads. The overlay ticks at ~60 Hz; we
// don't want to stat() the file that often.
constexpr auto kReadInterval = std::chrono::milliseconds(500);

std::wstring ResolveHostStatusPath()
{
	std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"facetracking", false);
	return dir.empty() ? std::wstring() : dir + L"\\host_status.json";
}

} // namespace

HostStatusPoller::HostStatusPoller()
{
	ResolvePath();
}

void HostStatusPoller::ResolvePath()
{
	std::wstring wpath = ResolveHostStatusPath();
	path_utf8_ = openvr_pair::common::WideToUtf8(wpath);
}

void HostStatusPoller::Tick()
{
	auto now = std::chrono::steady_clock::now();

	if (now - last_read_attempt_ < kReadInterval) {
		// Not yet time for another disk read; just refresh staleness flag.
		if (snapshot_.valid) {
			bool became_stale = (now - last_successful_read_) > kStaleAfter;
			snapshot_.stale = became_stale;
			if (became_stale && (now - last_stale_warn_) >= std::chrono::seconds(5)) {
				auto stale_s = std::chrono::duration_cast<std::chrono::seconds>(now - last_successful_read_).count();
				FT_LOG_OVL("[host-supervisor] host_status.json stale for %llds (pid=%d)", (long long)stale_s,
				           snapshot_.host_pid);
				last_stale_warn_ = now;
			}
		}
		return;
	}
	last_read_attempt_ = now;

	if (path_utf8_.empty()) {
		// Path resolution failed at construction; nothing to do.
		return;
	}

	std::wstring wpath = openvr_pair::common::Utf8ToWide(path_utf8_);
	int64_t mtime = openvr_pair::common::FileLastWriteTime(wpath);
	if (mtime == 0) {
		// File doesn't exist (host not running yet). Keep prior snapshot if
		// we ever had one, but flag it stale.
		if (snapshot_.valid) {
			bool became_stale = (now - last_successful_read_) > kStaleAfter;
			snapshot_.stale = became_stale;
			if (became_stale && (now - last_stale_warn_) >= std::chrono::seconds(5)) {
				auto stale_s = std::chrono::duration_cast<std::chrono::seconds>(now - last_successful_read_).count();
				FT_LOG_OVL("[host-supervisor] host_status.json stale for %llds (host not running?)",
				           (long long)stale_s);
				last_stale_warn_ = now;
			}
		}
		return;
	}

	if (mtime == last_observed_mtime_) {
		// File unchanged since last successful read. Refresh staleness.
		bool became_stale = (now - last_successful_read_) > kStaleAfter;
		snapshot_.stale = became_stale;
		if (became_stale && (now - last_stale_warn_) >= std::chrono::seconds(5)) {
			auto stale_s = std::chrono::duration_cast<std::chrono::seconds>(now - last_successful_read_).count();
			FT_LOG_OVL("[host-supervisor] host_status.json stale for %llds (mtime frozen, pid=%d)", (long long)stale_s,
			           snapshot_.host_pid);
			last_stale_warn_ = now;
		}
		return;
	}

	ReadFile();
	if (snapshot_.valid) {
		last_observed_mtime_ = mtime;
		last_successful_read_ = now;
		snapshot_.stale = false;
	}
}

void HostStatusPoller::ReadFile()
{
	std::ifstream is(openvr_pair::common::Utf8ToWide(path_utf8_));
	if (!is) return;

	std::stringstream ss;
	ss << is.rdbuf();
	std::string body = ss.str();

	picojson::value root;
	if (!openvr_pair::common::json::ParseObject(root, body)) {
		// Could happen briefly if we caught the file between .tmp write
		// and rename. Skip this read silently; the next tick retries.
		return;
	}

	HostStatusSnapshot s;
	s.valid = true;
	s.host_pid = openvr_pair::common::json::IntAt(root, "host_pid");
	s.host_started_at = openvr_pair::common::json::StringAt(root, "host_started_at");
	s.host_uptime_seconds = openvr_pair::common::json::IntAt(root, "host_uptime_s");
	s.host_shutting_down = openvr_pair::common::json::BoolAt(root, "host_shutting_down");
	s.phase = openvr_pair::common::json::StringAt(root, "phase");
	s.last_error = openvr_pair::common::json::StringAt(root, "last_error");
	s.module_count = openvr_pair::common::json::IntAt(root, "module_count");
	s.active_module_uuid = openvr_pair::common::json::StringAt(root, "active_module_uuid");
	s.active_module_name = openvr_pair::common::json::StringAt(root, "active_module_name");
	s.frames_written = static_cast<uint64_t>(openvr_pair::common::json::NumberAt(root, "frames_written"));
	s.frames_read = static_cast<uint64_t>(openvr_pair::common::json::NumberAt(root, "frames_read"));
	s.osc_messages_sent = static_cast<uint64_t>(openvr_pair::common::json::NumberAt(root, "osc_messages_sent"));
	s.last_exit_code = openvr_pair::common::json::IntAt(root, "last_exit_code");
	s.last_restart_time = openvr_pair::common::json::StringAt(root, "last_restart_time");

	if (const auto* am = openvr_pair::common::json::ValueAt(root, "active_module"); am && am->is<picojson::object>()) {
		HostStatusActiveModule m;
		m.uuid = openvr_pair::common::json::StringAt(*am, "uuid");
		m.name = openvr_pair::common::json::StringAt(*am, "name");
		m.vendor = openvr_pair::common::json::StringAt(*am, "vendor");
		m.version = openvr_pair::common::json::StringAt(*am, "version");
		if (!m.uuid.empty()) s.active_module = std::move(m);
	}

	if (const auto* im = openvr_pair::common::json::ArrayAt(root, "installed_modules")) {
		for (const auto& el : *im) {
			HostStatusInstalledModule mm;
			mm.uuid = openvr_pair::common::json::StringAt(el, "uuid");
			mm.name = openvr_pair::common::json::StringAt(el, "name");
			mm.vendor = openvr_pair::common::json::StringAt(el, "vendor");
			mm.version = openvr_pair::common::json::StringAt(el, "version");
			if (!mm.uuid.empty()) s.installed_modules.push_back(std::move(mm));
		}
	}

	FT_LOG_OVL("HostStatusPoller: refreshed (pid=%d uptime=%ds)", s.host_pid, s.host_uptime_seconds);

	snapshot_ = std::move(s);
}

} // namespace facetracking
