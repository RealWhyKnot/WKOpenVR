#define _CRT_SECURE_NO_DEPRECATE
#include "HostStatusPoller.h"

#include "JsonUtil.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

namespace captions {

namespace {
constexpr auto kStaleAfter = std::chrono::seconds(10);
constexpr auto kReadInterval = std::chrono::milliseconds(500);

std::wstring ResolveStatusPath()
{
	std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"captions", false);
	return dir.empty() ? std::wstring() : dir + L"\\host_status.json";
}
} // namespace

HostStatusPoller::HostStatusPoller()
{
	ResolvePath();
}

void HostStatusPoller::ResolvePath()
{
	std::wstring wpath = ResolveStatusPath();
	path_utf8_ = openvr_pair::common::WideToUtf8(wpath);
}

void HostStatusPoller::Tick()
{
	auto now = std::chrono::steady_clock::now();
	if (now - last_read_attempt_ < kReadInterval) {
		if (snapshot_.valid) snapshot_.stale = (now - last_successful_read_) > kStaleAfter;
		return;
	}
	last_read_attempt_ = now;

	if (path_utf8_.empty()) return;

	std::wstring wpath = openvr_pair::common::Utf8ToWide(path_utf8_);
	int64_t mtime = openvr_pair::common::FileLastWriteTime(wpath);
	if (mtime == 0) {
		if (snapshot_.valid) snapshot_.stale = (now - last_successful_read_) > kStaleAfter;
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

void HostStatusPoller::ReadFile()
{
	std::ifstream is(openvr_pair::common::Utf8ToWide(path_utf8_));
	if (!is) return;

	std::stringstream ss;
	ss << is.rdbuf();
	std::string body = ss.str();

	picojson::value root;
	if (!openvr_pair::common::json::ParseObject(root, body)) return;

	HostStatusSnapshot s;
	s.valid = true;
	s.host_halted = snapshot_.host_halted;
	s.last_exit_code = snapshot_.last_exit_code;
	s.last_exit_description = snapshot_.last_exit_description;
	s.host_pid = openvr_pair::common::json::IntAt(root, "host_pid");
	s.state = openvr_pair::common::json::IntAt(root, "state");
	s.phase = openvr_pair::common::json::StringAt(root, "phase");
	s.mic_name = openvr_pair::common::json::StringAt(root, "mic_name");
	s.last_transcript = openvr_pair::common::json::StringAt(root, "last_transcript");
	s.last_translation = openvr_pair::common::json::StringAt(root, "last_translation");
	s.last_error = openvr_pair::common::json::StringAt(root, "last_error");
	s.ptt_available = openvr_pair::common::json::BoolAt(root, "ptt_available");
	s.ptt_registered = openvr_pair::common::json::BoolAt(root, "ptt_registered");
	s.ptt_app_key = openvr_pair::common::json::StringAt(root, "ptt_app_key");
	s.ptt_error = openvr_pair::common::json::StringAt(root, "ptt_error");
	s.speech_pack_installed = openvr_pair::common::json::BoolAt(root, "speech_pack_installed");
	s.vad_runtime_available = openvr_pair::common::json::BoolAt(root, "vad_runtime_available");
	s.translation_runtime_available = openvr_pair::common::json::BoolAt(root, "translation_runtime_available");
	s.translation_pack_installed = openvr_pair::common::json::BoolAt(root, "translation_pack_installed");
	s.active_translation_pair = openvr_pair::common::json::StringAt(root, "active_translation_pair");
	s.packets_sent = static_cast<long long>(openvr_pair::common::json::NumberAt(root, "packets_sent"));

	snapshot_ = std::move(s);
}

} // namespace captions
