#define _CRT_SECURE_NO_DEPRECATE
#include "AvatarStatePoller.h"

#include "JsonUtil.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <chrono>
#include <fstream>
#include <sstream>
#include <utility>

namespace facetracking {
namespace {

constexpr auto kReadInterval = std::chrono::milliseconds(500);

std::wstring ResolveAvatarStatePath()
{
	std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"facetracking\\avatar_parameter_cache", false);
	return dir.empty() ? std::wstring() : dir + L"\\state.json";
}

std::string ReadAvatarNameFromConfigPath(const std::string& path)
{
	if (path.empty()) return {};

	std::ifstream is(openvr_pair::common::Utf8ToWide(path));
	if (!is) return {};

	std::stringstream ss;
	ss << is.rdbuf();

	picojson::value root;
	if (!openvr_pair::common::json::ParseObject(root, ss.str())) return {};

	return openvr_pair::common::json::StringAt(root, "name");
}

} // namespace

AvatarStatePoller::AvatarStatePoller()
{
	ResolvePath();
}

void AvatarStatePoller::ResolvePath()
{
	std::wstring wpath = ResolveAvatarStatePath();
	path_utf8_ = openvr_pair::common::WideToUtf8(wpath);
}

void AvatarStatePoller::Tick()
{
	const auto now = std::chrono::steady_clock::now();
	if (now - last_read_attempt_ < kReadInterval) return;
	last_read_attempt_ = now;

	if (path_utf8_.empty()) return;

	const std::wstring wpath = openvr_pair::common::Utf8ToWide(path_utf8_);
	const int64_t mtime = openvr_pair::common::FileLastWriteTime(wpath);
	if (mtime == 0) {
		last_observed_mtime_ = 0;
		snapshot_ = AvatarStateSnapshot{};
		return;
	}
	if (mtime == last_observed_mtime_) return;

	ReadFile();
	if (snapshot_.valid) {
		last_observed_mtime_ = mtime;
	}
}

void AvatarStatePoller::ReadFile()
{
	std::ifstream is(openvr_pair::common::Utf8ToWide(path_utf8_));
	if (!is) return;

	std::stringstream ss;
	ss << is.rdbuf();

	picojson::value root;
	if (!openvr_pair::common::json::ParseObject(root, ss.str())) return;

	AvatarStateSnapshot next;
	next.avatar_id = openvr_pair::common::json::StringAt(root, "AvatarId");
	next.config_path = openvr_pair::common::json::StringAt(root, "ConfigPath");
	next.avatar_name = ReadAvatarNameFromConfigPath(next.config_path);
	next.updated_at_utc = openvr_pair::common::json::StringAt(root, "UpdatedAtUtc");
	next.valid = !next.avatar_id.empty();
	snapshot_ = std::move(next);
}

} // namespace facetracking
