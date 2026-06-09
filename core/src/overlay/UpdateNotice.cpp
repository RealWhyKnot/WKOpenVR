#include "UpdateNotice.h"

#include "JsonUtil.h"

#include <picojson.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winhttp.h>

#pragma comment(lib, "winhttp.lib")

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef OPENVR_PAIR_VERSION_STRING
#define OPENVR_PAIR_VERSION_STRING "0.0.0.0-dev"
#endif

namespace openvr_pair::overlay {

namespace {

// Hard-coded so a hostile build server can't redirect the update probe
// somewhere else by mutating a config file. Editing this line is a
// deliberate recompile.
constexpr wchar_t kHost[] = L"api.github.com";
constexpr wchar_t kPath[] = L"/repos/RealWhyKnot/WKOpenVR-SpaceCalibrator/releases/latest";
constexpr wchar_t kUserAgent[] = L"WKOpenVR-UpdateNotice/1.0";

// Parse YYYY.M.D.N[-XXXX]? (with optional leading "v") into four numeric
// components. Returns false on malformed input. The trailing -XXXX hex
// suffix (dev-build hash) is dropped before comparison.
bool ParseStamp(const std::string& raw, int (&out)[4])
{
	std::string s = raw;
	if (!s.empty() && s[0] == 'v') s.erase(0, 1);
	const auto dash = s.find('-');
	if (dash != std::string::npos) s.resize(dash);

	int parsed[4] = {0, 0, 0, 0};
	int idx = 0;
	const char* p = s.c_str();
	while (idx < 4 && *p) {
		char* end = nullptr;
		const long v = std::strtol(p, &end, 10);
		if (end == p) return false;
		parsed[idx++] = static_cast<int>(v);
		p = end;
		if (*p == '.')
			++p;
		else if (*p != '\0')
			return false;
	}
	if (idx != 4) return false;
	for (int i = 0; i < 4; ++i)
		out[i] = parsed[i];
	return true;
}

bool IsRemoteNewer(const std::string& remote, const std::string& local)
{
	int r[4] = {0}, l[4] = {0};
	if (!ParseStamp(remote, r) || !ParseStamp(local, l)) return false;
	for (int i = 0; i < 4; ++i) {
		if (r[i] > l[i]) return true;
		if (r[i] < l[i]) return false;
	}
	return false;
}

// True when the local stamp carries the dev suffix that build.ps1 stamps
// onto every non-release build. release.yml strips this suffix when it
// stamps a real tag, so this also doubles as "did the binary come from
// the CI release path?"
bool IsLocalDevBuild()
{
	const std::string s = OPENVR_PAIR_VERSION_STRING;
	return s.find('-') != std::string::npos;
}

// Minimal WinHttp GET. Returns body on success, empty + sets err on failure.
// Follows the API's default redirect behaviour.
std::string HttpGet(const wchar_t* host, const wchar_t* path, std::string& err)
{
	HINTERNET hSession =
	    WinHttpOpen(kUserAgent, WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	if (!hSession) {
		err = "WinHttpOpen failed";
		return {};
	}

	HINTERNET hConnect = WinHttpConnect(hSession, host, INTERNET_DEFAULT_HTTPS_PORT, 0);
	if (!hConnect) {
		WinHttpCloseHandle(hSession);
		err = "WinHttpConnect failed";
		return {};
	}

	HINTERNET hRequest = WinHttpOpenRequest(hConnect, L"GET", path, nullptr, WINHTTP_NO_REFERER,
	                                        WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
	if (!hRequest) {
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		err = "WinHttpOpenRequest failed";
		return {};
	}

	// GitHub requires the Accept + API-version headers; without them the
	// API silently negotiates to a different content type that doesn't
	// preserve the JSON we want.
	const wchar_t kHeaders[] = L"Accept: application/vnd.github+json\r\n"
	                           L"X-GitHub-Api-Version: 2022-11-28\r\n";
	if (!WinHttpSendRequest(hRequest, kHeaders, (DWORD)wcslen(kHeaders), WINHTTP_NO_REQUEST_DATA, 0, 0, 0) ||
	    !WinHttpReceiveResponse(hRequest, nullptr)) {
		WinHttpCloseHandle(hRequest);
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		err = "WinHttp request failed";
		return {};
	}

	DWORD status = 0, statusSize = sizeof status;
	WinHttpQueryHeaders(hRequest, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX,
	                    &status, &statusSize, WINHTTP_NO_HEADER_INDEX);
	if (status != 200) {
		WinHttpCloseHandle(hRequest);
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		err = "GitHub API returned HTTP " + std::to_string(status);
		return {};
	}

	std::string body;
	for (;;) {
		DWORD avail = 0;
		if (!WinHttpQueryDataAvailable(hRequest, &avail)) {
			body.clear();
			err = "WinHttpQueryDataAvailable failed";
			break;
		}
		if (avail == 0) break;
		std::vector<char> chunk(avail);
		DWORD read = 0;
		if (!WinHttpReadData(hRequest, chunk.data(), avail, &read)) {
			body.clear();
			err = "WinHttpReadData failed";
			break;
		}
		body.append(chunk.data(), read);
	}

	WinHttpCloseHandle(hRequest);
	WinHttpCloseHandle(hConnect);
	WinHttpCloseHandle(hSession);
	return body;
}

// Process-wide singleton. Lives in this translation unit so the public
// header stays slim (free functions only). The worker thread is detached
// on shutdown via std::thread's destructor + the atomic guard, so an
// in-flight probe that hasn't returned by exit is abandoned cleanly.
struct UpdateNoticeSingleton
{
	std::thread worker;
	std::atomic<bool> inFlight{false};
	mutable std::mutex mu;
	UpdateNoticeState state;

	~UpdateNoticeSingleton()
	{
		if (worker.joinable()) worker.join();
	}
};

UpdateNoticeSingleton& Instance()
{
	static UpdateNoticeSingleton s;
	return s;
}

void RunCheck()
{
	auto& inst = Instance();
	UpdateNoticeState next;

	// Dev builds never have a sensible local stamp to compare against,
	// so a true release would always look "newer" and the notice would
	// fire constantly. Mark the check as complete without an available
	// flag and skip the network round trip entirely.
	if (IsLocalDevBuild()) {
		next.checkComplete = true;
		{
			std::lock_guard<std::mutex> lock(inst.mu);
			inst.state = std::move(next);
		}
		inst.inFlight.store(false);
		return;
	}

	std::string err;
	const std::string body = HttpGet(kHost, kPath, err);
	if (body.empty()) {
		next.checkComplete = true;
		next.errorMessage = err.empty() ? std::string("empty response") : err;
		{
			std::lock_guard<std::mutex> lock(inst.mu);
			inst.state = std::move(next);
		}
		inst.inFlight.store(false);
		return;
	}

	picojson::value v;
	std::string parseErr;
	if (!openvr_pair::common::json::ParseObject(v, body, &parseErr)) {
		next.checkComplete = true;
		next.errorMessage = "JSON parse failed: " + parseErr;
		{
			std::lock_guard<std::mutex> lock(inst.mu);
			inst.state = std::move(next);
		}
		inst.inFlight.store(false);
		return;
	}

	auto getStr = [&v](const char* key) -> std::string {
		return openvr_pair::common::json::StringAt(v, key);
	};

	next.latestTag = getStr("tag_name");
	next.releaseUrl = getStr("html_url");
	next.checkComplete = true;
	if (next.latestTag.empty()) {
		next.errorMessage = "GitHub response missing tag_name";
	}
	else {
		next.latestVersion = next.latestTag;
		if (!next.latestVersion.empty() && next.latestVersion.front() == 'v') next.latestVersion.erase(0, 1);
		next.available = IsRemoteNewer(next.latestTag, OPENVR_PAIR_VERSION_STRING);
	}

	{
		std::lock_guard<std::mutex> lock(inst.mu);
		inst.state = std::move(next);
	}
	inst.inFlight.store(false);
}

} // namespace

void StartUpdateCheck()
{
	auto& inst = Instance();
	bool expected = false;
	if (!inst.inFlight.compare_exchange_strong(expected, true)) return;
	if (inst.worker.joinable()) inst.worker.join();
	inst.worker = std::thread(&RunCheck);
}

UpdateNoticeState GetUpdateNoticeState()
{
	auto& inst = Instance();
	std::lock_guard<std::mutex> lock(inst.mu);
	return inst.state;
}

} // namespace openvr_pair::overlay
