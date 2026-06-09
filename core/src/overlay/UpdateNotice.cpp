#include "UpdateNotice.h"

#include "DiagnosticsLog.h"
#include "JsonUtil.h"
#include "UpdateNoticeLogic.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <picojson.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <bcrypt.h>
#include <shellapi.h>
#include <winhttp.h>

#pragma comment(lib, "bcrypt.lib")
#pragma comment(lib, "winhttp.lib")

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#ifndef OPENVR_PAIR_VERSION_STRING
#define OPENVR_PAIR_VERSION_STRING "0.0.0.0-dev"
#endif

namespace openvr_pair::overlay {

namespace {

constexpr wchar_t kApiHost[] = L"api.github.com";
constexpr wchar_t kCanonicalLatestPath[] = L"/repos/RealWhyKnot/WKOpenVR-SpaceCalibrator/releases/latest";
constexpr wchar_t kUserAgent[] = L"WKOpenVR-Updater/1.0";
constexpr int64_t kMaxInstallerBytes = 512ll * 1024ll * 1024ll;

struct ModuleReleaseSpec
{
	const char* flagFileName;
	const char* featureName;
	const char* displayName;
	const char* repoName;
};

constexpr ModuleReleaseSpec kModuleReleaseSpecs[] = {
    {"enable_calibration.flag", "Calibration", "Space Calibrator", "WKOpenVR-SpaceCalibrator"},
    {"enable_smoothing.flag", "Smoothing", "Smoothing", "WKOpenVR-Smoothing"},
    {"enable_inputhealth.flag", "InputHealth", "Input Health", "WKOpenVR-InputHealth"},
    {"enable_facetracking.flag", "FaceTracking", "Face Tracking", "WKOpenVR-FaceTracking"},
    {"enable_oscrouter.flag", "OSCRouter", "OSC Router", "WKOpenVR-OSCRouter"},
    {"enable_questapp.flag", "QuestApp", "Quest App", "WKOpenVR-QuestApp"},
    {"enable_captions.flag", "Captions", "Captions", "WKOpenVR-Captions"},
};

struct InstallerAsset
{
	std::string tagName;
	std::string version;
	std::string releaseUrl;
	std::string releaseBody;
	std::string assetName;
	std::string downloadUrl;
	std::string sha256;
	int64_t sizeBytes = 0;
};

struct PendingFile
{
	std::string tag;
	std::string version;
	std::string repo;
	std::string module;
	std::string asset;
	std::string path;
	std::string sha256;
};

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

bool IsLocalDevBuild()
{
	const std::string s = OPENVR_PAIR_VERSION_STRING;
	return s.find('-') != std::string::npos;
}

std::wstring QuotePowerShellString(const std::wstring& value)
{
	std::wstring out = L"'";
	for (wchar_t ch : value) {
		if (ch == L'\'')
			out += L"''";
		else
			out += ch;
	}
	out += L"'";
	return out;
}

std::wstring EncodePowerShellCommand(const std::wstring& script)
{
	std::vector<unsigned char> bytes;
	bytes.reserve(script.size() * 2);
	for (wchar_t ch : script) {
		bytes.push_back(static_cast<unsigned char>(ch & 0xFF));
		bytes.push_back(static_cast<unsigned char>((ch >> 8) & 0xFF));
	}

	static const char* kBase64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	std::wstring out;
	out.reserve(((bytes.size() + 2) / 3) * 4);
	size_t i = 0;
	while (i + 3 <= bytes.size()) {
		uint32_t v = (uint32_t(bytes[i]) << 16) | (uint32_t(bytes[i + 1]) << 8) | uint32_t(bytes[i + 2]);
		out += (wchar_t)kBase64[(v >> 18) & 0x3F];
		out += (wchar_t)kBase64[(v >> 12) & 0x3F];
		out += (wchar_t)kBase64[(v >> 6) & 0x3F];
		out += (wchar_t)kBase64[v & 0x3F];
		i += 3;
	}
	if (i < bytes.size()) {
		uint32_t v = uint32_t(bytes[i]) << 16;
		size_t rem = bytes.size() - i;
		if (rem == 2) v |= uint32_t(bytes[i + 1]) << 8;
		out += (wchar_t)kBase64[(v >> 18) & 0x3F];
		out += (wchar_t)kBase64[(v >> 12) & 0x3F];
		out += (wchar_t)(rem == 2 ? kBase64[(v >> 6) & 0x3F] : L'=');
		out += L'=';
	}
	return out;
}

bool FileExists(const std::wstring& path)
{
	const DWORD attr = GetFileAttributesW(path.c_str());
	return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

int64_t FileSizeBytes(const std::wstring& path)
{
	WIN32_FILE_ATTRIBUTE_DATA data{};
	if (!GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &data)) return -1;
	ULARGE_INTEGER size{};
	size.LowPart = data.nFileSizeLow;
	size.HighPart = data.nFileSizeHigh;
	return static_cast<int64_t>(size.QuadPart);
}

std::wstring UpdaterDir()
{
	return openvr_pair::common::WkOpenVrSubdirectoryPath(L"updater", true);
}

std::wstring PendingPath()
{
	const std::wstring dir = UpdaterDir();
	return dir.empty() ? std::wstring() : dir + L"\\pending-update.txt";
}

std::wstring WidePathFromUtf8(const std::string& path)
{
	return openvr_pair::common::Utf8ToWide(path);
}

std::string Utf8PathFromWide(const std::wstring& path)
{
	return openvr_pair::common::WideToUtf8(path);
}

std::wstring FullPath(const std::wstring& path)
{
	DWORD needed = GetFullPathNameW(path.c_str(), 0, nullptr, nullptr);
	if (needed == 0) return {};
	std::wstring out(needed, L'\0');
	DWORD written = GetFullPathNameW(path.c_str(), needed, out.data(), nullptr);
	if (written == 0 || written >= needed) return {};
	out.resize(written);
	return out;
}

bool PathIsUnderUpdaterDir(const std::wstring& path)
{
	std::wstring root = FullPath(UpdaterDir());
	std::wstring candidate = FullPath(path);
	if (root.empty() || candidate.empty()) return false;
	if (root.back() != L'\\' && root.back() != L'/') root.push_back(L'\\');
	if (candidate.size() < root.size()) return false;
	return _wcsnicmp(candidate.c_str(), root.c_str(), root.size()) == 0;
}

bool WriteTextFileAtomic(const std::wstring& path, const std::string& body)
{
	const std::wstring tmp = path + L".tmp";
	HANDLE h = CreateFileW(tmp.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) return false;
	DWORD written = 0;
	BOOL ok = WriteFile(h, body.data(), static_cast<DWORD>(body.size()), &written, nullptr);
	FlushFileBuffers(h);
	CloseHandle(h);
	if (!ok || written != static_cast<DWORD>(body.size())) {
		DeleteFileW(tmp.c_str());
		return false;
	}
	if (!MoveFileExW(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
		DeleteFileW(tmp.c_str());
		return false;
	}
	return true;
}

std::string HttpGet(const wchar_t* host, const wchar_t* path, std::string& err)
{
	HINTERNET hSession =
	    WinHttpOpen(kUserAgent, WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	if (!hSession) {
		err = "WinHttpOpen failed";
		return {};
	}
	WinHttpSetTimeouts(hSession, 5000, 5000, 15000, 15000);

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

std::wstring ReleasePath(const char* repoName)
{
	std::wstring path = L"/repos/RealWhyKnot/";
	path += openvr_pair::common::Utf8ToWide(repoName ? repoName : "");
	path += L"/releases/latest";
	return path;
}

bool HashFileSha256(const std::wstring& path, std::string& out, std::string& err)
{
	HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
	                          FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, nullptr);
	if (file == INVALID_HANDLE_VALUE) {
		err = "could not open file for SHA-256";
		return false;
	}

	BCRYPT_ALG_HANDLE alg = nullptr;
	BCRYPT_HASH_HANDLE hash = nullptr;
	DWORD objectLen = 0;
	DWORD cbData = 0;
	NTSTATUS status = BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr, 0);
	if (status < 0) {
		CloseHandle(file);
		err = "BCryptOpenAlgorithmProvider failed";
		return false;
	}
	status = BCryptGetProperty(alg, BCRYPT_OBJECT_LENGTH, reinterpret_cast<PUCHAR>(&objectLen), sizeof(objectLen),
	                           &cbData, 0);
	if (status < 0 || objectLen == 0) {
		BCryptCloseAlgorithmProvider(alg, 0);
		CloseHandle(file);
		err = "BCryptGetProperty failed";
		return false;
	}

	std::vector<unsigned char> object(objectLen);
	status = BCryptCreateHash(alg, &hash, object.data(), objectLen, nullptr, 0, 0);
	if (status < 0) {
		BCryptCloseAlgorithmProvider(alg, 0);
		CloseHandle(file);
		err = "BCryptCreateHash failed";
		return false;
	}

	std::vector<unsigned char> buffer(65536);
	for (;;) {
		DWORD got = 0;
		if (!ReadFile(file, buffer.data(), static_cast<DWORD>(buffer.size()), &got, nullptr)) {
			BCryptDestroyHash(hash);
			BCryptCloseAlgorithmProvider(alg, 0);
			CloseHandle(file);
			err = "file read failed during SHA-256";
			return false;
		}
		if (got == 0) break;
		status = BCryptHashData(hash, buffer.data(), got, 0);
		if (status < 0) {
			BCryptDestroyHash(hash);
			BCryptCloseAlgorithmProvider(alg, 0);
			CloseHandle(file);
			err = "BCryptHashData failed";
			return false;
		}
	}

	std::vector<unsigned char> digest(32);
	status = BCryptFinishHash(hash, digest.data(), static_cast<ULONG>(digest.size()), 0);
	BCryptDestroyHash(hash);
	BCryptCloseAlgorithmProvider(alg, 0);
	CloseHandle(file);
	if (status < 0) {
		err = "BCryptFinishHash failed";
		return false;
	}

	static constexpr char kHex[] = "0123456789abcdef";
	std::string hex;
	hex.reserve(digest.size() * 2);
	for (unsigned char b : digest) {
		hex.push_back(kHex[(b >> 4) & 0x0F]);
		hex.push_back(kHex[b & 0x0F]);
	}
	out = std::move(hex);
	return true;
}

void UpdateDownloadProgress(int64_t downloaded, int64_t total);

bool DownloadToFile(const std::string& url, const std::wstring& dest, int64_t expectedSize, std::atomic<bool>& cancel,
                    std::string& err)
{
	if (expectedSize <= 0 || expectedSize > kMaxInstallerBytes) {
		err = "installer size is outside the allowed range";
		return false;
	}

	const std::wstring wurl = openvr_pair::common::Utf8ToWide(url);
	URL_COMPONENTS uc{};
	uc.dwStructSize = sizeof(uc);
	wchar_t host[256] = {};
	wchar_t path[2048] = {};
	wchar_t extra[2048] = {};
	uc.lpszHostName = host;
	uc.dwHostNameLength = _countof(host);
	uc.lpszUrlPath = path;
	uc.dwUrlPathLength = _countof(path);
	uc.lpszExtraInfo = extra;
	uc.dwExtraInfoLength = _countof(extra);
	if (!WinHttpCrackUrl(wurl.c_str(), static_cast<DWORD>(wurl.size()), 0, &uc)) {
		err = "download URL parse failed";
		return false;
	}
	if (uc.nScheme != INTERNET_SCHEME_HTTPS) {
		err = "download URL was not HTTPS";
		return false;
	}
	std::wstring object(path, uc.dwUrlPathLength);
	object.append(extra, uc.dwExtraInfoLength);

	HINTERNET session =
	    WinHttpOpen(kUserAgent, WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	if (!session) {
		err = "WinHttpOpen failed";
		return false;
	}
	WinHttpSetTimeouts(session, 5000, 5000, 30000, 30000);

	const INTERNET_PORT port = uc.nPort != 0 ? uc.nPort : INTERNET_DEFAULT_HTTPS_PORT;
	HINTERNET conn = WinHttpConnect(session, host, port, 0);
	if (!conn) {
		WinHttpCloseHandle(session);
		err = "WinHttpConnect failed";
		return false;
	}

	HINTERNET req = WinHttpOpenRequest(conn, L"GET", object.c_str(), nullptr, WINHTTP_NO_REFERER,
	                                   WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
	if (!req) {
		WinHttpCloseHandle(conn);
		WinHttpCloseHandle(session);
		err = "WinHttpOpenRequest failed";
		return false;
	}

	if (!WinHttpSendRequest(req, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) ||
	    !WinHttpReceiveResponse(req, nullptr)) {
		WinHttpCloseHandle(req);
		WinHttpCloseHandle(conn);
		WinHttpCloseHandle(session);
		err = "installer download request failed";
		return false;
	}

	DWORD status = 0, statusSize = sizeof status;
	WinHttpQueryHeaders(req, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX,
	                    &status, &statusSize, WINHTTP_NO_HEADER_INDEX);
	if (status != 200) {
		WinHttpCloseHandle(req);
		WinHttpCloseHandle(conn);
		WinHttpCloseHandle(session);
		err = "installer download returned HTTP " + std::to_string(status);
		return false;
	}

	HANDLE out = CreateFileW(dest.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (out == INVALID_HANDLE_VALUE) {
		WinHttpCloseHandle(req);
		WinHttpCloseHandle(conn);
		WinHttpCloseHandle(session);
		err = "could not create installer cache file";
		return false;
	}

	int64_t downloaded = 0;
	bool ok = true;
	std::vector<unsigned char> buffer(65536);
	while (!cancel.load()) {
		DWORD avail = 0;
		if (!WinHttpQueryDataAvailable(req, &avail)) {
			ok = false;
			err = "WinHttpQueryDataAvailable failed";
			break;
		}
		if (avail == 0) break;
		const DWORD toRead = std::min<DWORD>(avail, static_cast<DWORD>(buffer.size()));
		DWORD got = 0;
		if (!WinHttpReadData(req, buffer.data(), toRead, &got) || got == 0) {
			ok = false;
			err = "WinHttpReadData failed";
			break;
		}
		downloaded += got;
		if (downloaded > expectedSize || downloaded > kMaxInstallerBytes) {
			ok = false;
			err = "installer download exceeded the expected size";
			break;
		}
		DWORD written = 0;
		if (!WriteFile(out, buffer.data(), got, &written, nullptr) || written != got) {
			ok = false;
			err = "installer cache write failed";
			break;
		}

		UpdateDownloadProgress(downloaded, expectedSize);
	}

	FlushFileBuffers(out);
	CloseHandle(out);
	WinHttpCloseHandle(req);
	WinHttpCloseHandle(conn);
	WinHttpCloseHandle(session);

	if (cancel.load()) {
		DeleteFileW(dest.c_str());
		err = "download cancelled";
		return false;
	}
	if (!ok) {
		DeleteFileW(dest.c_str());
		return false;
	}
	if (downloaded != expectedSize) {
		DeleteFileW(dest.c_str());
		err = "installer download size did not match GitHub metadata";
		return false;
	}
	return true;
}

struct UpdateNoticeSingleton
{
	std::thread checkWorker;
	std::thread downloadWorker;
	std::atomic<bool> checkInFlight{false};
	std::atomic<bool> downloadInFlight{false};
	std::atomic<bool> cancelDownload{false};
	mutable std::mutex mu;
	UpdateNoticeState state;

	~UpdateNoticeSingleton()
	{
		cancelDownload.store(true);
		if (checkWorker.joinable()) checkWorker.join();
		if (downloadWorker.joinable()) downloadWorker.join();
	}
};

UpdateNoticeSingleton& Instance()
{
	static UpdateNoticeSingleton s;
	return s;
}

void SetInstallState(UpdateInstallState install)
{
	auto& inst = Instance();
	std::lock_guard<std::mutex> lock(inst.mu);
	inst.state.install = std::move(install);
}

void UpdateDownloadProgress(int64_t downloaded, int64_t total)
{
	auto& inst = Instance();
	std::lock_guard<std::mutex> lock(inst.mu);
	inst.state.install.bytesDownloaded = downloaded;
	inst.state.install.totalBytes = total;
}

bool ParseReleaseJson(const std::string& body, picojson::value& out, std::string& err)
{
	std::string parseErr;
	if (!openvr_pair::common::json::ParseObject(out, body, &parseErr)) {
		err = "JSON parse failed: " + parseErr;
		return false;
	}
	return true;
}

bool FetchInstallerAsset(const ModuleReleaseSpec& spec, const std::string& expectedTag, InstallerAsset& out,
                         std::string& err)
{
	const std::wstring path = ReleasePath(spec.repoName);
	std::string body = HttpGet(kApiHost, path.c_str(), err);
	if (body.empty()) {
		if (err.empty()) err = "empty release response";
		return false;
	}

	picojson::value release;
	if (!ParseReleaseJson(body, release, err)) return false;

	out.tagName = openvr_pair::common::json::StringAt(release, "tag_name");
	out.releaseUrl = openvr_pair::common::json::StringAt(release, "html_url");
	out.releaseBody = openvr_pair::common::json::StringAt(release, "body");
	if (out.tagName.empty()) {
		err = "release response missing tag_name";
		return false;
	}
	if (!expectedTag.empty() && out.tagName != expectedTag) {
		err = "module release tag does not match the update notice";
		return false;
	}
	out.version = out.tagName;
	if (!out.version.empty() && out.version.front() == 'v') out.version.erase(0, 1);

	const std::string expectedAssetName = ExpectedInstallerAssetName(spec.featureName, out.version);
	const picojson::array* assets = openvr_pair::common::json::ArrayAt(release, "assets");
	if (!assets) {
		err = "release response missing assets";
		return false;
	}

	for (const picojson::value& asset : *assets) {
		const std::string name = openvr_pair::common::json::StringAt(asset, "name");
		if (name != expectedAssetName) continue;
		const std::string state = openvr_pair::common::json::StringAt(asset, "state");
		if (state != "uploaded") {
			err = "installer asset is not uploaded";
			return false;
		}
		out.assetName = name;
		out.downloadUrl = openvr_pair::common::json::StringAt(asset, "browser_download_url");
		out.sha256 = NormalizeSha256Digest(openvr_pair::common::json::StringAt(asset, "digest"));
		out.sizeBytes = static_cast<int64_t>(openvr_pair::common::json::NumberAt(asset, "size", 0.0));
		break;
	}

	if (out.assetName.empty()) {
		err = "release did not include the expected installer asset";
		return false;
	}
	if (out.sha256.empty()) {
		err = "installer asset missing SHA-256 digest";
		return false;
	}
	if (!ReleaseBodySha256Matches(out.releaseBody, out.sha256)) {
		err = "release notes SHA-256 does not match the asset digest";
		return false;
	}
	if (!IsTrustedGitHubReleaseAssetUrl(out.downloadUrl, spec.repoName, out.tagName, out.assetName)) {
		err = "installer download URL did not match the expected GitHub release path";
		return false;
	}
	if (out.sizeBytes <= 0 || out.sizeBytes > kMaxInstallerBytes) {
		err = "installer asset size is outside the allowed range";
		return false;
	}
	return true;
}

const ModuleReleaseSpec& SelectModuleSpec(const std::vector<std::string_view>& installedFlags)
{
	for (std::string_view flag : installedFlags) {
		for (const ModuleReleaseSpec& spec : kModuleReleaseSpecs) {
			if (flag == spec.flagFileName) return spec;
		}
	}
	return kModuleReleaseSpecs[0];
}

std::string PendingFileBody(const PendingFile& pending)
{
	std::ostringstream out;
	out << "tag=" << pending.tag << "\n";
	out << "version=" << pending.version << "\n";
	out << "repo=" << pending.repo << "\n";
	out << "module=" << pending.module << "\n";
	out << "asset=" << pending.asset << "\n";
	out << "path=" << pending.path << "\n";
	out << "sha256=" << pending.sha256 << "\n";
	return out.str();
}

bool WritePending(const PendingFile& pending)
{
	const std::wstring path = PendingPath();
	if (path.empty()) return false;
	return WriteTextFileAtomic(path, PendingFileBody(pending));
}

bool TryLoadPending(PendingFile& pending)
{
	const std::wstring path = PendingPath();
	if (path.empty() || !FileExists(path)) return false;

	std::ifstream in(std::filesystem::path(path), std::ios::binary);
	if (!in.is_open()) return false;
	std::string line;
	while (std::getline(in, line)) {
		if (!line.empty() && line.back() == '\r') line.pop_back();
		const size_t eq = line.find('=');
		if (eq == std::string::npos) continue;
		const std::string key = line.substr(0, eq);
		const std::string value = line.substr(eq + 1);
		if (key == "tag") pending.tag = value;
		if (key == "version") pending.version = value;
		if (key == "repo") pending.repo = value;
		if (key == "module") pending.module = value;
		if (key == "asset") pending.asset = value;
		if (key == "path") pending.path = value;
		if (key == "sha256") pending.sha256 = NormalizeSha256Digest(value);
	}

	const std::wstring installerPath = WidePathFromUtf8(pending.path);
	return !pending.version.empty() && !pending.repo.empty() && !pending.asset.empty() && !pending.path.empty() &&
	       !pending.sha256.empty() && PathIsUnderUpdaterDir(installerPath) && FileExists(installerPath);
}

void DeletePending()
{
	const std::wstring path = PendingPath();
	if (!path.empty()) DeleteFileW(path.c_str());
}

UpdateInstallState InstallStateFromPending(const PendingFile& pending)
{
	UpdateInstallState install;
	install.phase = UpdateInstallPhase::Ready;
	install.queuedForSteamVrExit = true;
	install.canQueue = false;
	install.targetTag = pending.tag;
	install.targetVersion = pending.version;
	install.selectedModule = pending.module;
	install.assetName = pending.asset;
	install.installerPath = pending.path;
	install.expectedSha256 = pending.sha256;
	install.statusMessage = "Installer verified. It will start after SteamVR closes.";
	return install;
}

void LoadPersistedPending()
{
	PendingFile pending;
	if (!TryLoadPending(pending)) return;
	SetInstallState(InstallStateFromPending(pending));
}

void ClearStalePendingIfCurrent(const std::string& latestTag, UpdateInstallState& install)
{
	if (!install.queuedForSteamVrExit || install.targetTag.empty()) return;
	if (install.targetTag == latestTag) return;
	if (IsRemoteNewer(latestTag, install.targetTag) || !IsRemoteNewer(install.targetTag, OPENVR_PAIR_VERSION_STRING)) {
		DeletePending();
		install = {};
	}
}

void RunCheck()
{
	auto& inst = Instance();
	UpdateInstallState installSnapshot;
	{
		std::lock_guard<std::mutex> lock(inst.mu);
		installSnapshot = inst.state.install;
	}

	UpdateNoticeState next;
	next.install = installSnapshot;

	if (IsLocalDevBuild()) {
		next.checkComplete = true;
		{
			std::lock_guard<std::mutex> lock(inst.mu);
			inst.state = std::move(next);
		}
		inst.checkInFlight.store(false);
		return;
	}

	std::string err;
	const std::string body = HttpGet(kApiHost, kCanonicalLatestPath, err);
	if (body.empty()) {
		next.checkComplete = true;
		next.errorMessage = err.empty() ? std::string("empty response") : err;
		{
			std::lock_guard<std::mutex> lock(inst.mu);
			inst.state = std::move(next);
		}
		inst.checkInFlight.store(false);
		return;
	}

	picojson::value release;
	if (!ParseReleaseJson(body, release, err)) {
		next.checkComplete = true;
		next.errorMessage = err;
		{
			std::lock_guard<std::mutex> lock(inst.mu);
			inst.state = std::move(next);
		}
		inst.checkInFlight.store(false);
		return;
	}

	next.latestTag = openvr_pair::common::json::StringAt(release, "tag_name");
	next.releaseUrl = openvr_pair::common::json::StringAt(release, "html_url");
	next.checkComplete = true;
	if (next.latestTag.empty()) {
		next.errorMessage = "GitHub response missing tag_name";
	}
	else {
		next.latestVersion = next.latestTag;
		if (!next.latestVersion.empty() && next.latestVersion.front() == 'v') next.latestVersion.erase(0, 1);
		next.available = IsRemoteNewer(next.latestTag, OPENVR_PAIR_VERSION_STRING);
		ClearStalePendingIfCurrent(next.latestTag, next.install);
		if (next.available && next.install.phase == UpdateInstallPhase::Idle) {
			next.install.canQueue = true;
		}
	}

	{
		std::lock_guard<std::mutex> lock(inst.mu);
		inst.state = std::move(next);
	}
	inst.checkInFlight.store(false);
}

void RunDownload(ModuleReleaseSpec spec, std::string expectedTag)
{
	auto& inst = Instance();
	UpdateInstallState install;
	{
		std::lock_guard<std::mutex> lock(inst.mu);
		install = inst.state.install;
	}

	auto fail = [&](std::string message) {
		openvr_pair::common::DiagnosticLog("updater", "queue_failed module='%s' error='%s'", spec.displayName,
		                                   message.c_str());
		install.phase = inst.cancelDownload.load() ? UpdateInstallPhase::Idle : UpdateInstallPhase::Failed;
		install.queuedForSteamVrExit = false;
		install.canQueue = true;
		install.errorMessage = std::move(message);
		install.statusMessage.clear();
		SetInstallState(install);
		inst.downloadInFlight.store(false);
		inst.cancelDownload.store(false);
	};

	std::string err;
	InstallerAsset asset;
	if (!FetchInstallerAsset(spec, expectedTag, asset, err)) {
		fail(err.empty() ? "could not read module release" : err);
		return;
	}

	const std::wstring dir = UpdaterDir();
	if (dir.empty()) {
		fail("could not create updater cache directory");
		return;
	}
	const std::wstring finalPath = dir + L"\\" + openvr_pair::common::Utf8ToWide(asset.assetName);
	const std::wstring tempPath = finalPath + L".download";
	if (!PathIsUnderUpdaterDir(finalPath)) {
		fail("installer cache path was outside the updater cache");
		return;
	}

	install.targetTag = asset.tagName;
	install.targetVersion = asset.version;
	install.assetName = asset.assetName;
	install.expectedSha256 = asset.sha256;
	install.installerPath = Utf8PathFromWide(finalPath);
	install.totalBytes = asset.sizeBytes;
	install.statusMessage = "Downloading installer...";
	SetInstallState(install);

	bool needDownload = true;
	if (FileExists(finalPath) && FileSizeBytes(finalPath) == asset.sizeBytes) {
		std::string existingHash;
		if (HashFileSha256(finalPath, existingHash, err) && existingHash == asset.sha256) {
			needDownload = false;
		}
	}

	if (needDownload) {
		DeleteFileW(tempPath.c_str());
		if (!DownloadToFile(asset.downloadUrl, tempPath, asset.sizeBytes, inst.cancelDownload, err)) {
			fail(err.empty() ? "installer download failed" : err);
			return;
		}
		UpdateDownloadProgress(asset.sizeBytes, asset.sizeBytes);

		std::string tempHash;
		if (!HashFileSha256(tempPath, tempHash, err) || tempHash != asset.sha256) {
			DeleteFileW(tempPath.c_str());
			fail("downloaded installer SHA-256 did not match GitHub metadata");
			return;
		}
		if (!MoveFileExW(tempPath.c_str(), finalPath.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
			DeleteFileW(tempPath.c_str());
			fail("could not move verified installer into the updater cache");
			return;
		}
	}

	std::string finalHash;
	if (!HashFileSha256(finalPath, finalHash, err) || finalHash != asset.sha256) {
		fail("cached installer SHA-256 did not match after final write");
		return;
	}
	if (FileSizeBytes(finalPath) != asset.sizeBytes) {
		fail("cached installer size did not match after final write");
		return;
	}

	PendingFile pending;
	pending.tag = asset.tagName;
	pending.version = asset.version;
	pending.repo = spec.repoName;
	pending.module = spec.displayName;
	pending.asset = asset.assetName;
	pending.path = Utf8PathFromWide(finalPath);
	pending.sha256 = asset.sha256;
	if (!WritePending(pending)) {
		fail("could not persist the update queue");
		return;
	}

	install = InstallStateFromPending(pending);
	SetInstallState(install);
	openvr_pair::common::DiagnosticLog("updater", "queue_ready module='%s' version='%s' asset='%s' sha256='%s'",
	                                   spec.displayName, asset.version.c_str(), asset.assetName.c_str(),
	                                   asset.sha256.c_str());
	inst.downloadInFlight.store(false);
	inst.cancelDownload.store(false);
}

} // namespace

void StartUpdateCheck()
{
	LoadPersistedPending();

	auto& inst = Instance();
	bool expected = false;
	if (!inst.checkInFlight.compare_exchange_strong(expected, true)) return;
	if (inst.checkWorker.joinable()) inst.checkWorker.join();
	inst.checkWorker = std::thread(&RunCheck);
}

UpdateNoticeState GetUpdateNoticeState()
{
	auto& inst = Instance();
	std::lock_guard<std::mutex> lock(inst.mu);
	return inst.state;
}

bool QueueUpdateForSteamVrClose(const std::vector<std::string_view>& installedFlags, std::string* error)
{
	auto& inst = Instance();
	UpdateNoticeState snapshot;
	{
		std::lock_guard<std::mutex> lock(inst.mu);
		snapshot = inst.state;
	}

	if (!snapshot.available || snapshot.latestTag.empty()) {
		if (error) *error = "No update is currently available.";
		return false;
	}
	if (snapshot.install.queuedForSteamVrExit && snapshot.install.phase == UpdateInstallPhase::Ready) {
		return true;
	}

	bool expected = false;
	if (!inst.downloadInFlight.compare_exchange_strong(expected, true)) {
		return true;
	}
	inst.cancelDownload.store(false);
	if (inst.downloadWorker.joinable()) inst.downloadWorker.join();

	const ModuleReleaseSpec& spec = SelectModuleSpec(installedFlags);
	UpdateInstallState install;
	install.phase = UpdateInstallPhase::Downloading;
	install.queuedForSteamVrExit = true;
	install.canQueue = false;
	install.selectedModule = spec.displayName;
	install.statusMessage = "Downloading installer...";
	SetInstallState(install);
	openvr_pair::common::DiagnosticLog("updater", "queue_start module='%s' tag='%s'", spec.displayName,
	                                   snapshot.latestTag.c_str());

	inst.downloadWorker = std::thread([spec, tag = snapshot.latestTag] { RunDownload(spec, tag); });
	return true;
}

void CancelQueuedUpdate()
{
	auto& inst = Instance();
	inst.cancelDownload.store(true);
	DeletePending();
	UpdateInstallState install;
	install.canQueue = true;
	SetInstallState(install);
	openvr_pair::common::DiagnosticLog("updater", "queue_cancelled");
}

bool LaunchQueuedUpdateAfterProcessExit(uint32_t currentProcessId, std::string* error)
{
	auto& inst = Instance();
	UpdateInstallState install;
	{
		std::lock_guard<std::mutex> lock(inst.mu);
		install = inst.state.install;
	}

	if (!install.queuedForSteamVrExit || install.phase != UpdateInstallPhase::Ready) {
		if (error) *error = "No verified update is queued.";
		return false;
	}

	const std::wstring installerPath = WidePathFromUtf8(install.installerPath);
	if (!PathIsUnderUpdaterDir(installerPath) || !FileExists(installerPath)) {
		if (error) *error = "Queued installer path is invalid.";
		return false;
	}

	std::string err;
	std::string hash;
	if (!HashFileSha256(installerPath, hash, err) || hash != NormalizeSha256Digest(install.expectedSha256)) {
		if (error) *error = "Queued installer SHA-256 check failed.";
		openvr_pair::common::DiagnosticLog("updater", "launch_blocked_sha expected='%s' actual='%s' error='%s'",
		                                   install.expectedSha256.c_str(), hash.c_str(), err.c_str());
		return false;
	}

	const std::wstring pendingPath = PendingPath();
	std::wstring script;
	script += L"$ErrorActionPreference = 'Stop';\r\n";
	script += L"$installer = " + QuotePowerShellString(installerPath) + L";\r\n";
	script += L"$pending = " + QuotePowerShellString(pendingPath) + L";\r\n";
	script += L"$pidToWait = " + std::to_wstring(currentProcessId) + L";\r\n";
	script += L"try { $p = Get-Process -Id $pidToWait -ErrorAction SilentlyContinue; ";
	script += L"if ($p) { [void]$p.WaitForExit(90000) } } catch {}\r\n";
	script += L"$names = @('vrserver','vrcompositor','vrdashboard','vrmonitor');\r\n";
	script += L"for ($i = 0; $i -lt 90; $i++) { ";
	script += L"$running = $false; foreach ($n in $names) { ";
	script += L"if (Get-Process -Name $n -ErrorAction SilentlyContinue) { $running = $true } }; ";
	script += L"if (-not $running) { break }; Start-Sleep -Seconds 1 }\r\n";
	script += L"$proc = Start-Process -FilePath $installer -Verb RunAs -Wait -PassThru;\r\n";
	script += L"if ($proc.ExitCode -eq 0 -and (Test-Path -LiteralPath $pending)) { ";
	script += L"Remove-Item -LiteralPath $pending -Force }\r\n";

	const std::wstring args = L"-NoProfile -ExecutionPolicy Bypass -EncodedCommand " + EncodePowerShellCommand(script);

	SHELLEXECUTEINFOW sei{};
	sei.cbSize = sizeof(sei);
	sei.fMask = SEE_MASK_NOCLOSEPROCESS;
	sei.lpFile = L"powershell.exe";
	sei.lpParameters = args.c_str();
	sei.nShow = SW_HIDE;
	if (!ShellExecuteExW(&sei) || sei.hProcess == nullptr) {
		const DWORD lastError = GetLastError();
		if (error) *error = "Could not start update helper.";
		openvr_pair::common::DiagnosticLog("updater", "launch_helper_failed error=%lu", lastError);
		return false;
	}
	CloseHandle(sei.hProcess);

	install.phase = UpdateInstallPhase::Launching;
	install.statusMessage = "Installer will open after WKOpenVR exits.";
	SetInstallState(install);
	openvr_pair::common::DiagnosticLog("updater", "launch_helper_started version='%s' asset='%s'",
	                                   install.targetVersion.c_str(), install.assetName.c_str());
	return true;
}

} // namespace openvr_pair::overlay
