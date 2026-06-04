#define _CRT_SECURE_NO_DEPRECATE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winhttp.h>
#include <shlobj.h>
#include <objbase.h>

#include "ModelDownloader.h"
#include "Logging.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <cstdio>
#include <cstring>
#include <string>

// Link against winhttp.lib (added in CMakeLists).

std::string ModelDownloader::DefaultModelDir()
{
	std::wstring root = openvr_pair::common::WkOpenVrSubdirectoryPath(L"captions\\models", true);
	return openvr_pair::common::WideToUtf8(root);
}

static std::wstring Utf8ToWide(const std::string& s)
{
	int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
	if (n <= 0) return {};
	std::wstring w(n, L'\0');
	MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
	return w;
}

bool ModelDownloader::Download(const std::string& url, const std::string& dest_path, ProgressCallback progress_cb,
                               std::string* error_out)
{
	auto fail = [&](const char* msg) -> bool {
		TH_LOG("[downloader] %s (err=%lu)", msg, GetLastError());
		if (error_out) *error_out = msg;
		return false;
	};

	// Parse URL.
	std::wstring wurl = Utf8ToWide(url);
	URL_COMPONENTS uc{};
	uc.dwStructSize = sizeof(uc);
	wchar_t hostbuf[256] = {}, pathbuf[2048] = {};
	uc.lpszHostName = hostbuf;
	uc.dwHostNameLength = _countof(hostbuf);
	uc.lpszUrlPath = pathbuf;
	uc.dwUrlPathLength = _countof(pathbuf);
	if (!WinHttpCrackUrl(wurl.c_str(), 0, 0, &uc)) {
		return fail("WinHttpCrackUrl failed");
	}

	bool use_https = (uc.nScheme == INTERNET_SCHEME_HTTPS);

	HINTERNET session = WinHttpOpen(L"WKOpenVR-ModelDownloader/1.0", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
	                                WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	if (!session) return fail("WinHttpOpen failed");

	HINTERNET conn = WinHttpConnect(session, hostbuf, uc.nPort, 0);
	if (!conn) {
		WinHttpCloseHandle(session);
		return fail("WinHttpConnect failed");
	}

	DWORD flags = use_https ? WINHTTP_FLAG_SECURE : 0;
	HINTERNET req =
	    WinHttpOpenRequest(conn, L"GET", pathbuf, nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, flags);
	if (!req) {
		WinHttpCloseHandle(conn);
		WinHttpCloseHandle(session);
		return fail("WinHttpOpenRequest failed");
	}

	if (!WinHttpSendRequest(req, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0)) {
		WinHttpCloseHandle(req);
		WinHttpCloseHandle(conn);
		WinHttpCloseHandle(session);
		return fail("WinHttpSendRequest failed");
	}

	if (!WinHttpReceiveResponse(req, nullptr)) {
		WinHttpCloseHandle(req);
		WinHttpCloseHandle(conn);
		WinHttpCloseHandle(session);
		return fail("WinHttpReceiveResponse failed");
	}

	// Read Content-Length if present.
	int64_t total_bytes = 0;
	{
		wchar_t clbuf[32] = {};
		DWORD clbuf_size = sizeof(clbuf);
		if (WinHttpQueryHeaders(req, WINHTTP_QUERY_CONTENT_LENGTH, WINHTTP_HEADER_NAME_BY_INDEX, clbuf, &clbuf_size,
		                        WINHTTP_NO_HEADER_INDEX)) {
			total_bytes = _wtoi64(clbuf);
		}
	}

	// Ensure destination directory exists.
	{
		std::wstring wdest = Utf8ToWide(dest_path);
		std::wstring dir = wdest;
		auto sep = dir.find_last_of(L"/\\");
		if (sep != std::wstring::npos) {
			dir.resize(sep);
			CreateDirectoryW(dir.c_str(), nullptr);
		}
	}

	// Open destination file.
	std::wstring wdest = Utf8ToWide(dest_path);
	HANDLE out_file =
	    CreateFileW(wdest.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (out_file == INVALID_HANDLE_VALUE) {
		WinHttpCloseHandle(req);
		WinHttpCloseHandle(conn);
		WinHttpCloseHandle(session);
		return fail("Failed to create destination file");
	}

	uint8_t buf[65536];
	int64_t downloaded = 0;
	DWORD avail = 0;
	bool ok = true;

	while (WinHttpQueryDataAvailable(req, &avail) && avail > 0) {
		DWORD to_read = (avail < sizeof(buf)) ? avail : sizeof(buf);
		DWORD got = 0;
		if (!WinHttpReadData(req, buf, to_read, &got) || got == 0) {
			ok = false;
			break;
		}
		DWORD written = 0;
		WriteFile(out_file, buf, got, &written, nullptr);
		downloaded += got;
		if (progress_cb) progress_cb(downloaded, total_bytes);
	}

	CloseHandle(out_file);
	WinHttpCloseHandle(req);
	WinHttpCloseHandle(conn);
	WinHttpCloseHandle(session);

	if (!ok) {
		DeleteFileW(wdest.c_str());
		return fail("Download read error");
	}

	TH_LOG("[downloader] downloaded %lld bytes to '%s'", (long long)downloaded, dest_path.c_str());
	return true;
}
