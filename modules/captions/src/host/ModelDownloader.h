#pragma once

#include <cstddef>
#include <functional>
#include <string>

// Downloads a model file via WinHTTP (GET request).
// Progress callback receives (bytes_downloaded, total_bytes). total_bytes may
// be 0 if the server does not send Content-Length.
class ModelDownloader
{
public:
	using ProgressCallback = std::function<void(int64_t downloaded, int64_t total)>;

	// Download `url` to `dest_path`. Creates parent directories as needed.
	// progress_cb is called on the download thread; it is safe to call this
	// from any thread. Returns true on success, false on any error.
	static bool Download(const std::string& url, const std::string& dest_path, ProgressCallback progress_cb = nullptr,
	                     std::string* error_out = nullptr);

	// Returns the default model storage directory:
	//   %LocalAppDataLow%/WKOpenVR/captions/models
	// (forward slashes used in the comment to avoid a trailing-backslash
	// line-continuation that would eat the declaration below).
	static std::string DefaultModelDir();
};
