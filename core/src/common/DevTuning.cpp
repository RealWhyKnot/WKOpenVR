#include "DevTuning.h"

#include "BuildChannel.h"

#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <unordered_map>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace openvr_pair::common::devtuning {

namespace {

using Map = std::unordered_map<std::string, double>;

// Snapshot swapped wholesale on reload so hot-path readers never see a
// half-updated map. A plain mutex is enough: readers copy the shared_ptr under
// the lock (uncontended in practice -- writes happen ~1 Hz), then look up on the
// copy outside it. Dev-only, so simplicity beats a lock-free scheme.
std::mutex g_mutex;
std::shared_ptr<const Map> g_snapshot = std::make_shared<const Map>();

// Reload bookkeeping. Only touched from MaybeReloadFromFile, which the driver
// calls from a single thread, so no synchronization is needed here.
uint64_t g_lastWrite = 0; // FILETIME of the file at the last successful read
bool g_hadFile = false;   // whether the file existed at the last check

std::string Trim(const std::string& s)
{
	size_t b = s.find_first_not_of(" \t\r\n");
	if (b == std::string::npos) return {};
	size_t e = s.find_last_not_of(" \t\r\n");
	return s.substr(b, e - b + 1);
}

std::shared_ptr<const Map> Parse(const std::string& text)
{
	auto map = std::make_shared<Map>();
	std::istringstream in(text);
	std::string line;
	while (std::getline(in, line)) {
		// Strip a trailing/whole-line comment.
		size_t hash = line.find_first_of("#;");
		if (hash != std::string::npos) line.resize(hash);
		size_t eq = line.find('=');
		if (eq == std::string::npos) continue;
		std::string key = Trim(line.substr(0, eq));
		std::string val = Trim(line.substr(eq + 1));
		if (key.empty() || val.empty()) continue;
		char* end = nullptr;
		double parsed = std::strtod(val.c_str(), &end);
		if (end == val.c_str()) continue; // no number consumed -> skip
		(*map)[key] = parsed;
	}
	return map;
}

void Swap(std::shared_ptr<const Map> next)
{
	std::lock_guard<std::mutex> lk(g_mutex);
	g_snapshot = std::move(next);
}

} // namespace

double Get(const char* key, double def)
{
	if (!key) return def;
	std::shared_ptr<const Map> snap;
	{
		std::lock_guard<std::mutex> lk(g_mutex);
		snap = g_snapshot;
	}
	if (!snap) return def;
	auto it = snap->find(key);
	return it != snap->end() ? it->second : def;
}

bool MaybeReloadFromFile(const std::wstring& path)
{
#if WKOPENVR_BUILD_IS_DEV
	WIN32_FILE_ATTRIBUTE_DATA attr{};
	const bool exists = GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &attr) != 0 &&
	                    !(attr.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY);

	if (!exists) {
		// File removed since last read -> revert to defaults exactly once.
		if (g_hadFile) {
			g_hadFile = false;
			g_lastWrite = 0;
			Swap(std::make_shared<const Map>());
			return true;
		}
		return false;
	}

	const uint64_t write =
	    (static_cast<uint64_t>(attr.ftLastWriteTime.dwHighDateTime) << 32) | attr.ftLastWriteTime.dwLowDateTime;
	if (g_hadFile && write == g_lastWrite) return false;

	std::ifstream f(path);
	if (!f) return false;
	std::ostringstream ss;
	ss << f.rdbuf();

	g_hadFile = true;
	g_lastWrite = write;
	Swap(Parse(ss.str()));
	return true;
#else
	(void)path;
	return false;
#endif
}

void ApplyTextForTest(const std::string& text)
{
	Swap(Parse(text));
}

} // namespace openvr_pair::common::devtuning
