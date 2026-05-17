#include "Config.h"

#include "Win32Paths.h"

#include <cstdio>
#include <string>
#include <vector>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace
{
    std::wstring ConfigDir()
    {
        return openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
    }

    std::wstring ConfigPath()
    {
        std::wstring dir = ConfigDir();
        if (dir.empty()) return {};
        return dir + L"\\smoothing.txt";
    }
}

SmoothingConfig LoadConfig()
{
    SmoothingConfig cfg;
    std::wstring path = ConfigPath();
    if (path.empty()) return cfg;

    FILE *f = _wfopen(path.c_str(), L"r");
    if (!f) return cfg;

    char line[256];
    while (fgets(line, sizeof line, f)) {
        // Strip trailing CR/LF.
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        const char *key = line;
        const char *val = eq + 1;

        if (strcmp(key, "smoothness") == 0) {
            int n = atoi(val);
            if (n < 0) n = 0;
            if (n > 100) n = 100;
            cfg.smoothness = n;
        } else if (strcmp(key, "finger_mask") == 0) {
            unsigned n = (unsigned)strtoul(val, nullptr, 0);
            cfg.finger_mask = (uint16_t)(n & 0xFFFFu);
        } else if (strncmp(key, "per_finger_smoothness.", 22) == 0) {
            // Key format: per_finger_smoothness.<index>. Value: 0..100.
            int idx = atoi(key + 22);
            int n   = atoi(val);
            if (n < 0) n = 0;
            if (n > 100) n = 100;
            if (idx >= 0 && idx < 10) cfg.per_finger_smoothness[idx] = n;
        } else if (strncmp(key, "tracker_smoothness.", 19) == 0) {
            // Key format: tracker_smoothness.<serial>. Value: 0..100.
            // Serials are alphanumeric/short so no escaping needed in either
            // half of the key=value line.
            int n = atoi(val);
            if (n < 0) n = 0;
            if (n > 100) n = 100;
            if (n > 0) cfg.trackerSmoothness[std::string(key + 19)] = n;
        } else if (strcmp(key, "smart_smoothing") == 0) {
            cfg.smart_smoothing = (atoi(val) != 0);
        }
    }
    fclose(f);
    return cfg;
}

void SaveConfig(const SmoothingConfig &cfg)
{
    std::wstring path = ConfigPath();
    if (path.empty()) return;

    // Serialize to an in-memory buffer first. Writing to <path>.tmp and then
    // MoveFileExW(REPLACE_EXISTING) gives us atomicity: a crash mid-write
    // leaves the existing smoothing.txt untouched rather than truncated.
    std::string body;
    body.reserve(512);
    auto appendf = [&](const char *fmt, auto&&... args) {
        char buf[256];
        int n = std::snprintf(buf, sizeof buf, fmt, std::forward<decltype(args)>(args)...);
        if (n > 0) body.append(buf, (size_t)n);
    };
    appendf("smoothness=%d\n", cfg.smoothness);
    appendf("finger_mask=%u\n", (unsigned)cfg.finger_mask);
    appendf("smart_smoothing=%d\n", cfg.smart_smoothing ? 1 : 0);
    for (int i = 0; i < 10; ++i) {
        // Only write non-zero entries so the on-disk file stays small and a
        // hand-edit removing a line restores the global-fallback default.
        if (cfg.per_finger_smoothness[i] > 0) {
            appendf("per_finger_smoothness.%d=%d\n", i, cfg.per_finger_smoothness[i]);
        }
    }
    for (const auto &kv : cfg.trackerSmoothness) {
        // 0 values are dropped from the map on slider release; anything
        // still here is meaningful and should round-trip.
        appendf("tracker_smoothness.%s=%d\n", kv.first.c_str(), kv.second);
    }

    std::wstring tmpPath = path + L".tmp";
    HANDLE h = CreateFileW(tmpPath.c_str(), GENERIC_WRITE, 0, nullptr,
        CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return;
    DWORD written = 0;
    BOOL ok = WriteFile(h, body.data(), (DWORD)body.size(), &written, nullptr);
    CloseHandle(h);
    if (!ok || written != (DWORD)body.size()) {
        DeleteFileW(tmpPath.c_str());
        return;
    }
    // Atomic replace. Logs nothing on failure -- the existing file is intact;
    // the next SaveConfig will retry.
    MoveFileExW(tmpPath.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING);
}
