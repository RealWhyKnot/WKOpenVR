#include "FaceOscPublisher.h"

#include <Windows.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>

namespace facetracking {

namespace {

static uint64_t FileTimeToU64(const FILETIME &ft)
{
    ULARGE_INTEGER uli;
    uli.LowPart = ft.dwLowDateTime;
    uli.HighPart = ft.dwHighDateTime;
    return uli.QuadPart;
}

static uint64_t AllowListStamp(const std::wstring &path)
{
    WIN32_FILE_ATTRIBUTE_DATA data;
    if (!GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &data)) return 0;
    return FileTimeToU64(data.ftLastWriteTime) ^ static_cast<uint64_t>(data.nFileSizeLow) ^
           (static_cast<uint64_t>(data.nFileSizeHigh) << 32);
}

static bool ReadWholeFile(const std::wstring &path, std::string &out)
{
    FILE *f = nullptr;
    if (_wfopen_s(&f, path.c_str(), L"rb") != 0 || !f) return false;

    if (std::fseek(f, 0, SEEK_END) != 0) {
        std::fclose(f);
        return false;
    }
    long len = std::ftell(f);
    if (len < 0) {
        std::fclose(f);
        return false;
    }
    if (std::fseek(f, 0, SEEK_SET) != 0) {
        std::fclose(f);
        return false;
    }

    out.assign(static_cast<size_t>(len), '\0');
    if (len > 0) {
        const size_t read = std::fread(out.data(), 1, static_cast<size_t>(len), f);
        if (read != static_cast<size_t>(len)) {
            std::fclose(f);
            return false;
        }
    }

    std::fclose(f);
    return true;
}

static std::string TrimAddressLine(const std::string &line)
{
    size_t first = 0;
    while (first < line.size() && (line[first] == ' ' || line[first] == '\t' || line[first] == '\r')) {
        ++first;
    }

    size_t last = line.size();
    while (last > first && (line[last - 1] == ' ' || line[last - 1] == '\t' ||
                            line[last - 1] == '\r')) {
        --last;
    }

    return line.substr(first, last - first);
}

static bool StartsWith(const std::string &value, const char *prefix)
{
    const size_t prefixLen = std::strlen(prefix);
    return value.size() >= prefixLen &&
           std::memcmp(value.data(), prefix, prefixLen) == 0;
}

static std::string CompatibleKey(const std::string &address)
{
    static const char kAvatarPrefix[] = "/avatar/parameters/";

    std::string body = StartsWith(address, kAvatarPrefix)
        ? address.substr(sizeof(kAvatarPrefix) - 1)
        : address;

    if (StartsWith(body, "v2/")) return body;

    const size_t nestedV2 = body.rfind("/v2/");
    if (nestedV2 != std::string::npos) {
        return body.substr(nestedV2 + 1);
    }

    return body;
}

} // namespace

FaceOscAddressFilter::FaceOscAddressFilter(std::wstring path)
    : path_(std::move(path))
{
}

bool FaceOscAddressFilter::ReloadIfChanged()
{
    if (path_.empty()) return false;

    const uint64_t stamp = AllowListStamp(path_);
    if (loaded_ && stamp == file_stamp_) return false;

    file_stamp_ = stamp;
    loaded_ = true;
    allowed_.clear();
    compatible_.clear();

    std::string body;
    if (stamp == 0 || !ReadWholeFile(path_, body)) return true;

    size_t offset = 0;
    while (offset <= body.size()) {
        const size_t newline = body.find('\n', offset);
        const size_t end = (newline == std::string::npos) ? body.size() : newline;
        const std::string line = TrimAddressLine(body.substr(offset, end - offset));
        if (!line.empty()) {
            allowed_.insert(line);
            compatible_.try_emplace(CompatibleKey(line), line);
        }
        if (newline == std::string::npos) break;
        offset = newline + 1;
    }

    return true;
}

bool FaceOscAddressFilter::Allows(const char *address) const
{
    if (path_.empty()) return true;
    if (!address || !loaded_) return false;
    return allowed_.find(address) != allowed_.end();
}

const std::string *FaceOscAddressFilter::CompatibleAddress(const char *address) const
{
    if (path_.empty() || !address || !loaded_) return nullptr;
    auto it = compatible_.find(CompatibleKey(address));
    return it == compatible_.end() ? nullptr : &it->second;
}

uint32_t FaceOscAddressFilter::AllowedCount() const
{
    return static_cast<uint32_t>(allowed_.size());
}

} // namespace facetracking
