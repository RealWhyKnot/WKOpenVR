#pragma once

#include "Protocol.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace facetracking {

struct FaceOscPublishCounts
{
    uint32_t sent = 0;
    uint32_t dropped = 0;

    void Add(const FaceOscPublishCounts &other)
    {
        sent += other.sent;
        dropped += other.dropped;
    }
};

class FaceOscAddressFilter
{
public:
    FaceOscAddressFilter() = default;
    explicit FaceOscAddressFilter(std::wstring path);

    bool ReloadIfChanged();
    bool Allows(const char *address) const;
    const std::string *CompatibleAddress(const char *address) const;
    bool Active() const { return !path_.empty(); }
    uint32_t AllowedCount() const;

private:
    std::wstring path_;
    uint64_t file_stamp_ = UINT64_MAX;
    bool loaded_ = false;
    std::unordered_set<std::string> allowed_;
    std::unordered_map<std::string, std::string> compatible_;
};

// Publishes one already-filtered face frame through the OSC router. Eye data
// and expression data are gated by FaceTrackingFrameBody::flags.
FaceOscPublishCounts PublishFaceFrameOsc(
    const protocol::FaceTrackingFrameBody &frame,
    const FaceOscAddressFilter *filter = nullptr);

const char *FaceExpressionOscName(uint32_t index);

} // namespace facetracking
