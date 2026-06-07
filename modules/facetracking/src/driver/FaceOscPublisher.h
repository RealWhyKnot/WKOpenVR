#pragma once

#include "Protocol.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace facetracking {

struct FaceOscPublishCounts
{
	uint32_t attempted = 0;
	uint32_t sent = 0;
	uint32_t dropped = 0;
	uint32_t filtered = 0;
	uint32_t deduped = 0;
	uint32_t remapped = 0;

	void Add(const FaceOscPublishCounts& other)
	{
		attempted += other.attempted;
		sent += other.sent;
		dropped += other.dropped;
		filtered += other.filtered;
		deduped += other.deduped;
		remapped += other.remapped;
	}
};

enum class FaceOscAddressFilterLoadStatus : uint8_t
{
	NotConfigured,
	Missing,
	ReadFailed,
	Loaded,
};

const char* FaceOscAddressFilterLoadStatusName(FaceOscAddressFilterLoadStatus status);

class FaceOscAddressFilter
{
public:
	FaceOscAddressFilter() = default;
	explicit FaceOscAddressFilter(std::wstring path);

	bool ReloadIfChanged();
	bool Allows(const char* address) const;
	const std::string* CompatibleAddress(const char* address) const;
	// Filtering only engages once a non-empty allowlist is loaded. An empty
	// allowlist means the active avatar's parameter list is unavailable (the
	// OSCQuery config has not arrived yet, or the file is empty); pass every
	// address through in that case rather than suppressing all output.
	bool Active() const { return !path_.empty() && !allowed_.empty(); }
	uint32_t AllowedCount() const;
	FaceOscAddressFilterLoadStatus LastLoadStatus() const { return load_status_; }

private:
	std::wstring path_;
	uint64_t file_stamp_ = UINT64_MAX;
	bool loaded_ = false;
	FaceOscAddressFilterLoadStatus load_status_ = FaceOscAddressFilterLoadStatus::NotConfigured;
	std::unordered_set<std::string> allowed_;
	std::unordered_map<std::string, std::string> compatible_;
};

// Publishes one already-filtered face frame through the OSC router. Eye data
// and expression data are gated by FaceTrackingFrameBody::flags. When `manifest`
// is non-null, every emitted (final, post-filter) address is appended to it --
// a one-shot diagnostic for seeing exactly which OSC parameters are sent.
FaceOscPublishCounts PublishFaceFrameOsc(const protocol::FaceTrackingFrameBody& frame,
                                         const FaceOscAddressFilter* filter = nullptr,
                                         std::vector<std::string>* manifest = nullptr);

const char* FaceExpressionOscName(uint32_t index);

} // namespace facetracking
