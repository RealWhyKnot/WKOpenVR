#pragma once

#include <chrono>
#include <string>

namespace facetracking {

struct AvatarStateSnapshot
{
	bool valid = false;
	std::string avatar_id;
	std::string config_path;
	std::string updated_at_utc;
};

class AvatarStatePoller
{
public:
	AvatarStatePoller();

	void Tick();

	const AvatarStateSnapshot& Snapshot() const noexcept { return snapshot_; }
	const std::string& PathUtf8() const noexcept { return path_utf8_; }

private:
	void ResolvePath();
	void ReadFile();

	std::string path_utf8_;
	std::chrono::steady_clock::time_point last_read_attempt_{};
	int64_t last_observed_mtime_ = 0;
	AvatarStateSnapshot snapshot_;
};

} // namespace facetracking
