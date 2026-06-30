#include "VirtualTrackerManager.h"

#include "DebugLogging.h"
#include "Logging.h"

#include <cstddef>

namespace phantom {

VirtualTrackerManager::VirtualTrackerManager() = default;

void VirtualTrackerManager::OnDriverInit()
{
	init_time_ = std::chrono::steady_clock::now();
}

void VirtualTrackerManager::SetEnabled(BodyRole role, bool enabled)
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= enabled_.size()) return;
	const bool was_enabled = enabled_[idx];
	enabled_[idx] = enabled;
	if (enabled && !was_enabled) {
		char profilePath[160] = {};
		const bool hasProfile = BodyRoleInputProfilePath(role, profilePath, sizeof(profilePath));
		LOG("[phantom] virtual role %s enabled; profile=%s publishing_gate=%d", BodyRoleToKey(role),
		    hasProfile ? profilePath : "(none)", MasterEnabled() ? 1 : 0);
	}
	else if (!enabled && devices_[idx]) {
		// SteamVR does not honour TrackedDeviceRemoved live for generic
		// trackers; the slot stays activated for the rest of the session.
		// Close the gate now so GetPose reports disconnected immediately;
		// the next Tick pushes a disconnected pose through the safe channel
		// so SteamVR hides the device promptly instead of floating its last
		// pose. Next vrserver restart will not re-activate it.
		devices_[idx]->SetReportConnected(false);
		LOG("[phantom] virtual role %s disabled; reporting disconnected "
		    "(virtual device retained until vrserver restart)",
		    BodyRoleToKey(role));
	}
}

bool VirtualTrackerManager::IsEnabled(BodyRole role) const
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= enabled_.size()) return false;
	return enabled_[idx];
}

void VirtualTrackerManager::SetRoleBlocked(BodyRole role, bool blocked)
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= blocked_.size()) return;
	const bool was_blocked = blocked_[idx];
	blocked_[idx] = blocked;
	// A newly-blocked role must stop floating just like a disabled one: close the
	// gate now (GetPose reports disconnected) and let the next Tick push the
	// disconnected pose. Unblocking reopens via Tick's next valid Publish.
	if (blocked && !was_blocked && devices_[idx]) {
		devices_[idx]->SetReportConnected(false);
	}
	if (was_blocked != blocked && BodyRoleToControllerType(role) != nullptr) {
		LOG("[phantom] virtual role %s %s by physical tracker assignment", BodyRoleToKey(role),
		    blocked ? "blocked" : "unblocked");
	}
}

bool VirtualTrackerManager::IsRoleBlocked(BodyRole role) const
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= blocked_.size()) return false;
	return blocked_[idx];
}

int VirtualTrackerManager::EnabledCount() const
{
	int n = 0;
	for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
		const auto role = static_cast<BodyRole>(i);
		if (BodyRoleToControllerType(role) != nullptr && enabled_[i]) ++n;
	}
	return n;
}

void VirtualTrackerManager::SetMasterEnabled(bool enabled)
{
	const bool previous = master_enabled_.exchange(enabled, std::memory_order_acq_rel);
	if (previous != enabled) {
		LOG("[phantom] virtual tracker publishing gate %s (enabled_roles=%d active=%d)",
		    enabled ? "enabled" : "disabled", EnabledCount(), ActiveCount());
	}
}

bool VirtualTrackerManager::MasterEnabled() const
{
	return master_enabled_.load(std::memory_order_acquire);
}

namespace {

vr::DriverPose_t PoseFromCompletion(const vr::DriverPose_t& hmd_pose, const BodyCompletionRoleOutput& role)
{
	vr::DriverPose_t out = hmd_pose;
	out.poseTimeOffset = 0.0;
	out.vecPosition[0] = role.pose.position[0];
	out.vecPosition[1] = role.pose.position[1];
	out.vecPosition[2] = role.pose.position[2];
	out.vecVelocity[0] = role.pose.velocity[0];
	out.vecVelocity[1] = role.pose.velocity[1];
	out.vecVelocity[2] = role.pose.velocity[2];
	out.vecAcceleration[0] = 0.0;
	out.vecAcceleration[1] = 0.0;
	out.vecAcceleration[2] = 0.0;
	out.qRotation = {
	    role.pose.rotation[0],
	    role.pose.rotation[1],
	    role.pose.rotation[2],
	    role.pose.rotation[3],
	};
	out.vecAngularVelocity[0] = 0.0;
	out.vecAngularVelocity[1] = 0.0;
	out.vecAngularVelocity[2] = 0.0;
	out.vecAngularAcceleration[0] = 0.0;
	out.vecAngularAcceleration[1] = 0.0;
	out.vecAngularAcceleration[2] = 0.0;
	out.result = vr::TrackingResult_Running_OK;
	out.poseIsValid = true;
	out.deviceIsConnected = true;
	out.shouldApplyHeadModel = false;
	return out;
}

} // namespace

void VirtualTrackerManager::MaybeActivate(BodyRole role)
{
	const auto idx = static_cast<size_t>(role);
	if (idx >= devices_.size()) return;
	// Already live. A device that exists but is no longer Activated() (SteamVR
	// called Deactivate after a disconnect) falls through and is re-registered
	// below, reusing the same object so re-enable works within a session.
	if (devices_[idx] && devices_[idx]->Activated()) return;
	if (!MasterEnabled()) return;
	if (!enabled_[idx]) return;
	if (blocked_[idx]) return;
	if (!hmd_pose_seen_.load(std::memory_order_acquire)) return;

	// openvr#1536 mitigation: wait at least settle_delay_ past driver
	// init before any TrackedDeviceAdded call so SteamVR has finished
	// enumerating real devices.
	if (std::chrono::steady_clock::now() - init_time_ < settle_delay_) {
		return;
	}

	const bool reactivating = static_cast<bool>(devices_[idx]);
	if (!devices_[idx]) {
		devices_[idx] = std::make_unique<VirtualTrackerDevice>(role);
	}
	const std::string serial = devices_[idx]->Serial();
	if (!vr::VRServerDriverHost()) {
		LOG("[phantom] cannot activate virtual %s: VRServerDriverHost null", BodyRoleToKey(role));
		return;
	}
	const bool ok = vr::VRServerDriverHost()->TrackedDeviceAdded(serial.c_str(), vr::TrackedDeviceClass_GenericTracker,
	                                                             devices_[idx].get());
	if (!ok) {
		LOG("[phantom] TrackedDeviceAdded failed for virtual %s (serial=%s)", BodyRoleToKey(role), serial.c_str());
		return;
	}
	LOG("[phantom] virtual %s %s (serial=%s)", BodyRoleToKey(role), reactivating ? "re-registered" : "activated",
	    serial.c_str());
}

void VirtualTrackerManager::Tick(const vr::DriverPose_t& hmd_pose, const BodyCompletionResult& body,
                                 double min_confidence)
{
	if (!MasterEnabled()) return;
	const bool debug_logging_enabled = openvr_pair::common::IsDebugLoggingEnabled();
	if (debug_logging_enabled) ++diag_ticks_;

	if (hmd_pose.poseIsValid && hmd_pose.deviceIsConnected && hmd_pose.result == vr::TrackingResult_Running_OK) {
		hmd_pose_seen_.store(true, std::memory_order_release);
	}

	for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
		const auto role = static_cast<BodyRole>(i);
		if (BodyRoleToControllerType(role) == nullptr) continue; // HMD / hand roles not publishable
		MaybeActivate(role);

		auto& dev = devices_[i];
		if (!dev || !dev->Activated()) continue;

		const bool allow = enabled_[i] && !blocked_[i];
		if (!allow) {
			// Disabled or blocked: push a disconnected pose once (one-shot) so
			// SteamVR hides the device instead of floating its last valid pose.
			if (dev->PushedConnected()) {
				dev->PublishDisconnect();
				LOG("[phantom] virtual role %s retracted; disconnected pose queued", BodyRoleToKey(role));
			}
			static_pub_ticks_[i] = 0;
			float_warned_[i] = false;
			continue;
		}

		const auto& solved = body.roles[i];
		bool fresh_publish = false;
		if (solved.valid && solved.confidence >= min_confidence) {
			const vr::DriverPose_t pose = PoseFromCompletion(hmd_pose, solved);
			const double dx = pose.vecPosition[0] - last_pub_pos_[i][0];
			const double dy = pose.vecPosition[1] - last_pub_pos_[i][1];
			const double dz = pose.vecPosition[2] - last_pub_pos_[i][2];
			fresh_publish = (dx * dx + dy * dy + dz * dz) >= 1e-12;
			last_pub_pos_[i] = {pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]};
			dev->Publish(pose);
			if (debug_logging_enabled) ++diag_published_;
		}
		else if (!solved.valid) {
			if (debug_logging_enabled) ++diag_skip_invalid_;
		}
		else {
			if (debug_logging_enabled) ++diag_skip_confidence_;
		}

		// Float watchdog: an enabled device whose published position has not
		// changed for kFloatWarnTicks ticks is stuck/floating; warn once.
		if (fresh_publish) {
			static_pub_ticks_[i] = 0;
			float_warned_[i] = false;
		}
		else if (++static_pub_ticks_[i] >= kFloatWarnTicks && !float_warned_[i]) {
			float_warned_[i] = true;
			LOG("[phantom][warn] virtual role %s pose unchanged for %u ticks; possible floating tracker",
			    BodyRoleToKey(role), static_pub_ticks_[i]);
		}
	}

	if (debug_logging_enabled) {
		const auto now = std::chrono::steady_clock::now();
		if (last_diag_log_ == std::chrono::steady_clock::time_point{}) {
			last_diag_log_ = now;
		}
		else if (now - last_diag_log_ >= std::chrono::seconds(5)) {
			LOG("[phantom][diag] virtual tick period ticks=%llu enabled=%d active=%d "
			    "hmd_seen=%u min_conf=%.2f published=%llu skip_invalid=%llu skip_conf=%llu",
			    static_cast<unsigned long long>(diag_ticks_), EnabledCount(), ActiveCount(),
			    hmd_pose_seen_.load(std::memory_order_acquire) ? 1u : 0u, min_confidence,
			    static_cast<unsigned long long>(diag_published_), static_cast<unsigned long long>(diag_skip_invalid_),
			    static_cast<unsigned long long>(diag_skip_confidence_));
			diag_ticks_ = 0;
			diag_published_ = 0;
			diag_skip_invalid_ = 0;
			diag_skip_confidence_ = 0;
			last_diag_log_ = now;
		}
	}
}

void VirtualTrackerManager::CollectPoseUpdates(std::vector<std::pair<uint32_t, vr::DriverPose_t>>& out)
{
	for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
		auto& dev = devices_[i];
		if (!dev || !dev->Activated()) continue;
		vr::DriverPose_t pose{};
		if (dev->TakePendingPose(pose)) {
			out.emplace_back(static_cast<uint32_t>(dev->ObjectId()), pose);
		}
	}
}

void VirtualTrackerManager::CollectDisconnects(std::vector<std::pair<uint32_t, vr::DriverPose_t>>& out)
{
	int n = 0;
	for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
		auto& dev = devices_[i];
		if (!dev || !dev->Activated()) continue;
		dev->SetReportConnected(false);
		out.emplace_back(static_cast<uint32_t>(dev->ObjectId()), MakeDisconnectedVirtualPose());
		++n;
	}
	if (n > 0) LOG("[phantom] collected %d virtual-tracker disconnect pose(s) for teardown", n);
}

void VirtualTrackerManager::LeakDevicesForProcessLifetime()
{
	// SteamVR keeps polling the raw device pointers it was handed (it ignores
	// live TrackedDeviceRemoved for generic trackers, openvr#1536), so the
	// objects must outlive this module. Move them into a never-freed store; their
	// gate is already closed, so GetPose reports disconnected until restart.
	static std::vector<std::unique_ptr<VirtualTrackerDevice>> s_retired;
	int n = 0;
	for (auto& dev : devices_) {
		if (!dev) continue;
		dev->SetReportConnected(false);
		s_retired.push_back(std::move(dev));
		++n;
	}
	if (n > 0) LOG("[phantom] retained %d virtual-tracker device(s) past shutdown (until vrserver restart)", n);
}

int VirtualTrackerManager::ActiveCount() const
{
	int n = 0;
	for (const auto& d : devices_)
		if (d && d->Activated()) ++n;
	return n;
}

} // namespace phantom
