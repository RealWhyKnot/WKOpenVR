#include "CalibrationProfileApply.h"

#include "CalibrationInternal.h"
#include "CalibrationMetrics.h"
#include "CalibrationPoseSampling.h"
#include "IpcSendQueue.h"
#include "MotionGate.h"
#include "HeadMountVisibility.h"
#include "ProtocolNames.h"
#include "TransformPayloadCompare.h"
#include "VRState.h"

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>

namespace {
// Per-device cache of the last SetDeviceTransform we sent to the driver. Used to
// suppress redundant IPC writes when ScanAndApplyProfile runs every tick during
// continuous calibration.
struct LastAppliedTransform
{
	bool valid = false;
	protocol::SetDeviceTransform payload{0u, false};
};
LastAppliedTransform g_lastApplied[vr::k_unMaxTrackedDeviceCount];

// Per-device serial cache. If a device ID gets reassigned to a different physical
// device (battery dies, pairing changes, SteamVR re-enumerates), we want to clear
// the stale per-ID state in the driver before applying any new transform.
std::string g_lastSeenSerial[vr::k_unMaxTrackedDeviceCount];

// Target system tracking. When the calibrated profile's target system changes
// (or the profile is cleared), we invalidate all per-ID caches so the next scan
// re-establishes correct enable/disable state.
std::string g_lastTargetSystem;
bool g_lastEnabled = false;

// AlignmentSpeedParams dedupe -- avoid spamming the driver with identical params.
protocol::AlignmentSpeedParams g_lastAlignmentSpeed{};
bool g_alignmentSpeedSent = false;

// Last per-tracking-system fallback we sent to the driver (deduped).
protocol::SetTrackingSystemFallback g_lastFallback{};
bool g_lastFallbackSent = false;

enum class ProfileApplySendKind
{
	Control,
	Device,
	Fallback
};

struct ProfileApplyStats
{
	double ipcMs = 0.0;
	int ipcSends = 0;
	int deviceSends = 0;
	int fallbackSends = 0;
};

constexpr double kSlowProfileApplyMs = 20.0;

using ProfileApplyClock = std::chrono::steady_clock;

double MsSince(ProfileApplyClock::time_point start)
{
	return std::chrono::duration<double, std::milli>(ProfileApplyClock::now() - start).count();
}

ProfileApplyStats* g_profileApplyStats = nullptr;

struct ProfileApplyStatsScope
{
	explicit ProfileApplyStatsScope(ProfileApplyStats& stats) : previous(g_profileApplyStats)
	{
		g_profileApplyStats = &stats;
	}

	~ProfileApplyStatsScope() { g_profileApplyStats = previous; }

	ProfileApplyStats* previous;
};

// Off-thread sender for the apply path. Transforms, fallbacks, and control
// params republish several times per second; pushing them through the render
// thread's blocking pipe measured 16-20 ms per apply cycle. The worker owns
// its own pipe connection (the driver serves unlimited instances) and
// preserves enqueue order; per-key coalescing keeps a slow driver from
// growing a backlog of superseded values. The tick-thread caches above stay
// tick-thread-only -- only the pipe write moves off-thread.
openvr_pair::overlay::IpcSendQueue g_profileApplyQueue;

void SendProfileApplyRequest(const protocol::Request& request, ProfileApplySendKind kind, std::string coalesceKey = {})
{
	if (!g_profileApplyQueue.IsRunning()) {
		g_profileApplyQueue.Start(OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME);
	}
	const auto start = ProfileApplyClock::now();
	g_profileApplyQueue.Enqueue(request, std::move(coalesceKey));
	if (!g_profileApplyStats) return;

	g_profileApplyStats->ipcMs += MsSince(start);
	++g_profileApplyStats->ipcSends;
	switch (kind) {
		case ProfileApplySendKind::Device:
			++g_profileApplyStats->deviceSends;
			break;
		case ProfileApplySendKind::Fallback:
			++g_profileApplyStats->fallbackSends;
			break;
		case ProfileApplySendKind::Control:
			break;
	}
}

void SetTargetSystemField(protocol::SetDeviceTransform& payload, const std::string& system)
{
	// Copy bounded by the buffer size; leave any remaining bytes zero so the
	// driver can read up to the first NUL or buffer end.
	memset(payload.target_system, 0, sizeof payload.target_system);
	size_t copyLen = system.size();
	if (copyLen >= sizeof payload.target_system) copyLen = sizeof payload.target_system - 1;
	memcpy(payload.target_system, system.data(), copyLen);
}

bool SendDeviceTransformIfChanged(uint32_t id, const protocol::SetDeviceTransform& payload)
{
	if (id >= vr::k_unMaxTrackedDeviceCount) return false;
	auto& cache = g_lastApplied[id];
	if (cache.valid && spacecal::apply::TransformPayloadNearEqual(cache.payload, payload)) {
		return false;
	}
	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = payload;
	SendProfileApplyRequest(req, ProfileApplySendKind::Device, "d" + std::to_string(id));
	cache.valid = true;
	cache.payload = payload;
	Metrics::LogAnnotationf("profile_apply_device_sent: id=%u enabled=%d target_system='%s'"
	                        " trans_cm=(%.2f,%.2f,%.2f) mag_cm=%.2f scale=%.4f lerp=%d quash=%d"
	                        " recalibrateOnMovement=%d state=%d",
	                        id, (int)payload.enabled, payload.target_system, payload.translation.v[0] * 100.0,
	                        payload.translation.v[1] * 100.0, payload.translation.v[2] * 100.0,
	                        std::sqrt(payload.translation.v[0] * payload.translation.v[0] +
	                                  payload.translation.v[1] * payload.translation.v[1] +
	                                  payload.translation.v[2] * payload.translation.v[2]) *
	                            100.0,
	                        payload.scale, (int)payload.lerp, (int)payload.quash, (int)payload.recalibrateOnMovement,
	                        (int)CalCtx.state);
	return true;
}

// Per-tracking-system cache so multi-ecosystem setups (3+ systems) don't
// thrash IPC: each system's fallback is compared against its OWN last-sent
// value. The previous single-slot g_lastFallback worked when only one
// system ever had a fallback active, but with extras we send N fallbacks
// per scan tick, and a single-slot cache would miss on every other call.
//
// Threading invariant: this map and the legacy single-slot g_lastFallback /
// g_lastFallbackSent above are written ONLY from the overlay's calibration
// tick (SpaceCalibratorUmbrellaRuntime::Tick -> CalibrationTick) and
// InvalidateAllTransformCaches, both of which run on the overlay main
// thread. Adding a UI handler or background worker that mutates these
// would introduce a race and requires adding synchronisation here first.
std::unordered_map<std::string, protocol::SetTrackingSystemFallback> g_lastFallbacksBySystem;

void SendFallbackIfChanged(const std::string& systemName, bool enabled, const Eigen::Vector3d& translationCm,
                           const Eigen::Quaterniond& rotation, double scale, uint8_t predictionSmoothness,
                           bool recalibrateOnMovement)
{
	protocol::SetTrackingSystemFallback payload{};
	size_t copyLen = systemName.size();
	if (copyLen >= sizeof payload.system_name) copyLen = sizeof payload.system_name - 1;
	memcpy(payload.system_name, systemName.data(), copyLen);
	payload.enabled = enabled;
	Eigen::Vector3d trans = translationCm * 0.01; // cm -> m, matches per-ID convention
	payload.translation.v[0] = trans.x();
	payload.translation.v[1] = trans.y();
	payload.translation.v[2] = trans.z();
	payload.rotation.w = rotation.w();
	payload.rotation.x = rotation.x();
	payload.rotation.y = rotation.y();
	payload.rotation.z = rotation.z();
	payload.scale = scale;
	payload.predictionSmoothness = predictionSmoothness;
	payload.recalibrateOnMovement = recalibrateOnMovement;

	auto it = g_lastFallbacksBySystem.find(systemName);
	if (it != g_lastFallbacksBySystem.end() && spacecal::apply::FallbackPayloadNearEqual(it->second, payload)) {
		return;
	}

	protocol::Request req(protocol::RequestSetTrackingSystemFallback);
	req.setTrackingSystemFallback = payload;
	SendProfileApplyRequest(req, ProfileApplySendKind::Fallback, "f" + systemName);
	g_lastFallbacksBySystem[systemName] = payload;
	// Legacy single-slot cache kept for any code that still reads it.
	g_lastFallback = payload;
	g_lastFallbackSent = true;
	Metrics::LogAnnotationf("profile_apply_fallback_sent: system='%s' enabled=%d"
	                        " trans_cm=(%.2f,%.2f,%.2f) mag_cm=%.2f scale=%.4f"
	                        " recalibrateOnMovement=%d state=%d",
	                        systemName.c_str(), (int)enabled, translationCm.x(), translationCm.y(), translationCm.z(),
	                        translationCm.norm(), scale, (int)recalibrateOnMovement, (int)CalCtx.state);
}

void InvalidateTransformCacheForId(uint32_t id)
{
	if (id >= vr::k_unMaxTrackedDeviceCount) return;
	g_lastApplied[id].valid = false;
	g_lastSeenSerial[id].clear();
}

bool ResolveHeadMountHide(uint32_t id, const std::string& trackingSystem)
{
	if (id >= vr::k_unMaxTrackedDeviceCount) return false;
	return wkopenvr::headmount::ShouldHideHeadMountTracker(CalCtx, id, g_lastSeenSerial[id], trackingSystem);
}

} // namespace

void InvalidateAllTransformCaches()
{
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		g_lastApplied[id].valid = false;
		g_lastSeenSerial[id].clear();
	}
	g_alignmentSpeedSent = false;
	g_lastFallbackSent = false;
	g_lastFallbacksBySystem.clear();
}

void StopProfileApplyWorker()
{
	g_profileApplyQueue.Stop();
}

void ResetAndDisableOffsets(uint32_t id, const std::string& trackingSystem)
{
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0;
	zeroQ.y = 0;
	zeroQ.z = 0;
	zeroQ.w = 1;

	protocol::SetDeviceTransform payload{id, false, zeroV, zeroQ, 1.0};
	SetTargetSystemField(payload, trackingSystem);
	// Carry head-mounted tracker hide through disable payloads too. This keeps
	// the selected tracker hidden by serial when continuous calibration is idle,
	// reacquiring, or temporarily not applying a live target transform.
	payload.updateQuash = true;
	payload.quash = ResolveHeadMountHide(id, trackingSystem);
	SendDeviceTransformIfChanged(id, payload);
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

// Per-scan record of which device IDs (and their human-friendly identity) we
// applied a per-target-system transform to last time. Used to log
// adopted/disconnected events when the set changes scan-over-scan.
namespace {
struct AdoptedTracker
{
	std::string serial;
	std::string model;
};
// Indexed by OpenVR ID. Empty entries mean the slot was not adopted last scan.
std::map<uint32_t, AdoptedTracker> g_lastAdoptedTrackers;
} // namespace

void ScanAndApplyProfile(CalibrationContext& ctx, bool forceSnapThisCycle, const char* forceSnapReason)
{
	if (!vr::VRSystem()) {
		ctx.enabled = ctx.validProfile;
		return;
	}

	const auto applyStart = ProfileApplyClock::now();
	ProfileApplyStats stats;
	ProfileApplyStatsScope statsScope(stats);

	// The dedupe caches below record what was ENQUEUED, not what reached the
	// driver. If the worker's pipe reconnected (driver lost its state) or any
	// entry was dropped (send failure, full queue), the caches are stale:
	// invalidate so this scan re-enqueues everything current.
	{
		const auto workerStatus = g_profileApplyQueue.GetStatus();
		static uint64_t s_lastSeenGeneration = 0;
		static uint64_t s_lastSeenFailures = 0;
		static uint64_t s_lastSeenDropped = 0;
		if (workerStatus.connectionGeneration != s_lastSeenGeneration ||
		    workerStatus.sendFailures != s_lastSeenFailures || workerStatus.dropped != s_lastSeenDropped) {
			const bool firstConnect =
			    (s_lastSeenGeneration == 0 && workerStatus.sendFailures == 0 && workerStatus.dropped == 0);
			s_lastSeenGeneration = workerStatus.connectionGeneration;
			s_lastSeenFailures = workerStatus.sendFailures;
			s_lastSeenDropped = workerStatus.dropped;
			if (!firstConnect) {
				InvalidateAllTransformCaches();
				Metrics::LogAnnotationf("profile_apply_caches_invalidated: reason=worker_state_change"
				                        " generation=%llu failures=%llu dropped=%llu",
				                        (unsigned long long)workerStatus.connectionGeneration,
				                        (unsigned long long)workerStatus.sendFailures,
				                        (unsigned long long)workerStatus.dropped);
			}
		}
	}

	std::unique_ptr<char[]> buffer_array(new char[vr::k_unMaxPropertyStringSize]);
	char* buffer = buffer_array.get();
	ctx.enabled = ctx.validProfile;

	// Auto-recovery snap (option-3 bundle, 2026-05-04). RecoverFromWedgedCalibration
	// sets g_snapNextProfileApply=true so the very next profile-apply cycle sends
	// every per-ID payload with lerp=false (driver snaps transform := target rather
	// than smoothly interpolating). Fallback payloads have no lerp field so they
	// can't snap directly -- but in practice every device that needs the cal has a
	// per-ID slot by the time recovery fires, so per-ID snap covers the user-visible
	// case. Captured at the top of the function and consumed at the end so all
	// per-ID sends in this cycle see the same value.
	const bool recoverySnapThisCycle = g_snapNextProfileApply;
	const bool snapThisCycle = recoverySnapThisCycle || forceSnapThisCycle;
	const char* snapReason =
	    recoverySnapThisCycle ? "recovery" : ((forceSnapReason && forceSnapReason[0]) ? forceSnapReason : "forced");

	// One-shot re-anchor: send reanchor=true (with lerp=true) so the driver ramps
	// to the profile at constant velocity. A snap this cycle takes precedence
	// (the driver clears the ramp on lerp=false), so they never conflict.
	const bool reanchorThisCycle = g_reanchorNextProfileApply && !snapThisCycle;

	// Snapshot of which IDs got adopted this scan and what serial/model they had.
	// Compared against g_lastAdoptedTrackers below to log new-adoption / disconnect events.
	std::map<uint32_t, AdoptedTracker> currentAdopted;
	int scanInvalidClass = 0;
	int scanDisabledProfile = 0;
	int scanTrackingSystemError = 0;
	int scanHmd = 0;
	int scanNonTargetSystem = 0;
	int scanTargetMatched = 0;
	int scanPayloadSent = 0;

	// If the calibrated target tracking system changed (or profile was loaded/cleared),
	// invalidate all per-ID caches so we re-establish correct state on every device.
	const bool targetSystemChanged = (ctx.targetTrackingSystem != g_lastTargetSystem);
	if (targetSystemChanged || ctx.enabled != g_lastEnabled) {
		// If we previously had a fallback registered for a now-stale system, tell
		// the driver to disable it so devices on that system stop receiving the
		// old offset. Done before InvalidateAllTransformCaches so the dedupe
		// shortcut doesn't suppress this.
		if (targetSystemChanged && !g_lastTargetSystem.empty() && g_lastFallbackSent && g_lastFallback.enabled) {
			protocol::SetTrackingSystemFallback disablePayload{};
			size_t copyLen = g_lastTargetSystem.size();
			if (copyLen >= sizeof disablePayload.system_name) copyLen = sizeof disablePayload.system_name - 1;
			memcpy(disablePayload.system_name, g_lastTargetSystem.data(), copyLen);
			disablePayload.enabled = false;
			disablePayload.rotation = {1, 0, 0, 0};
			disablePayload.scale = 1.0;
			protocol::Request req(protocol::RequestSetTrackingSystemFallback);
			req.setTrackingSystemFallback = disablePayload;
			SendProfileApplyRequest(req, ProfileApplySendKind::Fallback, "f" + g_lastTargetSystem);
		}

		InvalidateAllTransformCaches();
		g_lastTargetSystem = ctx.targetTrackingSystem;
		g_lastEnabled = ctx.enabled;
	}

	if (!g_alignmentSpeedSent ||
	    memcmp(&g_lastAlignmentSpeed, &ctx.alignmentSpeedParams, sizeof g_lastAlignmentSpeed) != 0) {
		protocol::Request setParamsReq(protocol::RequestSetAlignmentSpeedParams);
		setParamsReq.setAlignmentSpeedParams = ctx.alignmentSpeedParams;
		SendProfileApplyRequest(setParamsReq, ProfileApplySendKind::Control, "c-align");
		g_lastAlignmentSpeed = ctx.alignmentSpeedParams;
		g_alignmentSpeedSent = true;
	}

	// Push the per-tracking-system fallback so any device on `targetTrackingSystem`
	// that connects between scans inherits the calibrated offset on its first pose
	// update -- without waiting for the next per-ID scan. The fallback's freeze flag
	// fires whenever an external smoothing tool was detected and auto-suppress is on:
	// any newly-connected matching-system tracker (handled exclusively by the
	// fallback path until the next 1Hz scan tick promotes it to a per-ID transform)
	// gets clean-velocity behaviour from its very first pose update.
	if (ctx.enabled && !ctx.targetTrackingSystem.empty()) {
		auto euler = ctx.calibratedRotation * EIGEN_PI / 180.0;
		Eigen::Quaterniond rotQuat = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
		                             Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
		                             Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
		// Per-tracking-system fallback never carries a smoothness value: the
		// fallback applies to ANY device of that system that doesn't have an
		// active per-ID transform, including potentially the HMD or a freshly-
		// connected reference/target which we hard-block from suppression. The
		// per-ID path below sends per-tracker smoothness; the fallback path
		// stays at 0 to avoid surprise-suppressing a device the user didn't
		// individually opt in.
		SendFallbackIfChanged(ctx.targetTrackingSystem, true, ctx.calibratedTranslation, rotQuat, ctx.calibratedScale,
		                      /*predictionSmoothness=*/0, ctx.recalibrateOnMovement);
	}

	// Multi-ecosystem extras: each entry contributes its own per-tracking-
	// system fallback, applied to every device of that system that lacks a
	// per-ID transform. Driver-side, these go into separate slots in the
	// systemFallbacks[8] array, so they don't interfere. Each entry's
	// per-system fallback is sent only when the entry itself is valid + enabled
	// AND its target tracking system is non-empty AND distinct from the
	// primary's (sending a duplicate fallback for the primary's system would
	// race the primary's send above and cause flicker).
	for (const auto& extra : ctx.additionalCalibrations) {
		if (!extra.enabled || !extra.valid) continue;
		if (extra.targetTrackingSystem.empty()) continue;
		if (extra.targetTrackingSystem == ctx.targetTrackingSystem) continue;

		auto eulerE = extra.calibratedRotation * EIGEN_PI / 180.0;
		Eigen::Quaterniond rotQuatE = Eigen::AngleAxisd(eulerE(0), Eigen::Vector3d::UnitZ()) *
		                              Eigen::AngleAxisd(eulerE(1), Eigen::Vector3d::UnitY()) *
		                              Eigen::AngleAxisd(eulerE(2), Eigen::Vector3d::UnitX());
		SendFallbackIfChanged(extra.targetTrackingSystem, true, extra.calibratedTranslation, rotQuatE,
		                      extra.calibratedScale,
		                      /*predictionSmoothness=*/0, ctx.recalibrateOnMovement);
	}

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid) {
			++scanInvalidClass;
			// Device disappeared. Clear our cache for this slot so a future device
			// that gets assigned this ID starts from a known-clean state.
			if (!g_lastSeenSerial[id].empty() || g_lastApplied[id].valid) {
				InvalidateTransformCacheForId(id);
			}
			continue;
		}

		// Detect device-ID reuse: SteamVR can reassign an OpenVR ID to a different
		// physical device after the original disconnects. The driver's transforms[]
		// slot would otherwise apply the old offset to the new device.
		{
			char serialBuf[256] = {0};
			vr::ETrackedPropertyError serialErr = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, serialBuf,
			                                               sizeof serialBuf, &serialErr);
			std::string serial = (serialErr == vr::TrackedProp_Success) ? std::string(serialBuf) : std::string();
			if (g_lastSeenSerial[id] != serial) {
				const bool hadPriorSerial = !g_lastSeenSerial[id].empty();
				g_lastSeenSerial[id] = serial;
				if (hadPriorSerial) {
					// ID reassigned. Force a clean disable on the slot before any new
					// transform takes effect. Clear our local cache so the disable is
					// guaranteed to be sent (no dedupe match).
					g_lastApplied[id].valid = false;
					ResetAndDisableOffsets(id);
				}
			}
		}

		/*if (deviceClass == vr::TrackedDeviceClass_HMD) // for debugging unexpected universe switches
		{
		    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		    auto universeId = vr::VRSystem()->GetUint64TrackedDeviceProperty(id, vr::Prop_CurrentUniverseId_Uint64,
		&err); printf("uid %d err %d\n", universeId, err); ResetAndDisableOffsets(id); continue;
		}*/

		if (!ctx.enabled) {
			++scanDisabledProfile;
			ResetAndDisableOffsets(id);
			continue;
		}

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer,
		                                               vr::k_unMaxPropertyStringSize, &err);

		if (err != vr::TrackedProp_Success) {
			++scanTrackingSystemError;
			ResetAndDisableOffsets(id);
			continue;
		}

		std::string trackingSystem(buffer);

		if (id == vr::k_unTrackedDeviceIndex_Hmd) {
			++scanHmd;
			// auto p = ctx.devicePoses[id].mDeviceToAbsoluteTracking.m;
			// printf("HMD %d: %f %f %f\n", id, p[0][3], p[1][3], p[2][3]);

			// Check if the current HMD is a Pimax crystal
			if (trackingSystem == "aapvr") {
				// HMD is a Pimax HMD
				vr::HmdMatrix34_t eyeToHeadLeft = vr::VRSystem()->GetEyeToHeadTransform(vr::Eye_Left);
				// Crystal's projection matrix is constant 0s or 1s except for [0][3], which stores the IPD offset from
				// the nose
				bool isCrystalHmd =
				    eyeToHeadLeft.m[0][0] == 1 && eyeToHeadLeft.m[0][1] == 0 && eyeToHeadLeft.m[0][2] == 0 && // IPD
				    eyeToHeadLeft.m[1][0] == 0 && eyeToHeadLeft.m[1][1] == 1 && eyeToHeadLeft.m[1][2] == 0 &&
				    eyeToHeadLeft.m[1][3] == 0 && eyeToHeadLeft.m[2][0] == 0 && eyeToHeadLeft.m[2][1] == 0 &&
				    eyeToHeadLeft.m[2][2] == 1 && eyeToHeadLeft.m[2][3] == 0;

				if (isCrystalHmd) {
					// Move it outside the aapvr system ; we treat aapvr as if it were lighthouse
					trackingSystem = "Pimax Crystal HMD";
				}
			}

			if (trackingSystem != ctx.referenceTrackingSystem) {
				// Currently using an HMD with a different tracking system than the calibration.
				ctx.enabled = false;
			}

			ResetAndDisableOffsets(id, trackingSystem);
			continue;
		}

		// Detect Pimax crystal controllers and separate them too
		if (deviceClass == vr::TrackedDeviceClass_Controller) {
			if (trackingSystem == "oculus") {
				vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_RenderModelName_String, buffer,
				                                               vr::k_unMaxPropertyStringSize, &err);
				std::string renderModel(buffer);
				vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ConnectedWirelessDongle_String, buffer,
				                                               vr::k_unMaxPropertyStringSize, &err);
				std::string connectedWirelessDongle(buffer);

				// Check if the controller claims its an oculus controller but also pimax
				if (renderModel.find("{aapvr}") != std::string::npos &&
				    renderModel.find("crystal") != std::string::npos &&
				    connectedWirelessDongle.find("lighthouse") != std::string::npos) {
					trackingSystem = "Pimax Crystal Controllers";
				}
			}
		}

		if (trackingSystem != ctx.targetTrackingSystem) {
			++scanNonTargetSystem;
			ResetAndDisableOffsets(id, trackingSystem);
			continue;
		}
		++scanTargetMatched;

		const bool isFreshlyAdopted = !g_lastApplied[id].valid || !g_lastApplied[id].payload.enabled;

		protocol::SetDeviceTransform payload{id, true, VRTranslationVec(ctx.calibratedTranslation),
		                                     VRRotationQuat(ctx.calibratedRotation), ctx.calibratedScale};
		// During continuous calibration, lerp toward the smoothly-updating target so
		// the active offset doesn't snap on every cycle. EXCEPT when this is a freshly
		// adopted device -- those need to snap into place rather than ramping in from
		// identity, which would look like a slow drift to the user. ALSO except when
		// auto-recovery just fired (snapThisCycle): the recovery's brand-new cal must
		// land discontinuously, blending it would defeat the recovery.
		// Decision routed through the pure helper so test_motion_gate.cpp pins
		// the contract.
		payload.lerp = spacecal::motiongate::ShouldBlendCycle(
		    /*inContinuousState=*/CalCtx.state == CalibrationState::Continuous,
		    /*isFreshlyAdopted=*/isFreshlyAdopted,
		    /*snapThisCycle=*/snapThisCycle);
		// Re-anchor ramp request rides alongside lerp=true; the driver moves to
		// the target at a constant velocity instead of the proportional blend.
		payload.reanchor = reanchorThisCycle;
		// Hide intent: the continuous-target toggle hides the active target
		// during continuous calibration. The head-mounted tracker toggle is
		// serial-based so the same physical tracker remains hidden through
		// standby, reacquire, and OpenVR device-id churn.
		payload.quash =
		    wkopenvr::headmount::ShouldQuashPublishedTrackerPose(CalCtx, id, g_lastSeenSerial[id], trackingSystem);
		payload.updateQuash = true;
		SetTargetSystemField(payload, ctx.targetTrackingSystem);

		// predictionSmoothness moved to the Smoothing overlay on 2026-05-11
		// (Protocol v12). The driver ignores this field on SetDeviceTransform
		// from v12 onward; SC sends 0 to keep wire layout stable.
		payload.predictionSmoothness = 0;

		// Motion-gated blend -- when on, the driver-side BlendTransform's lerp
		// only advances proportional to detected per-frame motion. Hides offset
		// shifts in the user's natural movement; eliminates "phantom drift" while
		// stationary. Default on at the profile level.
		payload.recalibrateOnMovement = ctx.recalibrateOnMovement;

		if (SendDeviceTransformIfChanged(id, payload)) {
			++scanPayloadSent;
		}

		// Record this ID as adopted (it's receiving a per-target-system transform with
		// enabled=true). g_lastSeenSerial[id] is freshly populated above; pair it with
		// the model name for log readability. RenderModel falls back to empty string
		// on failure -- we don't gate the log on that.
		AdoptedTracker tracker;
		tracker.serial = g_lastSeenSerial[id];
		char modelBuf[256] = {0};
		vr::ETrackedPropertyError modelErr = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, modelBuf, sizeof modelBuf,
		                                               &modelErr);
		if (modelErr == vr::TrackedProp_Success) tracker.model = modelBuf;
		currentAdopted[id] = tracker;
	}

	// Diff against the previous scan: log new adoptions and disconnects. Skipped
	// when the profile is disabled so we don't spam the log on profile-clear.
	if (ctx.enabled) {
		for (const auto& kv : currentAdopted) {
			if (g_lastAdoptedTrackers.find(kv.first) == g_lastAdoptedTrackers.end()) {
				char buf[512];
				snprintf(buf, sizeof buf, "Adopted new tracker: %s/%s\n",
				         kv.second.model.empty() ? "(unknown model)" : kv.second.model.c_str(),
				         kv.second.serial.empty() ? "(no serial)" : kv.second.serial.c_str());
				CalCtx.Log(buf);
			}
		}
		for (const auto& kv : g_lastAdoptedTrackers) {
			if (currentAdopted.find(kv.first) == currentAdopted.end()) {
				char buf[512];
				snprintf(buf, sizeof buf, "Tracker disconnected: %s\n",
				         kv.second.serial.empty() ? "(no serial)" : kv.second.serial.c_str());
				CalCtx.Log(buf);
			}
		}
	}
	g_lastAdoptedTrackers = std::move(currentAdopted);
	{
		static double s_lastApplyScanSummary = -1e9;
		if (Metrics::CurrentTime - s_lastApplyScanSummary >= 2.0 || targetSystemChanged) {
			s_lastApplyScanSummary = Metrics::CurrentTime;
			const auto workerStatus = g_profileApplyQueue.GetStatus();
			char summaryBuf[768];
			snprintf(summaryBuf, sizeof summaryBuf,
			         "profile_apply_scan_summary: state=%d enabled=%d validProfile=%d"
			         " ref='%s' target='%s' profile_trans_cm=(%.2f,%.2f,%.2f)"
			         " profile_mag_cm=%.2f invalid=%d disabled=%d hmd=%d"
			         " trackingErr=%d nonTarget=%d targetMatched=%d payloadSent=%d"
			         " adopted=%zu fallbackSent=%d snap=%d"
			         " worker_connected=%d worker_depth=%u worker_sent=%llu worker_failed=%llu"
			         " worker_coalesced=%llu worker_dropped=%llu",
			         (int)CalCtx.state, (int)ctx.enabled, (int)ctx.validProfile, ctx.referenceTrackingSystem.c_str(),
			         ctx.targetTrackingSystem.c_str(), ctx.calibratedTranslation.x(), ctx.calibratedTranslation.y(),
			         ctx.calibratedTranslation.z(), ctx.calibratedTranslation.norm(), scanInvalidClass,
			         scanDisabledProfile, scanHmd, scanTrackingSystemError, scanNonTargetSystem, scanTargetMatched,
			         scanPayloadSent, g_lastAdoptedTrackers.size(), (int)g_lastFallbackSent, (int)snapThisCycle,
			         (int)workerStatus.connected, workerStatus.queueDepth, (unsigned long long)workerStatus.sent,
			         (unsigned long long)workerStatus.sendFailures, (unsigned long long)workerStatus.coalesced,
			         (unsigned long long)workerStatus.dropped);
			Metrics::WriteLogAnnotation(summaryBuf);
		}
	}

	// Boundary/floor subsystem disabled.
	// if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply) {
	// 	uint32_t quadCount = 0;
	// 	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);
	//
	// 	if (quadCount != ctx.chaperone.geometry.size()) {
	// 		ApplyChaperoneBounds();
	// 	}
	// }

	// Consume the one-shot auto-recovery snap flag -- only after every per-ID
	// payload in this cycle has been sent, so the snap reaches all devices.
	// Subsequent cycles return to normal lerp behaviour.
	if (snapThisCycle) {
		if (recoverySnapThisCycle) {
			g_snapNextProfileApply = false;
		}
		char snapBuf[220];
		snprintf(snapBuf, sizeof snapBuf, "profile_apply_snap_cycle_consumed: reason=%s payloadSent=%d", snapReason,
		         scanPayloadSent);
		Metrics::WriteLogAnnotation(snapBuf);
	}

	// Consume the one-shot re-anchor flag after all per-ID payloads are sent, so
	// every calibrated device starts its ramp on the same cycle.
	if (reanchorThisCycle) {
		g_reanchorNextProfileApply = false;
		char rbuf[160];
		snprintf(rbuf, sizeof rbuf, "profile_apply_reanchor_cycle_consumed: payloadSent=%d", scanPayloadSent);
		Metrics::WriteLogAnnotation(rbuf);
	}

	const double totalMs = MsSince(applyStart);
	if (totalMs >= kSlowProfileApplyMs) {
		const double nonIpcMs = std::max(0.0, totalMs - stats.ipcMs);
		Metrics::LogAnnotationf("profile_apply_slow: total_ms=%.1f ipc_ms=%.1f non_ipc_ms=%.1f"
		                        " ipc_sends=%d fallback_sends=%d device_sends=%d targetMatched=%d"
		                        " payloadSent=%d state=%d enabled=%d validProfile=%d snap=%d",
		                        totalMs, stats.ipcMs, nonIpcMs, stats.ipcSends, stats.fallbackSends, stats.deviceSends,
		                        scanTargetMatched, scanPayloadSent, (int)CalCtx.state, (int)ctx.enabled,
		                        (int)ctx.validProfile, (int)snapThisCycle);
	}
}
