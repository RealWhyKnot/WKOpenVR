#include "ServerTrackedDeviceProvider.h"
#include "DebugLogging.h"
#include "FeatureFlags.h"
#include "InterfaceHookInjector.h"
#include "IsometryTransform.h"
#include "Logging.h"
#include "ProcessPerfLog.h"
#include "PredictionSmoothingMath.h"
#include "RuntimeHealthSummary.h"
#include "ServerTrackedDeviceProviderConfigPacking.h"
#include "inputhealth/PathClassifier.h"
#include "inputhealth/SerialHash.h"
#include "quash/QuashPose.h"

// Forward declaration of the per-finger reseed entry point. Defined in
// modules/smoothing/src/driver/SkeletalHookInjector.cpp; the smoothing
// driver library is linked into this DLL via OPENVR_PAIR_FEATURE_LIBS.
// Including SkeletalHookInjector.h directly would require adding the
// smoothing module's source directory to this target's include path; one
// forward declaration here keeps the dependency edge narrow.
namespace skeletal { void MarkFingersNeedReseed(uint16_t fingerBits); }

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <random>

#ifndef OPENVR_PAIR_HAS_CALIBRATION_DRIVER
#define OPENVR_PAIR_HAS_CALIBRATION_DRIVER 0
#endif
#ifndef OPENVR_PAIR_HAS_SMOOTHING_DRIVER
#define OPENVR_PAIR_HAS_SMOOTHING_DRIVER 0
#endif
#ifndef OPENVR_PAIR_HAS_INPUTHEALTH_DRIVER
#define OPENVR_PAIR_HAS_INPUTHEALTH_DRIVER 0
#endif
#ifndef OPENVR_PAIR_HAS_FACETRACKING_DRIVER
#define OPENVR_PAIR_HAS_FACETRACKING_DRIVER 0
#endif
#ifndef OPENVR_PAIR_HAS_OSCROUTER_DRIVER
#define OPENVR_PAIR_HAS_OSCROUTER_DRIVER 0
#endif
#ifndef OPENVR_PAIR_HAS_CAPTIONS_DRIVER
#define OPENVR_PAIR_HAS_CAPTIONS_DRIVER 0
#endif
#ifndef OPENVR_PAIR_HAS_PHANTOM_DRIVER
#define OPENVR_PAIR_HAS_PHANTOM_DRIVER 0
#endif
#ifndef PAIRDRIVER_VERSION_STRING
#define PAIRDRIVER_VERSION_STRING "0.0.0.0-dev"
#endif

// Phantom hot-path entry points are declared in DriverModule.h next to the
// CreateDriverModule forward declaration; no additional include needed.

namespace {

std::string InputHealthPathString(const char *path)
{
	size_t n = 0;
	while (n < protocol::INPUTHEALTH_PATH_LEN && path[n] != '\0') ++n;
	return std::string(path, path + n);
}

// Smart-smoothing motion-ramp thresholds. Picked for hand-held VR motion:
// resting hand tremor sits around 5-10 deg/s and a few cm/s, while a walking
// head/hands sweep is on the order of 100+ deg/s and 50+ cm/s. Either axis
// going hot is enough to release suppression (max() not mean()) -- a slow
// aim adjustment moves the controller rotationally with almost no
// translation, and we still want to back off there.
constexpr double kSmartLinearStill   = 0.05;   // m/s
constexpr double kSmartLinearMoving  = 0.60;   // m/s
constexpr double kSmartAngularStill  = 0.10;   // rad/s  (~5.7 deg/s)
constexpr double kSmartAngularMoving = 2.00;   // rad/s  (~115  deg/s)
// EMA on the motion ramp itself. 0.15 ~= 5-frame lag at 90 Hz; keeps the
// effective factor from twitching on noisy IMU velocity without making the
// roll-off feel sticky.
constexpr double kSmartEmaAlpha      = 0.15;

double ComputeSmartMotionRamp(const vr::DriverPose_t &pose)
{
	const double vx = pose.vecVelocity[0];
	const double vy = pose.vecVelocity[1];
	const double vz = pose.vecVelocity[2];
	const double wx = pose.vecAngularVelocity[0];
	const double wy = pose.vecAngularVelocity[1];
	const double wz = pose.vecAngularVelocity[2];

	// SteamVR can fill the velocity fields with INF on a tracking
	// recovery edge (the driver-side IMU integrator briefly produces
	// non-finite values). sqrt(INF) = INF; clamp((INF - threshold) / span,
	// 0, 1) = 1; the EMA would saturate the motion ramp permanently and
	// disable the smart-motion blend suppressor. Treat any non-finite
	// component as "no motion data" and return the still ramp value.
	if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)
		|| !std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
		return 0.0;
	}

	const double linear = std::sqrt(vx * vx + vy * vy + vz * vz);
	const double angular = std::sqrt(wx * wx + wy * wy + wz * wz);

	const double posRamp = std::clamp(
		(linear  - kSmartLinearStill)  / (kSmartLinearMoving  - kSmartLinearStill),
		0.0, 1.0);
	const double rotRamp = std::clamp(
		(angular - kSmartAngularStill) / (kSmartAngularMoving - kSmartAngularStill),
		0.0, 1.0);
	return std::max(posRamp, rotRamp);
}

} // namespace

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	LOG("ServerTrackedDeviceProvider::Init begin version=%s protocol=%u build_is_dev=%d",
		PAIRDRIVER_VERSION_STRING, (unsigned)protocol::Version,
		(int)WKOPENVR_BUILD_IS_DEV);
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	// QPF is constant for the lifetime of the boot; capture it once instead of
	// querying inside BlendTransform on every pose update.
	QueryPerformanceFrequency(&qpcFreq);

	// Detect which subsystems to wire up. Driver scans its resources/ folder
	// for enable_calibration.flag and enable_smoothing.flag; the consumer
	// installers each drop their own flag at install time. With neither flag
	// the driver runs inert -- still loaded by SteamVR, but no hooks, no IPC,
	// no shmem.
	featureFlags = pairdriver::DetectFeatureFlags();
	LOG("Driver feature mask detected: 0x%08x", (unsigned)featureFlags);

	activeModules.clear();
	DriverModuleContext moduleContext{this, pDriverContext, featureFlags};
	auto activateModule = [&](std::unique_ptr<DriverModule> module) {
		if (!module) {
			LOG("Driver module factory returned null");
			return;
		}
		if ((featureFlags & module->FeatureMask()) == 0) {
			LOG("Driver module '%s' skipped; module_mask=0x%08x featureFlags=0x%08x",
				module->Name(), (unsigned)module->FeatureMask(), (unsigned)featureFlags);
			return;
		}
		if (!module->Init(moduleContext)) {
			LOG("Driver module '%s' failed to initialize", module->Name());
			return;
		}
		LOG("Driver module '%s' initialized", module->Name());
		activeModules.push_back(std::move(module));
	};

	// OscRouter must be first: other modules may call PublishOsc during Init.
#if OPENVR_PAIR_HAS_OSCROUTER_DRIVER
	activateModule(oscrouter::CreateDriverModule());
#endif
#if OPENVR_PAIR_HAS_CALIBRATION_DRIVER
	activateModule(calibration::CreateDriverModule());
#endif
#if OPENVR_PAIR_HAS_SMOOTHING_DRIVER
	activateModule(smoothing::CreateDriverModule());
#endif
#if OPENVR_PAIR_HAS_INPUTHEALTH_DRIVER
	activateModule(inputhealth::CreateDriverModule());
#endif
#if OPENVR_PAIR_HAS_FACETRACKING_DRIVER
	activateModule(facetracking::CreateDriverModule());
#endif
#if OPENVR_PAIR_HAS_CAPTIONS_DRIVER
	activateModule(captions::CreateDriverModule());
#endif
#if OPENVR_PAIR_HAS_PHANTOM_DRIVER
	activateModule(phantom::CreateDriverModule());
#endif

	if (featureFlags & pairdriver::kFeatureCalibration) {
		// Calibration setup: speed thresholds, pose telemetry shmem, IPC pipe.
		// transforms[] elements are zero/identity-initialized via DeviceTransform's
		// default member initializers and IsoTransform's default constructor; a
		// memset would be undefined behavior because Eigen members aren't trivially
		// copyable. AlignmentSpeedParams is a plain aggregate of doubles so memset
		// here is safe.
		memset(&alignmentSpeedParams, 0, sizeof alignmentSpeedParams);

		alignmentSpeedParams.thr_rot_tiny = 0.1f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_small = 1.0f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_large = 5.0f * (EIGEN_PI / 180.0f);

		alignmentSpeedParams.thr_trans_tiny = 0.1f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_small = 1.0f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_large = 20.0f / 1000.0; // mm

		alignmentSpeedParams.align_speed_tiny = 0.05f;
		alignmentSpeedParams.align_speed_small = 0.2f;
		alignmentSpeedParams.align_speed_large = 2.0f;

		// Slew-rate cap defaults. Stationary at the published lateral-drift
		// detection threshold in VR (~0.5 mm/sec); moving at 20x that so a
		// pending correction catches up in a few seconds once motion masks it.
		// Overridden per-session by the overlay's first SetAlignmentSpeedParams
		// after profile load.
		alignmentSpeedParams.slew_stationary_pos_rate = 0.0005;   // 0.5 mm/sec
		alignmentSpeedParams.slew_stationary_rot_rate = 0.000873; // ~0.05 deg/sec
		alignmentSpeedParams.slew_moving_pos_rate     = 0.010;    // 10  mm/sec
		alignmentSpeedParams.slew_moving_rot_rate     = 0.01745;  // ~1.0 deg/sec

		if (!shmem.Create(OPENVR_PAIRDRIVER_SHMEM_NAME)) {
			// Non-fatal: pose telemetry is not essential -- calibration still
			// works without it. Log so the overlay's diagnostics (or a post-
			// mortem grep) can surface the cause.
			LOG("shmem.Create(%s) failed (GetLastError=%u); pose telemetry disabled",
				OPENVR_PAIRDRIVER_SHMEM_NAME, (unsigned)GetLastError());
		}

		calibrationServer = std::make_unique<IPCServer>(
			this,
			OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME,
			pairdriver::kFeatureCalibration);
		LOG("Starting calibration IPC server pipe=%s", OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME);
		calibrationServer->Run();
	}

	if (featureFlags & pairdriver::kFeatureSmoothing) {
		smoothingServer = std::make_unique<IPCServer>(
			this,
			OPENVR_PAIRDRIVER_SMOOTHING_PIPE_NAME,
			pairdriver::kFeatureSmoothing);
		LOG("Starting smoothing IPC server pipe=%s", OPENVR_PAIRDRIVER_SMOOTHING_PIPE_NAME);
		smoothingServer->Run();
	}

	if (featureFlags & pairdriver::kFeatureInputHealth) {
		inputHealthServer = std::make_unique<IPCServer>(
			this,
			OPENVR_PAIRDRIVER_INPUTHEALTH_PIPE_NAME,
			pairdriver::kFeatureInputHealth);
		LOG("Starting inputhealth IPC server pipe=%s", OPENVR_PAIRDRIVER_INPUTHEALTH_PIPE_NAME);
		inputHealthServer->Run();
	}

	if (featureFlags & pairdriver::kFeatureFaceTracking) {
		faceTrackingServer = std::make_unique<IPCServer>(
			this,
			OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME,
			pairdriver::kFeatureFaceTracking);
		LOG("Starting facetracking IPC server pipe=%s", OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME);
		faceTrackingServer->Run();
	}

	if (featureFlags & pairdriver::kFeatureOscRouter) {
		oscRouterServer = std::make_unique<IPCServer>(
			this,
			OPENVR_PAIRDRIVER_OSCROUTER_PIPE_NAME,
			pairdriver::kFeatureOscRouter);
		LOG("Starting oscrouter IPC server pipe=%s", OPENVR_PAIRDRIVER_OSCROUTER_PIPE_NAME);
		oscRouterServer->Run();
	}

	if (featureFlags & pairdriver::kFeatureCaptions) {
		captionsServer = std::make_unique<IPCServer>(
			this,
			OPENVR_PAIRDRIVER_CAPTIONS_PIPE_NAME,
			pairdriver::kFeatureCaptions);
		LOG("Starting captions IPC server pipe=%s", OPENVR_PAIRDRIVER_CAPTIONS_PIPE_NAME);
		captionsServer->Run();
	}

	if (featureFlags & pairdriver::kFeaturePhantom) {
		phantomServer = std::make_unique<IPCServer>(
			this,
			OPENVR_PAIRDRIVER_PHANTOM_PIPE_NAME,
			pairdriver::kFeaturePhantom);
		LOG("Starting phantom IPC server pipe=%s", OPENVR_PAIRDRIVER_PHANTOM_PIPE_NAME);
		phantomServer->Run();
	}

	// Hook installation is gated inside the injector by the same feature
	// flags so the GetGenericInterface detour skips registering the
	// per-feature inner hooks for subsystems that aren't enabled.
	if (!InjectHooks(this, pDriverContext, featureFlags)) {
		LOG("InjectHooks failed; stopping IPC servers and shmem");
		// MH_Initialize failed. IPC servers and shmem are already up, but
		// without hooks the calibration and smoothing paths are dead. Report
		// failure so SteamVR unloads us cleanly rather than leaving the driver
		// in a zombie state where DisableHooks would call into uninitialized
		// MinHook on the way out.
		if (calibrationServer) calibrationServer->Stop();
		if (smoothingServer) smoothingServer->Stop();
		if (inputHealthServer) inputHealthServer->Stop();
		if (faceTrackingServer) faceTrackingServer->Stop();
		if (oscRouterServer) oscRouterServer->Stop();
		if (captionsServer) captionsServer->Stop();
		if (phantomServer) phantomServer->Stop();
		shmem.Close();
		VR_CLEANUP_SERVER_DRIVER_CONTEXT();
		return vr::VRInitError_Driver_Failed;
	}

	debugTransform = Eigen::Vector3d::Zero();
	debugRotation = Eigen::Quaterniond::Identity();

	LOG("ServerTrackedDeviceProvider::Init complete active_modules=%zu featureFlags=0x%08x",
		activeModules.size(), (unsigned)featureFlags);
	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::RunFrame()
{
	static openvr_pair::common::ProcessPerfSampler s_perfSampler;
	if (!openvr_pair::common::IsDebugLoggingEnabled()) {
		s_perfSampler.Reset();
		return;
	}

	openvr_pair::common::ProcessPerfSample perfSample{};
	if (!s_perfSampler.MaybeSample(perfSample)) return;

	const std::string line =
		openvr_pair::common::FormatProcessPerfSample("driver-host", perfSample);
	LOG("[perf] %s", line.c_str());
	openvr_pair::common::RecordRuntimeProcessSample("driver-host", perfSample);
	openvr_pair::common::MaybeWriteRuntimeHealthSummary(
		10000,
		L"runtime_health_driver_host.json");
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	LOG("ServerTrackedDeviceProvider::Cleanup begin active_modules=%zu featureFlags=0x%08x",
		activeModules.size(), (unsigned)featureFlags);

	// Order matters. The previous order (server.Stop -> shmem.Close ->
	// DisableHooks -> VR_CLEANUP) had a fatal race: DisableHooks removes
	// the MinHook patches but does NOT wait for in-flight detours to
	// return. SteamVR's pose-update detours fire ~kHz across all tracked
	// devices, the skeletal detour at ~340Hz/hand, so on every driver
	// unload there's a window where a detour body is still executing
	// inside our DLL while we tear down state below it -- and after
	// Cleanup returns SteamVR unmaps the DLL with that thread mid-call.
	//
	// New order:
	//   1. DisableHooks first -- removes patches AND drains in-flight
	//      detour callers before returning. After it returns no thread
	//      is executing inside any of our hook bodies.
	//   2. Stop both IPC servers (each joins its own worker thread).
	//   3. shmem.Close -- safe now because no detour can read it.
	//   4. VR_CLEANUP_SERVER_DRIVER_CONTEXT -- finalize.
	DisableHooks();
	for (auto it = activeModules.rbegin(); it != activeModules.rend(); ++it) {
		(*it)->Shutdown();
	}
	activeModules.clear();
	if (phantomServer) phantomServer->Stop();
	if (captionsServer) captionsServer->Stop();
	if (oscRouterServer) oscRouterServer->Stop();
	if (faceTrackingServer) faceTrackingServer->Stop();
	if (inputHealthServer) inputHealthServer->Stop();
	if (smoothingServer) smoothingServer->Stop();
	if (calibrationServer) calibrationServer->Stop();
	shmem.Close();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
	LOG("ServerTrackedDeviceProvider::Cleanup complete");
}

bool ServerTrackedDeviceProvider::HandleIpcRequest(
	uint32_t featureMask,
	const protocol::Request &request,
	protocol::Response &response)
{
	if (request.type == protocol::RequestHandshake) {
		response.type = protocol::ResponseHandshake;
		response.protocol.version = protocol::Version;
		return true;
	}

	for (auto &module : activeModules) {
		if ((module->FeatureMask() & featureMask) == 0) continue;
		if (module->HandleRequest(request, response)) return true;
	}

	return false;
}

void ServerTrackedDeviceProvider::OnGetGenericInterface(const char *pchInterface, void *iface)
{
	for (auto &module : activeModules) {
		module->OnGetGenericInterface(pchInterface, iface);
	}
}

namespace {


	vr::HmdQuaternion_t convert(const Eigen::Quaterniond& q) {
		vr::HmdQuaternion_t result;
		result.w = q.w();
		result.x = q.x();
		result.y = q.y();
		result.z = q.z();
		return result;
	}

	vr::HmdVector3_t convert(const Eigen::Vector3d& v) {
		vr::HmdVector3_t result;
		result.v[0] = (float) v.x();
		result.v[1] = (float) v.y();
		result.v[2] = (float) v.z();
		return result;
	}

	Eigen::Quaterniond convert(const vr::HmdQuaternion_t& q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
	}

	Eigen::Vector3d convert(const vr::HmdVector3d_t& v) {
		return Eigen::Vector3d(v.v[0], v.v[1], v.v[2]);
	}

	Eigen::Vector3d convert(const double* arr) {
		return Eigen::Vector3d(arr[0], arr[1], arr[2]);
	}

	IsoTransform toIsoWorldTransform(const vr::DriverPose_t& pose) {
		Eigen::Quaterniond rot(pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x, pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);
		Eigen::Vector3d trans(pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1], pose.vecWorldFromDriverTranslation[2]);

		return IsoTransform(rot, trans);
	}

	IsoTransform toIsoPose(const vr::DriverPose_t& pose) {
		auto worldXform = toIsoWorldTransform(pose);

		Eigen::Quaterniond rot(pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z);
		Eigen::Vector3d trans(pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]);

		return worldXform * IsoTransform(rot, trans);
	}
}





void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount) return;

	std::lock_guard<std::mutex> lock(stateMutex);

	auto &tf = transforms[newTransform.openVRID];
	const bool wasEnabled = tf.enabled;
	tf.enabled = newTransform.enabled;

	// Record the device's tracking system so we can match it against per-system
	// fallbacks for any future device that occupies this slot.
	{
		// target_system is a fixed-size buffer that may not be NUL-terminated if
		// fully populated; bound the read to the buffer length.
		size_t maxLen = sizeof newTransform.target_system;
		size_t len = 0;
		while (len < maxLen && newTransform.target_system[len] != '\0') ++len;
		deviceSystem[newTransform.openVRID].assign(newTransform.target_system, len);
		// Mark the lookup state so the pose-hook thread doesn't re-query the
		// property store for this slot. Empty target_system means "unknown" --
		// fall back to NotTried so the lazy lookup can still try (and then
		// throttle if it keeps failing).
		lookupState[newTransform.openVRID] = (len > 0) ? LookupState::Cached : LookupState::NotTried;
	}

	// Whenever the per-ID transform is updated (whether enabling or disabling), the
	// slot is no longer following a fallback. Subsequent HandleDevicePoseUpdated
	// calls will re-evaluate fallback eligibility.
	tf.fallbackActive = false;

	// predictionSmoothness used to be copied from newTransform here. From
	// Protocol v12 (2026-05-11) onward that slot is owned by the Smoothing
	// overlay and updated exclusively through SetDevicePrediction (see below).
	// The field still travels inside SetDeviceTransform for wire
	// compatibility, but the driver ignores it here so SC's per-frame
	// calibration pushes can't clobber what Smoothing last wrote.

	// Motion-gated blend stays owned by SC -- the lerp it gates is the
	// calibration blend, not pose prediction. SetDeviceTransform remains the
	// authoritative path for this flag.
	if (tf.recalibrateOnMovement && !newTransform.recalibrateOnMovement) {
		tf.blendMotionInitialized = false;
	}
	tf.recalibrateOnMovement = newTransform.recalibrateOnMovement;

	if (newTransform.updateTranslation) {
		tf.targetTransform.translation = convert(newTransform.translation);
		if (!newTransform.lerp) {
			tf.transform.translation = tf.targetTransform.translation;
		}
	}

	if (newTransform.updateRotation) {
		tf.targetTransform.rotation = convert(newTransform.rotation);

		if (!newTransform.lerp) {
			tf.transform.rotation = tf.targetTransform.rotation;
		}
	}

	if (newTransform.updateScale)
		tf.scale = newTransform.scale;

	// v23 (2026-05-19): only mutate stored hide when the caller explicitly
	// signals intent. Prior unconditional assignment let any partial-init
	// payload (e.g. ResetAndDisableOffsets) silently wipe a user-marked
	// always-hidden tracker.
	if (newTransform.updateQuash)
		tf.quash = newTransform.quash;

	// On enable transition, the slot's `transform` may be stale from a prior session
	// or never initialized. Snap to the target so we don't ramp in from a junk state.
	if (!wasEnabled && tf.enabled) {
		tf.transform = tf.targetTransform;
		// Forensic diagnostic for audit row #8 (project_upstream_regression_audit_2026-05-04).
		// If a sleeper bug ever puts a stale fallback transform into
		// `tf.transform` before this snap, the snap would lock-in the
		// staleness for the user. Surfacing every snap-on-enable lets
		// post-mortem grep `device_transform_snap_on_enable` reveal the
		// pattern. Once-per-event (driven by SetDeviceTransform IPC), so
		// no throttling needed.
		LOG("device_transform_snap_on_enable: id=%u target=(%.3f,%.3f,%.3f) prevFallbackActive=%d",
			(unsigned)newTransform.openVRID,
			tf.targetTransform.translation.x(),
			tf.targetTransform.translation.y(),
			tf.targetTransform.translation.z(),
			(int)tf.fallbackActive);
	}

	// On disable transition, drop any pending lerp target so a future re-enable
	// doesn't pick up where the last one left off.
	if (wasEnabled && !tf.enabled) {
		tf.targetTransform = tf.transform;
	}

	// Always reset the lerp clock and rate. If the device went offline for a long time,
	// the next BlendTransform would otherwise compute a huge dt and saturate to 1.0
	// (instant jump). Restarting the clock keeps lerp behavior bounded.
	QueryPerformanceCounter(&tf.lastPoll);
	tf.currentRate = DeltaSize::TINY;
}


void ServerTrackedDeviceProvider::SetDevicePrediction(const protocol::SetDevicePrediction &cfg)
{
	if (cfg.openVRID >= vr::k_unMaxTrackedDeviceCount) return;

	std::lock_guard<std::mutex> lock(stateMutex);

	auto &tf = transforms[cfg.openVRID];
	const uint8_t oldSmoothness = tf.predictionSmoothness;
	const bool    oldSmart      = tf.smartEnabled;
	const bool    newSmart      = cfg.smart_enabled != 0;

	// Cheap to write unconditionally; HandleDevicePoseUpdated reads it once
	// per pose update for the velocity / acceleration / poseTimeOffset
	// scaling. 0 = pose untouched, 100 = predictor fully defeated.
	tf.predictionSmoothness = cfg.predictionSmoothness;
	tf.smartEnabled         = newSmart;
#if WKOPENVR_BUILD_IS_DEV
	tf.smartShadowParams = prediction::smart_shadow::BuildParams(cfg.predictionSmoothness);
	if (oldSmoothness != cfg.predictionSmoothness) {
		tf.smartShadow = SmartSmoothingShadowState{};
	}
#endif
	if (newSmart && !oldSmart) {
		// Re-enable starts from a known state so the first frame after
		// toggle doesn't inherit a stale motion estimate from an earlier
		// enabled stretch.
		tf.smartMotionEma = 0.0;
	}
	if (oldSmoothness != cfg.predictionSmoothness || oldSmart != newSmart) {
		LOG("[prediction] SetDevicePrediction id=%u smoothness=%u old=%u smart=%s",
			cfg.openVRID,
			static_cast<unsigned>(cfg.predictionSmoothness),
			static_cast<unsigned>(oldSmoothness),
			newSmart ? "on" : "off");
	}
}

void ServerTrackedDeviceProvider::SetTrackingSystemFallback(const protocol::SetTrackingSystemFallback& newFallback)
{
	size_t maxLen = sizeof newFallback.system_name;
	size_t len = 0;
	while (len < maxLen && newFallback.system_name[len] != '\0') ++len;
	if (len == 0) return;

	std::lock_guard<std::mutex> lock(stateMutex);

	if (!newFallback.enabled) {
		// Disabling the fallback. Drop any per-ID slots that were following it so
		// they don't keep applying the stale offset on subsequent pose updates.
		if (auto* slot = FindFallbackSlot(newFallback.system_name, len)) {
			slot->tf.enabled = false;
		}
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			if (!transforms[id].fallbackActive) continue;
			const std::string& sys = deviceSystem[id];
			if (sys.size() == len && memcmp(sys.data(), newFallback.system_name, len) == 0) {
				transforms[id].fallbackActive = false;
				transforms[id].targetTransform = transforms[id].transform;
				transforms[id].currentRate = DeltaSize::TINY;
				QueryPerformanceCounter(&transforms[id].lastPoll);
			}
		}
		return;
	}

	auto* slot = AcquireFallbackSlot(newFallback.system_name, len);
	if (!slot) {
		// Flat array is full. With MaxFallbackSlots=8 this should never happen
		// in practice; log so we notice if it does.
		LOG("Fallback slot table full; ignoring fallback for new tracking system");
		return;
	}
	slot->tf.enabled = true;
	slot->tf.transform.translation = Eigen::Vector3d(
		newFallback.translation.v[0], newFallback.translation.v[1], newFallback.translation.v[2]);
	slot->tf.transform.rotation = Eigen::Quaterniond(
		newFallback.rotation.w, newFallback.rotation.x, newFallback.rotation.y, newFallback.rotation.z);
	slot->tf.scale = newFallback.scale;
	// predictionSmoothness from the fallback path is ignored from
	// Protocol v12 onward (2026-05-11). Per-device prediction is now
	// owned by the Smoothing overlay's SetDevicePrediction; the
	// FallbackTransform field stays in the struct for wire compat.
	slot->tf.recalibrateOnMovement = newFallback.recalibrateOnMovement;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose)
{
	// Apply debug pose before anything else
	if (openVRID > 0) {
		auto dbgPos = convert(pose.vecPosition) + debugTransform;
		auto dbgRot = convert(pose.qRotation) * debugRotation;
		pose.qRotation = convert(dbgRot);
		pose.vecPosition[0] = dbgPos(0);
		pose.vecPosition[1] = dbgPos(1);
		pose.vecPosition[2] = dbgPos(2);
	}

	// Lock guards every read/write of transforms[], deviceSystem[], systemFallbacks
	// against the IPC server thread's SetDeviceTransform / SetTrackingSystemFallback
	// handlers. IPC writes are infrequent (once per ScanAndApplyProfile tick at
	// most ~1 Hz) and brief, so contention is negligible at the hook's hundreds-of-
	// pose-updates-per-second cadence. Held for the full hook body -- simpler than
	// copy-out-under-lock + math-without-lock + write-back. shmem.SetPose writes
	// to a different-process ring buffer (overlay reader) so the mutex doesn't
	// synchronize that path; the lock only matters for in-process state.
	//
	// `unique_lock` (instead of `lock_guard`) so the lookup-fallback path below
	// can briefly release the mutex around its `vr::VRProperties()` call. That
	// OpenVR API call lands inside SteamVR and can block for milliseconds on
	// the runtime's own state lock. Holding our mutex across it stalled every
	// other tracked device's pose-update path -- the documented cause of "70+
	// HMD stalls per session" the user reported. We re-acquire before touching
	// any in-process state again.
	std::unique_lock<std::mutex> lock(stateMutex);

	auto& tf = transforms[openVRID];
	const vr::DriverPose_t rawSmoothingInput = pose;

	// Native pose-prediction suppression. Scales the velocity / acceleration /
	// poseTimeOffset fields by (1 - smoothness/100), where smoothness comes from
	// either the per-ID slot or the matching tracking-system fallback (per-ID
	// wins when both are set). The user picks the value from a 0..100 slider in
	// the overlay's Prediction tab; 100 fully zeros the fields (matching the old
	// "freeze" behaviour) and effectively defeats SteamVR's pose extrapolation.
	// The HMD / calibration ref / calibration target are hard-blocked to 0
	// upstream by the overlay, so by the time we read smoothness here it's
	// already been vetted as safe to apply.
	uint8_t smoothness = tf.predictionSmoothness;
	if (smoothness == 0 && !tf.enabled && !deviceSystem[openVRID].empty()) {
		const auto& sys = deviceSystem[openVRID];
		const FallbackSlot* slot = FindFallbackSlot(sys.data(), sys.size());
		if (slot && slot->tf.enabled && slot->tf.predictionSmoothness > 0) {
			smoothness = slot->tf.predictionSmoothness;
		}
	}
	if (smoothness > 0) {
		// Clamp defensively -- a buggy overlay (or a stale-protocol mismatch)
		// shouldn't be able to push a value above 100 here.
		if (smoothness > 100) smoothness = 100;

		// Squared curve: factor = (1 - s/100)^2.  Widens the perceptual gap
		// between s=80 (factor=0.04) and s=100 (factor=0) compared to the old
		// linear map (was 0.20 vs 0.00).
		double factor = prediction::SmoothnessToFactor(smoothness);

		// Position EWM alpha: separate from velocity factor so the positional
		// filter can be softer at mid-settings without killing velocity too hard.
		double posAlpha = prediction::PositionEwmAlpha(smoothness);

		if (tf.smartEnabled) {
			// Treat the slider as the maximum strength to apply when the
			// device is stationary, and blend toward "no suppression"
			// (factor = 1.0 / posAlpha = 1.0) as motion rises. EMA the
			// motion ramp so the effective factor doesn't twitch on noisy
			// IMU velocity.
			const double motion   = ComputeSmartMotionRamp(pose);
			tf.smartMotionEma     = (1.0 - kSmartEmaAlpha) * tf.smartMotionEma
			                      + kSmartEmaAlpha * motion;
			factor   = factor   + (1.0 - factor)   * tf.smartMotionEma;
			posAlpha = posAlpha + (1.0 - posAlpha)  * tf.smartMotionEma;
		}

		// Velocity / acceleration / lookahead suppression.
		pose.vecVelocity[0] *= factor;
		pose.vecVelocity[1] *= factor;
		pose.vecVelocity[2] *= factor;
		pose.vecAcceleration[0] *= factor;
		pose.vecAcceleration[1] *= factor;
		pose.vecAcceleration[2] *= factor;
		pose.vecAngularVelocity[0] *= factor;
		pose.vecAngularVelocity[1] *= factor;
		pose.vecAngularVelocity[2] *= factor;
		pose.vecAngularAcceleration[0] *= factor;
		pose.vecAngularAcceleration[1] *= factor;
		pose.vecAngularAcceleration[2] *= factor;
		// poseTimeOffset is multiplicative on the predictor: zeroing it cancels
		// extrapolation; halving it halves the lookahead time. Same factor as
		// velocity for consistency.
		pose.poseTimeOffset *= factor;

		// Position-space low-pass filter.  Seeds from the first pose seen after
		// smoothness becomes non-zero; thereafter blends the running average
		// toward the raw position at rate posAlpha each frame.  This attenuates
		// physical jitter that velocity suppression cannot touch.
		//
		// Large-jump guard: if the raw position is more than 0.5 m from the
		// current EMA (tracking loss / teleport recovery), reseed immediately
		// so the filter doesn't lag behind an intentional large displacement.
		if (tf.posEmaSeeded) {
			const double dx = pose.vecPosition[0] - tf.posEma[0];
			const double dy = pose.vecPosition[1] - tf.posEma[1];
			const double dz = pose.vecPosition[2] - tf.posEma[2];
			if (dx*dx + dy*dy + dz*dz > 0.25) { // 0.5 m radius
				tf.posEmaSeeded = false;
			}
		}
		if (!tf.posEmaSeeded) {
			tf.posEma[0] = pose.vecPosition[0];
			tf.posEma[1] = pose.vecPosition[1];
			tf.posEma[2] = pose.vecPosition[2];
			tf.posEmaSeeded = true;
		} else {
			tf.posEma[0] += posAlpha * (pose.vecPosition[0] - tf.posEma[0]);
			tf.posEma[1] += posAlpha * (pose.vecPosition[1] - tf.posEma[1]);
			tf.posEma[2] += posAlpha * (pose.vecPosition[2] - tf.posEma[2]);
		}
		pose.vecPosition[0] = tf.posEma[0];
		pose.vecPosition[1] = tf.posEma[1];
		pose.vecPosition[2] = tf.posEma[2];
	} else {
		// Smoothness dropped to 0: reset EMA state so a future re-enable
		// seeds from the current raw pose rather than a stale snapshot.
		tf.posEmaSeeded = false;
	}

#if WKOPENVR_BUILD_IS_DEV
	UpdateSmartSmoothingShadow(openVRID, tf, rawSmoothingInput, pose);
#endif

	shmem.SetPose(openVRID, pose);

#if WKOPENVR_BUILD_IS_DEV
	// DriverSynth: dev-only experimental path that replaces the upstream Quest
	// HMD pose with one synthesized from the head-mounted lighthouse tracker.
	// The tracker snapshot is written later in this function, after the normal
	// calibration transform has been applied to a copy of the tracker pose. That
	// makes the HMD follow the calibrated lighthouse pose instead of raw tracker
	// driver coordinates, and still works when the visible tracker output is
	// quashed.
	bool driverSynthTrackerSlot = false;
	bool driverSynthHmdMode = false;
	bool driverSynthComposeOk = false;
	const char* driverSynthFallbackReason = "inactive";
	int64_t driverSynthTrackerAgeMs = -1;
	auto driverSynthNow = std::chrono::steady_clock::now();
	wkopenvr::headmount::DriverSynthTimingConfig driverSynthTiming{};
	vr::DriverPose_t driverSynthPose{};
	{
		std::lock_guard<std::mutex> hmLk(m_headMountStateMutex);
		driverSynthTrackerSlot =
			m_headMountState.mode == 3
			&& m_headMountState.deviceId >= 0
			&& static_cast<int32_t>(openVRID) == m_headMountState.deviceId;
	}
	auto snapshotDriverSynthTracker = [&](const vr::DriverPose_t& snapshotPose) {
		if (!driverSynthTrackerSlot) return;
		std::lock_guard<std::mutex> snapLk(m_trackerSnapMutex);
		m_trackerSnap.pose = snapshotPose;
		m_trackerSnap.capturedAt = std::chrono::steady_clock::now();
		m_trackerSnap.capturedForDeviceId = static_cast<int32_t>(openVRID);
		m_trackerSnap.valid = true;
	};

	if (openVRID == 0) {
		// Attempt HMD synthesis. Copy the cached state out under its mutex
		// (tiny), release, then work on the copy.
		HeadMountDriverState hmState{};
		{
			std::lock_guard<std::mutex> hmLk(m_headMountStateMutex);
			hmState = m_headMountState;
		}

		if (hmState.mode == 3 /*DriverSynth*/) {
			driverSynthHmdMode = true;
			// Copy the latest calibrated tracker snapshot and attempt composition.
			driver_synth::TrackerSnapshot trackerCopy;
			{
				std::lock_guard<std::mutex> snapLk(m_trackerSnapMutex);
				trackerCopy = m_trackerSnap;
			}

			driver_synth::SynthState synthState{};
			synthState.mode             = hmState.mode;
			synthState.deviceId         = hmState.deviceId;
			synthState.offsetCalibrated = hmState.offsetCalibrated;
			memcpy(synthState.headFromTrackerTrans, hmState.headFromTrackerTrans,
			       sizeof synthState.headFromTrackerTrans);
			memcpy(synthState.headFromTrackerRot, hmState.headFromTrackerRot,
			       sizeof synthState.headFromTrackerRot);
			driverSynthTiming = hmState.driverSynthTiming;

			driverSynthNow = std::chrono::steady_clock::now();
			driverSynthTrackerAgeMs =
				driver_synth::SnapshotAgeMs(trackerCopy, driverSynthNow);
			driverSynthComposeOk = driver_synth::Compose(
				synthState, trackerCopy, driverSynthNow, driverSynthPose,
				driverSynthTiming);

			// Synthesis failure no longer hard-switches the HMD pose source here.
			// The normal HMD path below first builds the Quest fallback pose, then
			// the source blender crossfades between fallback and tracker synth.
			if (!driverSynthComposeOk)
			{
				auto markReasonLogged = [](std::atomic<uint8_t>& flags, uint8_t bit) {
					uint8_t observed = flags.load(std::memory_order_relaxed);
					while ((observed & bit) == 0) {
						if (flags.compare_exchange_weak(
							observed, static_cast<uint8_t>(observed | bit),
							std::memory_order_relaxed, std::memory_order_relaxed)) {
							return true;
						}
					}
					return false;
				};
				static std::atomic<uint8_t> s_loggedReasons{0};
				enum { R_UNRESOLVED=1, R_OFFSET=2, R_INVALID=4, R_STALE=8, R_MISMATCH=16, R_UNKNOWN=32 };
				const char* reason = "unknown";
				uint8_t bit = 0;
				if (synthState.deviceId < 0) {
					reason = "tracker_unresolved"; bit = R_UNRESOLVED;
				} else if (!synthState.offsetCalibrated) {
					reason = "offset_not_calibrated"; bit = R_OFFSET;
				} else if (!trackerCopy.valid || !trackerCopy.pose.poseIsValid) {
					reason = "tracker_invalid"; bit = R_INVALID;
				} else if (!driver_synth::IsTrackerFresh(trackerCopy, driverSynthNow)) {
					reason = "tracker_stale"; bit = R_STALE;
				} else if (trackerCopy.capturedForDeviceId != synthState.deviceId) {
					reason = "snapshot_device_mismatch"; bit = R_MISMATCH;
				} else {
					reason = "unknown"; bit = R_UNKNOWN;
				}
				if (markReasonLogged(s_loggedReasons, bit)) {
					LOG("[driver-synth] fallback: reason='%s'", reason);
				}
				driverSynthFallbackReason = reason;
				shmem.IncrementTelemetry(protocol::DriverPoseShmem::TELEMETRY_DRIVER_SYNTH_FALLBACK);
			}
		}
	}
#endif // WKOPENVR_BUILD_IS_DEV

	const bool quashTransition = tf.quash && !tf.prevQuash;
	tf.prevQuash = tf.quash;

	if (tf.quash) {
#if WKOPENVR_BUILD_IS_DEV
		if (driverSynthTrackerSlot && tf.enabled) {
			vr::DriverPose_t synthTrackerPose = pose;
			synthTrackerPose.vecPosition[0] *= tf.scale;
			synthTrackerPose.vecPosition[1] *= tf.scale;
			synthTrackerPose.vecPosition[2] *= tf.scale;

			auto deviceWorldPose = toIsoPose(synthTrackerPose);
			tf.currentRate = GetTransformDeltaSize(
				tf.currentRate, deviceWorldPose, tf.transform, tf.targetTransform);
			BlendTransform(tf, deviceWorldPose);
			ApplyTransform(tf, synthTrackerPose);
			snapshotDriverSynthTracker(synthTrackerPose);
		} else if (driverSynthTrackerSlot) {
			static std::atomic<bool> s_loggedHiddenNoTransform{false};
			if (!s_loggedHiddenNoTransform.exchange(true, std::memory_order_relaxed)) {
				LOG("[driver-synth] tracker snapshot skipped: hidden selected tracker has no enabled transform");
			}
		}
#endif
		openvr_pair::common::quash::ApplyQuashToPose(pose);
		shmem.IncrementTelemetry(protocol::DriverPoseShmem::TELEMETRY_QUASH_APPLY);

		if (quashTransition) {
			lock.unlock();
			char serial[256] = {};
			if (auto* helpers = vr::VRProperties()) {
				auto handle = helpers->TrackedDeviceToPropertyContainer(openVRID);
				if (handle != vr::k_ulInvalidPropertyContainer) {
					vr::ETrackedPropertyError err = vr::TrackedProp_Success;
					std::string s = helpers->GetStringProperty(handle, vr::Prop_SerialNumber_String, &err);
					if (err == vr::TrackedProp_Success && !s.empty())
						snprintf(serial, sizeof serial, "%s", s.c_str());
				}
			}
			LOG("[calibration] hide-tracker active for %s; pose offset by (%.0f,%.0f,%.0f) m -- model lives outside play space",
				serial[0] ? serial : "(unknown)",
				openvr_pair::common::quash::kQuashOffsetX,
				openvr_pair::common::quash::kQuashOffsetY,
				openvr_pair::common::quash::kQuashOffsetZ);
			lock.lock();
		}
	} else if (tf.enabled)
	{
		// Scale is applied to driver-local position before the calibration
		// transform (rotation + translation) is composed in below. This
		// scales positions around the driver-local origin, which is correct
		// only when the calibration anchor coincides with that origin --
		// true for the common case of a 1.0 scale, increasingly wrong as
		// scale diverges from 1.0 and the user's playspace shifts away
		// from the driver origin. A proper fix needs an explicit anchor
		// field on DeviceTransform: subtract anchor, scale, add anchor
		// back. The upstream SC driver carries the same limitation
		// (compare git history at e0d9eaf); the right anchor semantics
		// (calibration reference position? per-device rest pose?) were
		// never specified, so leave the naive form in place until that
		// decision is made rather than guess and break tracking.
		pose.vecPosition[0] *= tf.scale;
		pose.vecPosition[1] *= tf.scale;
		pose.vecPosition[2] *= tf.scale;

		auto deviceWorldPose = toIsoPose(pose);
		tf.currentRate = GetTransformDeltaSize(tf.currentRate, deviceWorldPose, tf.transform, tf.targetTransform);
		double lerp = GetTransformRate(tf.currentRate);

		BlendTransform(tf, deviceWorldPose);
		ApplyTransform(tf, pose);
#if WKOPENVR_BUILD_IS_DEV
		snapshotDriverSynthTracker(pose);
#endif
		shmem.IncrementTelemetry(protocol::DriverPoseShmem::TELEMETRY_PER_ID_APPLY);
	}
	else
	{
		// Per-ID transform is disabled. Check for a per-tracking-system fallback --
		// this lets a tracker that connected after the last overlay scan inherit
		// the calibrated offset on its very first pose update.
		//
		// If the overlay hasn't told us this device's tracking system yet (no
		// SetDeviceTransform has arrived for this slot), query it directly via the
		// driver-side property API. Without throttling this fires for every
		// unoccupied slot up to k_unMaxTrackedDeviceCount on every pose update;
		// gate it on lookupState + a 1-second backoff for failures.
		if (deviceSystem[openVRID].empty() && lookupState[openVRID] != LookupState::Cached) {
			bool shouldTry = true;
			if (lookupState[openVRID] == LookupState::Failed) {
				LARGE_INTEGER now;
				QueryPerformanceCounter(&now);
				// Retry no more than once per second. qpcFreq.QuadPart is the
				// number of QPC ticks per second.
				if (qpcFreq.QuadPart > 0 &&
					(now.QuadPart - lastLookupAttempt[openVRID].QuadPart) < qpcFreq.QuadPart) {
					shouldTry = false;
				}
			}
			if (shouldTry) {
				// The `vr::VRProperties()` call below lands inside SteamVR and
				// can block for milliseconds. Drop our mutex for the duration
				// so other devices' pose-update paths aren't stalled behind us
				// (the cause of the user's reported 70+ HMD stalls/session).
				// We've already read everything we needed under the lock --
				// the in-process state we care about (deviceSystem[],
				// lookupState[], transforms[]) is untouched between unlock and
				// the re-lock below; reads/writes of `tf` earlier in the
				// function happened under the lock and reads/writes of
				// `tf.targetTransform` etc. below happen after we re-acquire.
				lock.unlock();

				std::string queriedSys;
				bool queryOk = false;
				if (auto* helpers = vr::VRProperties()) {
					auto handle = helpers->TrackedDeviceToPropertyContainer(openVRID);
					if (handle != vr::k_ulInvalidPropertyContainer) {
						vr::ETrackedPropertyError err = vr::TrackedProp_Success;
						std::string sys = helpers->GetStringProperty(handle, vr::Prop_TrackingSystemName_String, &err);
						if (err == vr::TrackedProp_Success && !sys.empty()) {
							queriedSys = std::move(sys);
							queryOk = true;
						}
					}
				}

				lock.lock();

				// Race check: while we were unlocked, the IPC server thread
				// could have populated this slot via SetDeviceTransform. If it
				// won the race, defer to its value rather than overwriting
				// with our (possibly stale) query result.
				if (lookupState[openVRID] != LookupState::Cached) {
					if (queryOk) {
						deviceSystem[openVRID] = std::move(queriedSys);
						lookupState[openVRID] = LookupState::Cached;
					} else {
						lookupState[openVRID] = LookupState::Failed;
						QueryPerformanceCounter(&lastLookupAttempt[openVRID]);
					}
				}
			}
		}

		if (deviceSystem[openVRID].empty()) {
#if WKOPENVR_BUILD_IS_DEV
			if (driverSynthTrackerSlot) {
				static std::atomic<bool> s_loggedUnknownSystem{false};
				if (!s_loggedUnknownSystem.exchange(true, std::memory_order_relaxed)) {
					LOG("[driver-synth] tracker snapshot skipped: tracking system unknown for selected tracker");
				}
			}
#endif
			return true; // No system known yet; nothing to fall back to.
		}

		const auto& sys = deviceSystem[openVRID];
		FallbackSlot* slot = FindFallbackSlot(sys.data(), sys.size());
		if (slot && slot->tf.enabled) {
			const auto& fb = slot->tf;

			// Update the slot's blend target from the (possibly newly updated) fallback.
			tf.targetTransform = fb.transform;
			tf.scale = fb.scale;
			// Propagate motion-gate setting so the fallback path also gets the
			// movement-masked blend if the user has it enabled. Reset the captured
			// previous-frame pose if the flag is transitioning off.
			if (tf.recalibrateOnMovement && !fb.recalibrateOnMovement) {
				tf.blendMotionInitialized = false;
			}
			tf.recalibrateOnMovement = fb.recalibrateOnMovement;

			// First activation: snap so the device doesn't ramp in from an identity
			// or otherwise stale `transform` value.
			if (!tf.fallbackActive) {
				tf.transform = fb.transform;
				tf.fallbackActive = true;
				tf.currentRate = DeltaSize::TINY;
				QueryPerformanceCounter(&tf.lastPoll);
			}

			pose.vecPosition[0] *= tf.scale;
			pose.vecPosition[1] *= tf.scale;
			pose.vecPosition[2] *= tf.scale;

			auto deviceWorldPose = toIsoPose(pose);
			tf.currentRate = GetTransformDeltaSize(tf.currentRate, deviceWorldPose, tf.transform, tf.targetTransform);

			BlendTransform(tf, deviceWorldPose);
			ApplyTransform(tf, pose);
#if WKOPENVR_BUILD_IS_DEV
			snapshotDriverSynthTracker(pose);
#endif
			shmem.IncrementTelemetry(protocol::DriverPoseShmem::TELEMETRY_FALLBACK_APPLY);
		}
		else if (tf.fallbackActive) {
			// Fallback was removed/disabled while we were following it. Clear our
			// blend state so a future re-enable starts clean.
			tf.fallbackActive = false;
			tf.targetTransform = tf.transform;
			tf.currentRate = DeltaSize::TINY;
			QueryPerformanceCounter(&tf.lastPoll);
		}
	}

#if WKOPENVR_BUILD_IS_DEV
	if (openVRID == 0 && driverSynthHmdMode) {
		if (m_driverSynthBlendReset.exchange(false, std::memory_order_relaxed)) {
			m_driverSynthBlendState = driver_synth::SourceBlendState{};
		}
		vr::DriverPose_t blendedPose{};
		const auto blendResult = driver_synth::StepSourceBlend(
			m_driverSynthBlendState,
			pose,
			driverSynthComposeOk ? &driverSynthPose : nullptr,
			driverSynthComposeOk,
			driverSynthNow,
			blendedPose,
			driverSynthTiming);
		pose = blendedPose;
		if (blendResult.phaseChanged) {
			LOG("[driver-synth] source_blend phase=%s prev=%s reason='%s' alpha=%.3f tracker_age_ms=%lld",
			    driver_synth::PhaseName(blendResult.phase),
			    driver_synth::PhaseName(blendResult.previousPhase),
			    driverSynthComposeOk ? "tracker_ready" : driverSynthFallbackReason,
			    blendResult.alpha,
			    (long long)driverSynthTrackerAgeMs);
		}
	}
#endif

#if OPENVR_PAIR_HAS_PHANTOM_DRIVER
	// Phantom-tracker pipeline. OnRealPoseObserved records the pose AFTER
	// existing transforms (calibration / smoothing) have run so the
	// dropout-bridging stream stays visually consistent with what the user
	// normally sees. MaybeOverridePose may replace `pose` with a
	// dead-reckoned / IK / ML synthesis, or return false to suppress the
	// downstream pose update entirely (LOST state -> SteamVR treats the
	// device as disconnected after its own timeout).
	//
	// Skip the phantom pipeline entirely for quashed (hidden) devices. The
	// hide path has already translated the pose by ~14 km; recording that
	// offset position in the dropout history would corrupt the IK fallback
	// the moment the user toggles hide off. Phantom's job is masking real
	// dropouts on visible trackers, not babysitting an intentionally-hidden
	// one.
	if ((featureFlags & pairdriver::kFeaturePhantom) && !tf.quash) {
		LARGE_INTEGER qpcNow{};
		QueryPerformanceCounter(&qpcNow);
		phantom::OnRealPoseObserved(openVRID, qpcNow.QuadPart, pose);
		if (!phantom::MaybeOverridePose(openVRID, qpcNow.QuadPart, qpcFreq.QuadPart, pose)) {
			return false;
		}
	}
#endif

	return true;
}

void ServerTrackedDeviceProvider::HandleApplyRandomOffset() {
	std::random_device gen;
	std::uniform_real_distribution<double> d(-1, 1);
	auto init = Eigen::Vector3d(d(gen), d(gen), d(gen));
	auto posOffset = init * 0.25f;

	debugTransform = posOffset;
	debugRotation = Eigen::Quaterniond::Identity();

	std::ostringstream oss;
	oss << "Applied random offset: " << posOffset << " from init " << init << std::endl;
	LOG("%s", oss.str().c_str());
}

void ServerTrackedDeviceProvider::SetHeadMountConfig(const protocol::SetHeadMountConfig &cfg)
{
#if WKOPENVR_BUILD_IS_DEV
	// Copy the wire payload into the cached state under its own mutex. The
	// pose hook copies this state out (under lock, instantly) and then works
	// on the copy without holding the lock, so the critical section here is
	// just a struct write.
	HeadMountDriverState next{};
	next.mode             = static_cast<int>(cfg.mode);
	next.deviceId         = cfg.deviceId;
	next.hideTracker      = cfg.hideTracker;
	next.offsetCalibrated = cfg.offsetCalibrated;
	next.driverSynthTiming = wkopenvr::headmount::ClampDriverSynthTimingConfig({
		(int)cfg.driverSynthStaleLimitMs,
		(int)cfg.driverSynthGraceHoldMs,
		(int)cfg.driverSynthBlendToFallbackMs,
		(int)cfg.driverSynthStableBeforeSynthMs,
		(int)cfg.driverSynthBlendToSynthMs,
	});

	// NUL-safe copy of the fixed-size name buffers.
	{
		constexpr size_t kNameLen = protocol::MaxTrackingSystemNameLen;
		const size_t serialLen = strnlen(cfg.trackerSerial, kNameLen);
		memcpy(next.trackerSerial, cfg.trackerSerial, serialLen);
		const size_t sysLen = strnlen(cfg.trackerTrackingSystem, kNameLen);
		memcpy(next.trackerTrackingSystem, cfg.trackerTrackingSystem, sysLen);
	}

	memcpy(next.headFromTrackerTrans, cfg.headFromTrackerTrans,
	       sizeof cfg.headFromTrackerTrans);
	memcpy(next.headFromTrackerRot, cfg.headFromTrackerRot,
	       sizeof cfg.headFromTrackerRot);

	bool stateChanged = false;
	bool sourceChanged = false;
	{
		HeadMountDriverState prev{};
		std::lock_guard<std::mutex> lk(m_headMountStateMutex);
		prev = m_headMountState;
		m_headMountState = next;
		sourceChanged =
			next.mode != prev.mode
			|| next.deviceId != prev.deviceId
			|| next.hideTracker != prev.hideTracker
			|| next.offsetCalibrated != prev.offsetCalibrated
			|| memcmp(next.trackerSerial, prev.trackerSerial, sizeof next.trackerSerial) != 0
			|| memcmp(next.trackerTrackingSystem, prev.trackerTrackingSystem,
			          sizeof next.trackerTrackingSystem) != 0
			|| memcmp(next.headFromTrackerTrans, prev.headFromTrackerTrans,
			          sizeof next.headFromTrackerTrans) != 0
			|| memcmp(next.headFromTrackerRot, prev.headFromTrackerRot,
			          sizeof next.headFromTrackerRot) != 0;
		stateChanged = sourceChanged
			|| next.driverSynthTiming.staleLimitMs != prev.driverSynthTiming.staleLimitMs
			|| next.driverSynthTiming.graceHoldMs != prev.driverSynthTiming.graceHoldMs
			|| next.driverSynthTiming.blendToFallbackMs != prev.driverSynthTiming.blendToFallbackMs
			|| next.driverSynthTiming.stableBeforeSynthMs != prev.driverSynthTiming.stableBeforeSynthMs
			|| next.driverSynthTiming.blendToSynthMs != prev.driverSynthTiming.blendToSynthMs;
		// Log on change only to avoid flooding when the overlay re-sends
		// the same config on every AssignTargets scan.
		if (stateChanged) {
			LOG("[driver-head-mount] config: mode=%d deviceID=%d offsetCalibrated=%d"
				" synth_stale_ms=%d grace_ms=%d blend_fallback_ms=%d"
				" stable_synth_ms=%d blend_synth_ms=%d",
			    next.mode, next.deviceId, (int)next.offsetCalibrated,
			    next.driverSynthTiming.staleLimitMs,
			    next.driverSynthTiming.graceHoldMs,
			    next.driverSynthTiming.blendToFallbackMs,
			    next.driverSynthTiming.stableBeforeSynthMs,
			    next.driverSynthTiming.blendToSynthMs);
		}
	}
	if (sourceChanged) {
		std::lock_guard<std::mutex> snapLk(m_trackerSnapMutex);
		m_trackerSnap = driver_synth::TrackerSnapshot{};
		m_driverSynthBlendReset.store(true, std::memory_order_relaxed);
	}
#else
	(void)cfg;
#endif // WKOPENVR_BUILD_IS_DEV
}

void ServerTrackedDeviceProvider::SetFingerSmoothingConfig(const protocol::FingerSmoothingConfig &cfg)
{
	const uint64_t newHeader = pairdriver::PackFingerHeader(cfg);
	const uint64_t newLow    = pairdriver::PackFingerLow(cfg);

	// Two exchanges: header carries the master/smoothness/mask plus fingers 8&9;
	// low carries fingers 0..7. The detour reads both atomically with acquire;
	// a partial update during the brief gap is harmless (one frame, one finger).
	const uint64_t oldHeader = fingerCfgPacked.exchange(newHeader, std::memory_order_acq_rel);
	const uint64_t oldLow    = perFingerSmoothness0to7Packed.exchange(newLow, std::memory_order_acq_rel);

	// Log only on real changes so a slider drag (60 Hz no-op tick) doesn't
	// flood the log file.
	if (oldHeader != newHeader || oldLow != newLow) {
		const protocol::FingerSmoothingConfig prev =
			pairdriver::UnpackFingerSmoothing(oldHeader, oldLow);

		// Compute which fingers transitioned from "not smoothed" to
		// "smoothed" so the detour can reseed its per-finger state.previous
		// from live input on the next frame. Without this, the first
		// smoothed frame after a toggle slerps from whatever the cached
		// previous-frame value happens to be -- visible as a snap or
		// transient spasm.
		//
		// A finger is "not smoothed" if any of: master disabled, mask bit
		// clear, or effective smoothness (per-finger > 0 wins, else
		// global) is zero. The IPC reconnect path arrives with
		// prev.master_enabled == 0 (atomics start at zero), so the very
		// first SetFingerSmoothingConfig after driver init reseeds every
		// currently-enabled finger automatically.
		const uint16_t reseedBits =
			pairdriver::ComputeFingerSmoothingReseedBits(prev, cfg);
		if (reseedBits) skeletal::MarkFingersNeedReseed(reseedBits);

		LOG("[skeletal] SetFingerSmoothingConfig via IPC: enabled=%d global=%u mask=0x%04x per_finger=[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u] (was: enabled=%d global=%u mask=0x%04x)",
			(int)cfg.master_enabled, (unsigned)cfg.smoothness, (unsigned)cfg.finger_mask,
			(unsigned)cfg.per_finger_smoothness[0], (unsigned)cfg.per_finger_smoothness[1],
			(unsigned)cfg.per_finger_smoothness[2], (unsigned)cfg.per_finger_smoothness[3],
			(unsigned)cfg.per_finger_smoothness[4], (unsigned)cfg.per_finger_smoothness[5],
			(unsigned)cfg.per_finger_smoothness[6], (unsigned)cfg.per_finger_smoothness[7],
			(unsigned)cfg.per_finger_smoothness[8], (unsigned)cfg.per_finger_smoothness[9],
			(int)prev.master_enabled, (unsigned)prev.smoothness, (unsigned)prev.finger_mask);
	}
}

protocol::FingerSmoothingConfig ServerTrackedDeviceProvider::GetFingerSmoothingConfig() const
{
	// Hot path: ~680 Hz (340 Hz x 2 hands) skeletal detour reads. Two atomic
	// loads + acquire fences on x64 are both single movs each; the cost is
	// one extra cache-line touch per call vs the old single-atomic version.
	const uint64_t header = fingerCfgPacked.load(std::memory_order_acquire);
	const uint64_t low    = perFingerSmoothness0to7Packed.load(std::memory_order_acquire);
	return pairdriver::UnpackFingerSmoothing(header, low);
}

void ServerTrackedDeviceProvider::SetInputHealthConfig(const protocol::InputHealthConfig &cfg)
{
	const uint64_t newPacked = pairdriver::PackInputHealthConfig(cfg);

	const uint64_t oldPacked = inputHealthCfgPacked.exchange(newPacked, std::memory_order_acq_rel);

	if (oldPacked != newPacked) {
		const protocol::InputHealthConfig prev =
			pairdriver::UnpackInputHealthConfig(oldPacked);
		LOG("[inputhealth] SetInputHealthConfig via IPC: master=%d diag_only=%d rest=%d trig=%d (was: master=%d diag_only=%d rest=%d trig=%d)",
			(int)cfg.master_enabled, (int)cfg.diagnostics_only,
			(int)cfg.enable_rest_recenter, (int)cfg.enable_trigger_remap,
			(int)prev.master_enabled, (int)prev.diagnostics_only,
			(int)prev.enable_rest_recenter, (int)prev.enable_trigger_remap);
	}
}

protocol::InputHealthConfig ServerTrackedDeviceProvider::GetInputHealthConfig() const
{
	// Hot path: every UpdateBooleanComponent / UpdateScalarComponent detour
	// once Stage 1B lands. Single atomic load + acquire fence + memcpy.
	const uint64_t packed = inputHealthCfgPacked.load(std::memory_order_acquire);
	return pairdriver::UnpackInputHealthConfig(packed);
}

void ServerTrackedDeviceProvider::SetInputHealthCompensation(const protocol::InputHealthCompensationEntry &entry)
{
	const std::string path = InputHealthPathString(entry.path);
	if (entry.device_serial_hash == 0 || path.empty()) return;

	// Reject paths that should never carry compensation. The overlay's learning
	// engine applies the same classifier so this is a belt-and-suspenders guard
	// against stale entries arriving from an older overlay build.
	if (!inputhealth::IsCompensationPath(inputhealth::ClassifyInputPath(path))) {
		LOG("[inputhealth] SetInputHealthCompensation: rejected unsupported path serial_hash=0x%016llx path='%s'",
			(unsigned long long)entry.device_serial_hash, path.c_str());
		return;
	}

	std::unique_lock<std::shared_mutex> lk(inputHealthCompMutex);
	auto serialIt = inputHealthComp.find(entry.device_serial_hash);
	if (!entry.enabled) {
		if (serialIt != inputHealthComp.end()) {
			serialIt->second.erase(path);
			if (serialIt->second.empty()) inputHealthComp.erase(serialIt);
		}
		LOG("[inputhealth] SetInputHealthCompensation: serial_hash=0x%016llx path='%s' enabled=0",
			(unsigned long long)entry.device_serial_hash, path.c_str());
		return;
	}

	inputHealthComp[entry.device_serial_hash][path] = entry;
	LOG("[inputhealth] SetInputHealthCompensation: serial_hash=0x%016llx path='%s' enabled=1 kind=%u"
		" offset=%.5f trig_min=%.5f trig_max=%.5f dead=%.5f debounce_us=%u",
		(unsigned long long)entry.device_serial_hash,
		path.c_str(),
		(unsigned)entry.kind,
		entry.learned_rest_offset,
		entry.learned_trigger_min,
		entry.learned_trigger_max,
		entry.learned_deadzone_radius,
		(unsigned)entry.learned_debounce_us);
}

bool ServerTrackedDeviceProvider::LookupInputHealthCompensation(
	uint64_t serial_hash,
	const std::string &path,
	protocol::InputHealthCompensationEntry &out) const
{
	if (serial_hash == 0 || path.empty()) return false;
	// Block until the shared lock is acquired. Writes (SetInputHealthCompensation /
	// ClearInputHealthCompensation) are IPC-driven and rare, so contention is
	// negligible. try_to_lock silently dropped entries on any write-side contention,
	// which caused compensation to be skipped for that pose frame.
	// inputHealthCompContentionCount is incremented if another thread held the lock
	// so post-hoc contention analysis is possible under a debugger.
	std::shared_lock<std::shared_mutex> lk(inputHealthCompMutex, std::defer_lock);
	if (!lk.try_lock()) {
		inputHealthCompContentionCount.fetch_add(1, std::memory_order_relaxed);
		lk.lock();
	}

	auto serialIt = inputHealthComp.find(serial_hash);
	if (serialIt == inputHealthComp.end()) return false;
	auto pathIt = serialIt->second.find(path);
	if (pathIt == serialIt->second.end()) return false;
	out = pathIt->second;
	return out.enabled != 0;
}

void ServerTrackedDeviceProvider::ClearInputHealthCompensation(uint64_t serial_hash)
{
	std::unique_lock<std::shared_mutex> lk(inputHealthCompMutex);
	if (serial_hash == inputhealth::kSerialHashAllDevices) {
		const size_t count = inputHealthComp.size();
		inputHealthComp.clear();
		LOG("[inputhealth] ClearInputHealthCompensation: all serials cleared=%zu", count);
		return;
	}
	const size_t erased = inputHealthComp.erase(serial_hash);
	LOG("[inputhealth] ClearInputHealthCompensation: serial_hash=0x%016llx erased=%zu",
		(unsigned long long)serial_hash, erased);
}
