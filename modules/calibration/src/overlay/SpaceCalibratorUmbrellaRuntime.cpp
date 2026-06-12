#include "SpaceCalibratorUmbrellaRuntime.h"

#include "BuildChannel.h"
#include "Calibration.h"
#include "CalibrationAnchor.h"
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "TrackingStyle.h"
#include "UserInterface.h"

#include <openvr.h>

#include <chrono>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

namespace {

bool g_profileLoaded = false;
bool g_vrReady = false;
std::string g_lastVRError;
std::chrono::steady_clock::time_point g_lastRetry{};
const auto g_retryPeriod = std::chrono::seconds(1);

double SecondsSinceStart()
{
	static const auto start = std::chrono::steady_clock::now();
	const auto now = std::chrono::steady_clock::now();
	return std::chrono::duration<double>(now - start).count();
}

bool TryConnect()
{
	if (g_vrReady) return true;

	if (!vr::VRSystem()) {
		g_lastVRError = "Waiting for SteamVR";
		return false;
	}

	if (!vr::VR_IsInterfaceVersionValid(vr::IVRSystem_Version) ||
	    !vr::VR_IsInterfaceVersionValid(vr::IVRSettings_Version)) {
		g_lastVRError = "OpenVR interface version mismatch";
		return false;
	}

	try {
		InitCalibrator();
	}
	catch (const std::exception& e) {
		g_lastVRError = e.what();
		return false;
	}

	g_lastVRError.clear();
	g_vrReady = true;
	Metrics::WriteLogAnnotation("[umbrella] vr_ready");
	return true;
}

std::string ReadDeviceSerial(int32_t id)
{
	if (id < 0 || id >= (int32_t)vr::k_unMaxTrackedDeviceCount) return {};
	auto* vrSystem = vr::VRSystem();
	if (!vrSystem) return {};

	char buf[vr::k_unMaxPropertyStringSize] = {};
	vr::ETrackedPropertyError err = vr::TrackedProp_Success;
	vrSystem->GetStringTrackedDeviceProperty((uint32_t)id, vr::Prop_SerialNumber_String, buf, sizeof buf, &err);
	if (err != vr::TrackedProp_Success || buf[0] == '\0') return {};
	return buf;
}

void AddCalibrationLock(std::vector<openvr_pair::overlay::CalibrationDeviceLock>& locks,
                        openvr_pair::overlay::CalibrationDeviceLockKind kind, int32_t liveId,
                        const std::string& fallbackSerial)
{
	std::string serial = ReadDeviceSerial(liveId);
	if (serial.empty()) serial = fallbackSerial;
	if (serial.empty()) return;

	for (const auto& existing : locks) {
		if (existing.serial == serial) return;
	}
	locks.push_back({serial, kind});
}

} // namespace

void CCal_UmbrellaStart()
{
	if (!g_profileLoaded) {
		LoadProfile(CalCtx);
		g_profileLoaded = true;
	}
	g_lastRetry = std::chrono::steady_clock::now() - g_retryPeriod;
}

void CCal_UmbrellaTick()
{
	const auto now = std::chrono::steady_clock::now();
	if (!g_vrReady && now - g_lastRetry >= g_retryPeriod) {
		g_lastRetry = now;
		TryConnect();
	}

	if (g_vrReady) {
		CalibrationTick(SecondsSinceStart());

		std::vector<openvr_pair::overlay::CalibrationDeviceLock> locks;
		const bool continuous =
		    CalCtx.state == CalibrationState::Continuous || CalCtx.state == CalibrationState::ContinuousStandby;
		const bool publishCalibrationLocks =
		    continuous && TrackingStylePublishesCalibrationDeviceLocks(CalCtx.trackingStyle);
		if (publishCalibrationLocks) {
			AddCalibrationLock(locks, openvr_pair::overlay::CalibrationDeviceLockKind::Reference, CalCtx.referenceID,
			                   CalCtx.referenceStandby.serial);
			AddCalibrationLock(locks, openvr_pair::overlay::CalibrationDeviceLockKind::Target, CalCtx.targetID,
			                   CalCtx.targetStandby.serial);
			for (const auto& extra : CalCtx.additionalCalibrations) {
				if (!extra.enabled) continue;
				AddCalibrationLock(locks, openvr_pair::overlay::CalibrationDeviceLockKind::Target, extra.targetID,
				                   extra.targetStandby.serial);
			}
		}
		openvr_pair::overlay::SetCalibrationDeviceLocks(locks);

		std::string headsetSynthesisTrackerSerial;
		if (TrackingStyleUsesHeadsetSynthesis(CalCtx.trackingStyle) &&
		    CalCtx.headMount.mode == HeadMountMode::DriverSynth) {
			headsetSynthesisTrackerSerial = ReadDeviceSerial(CalCtx.headMount.deviceID);
			if (headsetSynthesisTrackerSerial.empty()) headsetSynthesisTrackerSerial = CalCtx.headMount.trackerSerial;
		}
		openvr_pair::overlay::SetHeadsetSynthesisTrackerSerial(headsetSynthesisTrackerSerial);
	}
	else {
		static auto s_lastWaitingLog = std::chrono::steady_clock::time_point{};
		if (s_lastWaitingLog.time_since_epoch().count() == 0 || now - s_lastWaitingLog >= std::chrono::seconds(5)) {
			s_lastWaitingLog = now;
			char buf[192];
			snprintf(buf, sizeof buf, "[umbrella] calibration_tick_skipped reason=vr_not_ready detail='%s'",
			         g_lastVRError.empty() ? "unknown" : g_lastVRError.c_str());
			Metrics::WriteLogAnnotation(buf);
		}
		openvr_pair::overlay::SetCalibrationDeviceLocks({});
		openvr_pair::overlay::SetHeadsetSynthesisTrackerSerial({});
	}
}

void CCal_UmbrellaShutdown()
{
	g_vrReady = false;
	// Persist any continuous-mode offset the per-tick throttle left pending, so a
	// session that quits mid-continuous-calibration still writes its final value.
	FlushPendingContinuousSave();
	openvr_pair::overlay::SetCalibrationDeviceLocks({});
	openvr_pair::overlay::SetHeadsetSynthesisTrackerSerial({});
}

void RequestImmediateRedraw() {}

void RequestExit() {}

bool IsVRReady()
{
	return g_vrReady;
}

const std::string& LastVRConnectError()
{
	return g_lastVRError;
}
