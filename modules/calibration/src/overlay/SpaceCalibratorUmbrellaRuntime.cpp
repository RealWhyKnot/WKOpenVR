#include "SpaceCalibratorUmbrellaRuntime.h"

#include "Calibration.h"
#include "CalibrationAnchor.h"
#include "Configuration.h"
#include "UserInterface.h"

#include <openvr.h>

#include <chrono>
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

	auto initError = vr::VRInitError_None;
	vr::VR_Init(&initError, vr::VRApplication_Background);
	if (initError != vr::VRInitError_None) {
		g_lastVRError = std::string("Waiting for SteamVR: ") +
			vr::VR_GetVRInitErrorAsEnglishDescription(initError);
		return false;
	}

	if (!vr::VR_IsInterfaceVersionValid(vr::IVRSystem_Version) ||
		!vr::VR_IsInterfaceVersionValid(vr::IVRSettings_Version)) {
		g_lastVRError = "OpenVR interface version mismatch";
		vr::VR_Shutdown();
		return false;
	}

	try {
		InitCalibrator();
	} catch (const std::exception &e) {
		g_lastVRError = e.what();
		vr::VR_Shutdown();
		return false;
	}

	g_lastVRError.clear();
	g_vrReady = true;
	return true;
}

std::string ReadDeviceSerial(int32_t id)
{
	if (id < 0 || id >= (int32_t)vr::k_unMaxTrackedDeviceCount) return {};
	auto *vrSystem = vr::VRSystem();
	if (!vrSystem) return {};

	char buf[vr::k_unMaxPropertyStringSize] = {};
	vr::ETrackedPropertyError err = vr::TrackedProp_Success;
	vrSystem->GetStringTrackedDeviceProperty(
		(uint32_t)id,
		vr::Prop_SerialNumber_String,
		buf, sizeof buf, &err);
	if (err != vr::TrackedProp_Success || buf[0] == '\0') return {};
	return buf;
}

void AddCalibrationLock(std::vector<openvr_pair::overlay::CalibrationDeviceLock> &locks,
	openvr_pair::overlay::CalibrationDeviceLockKind kind,
	int32_t liveId,
	const std::string &fallbackSerial)
{
	std::string serial = ReadDeviceSerial(liveId);
	if (serial.empty()) serial = fallbackSerial;
	if (serial.empty()) return;

	for (const auto &existing : locks) {
		if (existing.serial == serial) return;
	}
	locks.push_back({ serial, kind });
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
			CalCtx.state == CalibrationState::Continuous ||
			CalCtx.state == CalibrationState::ContinuousStandby;
		if (continuous) {
			AddCalibrationLock(locks,
				openvr_pair::overlay::CalibrationDeviceLockKind::Reference,
				CalCtx.referenceID, CalCtx.referenceStandby.serial);
			AddCalibrationLock(locks,
				openvr_pair::overlay::CalibrationDeviceLockKind::Target,
				CalCtx.targetID, CalCtx.targetStandby.serial);
			for (const auto &extra : CalCtx.additionalCalibrations) {
				if (!extra.enabled) continue;
				AddCalibrationLock(locks,
					openvr_pair::overlay::CalibrationDeviceLockKind::Target,
					extra.targetID, extra.targetStandby.serial);
			}
		}
		openvr_pair::overlay::SetCalibrationDeviceLocks(locks);
	} else {
		openvr_pair::overlay::SetCalibrationDeviceLocks({});
	}
}

void CCal_UmbrellaShutdown()
{
	if (g_vrReady) {
		vr::VR_Shutdown();
	}
	g_vrReady = false;
}

void RequestImmediateRedraw()
{
}

void RequestExit()
{
}

bool IsVRReady()
{
	return g_vrReady;
}

const std::string &LastVRConnectError()
{
	return g_lastVRError;
}
