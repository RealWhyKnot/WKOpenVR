#pragma once

#include "Logging.h"

#include <openvr_driver.h>

class VRWatchdogProvider : public vr::IVRWatchdogProvider
{
	/** initializes the driver in watchdog mode. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext)
	{
		LOG("VRWatchdogProvider::Init()");
		VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
		return vr::VRInitError_None;
	}

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup()
	{
		LOG("VRWatchdogProvider::Cleanup()");
		VR_CLEANUP_WATCHDOG_DRIVER_CONTEXT()
	}
};
