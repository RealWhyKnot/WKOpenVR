#include "DriverModule.h"
#include "FeatureFlags.h"
#include "Logging.h"
#include "ModuleRegistry.h"
#include "ServerTrackedDeviceProvider.h"

namespace calibration {
namespace {

class CalibrationDriverModule final : public DriverModule
{
public:
	const char* Name() const override { return "Space Calibrator"; }
	uint32_t FeatureMask() const override { return pairdriver::kFeatureCalibration; }
	const char* PipeName() const override
	{
		return openvr_pair::common::modules::PipeName(openvr_pair::common::modules::ModuleId::Calibration);
	}

	bool Init(DriverModuleContext& context) override
	{
		provider_ = context.provider;
		return provider_ != nullptr;
	}

	void Shutdown() override { provider_ = nullptr; }

	bool HandleRequest(const protocol::Request& request, protocol::Response& response) override
	{
		if (!provider_) return false;
		switch (request.type) {
			case protocol::RequestSetDeviceTransform:
				provider_->SetDeviceTransform(request.setDeviceTransform);
				response.type = protocol::ResponseSuccess;
				return true;
			case protocol::RequestDebugOffset:
				provider_->HandleApplyRandomOffset();
				response.type = protocol::ResponseSuccess;
				return true;
			case protocol::RequestSetAlignmentSpeedParams:
				provider_->HandleSetAlignmentSpeedParams(request.setAlignmentSpeedParams);
				response.type = protocol::ResponseSuccess;
				return true;
			case protocol::RequestSetTrackingSystemFallback:
				provider_->SetTrackingSystemFallback(request.setTrackingSystemFallback);
				response.type = protocol::ResponseSuccess;
				return true;
			// v25: head-mount tracker config. Caches the payload in the driver
			// for the DriverSynth pose-synthesis path. The overlay sends this
			// whenever the user changes head-mount mode or the resolved tracker
			// deviceId changes.
			case protocol::RequestSetHeadMountConfig:
				LOG("[driver-ipc] head-mount config received: mode=%d deviceId=%d",
				    (int)request.setHeadMountConfig.mode, (int)request.setHeadMountConfig.deviceId);
				provider_->SetHeadMountConfig(request.setHeadMountConfig);
				response.type = protocol::ResponseSuccess;
				return true;
			default:
				return false;
		}
	}

private:
	ServerTrackedDeviceProvider* provider_ = nullptr;
};

} // namespace

std::unique_ptr<DriverModule> CreateDriverModule()
{
	return std::make_unique<CalibrationDriverModule>();
}

} // namespace calibration
