#include "DriverModule.h"
#include "FeatureFlags.h"
#include "Logging.h"
#include "ModuleRegistry.h"
#include "ServerTrackedDeviceProvider.h"
#include "SkeletalHookInjector.h"

#include <cstring>
#include <string>

namespace smoothing {
namespace {

class SmoothingDriverModule final : public DriverModule
{
public:
	const char *Name() const override { return "Smoothing"; }
	uint32_t FeatureMask() const override { return pairdriver::kFeatureSmoothing; }
	const char *PipeName() const override { return openvr_pair::common::modules::PipeName(openvr_pair::common::modules::ModuleId::Smoothing); }

	bool Init(DriverModuleContext &context) override
	{
		provider_ = context.provider;
		skeletal::Init(provider_);
		return true;
	}

	void Shutdown() override
	{
		skeletal::Shutdown();
		provider_ = nullptr;
	}

	void OnGetGenericInterface(const char *pchInterface, void *iface) override
	{
		if (!pchInterface || !iface) return;
		// strstr on the raw C string avoids a std::string allocation per
		// interface query (this fires for every interface SteamVR asks for
		// during driver init, a few dozen times per boot).
		if (std::strstr(pchInterface, "IVRDriverInput_") != nullptr
			&& std::strstr(pchInterface, "Internal") == nullptr) {
			LOG("[skeletal] %s queried via context: iface=%p", pchInterface, iface);
			skeletal::TryInstallPublicHooks(iface);
		}
	}

	bool HandleRequest(const protocol::Request &request, protocol::Response &response) override
	{
		if (!provider_) return false;
		switch (request.type) {
		case protocol::RequestSetFingerSmoothing:
			provider_->SetFingerSmoothingConfig(request.setFingerSmoothing);
			response.type = protocol::ResponseSuccess;
			return true;
		case protocol::RequestSetDevicePrediction:
			provider_->SetDevicePrediction(request.setDevicePrediction);
			response.type = protocol::ResponseSuccess;
			return true;
		default:
			return false;
		}
	}

private:
	ServerTrackedDeviceProvider *provider_ = nullptr;
};

} // namespace

std::unique_ptr<DriverModule> CreateDriverModule()
{
	return std::make_unique<SmoothingDriverModule>();
}

} // namespace smoothing
