#include "DriverModule.h"
#include "FeatureFlags.h"
#include "Logging.h"
#include "ModuleRegistry.h"
#include "ServerTrackedDeviceProvider.h"
#include "SkeletalHookInjector.h"

#include <cstring>
#include <memory>

namespace dashboardinput {
namespace {

class DashboardInputDriverModule final : public DriverModule
{
public:
	const char* Name() const override { return "Dashboard Input"; }
	uint32_t FeatureMask() const override { return pairdriver::kFeatureDashboardInput; }
	const char* PipeName() const override
	{
		return openvr_pair::common::modules::PipeName(openvr_pair::common::modules::ModuleId::DashboardInput);
	}

	bool Init(DriverModuleContext& context) override
	{
		provider_ = context.provider;
		skeletal::Init(provider_, pairdriver::kFeatureDashboardInput);
		return true;
	}

	void Shutdown() override
	{
		skeletal::Shutdown(pairdriver::kFeatureDashboardInput);
		provider_ = nullptr;
	}

	void OnGetGenericInterface(const char* pchInterface, void* iface) override
	{
		if (!pchInterface || !iface) return;
		if (std::strstr(pchInterface, "IVRDriverInput_") != nullptr &&
		    std::strstr(pchInterface, "Internal") == nullptr) {
			LOG("[dashboardinput] %s queried via context: iface=%p", pchInterface, iface);
			skeletal::TryInstallPublicHooks(iface);
		}
	}

	bool HandleRequest(const protocol::Request& request, protocol::Response& response) override
	{
		if (!provider_) return false;
		switch (request.type) {
			case protocol::RequestSetDashboardHandTrackingState:
				provider_->SetDashboardHandTrackingState(request.setDashboardHandTrackingState);
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
	return std::make_unique<DashboardInputDriverModule>();
}

} // namespace dashboardinput
