#include "DriverModule.h"
#include "FeatureFlags.h"
#include "InputHealthHookInjector.h"
#include "InputHealthSnapshotPublisher.h"
#include "InputHealthSnapshotStaging.h"
#include "Logging.h"
#include "ModuleRegistry.h"
#include "ServerTrackedDeviceProvider.h"

#include <cstring>
#include <string>

namespace inputhealth {
namespace {

class InputHealthDriverModule final : public DriverModule
{
public:
	const char* Name() const override { return "Input Health"; }
	uint32_t FeatureMask() const override { return pairdriver::kFeatureInputHealth; }
	const char* PipeName() const override
	{
		return openvr_pair::common::modules::PipeName(openvr_pair::common::modules::ModuleId::InputHealth);
	}

	bool Init(DriverModuleContext& context) override
	{
		provider_ = context.provider;
		inputhealth::Init(provider_);
		inputhealth::SnapshotPublisherInit();
		return true;
	}

	void Shutdown() override
	{
		inputhealth::SnapshotPublisherShutdown();
		inputhealth::Shutdown();
		provider_ = nullptr;
	}

	void OnGetGenericInterface(const char* pchInterface, void* iface) override
	{
		if (!pchInterface || !iface) return;
		// strstr on the raw C string avoids a std::string allocation per
		// interface query (this fires for every interface SteamVR asks for
		// during driver init, a few dozen times per boot).
		if (std::strstr(pchInterface, "IVRDriverInput_") != nullptr &&
		    std::strstr(pchInterface, "Internal") == nullptr) {
			inputhealth::TryInstallScalarBooleanHooks(iface);
		}
	}

	bool HandleRequest(const protocol::Request& request, protocol::Response& response) override
	{
		if (!provider_) return false;
		switch (request.type) {
			case protocol::RequestSetInputHealthConfig:
				provider_->SetInputHealthConfig(request.setInputHealthConfig);
				response.type = protocol::ResponseSuccess;
				return true;
			case protocol::RequestResetInputHealthStats:
				inputhealth::ApplyResetRequest(request.resetInputHealthStats);
				response.type = protocol::ResponseSuccess;
				return true;
			case protocol::RequestSetInputHealthCompensation:
				provider_->SetInputHealthCompensation(request.setInputHealthCompensation);
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
	return std::make_unique<InputHealthDriverModule>();
}

} // namespace inputhealth
