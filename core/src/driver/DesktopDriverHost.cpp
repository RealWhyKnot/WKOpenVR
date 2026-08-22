#include "DesktopDriverHost.h"

#include "DiagnosticsLog.h"
#include "DriverModule.h"
#include "FeatureFlags.h"
#include "Logging.h"
#include "ModuleRegistry.h"

#include <cstdarg>
#include <exception>

namespace module_registry = openvr_pair::common::modules;

uint32_t DesktopFeatureMask()
{
	size_t count = 0;
	const module_registry::ModuleInfo* modules = module_registry::All(&count);
	uint32_t mask = 0;
	for (size_t i = 0; i < count; ++i) {
		if (modules[i].supports_desktop) mask |= pairdriver::FeatureMaskForModule(modules[i].id);
	}
	return mask;
}

uint32_t DesktopHostableMask()
{
	uint32_t mask = 0;
#if OPENVR_PAIR_HAS_OSCROUTER_DRIVER
	mask |= pairdriver::kFeatureOscRouter;
#endif
#if OPENVR_PAIR_HAS_FACETRACKING_DRIVER
	mask |= pairdriver::kFeatureFaceTracking;
#endif
#if OPENVR_PAIR_HAS_CAPTIONS_DRIVER
	mask |= pairdriver::kFeatureCaptions;
#endif
	return mask;
}

DesktopDriverHost::~DesktopDriverHost()
{
	Stop();
}

uint32_t DesktopDriverHost::Start(uint32_t featureMask, const std::wstring& resourcesDir)
{
	if (running_) return activeMask_;

	const uint32_t wanted = featureMask & DesktopHostableMask();
	LOG("desktop host starting requested_mask=0x%08x hostable_mask=0x%08x", (unsigned)featureMask, (unsigned)wanted);
	DriverModuleContext context{nullptr, nullptr, wanted, resourcesDir};

	// Modules are constructed before any pipe opens: a server thread iterating
	// hosted_ while Start() is still appending would race the reallocation.
	auto activate = [&](std::unique_ptr<DriverModule> module) {
		if (!module) return;
		const uint32_t moduleMask = module->FeatureMask();
		if ((wanted & moduleMask) == 0) return;
		bool initialized = false;
		try {
			initialized = module->Init(context);
		}
		catch (const std::exception& ex) {
			LOG("desktop host module '%s' init threw: %s", module->Name(), ex.what());
			return;
		}
		catch (...) {
			LOG("desktop host module '%s' init threw an unknown exception", module->Name());
			return;
		}
		if (!initialized) {
			LOG("desktop host module '%s' failed to initialize", module->Name());
			return;
		}
		activeMask_ |= moduleMask;
		hosted_.push_back({std::move(module), nullptr});
	};

	// OscRouter first: the other modules publish through it during Init.
#if OPENVR_PAIR_HAS_OSCROUTER_DRIVER
	activate(oscrouter::CreateDriverModule());
#endif
#if OPENVR_PAIR_HAS_FACETRACKING_DRIVER
	activate(facetracking::CreateDriverModule());
#endif
#if OPENVR_PAIR_HAS_CAPTIONS_DRIVER
	activate(captions::CreateDriverModule());
#endif

	for (Hosted& entry : hosted_) {
		const char* pipe = entry.module->PipeName();
		if (!pipe || !*pipe) continue;
		entry.server = std::make_unique<IPCServer>(this, pipe, entry.module->FeatureMask());
		entry.server->Run();
	}

	running_ = !hosted_.empty();
	LOG("desktop host started modules=%zu active_mask=0x%08x", hosted_.size(), (unsigned)activeMask_);
	return activeMask_;
}

void DesktopDriverHost::Stop()
{
	if (hosted_.empty()) {
		running_ = false;
		activeMask_ = 0;
		return;
	}
	LOG("desktop host stopping modules=%zu", hosted_.size());

	// Every pipe closes before any module is destroyed: IPCServer::Stop joins
	// its thread, so no dispatch can still be inside a module afterwards.
	for (Hosted& entry : hosted_) {
		if (entry.server) entry.server->Stop();
	}
	for (auto it = hosted_.rbegin(); it != hosted_.rend(); ++it) {
		it->server.reset();
		try {
			it->module->Shutdown();
		}
		catch (const std::exception& ex) {
			LOG("desktop host module '%s' shutdown threw: %s", it->module->Name(), ex.what());
		}
		catch (...) {
			LOG("desktop host module shutdown threw an unknown exception");
		}
	}
	hosted_.clear();
	running_ = false;
	activeMask_ = 0;
	LOG("desktop host stopped");
}

bool DesktopDriverHost::HandleIpcRequest(uint32_t featureMask, const protocol::Request& request,
                                         protocol::Response& response)
{
	if (request.type == protocol::RequestHandshake) {
		response.type = protocol::ResponseHandshake;
		response.protocol.version = protocol::Version;
		return true;
	}
	for (Hosted& entry : hosted_) {
		if ((entry.module->FeatureMask() & featureMask) == 0) continue;
		try {
			if (entry.module->HandleRequest(request, response)) return true;
		}
		catch (const std::exception& ex) {
			LOG("desktop host module '%s' request threw: %s", entry.module->Name(), ex.what());
			return false;
		}
		catch (...) {
			LOG("desktop host module '%s' request threw an unknown exception", entry.module->Name());
			return false;
		}
	}
	return false;
}

// The driver DLL's Logging.cpp cannot link into the app: it defines the same
// LogFile/OpenLogFile globals the inputhealth and smoothing overlays already
// own there. Shared driver sources reach the diagnostics log through this.
void LogLine(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	openvr_pair::common::DiagnosticLogV("driver-host", fmt, args);
	va_end(args);
}
