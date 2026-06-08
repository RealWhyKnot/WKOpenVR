#define _CRT_SECURE_NO_DEPRECATE
#include "HostSupervisor.h"
#include "Logging.h"

#include "DriverModule.h"
#include "FeatureFlags.h"
#include "ModuleRegistry.h"
#include "Protocol.h"
#include "ServerTrackedDeviceProvider.h"
#include "Win32Paths.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shlobj.h>
#include <objbase.h>
#include <openvr_driver.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

namespace captions {
namespace {

// Resolve the captions host exe path relative to the driver DLL.
// driver_wkopenvr.dll lives at <root>\bin\win64\; the host is at
// <root>\resources\captions\host\WKOpenVR.CaptionsHost.exe.
std::string ResolveHostExePath()
{
	HMODULE hSelf = nullptr;
	GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
	                   reinterpret_cast<LPCSTR>(&ResolveHostExePath), &hSelf);

	if (!hSelf) return {};

	char dllPath[MAX_PATH] = {};
	GetModuleFileNameA(hSelf, dllPath, MAX_PATH);
	std::string path(dllPath);
	// DLL lives at <root>/bin/win64/driver_wkopenvr.dll. Pop the filename,
	// then "win64", then "bin" -- three pops -- to reach <root>, then
	// append resources/captions/host/. The original two-pop landed at
	// <root>/bin and produced a phantom <root>/bin/resources/... path
	// that does not exist; CreateProcessW returned err=3 PATH_NOT_FOUND.
	// Same bug class the facetracking driver had; fixed there in 68fd11d.
	for (int up = 0; up < 3; ++up) {
		auto sep = path.find_last_of("/\\");
		if (sep == std::string::npos) break;
		path = path.substr(0, sep);
	}
	path += "\\resources\\captions\\host\\WKOpenVR.CaptionsHost.exe";
	return path;
}

bool ReadSavedSidecarEnabled()
{
	std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", false);
	if (dir.empty()) return true;
	std::wstring path = dir + L"\\captions.txt";

	FILE* f = _wfopen(path.c_str(), L"r");
	if (!f) return true;

	char line[512];
	while (fgets(line, sizeof line, f)) {
		size_t len = strlen(line);
		while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
			line[--len] = '\0';
		}
		static constexpr const char* kKey = "sidecar_enabled=";
		const size_t key_len = strlen(kKey);
		if (strncmp(line, kKey, key_len) != 0) continue;
		bool enabled = strcmp(line + key_len, "0") != 0;
		fclose(f);
		return enabled;
	}
	fclose(f);
	return true;
}

class CaptionsDriverModule final : public DriverModule
{
public:
	const char* Name() const override { return "Captions"; }
	uint32_t FeatureMask() const override { return pairdriver::kFeatureCaptions; }
	const char* PipeName() const override
	{
		return openvr_pair::common::modules::PipeName(openvr_pair::common::modules::ModuleId::Captions);
	}

	bool Init(DriverModuleContext&) override
	{
		TrDrvOpenLogFile();
		TR_LOG_DRV("[captions] driver module Init() entered");

		std::string host_path = ResolveHostExePath();
		TR_LOG_DRV("[captions] resolved host exe path: %s", host_path.c_str());

		// Verify the exe exists on disk before handing off to HostSupervisor.
		// CreateProcessW err=3 (PATH_NOT_FOUND) produces no obvious log without this.
		bool host_on_disk = false;
		if (!host_path.empty()) {
			DWORD attr = GetFileAttributesA(host_path.c_str());
			host_on_disk = (attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY) == 0);
		}
		TR_LOG_DRV("[captions] host exe exists on disk: %s", host_on_disk ? "true" : "false");
		if (!host_on_disk) {
			TR_LOG_DRV("[captions] host exe missing -- Captions feature will be inert until redeploy");
		}

		// Check launch-time dependencies that must be staged beside the host.
		// Optional inference runtimes are reported by the host status file
		// after it starts, so they are not treated as spawn blockers here.
		{
			std::string host_dir = host_path;
			size_t slash = host_dir.find_last_of("\\/");
			if (slash != std::string::npos) host_dir.resize(slash);
			std::string openvr_dll = host_dir + "\\openvr_api.dll";
			DWORD attr = GetFileAttributesA(openvr_dll.c_str());
			bool openvr_on_disk = attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY) == 0;
			TR_LOG_DRV("[captions] host launch dependency openvr_api.dll: %s (%s)",
			           openvr_on_disk ? "present" : "missing", openvr_dll.c_str());
			if (!openvr_on_disk) {
				TR_LOG_DRV("[captions] missing openvr_api.dll beside host; "
				           "CreateProcess may fail with STATUS_DLL_NOT_FOUND (0xC0000135)");
			}
		}

		supervisor_ = std::make_unique<HostSupervisor>(host_path);
		config_.master_enabled = ReadSavedSidecarEnabled() ? 1 : 0;
		TR_LOG_DRV("[module] saved captions sidecar setting: %s", config_.master_enabled ? "enabled" : "disabled");
		if (config_.master_enabled) {
			ApplySidecarState(true, BuildHostConfigCommand(config_));
		}
		else {
			TR_LOG_DRV("[module] captions sidecar disabled; host will not be started");
		}

		TR_LOG_DRV("[module] Init complete");
		return true;
	}

	void Shutdown() override
	{
		TR_LOG_DRV("[module] Shutdown()");
		StopSupervisor();
		supervisor_.reset();
		TR_LOG_DRV("[module] shutdown complete");
	}

	bool HandleRequest(const protocol::Request& req, protocol::Response& resp) override
	{
		switch (req.type) {
			case protocol::RequestSetCaptionsConfig: {
				std::string cmd;
				bool enabled = false;
				{
					std::lock_guard<std::mutex> lk(config_mutex_);
					config_ = req.setCaptionsConfig;
					enabled = config_.master_enabled != 0;
					// Forward relevant settings to the host via the control pipe.
					// The host re-reads its config via a simple tagged line protocol.
					cmd = BuildHostConfigCommand(config_);
				}
				ApplySidecarState(enabled, cmd);
				resp.type = protocol::ResponseSuccess;
				return true;
			}
			case protocol::RequestCaptionsRestartHost: {
				TR_LOG_DRV("[module] host restart requested by overlay");
				std::string cmd;
				bool enabled = false;
				{
					std::lock_guard<std::mutex> lk(config_mutex_);
					enabled = config_.master_enabled != 0;
					cmd = BuildHostConfigCommand(config_);
				}
				if (enabled) {
					RestartSupervisor(cmd);
				}
				else {
					TR_LOG_DRV("[module] restart ignored; captions sidecar disabled");
				}
				resp.type = protocol::ResponseSuccess;
				return true;
			}
			case protocol::RequestCaptionsGetSupervisorStatus: {
				resp.type = protocol::ResponseCaptionsSupervisorStatus;
				resp.captionsSupervisorStatus.host_halted = (supervisor_ && supervisor_->IsHalted()) ? 1 : 0;
				resp.captionsSupervisorStatus.last_exit_code = supervisor_ ? supervisor_->LastExitCode() : 0;
				const std::string desc = supervisor_ ? supervisor_->LastExitDescription() : std::string();
				std::snprintf(resp.captionsSupervisorStatus.last_exit_description,
				              sizeof(resp.captionsSupervisorStatus.last_exit_description), "%s", desc.c_str());
				return true;
			}
			default:
				return false;
		}
	}

private:
	void StartSupervisor(const std::string& cmd)
	{
		std::lock_guard<std::mutex> lk(supervisor_mutex_);
		if (!supervisor_) return;
		supervisor_->SetHostConfigCommand(cmd);
		if (supervisor_started_) return;

		supervisor_->CleanupStaleHostIfWedged();
		if (!supervisor_->Start()) {
			TR_LOG_DRV("[module] host initial spawn failed; supervisor monitor will retry");
		}
		supervisor_started_ = true;
	}

	void StopSupervisor()
	{
		std::lock_guard<std::mutex> lk(supervisor_mutex_);
		if (!supervisor_ || !supervisor_started_) return;

		TR_LOG_DRV("[module] stopping captions sidecar");
		supervisor_->RequestHostShutdown();
		supervisor_->Stop();
		supervisor_started_ = false;
	}

	void RestartSupervisor(const std::string& cmd)
	{
		std::lock_guard<std::mutex> lk(supervisor_mutex_);
		if (!supervisor_) return;
		supervisor_->SetHostConfigCommand(cmd);
		if (!supervisor_started_) {
			supervisor_->CleanupStaleHostIfWedged();
			if (!supervisor_->Start()) {
				TR_LOG_DRV("[module] host initial spawn failed; supervisor monitor will retry");
			}
			supervisor_started_ = true;
			return;
		}
		supervisor_->Restart();
	}

	void ApplySidecarState(bool enabled, const std::string& cmd)
	{
		if (enabled) {
			StartSupervisor(cmd);
		}
		else {
			StopSupervisor();
		}
	}

	static std::string BuildHostConfigCommand(const protocol::CaptionsConfig& config)
	{
		return std::string("config:") + "src=" + config.source_lang + ",tgt=" + config.target_lang +
		       ",mode=" + std::to_string((int)config.mode) + ",addr=" + config.chatbox_address +
		       ",port=" + std::to_string((int)config.chatbox_port) +
		       ",notify=" + std::to_string((int)config.notify_sound) +
		       ",chatbox=" + std::to_string((int)config.chatbox_enabled) +
		       ",log=" + std::to_string((int)config.transcript_logging) +
		       ",flags=" + std::to_string((int)config.realtime_flags) +
		       ",model=" + std::to_string((int)config.speech_model) + "\n";
	}

	std::unique_ptr<HostSupervisor> supervisor_;
	bool supervisor_started_ = false;
	mutable std::mutex supervisor_mutex_;

	protocol::CaptionsConfig config_{};
	mutable std::mutex config_mutex_;
};

} // namespace

std::unique_ptr<DriverModule> CreateDriverModule()
{
	return std::make_unique<CaptionsDriverModule>();
}

} // namespace captions
