#pragma once

#include "FeaturePlugin.h"
#include "CaptionPreviewHistory.h"
#include "CaptionsRealtimeFlags.h"
#include "HostStatusPoller.h"
#include "Protocol.h"
#include "CaptionsIpcClient.h"
#include "CaptionsSpeechModels.h"

#include <chrono>
#include <string>

class CaptionsPlugin;

namespace captions::ui {
void DrawCaptionsTab(CaptionsPlugin& plugin);
} // namespace captions::ui

class CaptionsPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
	CaptionsPlugin();

	const char* Name() const override { return ModuleName(ModuleId::Captions); }
	const char* FlagFileName() const override { return ModuleFlagFileName(ModuleId::Captions); }
	const char* PipeName() const override { return ModulePipeName(ModuleId::Captions); }
	openvr_pair::overlay::FeaturePluginChannel Channel() const override
	{
		return openvr_pair::overlay::FeaturePluginChannel::Development;
	}

	void OnStart(openvr_pair::overlay::ShellContext& ctx) override;
	void OnShutdown(openvr_pair::overlay::ShellContext& ctx) override;
	void Tick(openvr_pair::overlay::ShellContext& ctx) override;
	void DrawTab(openvr_pair::overlay::ShellContext& ctx) override;

	// Config push to driver.
	void PushConfigToDriver();

	// Send a restart-host request to the driver.
	void SendRestartHost();

	void InstallSpeechPack();
	void UninstallSpeechPack();
	void InstallTranslationPack();
	void UninstallTranslationPack();
	bool IsPackActionRunning() const { return pack_process_ != nullptr; }
	const std::string& PackActionStatus() const { return pack_status_; }
	std::string CurrentTranslationPackId() const;
	bool HasManagedTranslationPack() const;

	// Query the driver supervisor state and propagate host_halted to the
	// host status snapshot so the tab can show appropriate guidance.
	void PollSupervisorStatus();

	// Accessors for the UI helpers.
	captions::HostStatusPoller& HostStatus() { return host_status_; }
	const captions::CaptionPreviewHistory& PreviewHistory() const { return preview_history_; }
	void ClearPreviewHistory() { preview_history_.Clear(); }

	int GetMode() const { return mode_; }
	void SetMode(int m);
	bool HasAlwaysOnConsent() const { return always_on_consented_; }
	void SetAlwaysOnConsented(bool v);
	bool GetSidecarEnabled() const { return sidecar_enabled_; }

	const std::string& GetSourceLang() const { return source_lang_; }
	const std::string& GetTargetLang() const { return target_lang_; }
	bool GetChatboxEnabled() const { return chatbox_enabled_; }
	const std::string& GetChatboxAddress() const { return chatbox_address_; }
	bool GetNotifySound() const { return notify_sound_; }
	const std::string& GetInputDevice() const { return input_device_; }
	bool GetRealtimeOption(uint8_t flag) const { return captions::CaptionsRealtimeFlagEnabled(realtime_flags_, flag); }
	bool GetRealtimeOptionMask(uint8_t mask) const
	{
		return captions::CaptionsRealtimeMaskEnabled(realtime_flags_, mask);
	}
	uint8_t GetRealtimeFlags() const { return realtime_flags_; }
	uint8_t GetSpeechModel() const { return speech_model_; }

	void SetSourceLang(const std::string& s);
	void SetTargetLang(const std::string& s);
	void SetSidecarEnabled(bool v);
	void SetChatboxEnabled(bool v);
	void SetChatboxAddress(const std::string& s);
	void SetNotifySound(bool v);
	void SetRealtimeOption(uint8_t flag, bool enabled);
	void SetRealtimeOptionMask(uint8_t mask, bool enabled);
	void SetSpeechModel(uint8_t model);
	// Select the capture endpoint (IMMDevice id; empty = system default).
	// Persists to captions.txt and writes the host-readable audio_input.txt.
	void SetInputDevice(const std::string& endpointId);

private:
	friend void captions::ui::DrawCaptionsTab(CaptionsPlugin& plugin);

	CaptionsIpcClient ipc_;
	captions::HostStatusPoller host_status_;
	captions::CaptionPreviewHistory preview_history_;

	// Settings cached here and pushed to the driver.
	bool sidecar_enabled_ = true;
	int mode_ = 0; // 0=PTT, 1=always-on
	bool always_on_consented_ = false;
	std::string source_lang_ = "auto";
	std::string target_lang_ = "";
	bool chatbox_enabled_ = false;
	std::string chatbox_address_ = "/chatbox/input";
	bool notify_sound_ = false;
	uint8_t realtime_flags_ = captions::kCaptionsRealtimeDefaultFlags;
	uint8_t speech_model_ = captions::kCaptionsSpeechModelBalanced;
	std::string input_device_ = ""; // capture endpoint id; empty = system default

	std::string last_error_;
	std::string pack_status_;
	void* pack_process_ = nullptr;
	std::wstring pack_script_path_;
	std::wstring pack_manifest_path_;
	uint64_t observed_ipc_generation_ = 0;

	std::chrono::steady_clock::time_point last_connection_check_{};

	void MaintainDriverConnection();
	void DrawStatusBanner();
	void RefreshPackResourcePaths(const openvr_pair::overlay::ShellContext& ctx);
	void StartPackAction(const std::string& pack_id, bool uninstall);
	void PollPackAction();

	// Persist the current settings snapshot to profiles/captions.txt.
	// Called from every Set* method so the next launch reads the same state.
	// Atomic write via Move-temp pattern (see CaptionsConfig.cpp).
	void Persist();
};
