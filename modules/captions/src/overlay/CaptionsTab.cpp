#include "CaptionsTab.h"
#include "AudioInputDevices.h"
#include "CaptionsPlugin.h"
#include "CaptionsTabLogic.h"
#include "HostStatusPoller.h"
#include "UiHelpers.h"
#include "Win32Paths.h"

// Win32Paths.h pulls in windows.h with WIN32_LEAN_AND_MEAN already set.
// imgui comes after to avoid any Windows header order conflicts.
#include <imgui/imgui.h>

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Consent dialog state (per-session; persisted in the plugin's profile).
// ---------------------------------------------------------------------------

static bool s_consent_pending = false;

// Capture-device list, refreshed when the picker combo is opened.
static std::vector<captions::AudioInputDevice> s_devices;
static bool s_devices_loaded = false;

// "No audio reaching the host" tracking: remember the last frame counter and
// when it last advanced so the warning only fires after a sustained stall.
static long long s_last_frames = -1;
static std::chrono::steady_clock::time_point s_last_frames_change{};

// ---------------------------------------------------------------------------
// Diagnostics: read last N lines from the newest captions log file.
// Rate-limited to at most once per 2 s to avoid constant directory scans.
// ---------------------------------------------------------------------------

namespace {

struct DiagState
{
	bool expanded = false;
	std::string log_tail; // last 20 lines, joined with \n
	std::chrono::steady_clock::time_point last_refresh{};
	static constexpr int kLines = 20;
	static constexpr auto kRefreshInterval = std::chrono::seconds(2);
};

static DiagState s_diag;

// Find the newest file in dir matching prefix (wide strings).
static std::wstring FindNewestLogFile(const std::wstring& dir, const std::wstring& prefix)
{
	std::wstring pat = dir + L"\\" + prefix + L"*";
	WIN32_FIND_DATAW fd{};
	HANDLE h = FindFirstFileW(pat.c_str(), &fd);
	if (h == INVALID_HANDLE_VALUE) return {};

	std::wstring best_name;
	FILETIME best_ft{};
	do {
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
		// Compare by last-write time.
		if (best_name.empty() || CompareFileTime(&fd.ftLastWriteTime, &best_ft) > 0) {
			best_name = fd.cFileName;
			best_ft = fd.ftLastWriteTime;
		}
	} while (FindNextFileW(h, &fd));
	FindClose(h);

	if (best_name.empty()) return {};
	return dir + L"\\" + best_name;
}

static void RefreshDiagnostics()
{
	auto now = std::chrono::steady_clock::now();
	if (now - s_diag.last_refresh < DiagState::kRefreshInterval) return;
	s_diag.last_refresh = now;

	std::wstring logs_dir = openvr_pair::common::WkOpenVrLogsPath(false);
	if (logs_dir.empty()) {
		s_diag.log_tail = "(could not resolve log directory)";
		return;
	}

	// Prefer the host log; also look for crash dumps.
	std::wstring host_log = FindNewestLogFile(logs_dir, L"captions_host_log.");
	std::wstring crash_log = FindNewestLogFile(logs_dir, L"captions_host_crash_");

	std::string tail;

	// Crash dump takes priority if newer than the host log.
	if (!crash_log.empty()) {
		std::ifstream cf(crash_log);
		if (cf) {
			std::ostringstream ss;
			ss << cf.rdbuf();
			tail = "--- crash dump ---\n" + ss.str();
		}
	}

	if (!host_log.empty()) {
		std::ifstream lf(host_log);
		if (lf) {
			std::vector<std::string> lines;
			std::string ln;
			while (std::getline(lf, ln))
				lines.push_back(ln);
			int start = std::max(0, (int)lines.size() - DiagState::kLines);
			std::ostringstream ss;
			for (int i = start; i < (int)lines.size(); ++i) {
				ss << lines[i] << "\n";
			}
			if (!tail.empty()) tail += "\n--- host log (last 20 lines) ---\n";
			tail += ss.str();
		}
	}

	if (tail.empty()) tail = "(no log files found yet)";
	s_diag.log_tail = std::move(tail);
}

} // namespace

namespace captions::ui {

static const char* StateLabel(int state)
{
	switch (state) {
		case 0:
			return "Idle";
		case 1:
			return "Listening";
		case 2:
			return "Transcribing";
		case 3:
			return "Translating";
		case 4:
			return "Sending";
		case 5:
			return "Error";
		default:
			return "Unknown";
	}
}

static bool IsSetupStatus(const std::string& message)
{
	return message == "Speech pack not installed." || message == "Whisper model not installed." ||
	       message == "Speech VAD model not installed." || message == "Speech detection runtime not installed." ||
	       message == "Translation runtime not installed." || message.rfind("Translation pack ", 0) == 0;
}

static const char* SpeechPackStatus(const captions::HostStatusSnapshot& snap)
{
	if (!snap.valid) return "Unknown";
	if (!snap.speech_pack_installed) return "Not installed";
	if (!snap.vad_runtime_available) return "Runtime missing";
	return "Installed";
}

static const char* TranslationPackStatus(const captions::HostStatusSnapshot& snap, const std::string& pack_id)
{
	if (pack_id.empty()) return "Unavailable";
	if (!snap.valid) return "Unknown";

	constexpr const char* kPrefix = "translation-";
	std::string expected_pair = pack_id;
	if (expected_pair.rfind(kPrefix, 0) == 0) {
		expected_pair = expected_pair.substr(strlen(kPrefix));
	}
	if (!expected_pair.empty() && !snap.active_translation_pair.empty() &&
	    snap.active_translation_pair != expected_pair) {
		return "Updating";
	}
	if (!snap.translation_pack_installed) return "Not installed";
	if (!snap.translation_runtime_available) return "Runtime missing";
	return "Installed";
}

// Map a pack status string to a status-cell tone.
static openvr_pair::overlay::ui::StatusTone PackTone(const char* status)
{
	using T = openvr_pair::overlay::ui::StatusTone;
	if (strcmp(status, "Installed") == 0) return T::Ok;
	if (strcmp(status, "Runtime missing") == 0) return T::Warn;
	if (strcmp(status, "Updating") == 0) return T::Pending;
	return T::Idle; // Not installed / Unknown / Unavailable
}

// A pack action button: greyed via DisabledSection, with a tooltip that always
// shows (even while disabled) so the reason for a greyed button is visible.
static void DrawPackActionButton(const char* label, const char* id, bool disabled, const char* tooltip,
                                 const std::function<void()>& action)
{
	openvr_pair::overlay::ui::DisabledSection gate(disabled);
	std::string button = std::string(label) + "##" + id;
	if (ImGui::SmallButton(button.c_str())) {
		action();
	}
	if (tooltip && tooltip[0] && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
		ImGui::SetTooltip("%s", tooltip);
	}
}

static void DrawSetup(CaptionsPlugin& plugin, const captions::HostStatusSnapshot& snap)
{
	using namespace openvr_pair::overlay::ui;
	DrawSectionHeading("Setup");

	const bool busy = plugin.IsPackActionRunning();
	const std::string translation_pack = plugin.CurrentTranslationPackId();
	const char* kBusyReason = "A captions pack action is already running.";

	TableScope table("captions_setup", 4,
	                 ImGuiTableFlags_BordersOuter | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
	if (table) {
		SetupStretchColumn("Pack", 2.0f);
		SetupStretchColumn("Status", 1.2f);
		SetupFixedColumn("Install", 98.0f);
		SetupFixedColumn("Remove", 98.0f);
		DrawTableHeader();

		// --- Speech row ---
		const bool speechInstalled = SpeechInstalled(snap);
		NextRow();
		SetColumn(0);
		ImGui::TextUnformatted("Speech");
		TooltipOnHover("Whisper base model, Silero VAD model, and ONNX Runtime.\n"
		               "Stored under %LocalAppDataLow%\\WKOpenVR\\captions.");
		SetColumn(1);
		{
			const char* st = SpeechPackStatus(snap);
			DrawStatusCell(st, PackTone(st));
		}
		SetColumn(2);
		DrawPackActionButton(speechInstalled ? "Installed" : "Install", "speech_install",
		                     !SpeechInstallEnabled(snap, busy),
		                     speechInstalled ? "The speech pack is already installed."
		                     : busy          ? kBusyReason
		                                     : "Download and verify the speech pack.",
		                     [&]() { plugin.InstallSpeechPack(); });
		SetColumn(3);
		DrawPackActionButton("Uninstall", "speech_uninstall", !SpeechUninstallEnabled(snap, busy),
		                     !speechInstalled ? "The speech pack is not installed."
		                     : busy           ? kBusyReason
		                                      : "Remove speech models and runtime files from this PC.",
		                     [&]() { plugin.UninstallSpeechPack(); });

		// --- Translation row ---
		const bool translationInstalled = TranslationInstalled(snap, translation_pack);
		NextRow();
		SetColumn(0);
		ImGui::TextUnformatted("Translation");
		TooltipOnHover("CTranslate2 CPU runtime plus the model for the selected language pair.\n"
		               "Runtime files are removed when the last translation pack is uninstalled.");
		SetColumn(1);
		if (plugin.GetTargetLang().empty()) {
			DrawStatusCell("Transcribe only", StatusTone::Idle);
		}
		else if (translation_pack.empty()) {
			DrawStatusCell("No pack", StatusTone::Idle);
			TooltipOnHover("Managed packs currently cover English to German, Spanish, French, Russian, and Chinese.");
		}
		else {
			const char* st = TranslationPackStatus(snap, translation_pack);
			DrawStatusCell(st, PackTone(st));
		}
		SetColumn(2);
		DrawPackActionButton(translationInstalled ? "Installed" : "Install", "translation_install",
		                     !TranslationInstallEnabled(snap, busy, translation_pack),
		                     translation_pack.empty() ? "Pick a target language with a managed pack first."
		                     : translationInstalled   ? "This translation pack is already installed."
		                     : busy                   ? kBusyReason
		                                              : "Download and verify the selected translation pack.",
		                     [&]() { plugin.InstallTranslationPack(); });
		SetColumn(3);
		DrawPackActionButton("Uninstall", "translation_uninstall",
		                     !TranslationUninstallEnabled(snap, busy, translation_pack),
		                     translation_pack.empty() ? "No translation pack is selected."
		                     : !translationInstalled  ? "This translation pack is not installed."
		                     : busy                   ? kBusyReason
		                                              : "Remove this language-pair model. Shared runtime files stay "
		                                                "until no installed translation pack needs them.",
		                     [&]() { plugin.UninstallTranslationPack(); });
	}

	if (!plugin.PackActionStatus().empty()) {
		DrawStatusText(plugin.PackActionStatus().c_str(), StatusTone::Info);
	}
}

// Microphone picker: "System default" plus each active capture endpoint.
static void DrawMicPicker(CaptionsPlugin& plugin)
{
	using namespace openvr_pair::overlay::ui;
	const std::string& current = plugin.GetInputDevice();

	if (!s_devices_loaded) {
		s_devices = captions::EnumerateCaptureDevices();
		s_devices_loaded = true;
	}

	// Resolve a friendly label for the current selection.
	std::string label = "System default";
	if (!current.empty()) {
		label = current; // fall back to the raw id if the device is gone
		for (const auto& d : s_devices) {
			if (d.id == current) {
				label = d.name;
				break;
			}
		}
	}

	if (ImGui::BeginCombo("Microphone", label.c_str())) {
		// Re-enumerate while open so hot-plugged devices appear.
		s_devices = captions::EnumerateCaptureDevices();

		bool defSel = current.empty();
		if (ImGui::Selectable("System default", defSel)) {
			plugin.SetInputDevice("");
		}
		if (defSel) ImGui::SetItemDefaultFocus();

		for (const auto& d : s_devices) {
			bool sel = (d.id == current);
			if (ImGui::Selectable(d.name.c_str(), sel)) {
				plugin.SetInputDevice(d.id);
			}
			if (sel) ImGui::SetItemDefaultFocus();
		}
		ImGui::EndCombo();
	}
	TooltipForLastItem("Which microphone the captions host listens to.\n"
	                   "\"System default\" follows your Windows default input device.");
}

static void DrawRealtimeOption(CaptionsPlugin& plugin, const char* label, uint8_t flag, const char* tooltip)
{
	bool enabled = plugin.GetRealtimeOption(flag);
	if (openvr_pair::overlay::ui::CheckboxWithTooltip(label, &enabled, tooltip)) {
		plugin.SetRealtimeOption(flag, enabled);
		plugin.PushConfigToDriver();
	}
}

static void DrawRealtimeTuning(CaptionsPlugin& plugin)
{
	using namespace openvr_pair::overlay::ui;
	DrawSectionHeading("Realtime tuning");

	DrawRealtimeOption(plugin, "Extended speech timing", captions::kCaptionsRealtimeExtendedTiming,
	                   "Uses a longer pre-roll, a slightly longer silence tail, faster continuous-speech flushes, "
	                   "and a larger overlap between continuous chunks.");
	DrawRealtimeOption(plugin, "Require speech evidence", captions::kCaptionsRealtimeSpeechEvidenceGate,
	                   "Ignores very short always-on opens unless VAD or input level actually indicated speech.");
	DrawRealtimeOption(
	    plugin, "Confidence filter", captions::kCaptionsRealtimeConfidenceFilter,
	    "Suppresses likely silence hallucinations, low-confidence output, and repeated decode artifacts.");
	DrawRealtimeOption(plugin, "Trim overlap duplicates", captions::kCaptionsRealtimeOverlapCleanup,
	                   "Removes repeated words created by overlapping continuous speech chunks.");
	DrawRealtimeOption(
	    plugin, "Split long chatbox messages", captions::kCaptionsRealtimeChatboxSplitting,
	    "Sends long captions as multiple paced messages instead of allowing VRChat output to be cut off.");
}

static void DrawPrivatePreview(CaptionsPlugin& plugin)
{
	using namespace openvr_pair::overlay::ui;
	DrawSectionHeading("Private preview");
	ImGui::SameLine();
	if (ImGui::SmallButton("Clear##captions_private_preview")) {
		plugin.ClearPreviewHistory();
	}

	const auto& entries = plugin.PreviewHistory().Entries();
	if (entries.empty()) {
		DrawEmptyState("No captions yet");
		return;
	}

	TableScope table("captions_private_preview_table", 2,
	                 ImGuiTableFlags_BordersOuter | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
	if (table) {
		SetupStretchColumn("Caption", 2.2f);
		SetupStretchColumn("Source", 1.4f);
		DrawTableHeader();
		for (const auto& entry : entries) {
			const std::string& caption = entry.translation.empty() ? entry.transcript : entry.translation;
			NextRow();
			SetColumn(0);
			ImGui::TextWrapped("%s", caption.c_str());
			SetColumn(1);
			if (!entry.translation.empty() && entry.translation != entry.transcript) {
				ImGui::TextWrapped("%s", entry.transcript.c_str());
			}
			else {
				DrawEmptyState("Transcribe only");
			}
		}
	}
}

// Live status: mic + input level + state, plus contextual banners.
static void DrawStatusStrip(CaptionsPlugin& plugin, const captions::HostStatusSnapshot& snap)
{
	using namespace openvr_pair::overlay::ui;

	if (snap.valid) {
		ImGui::Text("Mic: %s", snap.mic_name.empty() ? "(unknown)" : snap.mic_name.c_str());

		// Input level meter -- the direct proof that audio is reaching the host.
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Input level");
		ImGui::SameLine();
		char pct[16];
		std::snprintf(pct, sizeof(pct), "%d%%", (int)(snap.audio_level * 100.0f + 0.5f));
		ImGui::ProgressBar(snap.audio_level, ImVec2(180.0f, 0.0f), pct);

		ImGui::Text("State: %s  |  Output: %s  |  Sent: %lld", StateLabel(snap.state),
		            plugin.GetChatboxEnabled() ? "VRChat chatbox" : "local preview", snap.packets_sent);
		if (!snap.phase.empty() && snap.phase != "running") {
			ImGui::TextDisabled("Host phase: %s", snap.phase.c_str());
		}

		// No-audio warning: the endpoint has delivered no new frames for a while.
		auto now = std::chrono::steady_clock::now();
		if (snap.frames_captured != s_last_frames) {
			s_last_frames = snap.frames_captured;
			s_last_frames_change = now;
		}
		double since = std::chrono::duration<double>(now - s_last_frames_change).count();
		if (ShouldWarnNoAudio(snap.valid, since)) {
			DrawWaitingBanner("No audio is reaching the captions host from this device. Pick a different "
			                  "microphone under \"Input\" below, or check that it is active and receiving sound.");
		}

		if (plugin.GetMode() == 0 && !snap.ptt_registered) {
			const char* detail =
			    snap.ptt_error.empty() ? "SteamVR push-to-talk binding is not registered." : snap.ptt_error.c_str();
			DrawWaitingBanner(detail);
		}
		if (!snap.last_transcript.empty()) ImGui::Text("Transcript: %s", snap.last_transcript.c_str());
		if (!snap.last_translation.empty()) ImGui::Text("Translation: %s", snap.last_translation.c_str());
		if (!snap.last_error.empty()) {
			if (IsSetupStatus(snap.last_error)) {
				DrawWaitingBanner(snap.last_error.c_str());
			}
			else {
				DrawErrorBanner("Host error", snap.last_error.c_str());
			}
		}
		return;
	}

	if (snap.host_halted) {
		char detail[512];
		if (!snap.last_exit_description.empty()) {
			std::snprintf(detail, sizeof(detail), "%s\nExit code: 0x%08X. Open diagnostics for the latest host log.",
			              snap.last_exit_description.c_str(), (unsigned)snap.last_exit_code);
		}
		else {
			std::snprintf(detail, sizeof(detail),
			              "The host exited repeatedly before reporting status.\n"
			              "Open diagnostics for the latest host log and crash note.");
		}
		DrawErrorBanner("Captions host failed to start", detail);

		// Diagnostics collapsible: shows log tail + crash dump on demand.
		if (ImGui::TreeNode("Show diagnostics")) {
			s_diag.expanded = true;
			RefreshDiagnostics();

			DrawFilePath("%LocalAppDataLow%\\WKOpenVR\\Logs\\");
			ImGui::Spacing();

			if (!s_diag.log_tail.empty()) {
				ImGui::InputTextMultiline("##diag_log", const_cast<char*>(s_diag.log_tail.c_str()),
				                          s_diag.log_tail.size() + 1, ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * 12),
				                          ImGuiInputTextFlags_ReadOnly);
			}

			if (ImGui::Button("Refresh")) {
				s_diag.last_refresh = {}; // force re-read on next frame
			}
			ImGui::TreePop();
		}
		else {
			s_diag.expanded = false;
		}
		return;
	}

	DrawEmptyState("Host not running");
}

void DrawCaptionsTab(CaptionsPlugin& plugin)
{
	using namespace openvr_pair::overlay::ui;
	const auto& snap = plugin.HostStatus().Snapshot();

	// -----------------------------------------------------------------------
	// Always-on consent modal (detail lives in the help marker, not a blob).
	// -----------------------------------------------------------------------
	if (s_consent_pending) {
		ImGui::OpenPopup("##tr_aon_consent");
		s_consent_pending = false;
	}

	if (ImGui::BeginPopupModal("##tr_aon_consent", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
		ImGui::TextUnformatted("Always-on captures your microphone while WKOpenVR is open.");
		ImGui::SameLine();
		DrawHelpMarker("Audio is processed in memory and recognized locally (whisper.cpp + Silero VAD); nothing is "
		               "written to disk unless you enable transcript logging, and nothing leaves your PC.\n"
		               "To stop capture: switch back to Push-to-Talk or close WKOpenVR.");
		ImGui::Spacing();
		if (ImGui::Button("Enable always-on")) {
			plugin.SetAlwaysOnConsented(true);
			plugin.SetMode(1);
			plugin.PushConfigToDriver();
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button("Cancel")) {
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}

	// -----------------------------------------------------------------------
	// Live status
	// -----------------------------------------------------------------------
	DrawStatusStrip(plugin, snap);

	ImGui::Separator();
	DrawSetup(plugin, snap);

	// -----------------------------------------------------------------------
	// Mode
	// -----------------------------------------------------------------------
	ImGui::Separator();
	DrawSectionHeading("Mode");
	{
		int mode = plugin.GetMode();
		if (RadioButtonWithTooltip("Push-to-Talk", mode == 0, "Capture only while the bound SteamVR button is held.")) {
			plugin.SetMode(0);
			plugin.PushConfigToDriver();
		}
		ImGui::SameLine();
		if (RadioButtonWithTooltip("Always-on", mode == 1,
		                           "Continuously capture your microphone while WKOpenVR is open.\n"
		                           "Audio is processed locally; nothing is transmitted.")) {
			if (!plugin.HasAlwaysOnConsent()) {
				s_consent_pending = true;
			}
			else {
				plugin.SetMode(1);
				plugin.PushConfigToDriver();
			}
		}
	}

	// -----------------------------------------------------------------------
	// Realtime tuning
	// -----------------------------------------------------------------------
	ImGui::Separator();
	DrawRealtimeTuning(plugin);

	// -----------------------------------------------------------------------
	// Input (microphone) + language
	// -----------------------------------------------------------------------
	ImGui::Separator();
	DrawSectionHeading("Input");
	DrawMicPicker(plugin);

	{
		static const char* kSrcLangs[] = {"auto", "en", "zh", "ja", "ko", "ru", "de", "fr", "es", "pt"};
		static const char* kTgtLangs[] = {"(transcribe only)", "en", "zh", "ja", "ko", "ru", "de", "fr", "es", "pt"};

		// Source.
		const std::string& src = plugin.GetSourceLang();
		int src_idx = 0;
		for (int i = 0; i < (int)(sizeof(kSrcLangs) / sizeof(kSrcLangs[0])); ++i) {
			if (src == kSrcLangs[i]) {
				src_idx = i;
				break;
			}
		}
		if (ImGui::BeginCombo("Source language", kSrcLangs[src_idx])) {
			for (int i = 0; i < (int)(sizeof(kSrcLangs) / sizeof(kSrcLangs[0])); ++i) {
				bool sel = (i == src_idx);
				if (ImGui::Selectable(kSrcLangs[i], sel)) {
					plugin.SetSourceLang(kSrcLangs[i]);
					plugin.PushConfigToDriver();
				}
				if (sel) ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
		TooltipForLastItem("\"auto\" lets whisper.cpp detect the language per utterance.\n"
		                   "Setting an explicit code skips detection and saves ~50-100 ms per chunk.");

		// Target.
		const std::string& tgt = plugin.GetTargetLang();
		int tgt_idx = 0;
		for (int i = 0; i < (int)(sizeof(kTgtLangs) / sizeof(kTgtLangs[0])); ++i) {
			if (tgt == kTgtLangs[i]) {
				tgt_idx = i;
				break;
			}
		}
		if (ImGui::BeginCombo("Target language", kTgtLangs[tgt_idx])) {
			for (int i = 0; i < (int)(sizeof(kTgtLangs) / sizeof(kTgtLangs[0])); ++i) {
				bool sel = (i == tgt_idx);
				if (ImGui::Selectable(kTgtLangs[i], sel)) {
					plugin.SetTargetLang(i == 0 ? "" : kTgtLangs[i]);
					plugin.PushConfigToDriver();
				}
				if (sel) ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
		TooltipForLastItem("\"transcribe only\" sends speech text without translation.\n"
		                   "Pick a target to download an OPUS-MT checkpoint for that pair (~75-120 MB).");
	}

	// -----------------------------------------------------------------------
	// Output
	// -----------------------------------------------------------------------
	ImGui::Separator();
	DrawSectionHeading("Output");
	{
		bool chatbox = plugin.GetChatboxEnabled();
		if (CheckboxWithTooltip("Publish to VRChat chatbox", &chatbox,
		                        "Off keeps recognized speech local to WKOpenVR.\n"
		                        "On sends completed captions to the configured OSC chatbox address.")) {
			plugin.SetChatboxEnabled(chatbox);
			plugin.PushConfigToDriver();
		}

		DisabledSection chatboxGate(!chatbox, "Enable chatbox publishing before editing VRChat output settings.");

		static const char* kPresets[] = {"VRChat", "Custom"};
		int preset = 0; // default to VRChat
		const std::string& addr = plugin.GetChatboxAddress();
		if (addr != "/chatbox/input") preset = 1;

		if (ImGui::BeginCombo("Game preset", kPresets[preset])) {
			if (ImGui::Selectable("VRChat", preset == 0)) {
				plugin.SetChatboxAddress("/chatbox/input");
				plugin.PushConfigToDriver();
			}
			if (ImGui::Selectable("Custom", preset == 1)) {
				preset = 1;
			}
			ImGui::EndCombo();
		}
		TooltipForLastItem("ChilloutVR's OSC spec is not publicly machine-readable; use Custom\n"
		                   "and verify the address from the CVR docs.");
		if (!chatbox) chatboxGate.AttachReasonTooltip();

		if (preset == 1) {
			char addr_buf[64];
			std::snprintf(addr_buf, sizeof(addr_buf), "%s", addr.c_str());
			if (ImGui::InputText("OSC address", addr_buf, sizeof(addr_buf))) {
				plugin.SetChatboxAddress(addr_buf);
				plugin.PushConfigToDriver();
			}
			if (!chatbox) chatboxGate.AttachReasonTooltip();
		}

		bool notify = plugin.GetNotifySound();
		if (CheckboxWithTooltip("Notify listeners", &notify,
		                        "Play the VRChat chatbox notification sound for each message.\n"
		                        "Default off to avoid spamming nearby players.")) {
			plugin.SetNotifySound(notify);
			plugin.PushConfigToDriver();
		}
		if (!chatbox) chatboxGate.AttachReasonTooltip();
	}

	// -----------------------------------------------------------------------
	// Recent captions
	// -----------------------------------------------------------------------
	ImGui::Separator();
	DrawPrivatePreview(plugin);

	// -----------------------------------------------------------------------
	// Host controls
	// -----------------------------------------------------------------------
	ImGui::Separator();
	DrawSectionHeading("Host controls");
	if (ImGui::Button("Restart host")) {
		plugin.SendRestartHost();
		// Optimistically clear the halted indicator; PollSupervisorStatus will
		// confirm the new state on the next tick.
		plugin.HostStatus().SetSupervisorStatus(false, 0, {});
	}
	TooltipForLastItem("Terminate and respawn the captions sidecar process.\n"
	                   "Use when the host appears stuck or crashed.");
}

} // namespace captions::ui
