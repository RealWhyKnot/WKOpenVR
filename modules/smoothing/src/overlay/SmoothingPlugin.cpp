// openvr.h must precede anything that may pull in openvr_driver.h via
// Protocol.h (IPCClient -> Protocol). Defining _OPENVR_API early makes
// Protocol.h skip the driver header, so the openvr.h declarations below
// don't redefine the same vr:: symbols.
#include <openvr.h>

#include "SmoothingPlugin.h"

#include "CalibrationAnchor.h"
#include "DebugLogging.h"
#include "DiscordPresenceComposer.h"
#include "DeviceFilters.h"
#include "Logging.h"
#include "Protocol.h"
#include "ShellContext.h"
#include "ShellFooter.h"
#include "UiHelpers.h"

#include <imgui.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <exception>
#include <memory>
#include <sstream>
#include <set>
#include <string>

void SmoothingPlugin::OnStart(openvr_pair::overlay::ShellContext &)
{
	smoothing::logging::OpenLogFile();
	SM_LOG("WKOpenVR-Smoothing plugin starting (protocol=v%u)", (unsigned)protocol::Version);
	const bool connectedNow = ConnectIfNeeded();
	if (ipc_.IsConnected()) {
		SM_LOG("[ipc] connected on first try (new=%d)", connectedNow ? 1 : 0);
	} else if (!connectError_.empty()) {
		SM_LOG("[ipc] initial connect failed: %s", connectError_.c_str());
	}
	SendConfig();
	SM_LOG("[config] pushed: smoothness=%d finger_mask=0x%04x",
		cfg_.smoothness, (unsigned)cfg_.finger_mask);
	ReplayDevicePredictions("startup");
}

void SmoothingPlugin::Tick(openvr_pair::overlay::ShellContext &)
{
	if (!ipc_.IsConnected() && ConnectIfNeeded()) {
		SM_LOG("[ipc] connected; replaying smoothing state");
		SendConfig();
		ReplayDevicePredictions("ipc-reconnect");
	}
	TickPredictionRestore();
	TickExternalToolDetection();
	TickCalibrationLockClear();
}

bool SmoothingPlugin::ConnectIfNeeded()
{
	if (ipc_.IsConnected()) return false;
	try {
		ipc_.Connect();
		if (!connectError_.empty()) {
			SM_LOG("[ipc] reconnect succeeded after error: %s", connectError_.c_str());
		}
		connectError_.clear();
		return true;
	} catch (const std::exception &e) {
		// Only log the message the first time it changes to avoid per-tick spam
		// while the driver is unavailable.
		if (connectError_ != e.what()) {
			SM_LOG("[ipc] connect failed: %s", e.what());
		}
		connectError_ = e.what();
	}
	return false;
}

void SmoothingPlugin::SendConfig()
{
	if (!ipc_.IsConnected()) return;
	protocol::Request req(protocol::RequestSetFingerSmoothing);
	// master_enabled is deprecated on the wire (see Protocol.h) -- the
	// resources/enable_smoothing.flag file is the real master toggle, gated
	// at driver load. Sending 1 unconditionally so the driver's reseed-mask
	// computation in SetFingerSmoothingConfig treats the connection as
	// "smoothing is live" for diff purposes.
	req.setFingerSmoothing.master_enabled = 1;
	cfg_.smoothness = std::clamp(cfg_.smoothness, 0, 100);
	req.setFingerSmoothing.smoothness = (uint8_t)cfg_.smoothness;
	req.setFingerSmoothing.finger_mask = cfg_.finger_mask;
	req.setFingerSmoothing._reserved = 0;
	for (int i = 0; i < 10; ++i) {
		int v = std::clamp(cfg_.per_finger_smoothness[i], 0, 100);
		cfg_.per_finger_smoothness[i] = v;
		req.setFingerSmoothing.per_finger_smoothness[i] = (uint8_t)v;
	}
	req.setFingerSmoothing._reserved2[0] = 0;
	req.setFingerSmoothing._reserved2[1] = 0;
	try {
		ipc_.SendBlocking(req);
		connectError_.clear();
	} catch (const std::exception &e) {
		connectError_ = e.what();
	}
}

void SmoothingPlugin::SendDevicePrediction(uint32_t openVRID, int smoothness)
{
	if (!ipc_.IsConnected()) return;
	if (smoothness < 0) smoothness = 0;
	if (smoothness > 100) smoothness = 100;
	protocol::Request req(protocol::RequestSetDevicePrediction);
	req.setDevicePrediction.openVRID = openVRID;
	req.setDevicePrediction.predictionSmoothness = (uint8_t)smoothness;
	// Smart flag rides with every per-device push; the driver caches it
	// alongside predictionSmoothness and only consumes it when smoothness
	// is non-zero, so sending it for clear-to-zero calls is harmless and
	// keeps the wire path uniform.
	req.setDevicePrediction.smart_enabled = cfg_.smart_smoothing ? 1 : 0;
	req.setDevicePrediction._reserved[0] = 0;
	req.setDevicePrediction._reserved[1] = 0;
	try {
		ipc_.SendBlocking(req);
		connectError_.clear();
	} catch (const std::exception &e) {
		connectError_ = e.what();
	}
}

static std::string BuildPredictionDeviceSignature()
{
	auto *vrSystem = vr::VRSystem();
	if (!vrSystem) return {};

	std::ostringstream sig;
	char buffer[vr::k_unMaxPropertyStringSize];
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		const auto deviceClass = vrSystem->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid) continue;
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, sizeof buffer, &err);
		if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
		sig << id << ":" << static_cast<int>(deviceClass) << ":" << buffer << ";";
	}
	return sig.str();
}

void SmoothingPlugin::ReplayDevicePredictions(const char *reason)
{
	// On startup / reconnect, walk the saved tracker_smoothness map and push
	// each entry to the driver. Without this, restored values would sit on
	// disk until the user happened to wiggle the slider for that device.
	if (!ipc_.IsConnected()) return;
	auto *vrSystem = vr::VRSystem();
	if (!vrSystem) return;

	int restored = 0;
	int cleared = 0;
	int skipped = 0;
	char buffer[vr::k_unMaxPropertyStringSize];
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		const auto deviceClass = vrSystem->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid) continue;
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, sizeof buffer, &err);
		if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;

		std::string serial = buffer;
		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, buffer, sizeof buffer, &err);
		std::string model = (err == vr::TrackedProp_Success) ? buffer : "";
		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, sizeof buffer, &err);
		std::string trackingSystem = (err == vr::TrackedProp_Success) ? buffer : "";
		if (!openvr_pair::overlay::ShouldShowInSmoothingPredictionList(
				deviceClass, serial, model, trackingSystem)) {
			if (cfg_.trackerSmoothness.find(serial) != cfg_.trackerSmoothness.end()) {
				SendDevicePrediction(id, 0);
				SM_LOG("[prediction] replay reason=%s id=%u serial='%s' value=0 hidden/internal",
					reason ? reason : "unknown", id, serial.c_str());
				++cleared;
			} else {
				++skipped;
			}
			continue;
		}
		if (deviceClass == vr::TrackedDeviceClass_HMD) {
			if (cfg_.trackerSmoothness.find(serial) != cfg_.trackerSmoothness.end()) {
				SendDevicePrediction(id, 0);
				SM_LOG("[prediction] replay reason=%s id=%u serial='%s' value=0 hmd-locked",
					reason ? reason : "unknown", id, serial.c_str());
				++cleared;
			} else {
				++skipped;
			}
			continue;
		}

		openvr_pair::overlay::CalibrationDeviceLockKind lockKind{};
		if (openvr_pair::overlay::TryGetCalibrationDeviceLockKind(serial, lockKind)) {
			if (cfg_.trackerSmoothness.find(serial) != cfg_.trackerSmoothness.end()) {
				SendDevicePrediction(id, 0);
				SM_LOG("[prediction] replay reason=%s id=%u serial='%s' value=0 calibration-locked",
					reason ? reason : "unknown", id, serial.c_str());
				++cleared;
			} else {
				++skipped;
			}
			continue;
		}

		auto it = cfg_.trackerSmoothness.find(serial);
		if (it == cfg_.trackerSmoothness.end()) {
			++skipped;
			continue;
		}
		SendDevicePrediction(id, it->second);
		SM_LOG("[prediction] replay reason=%s id=%u serial='%s' value=%d",
			reason ? reason : "unknown", id, serial.c_str(), it->second);
		++restored;
	}
	SM_LOG("[prediction] replay complete reason=%s restored=%d cleared=%d skipped=%d saved=%zu",
		reason ? reason : "unknown", restored, cleared, skipped, cfg_.trackerSmoothness.size());
}

void SmoothingPlugin::TickPredictionRestore()
{
	if (!ipc_.IsConnected()) return;
	const auto now = std::chrono::steady_clock::now();
	if (now - lastPredictionReplayScan_ < std::chrono::seconds(2)) return;
	lastPredictionReplayScan_ = now;

	std::string sig = BuildPredictionDeviceSignature();
	if (sig.empty()) return;
	if (lastPredictionDeviceSignature_.empty()) {
		lastPredictionDeviceSignature_ = std::move(sig);
		return;
	}
	if (sig == lastPredictionDeviceSignature_) return;

	lastPredictionDeviceSignature_ = std::move(sig);
	SM_LOG("[prediction] tracked-device list changed; replaying saved smoothing values");
	ReplayDevicePredictions("device-list-change");
}

void SmoothingPlugin::TickCalibrationLockClear()
{
	std::set<std::string> locks;

	for (const auto &kv : cfg_.trackerSmoothness) {
		openvr_pair::overlay::CalibrationDeviceLockKind lockKind{};
		if (openvr_pair::overlay::TryGetCalibrationDeviceLockKind(kv.first, lockKind)) {
			locks.insert(kv.first);
		}
	}
	if (locks == lastKnownCalibrationLocks_) return;

	auto *vrSystem = vr::VRSystem();
	if (!vrSystem || !ipc_.IsConnected()) return;

	char buffer[vr::k_unMaxPropertyStringSize];
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		if (vrSystem->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_Invalid) continue;
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, sizeof buffer, &err);
		if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
		std::string serial = buffer;
		if (locks.find(serial) == locks.end()) continue;
		if (lastKnownCalibrationLocks_.find(serial) != lastKnownCalibrationLocks_.end()) continue;
		SM_LOG("[smoothing] tracker %s is used by continuous calibration; smoothing disabled while locked", serial.c_str());
		SendDevicePrediction(id, 0);
	}

	lastKnownCalibrationLocks_ = std::move(locks);
}

void SmoothingPlugin::DrawTab(openvr_pair::overlay::ShellContext &)
{
	if (!connectError_.empty()) {
		openvr_pair::overlay::ui::DrawErrorBanner(
			"Smoothing driver connection failed",
			connectError_.c_str());
		ImGui::Separator();
	}

	if (ImGui::BeginTabBar("smoothing_tabs")) {
		if (ImGui::BeginTabItem("Settings")) { DrawSettingsTab(); ImGui::EndTabItem(); }
		// Advanced tab is a placeholder with no knobs of its own right now.
		// Commented out (not deleted) so the DrawAdvancedTab() implementation
		// below is preserved for the next time we have an advanced knob to
		// land -- re-enable this tab item rather than re-discovering where
		// the entry point lives.
		// if (ImGui::BeginTabItem("Advanced")) { DrawAdvancedTab(); ImGui::EndTabItem(); }
		// Logs moved to the umbrella's global Logs tab; DrawLogsTab's body is
		// hosted there via FeaturePlugin::DrawLogsSection.
		ImGui::EndTabBar();
	}

	openvr_pair::overlay::ShellFooterStatus footer;
	footer.driverConnected = ipc_.IsConnected();
	footer.driverLabel = "Smoothing driver";
	openvr_pair::overlay::DrawShellFooter(footer);
}

void SmoothingPlugin::DrawSettingsTab()
{
	bool smart = cfg_.smart_smoothing;
	if (ImGui::Checkbox("Smart smoothing", &smart)) {
		cfg_.smart_smoothing = smart;
		SaveConfig(cfg_);
		// Push the new flag down to every device that already has a saved
		// per-tracker value. ReplayDevicePredictions walks the saved map
		// and calls SendDevicePrediction, which now carries cfg_.smart_smoothing
		// alongside each device's smoothness. Devices without a saved value
		// pick up the flag the first time the user touches their slider.
		ReplayDevicePredictions(smart ? "smart-toggle-on" : "smart-toggle-off");
		SM_LOG("[smart] master toggle set to %s", smart ? "on" : "off");
	}
	openvr_pair::overlay::ui::TooltipForLastItem(
		"Adapts the prediction strength to how much each tracker is moving.\n"
		"Stationary: full slider strength (kills resting jitter when holding\n"
		"or aiming the controller).\n"
		"Walking / fast aim: relaxed toward 0 (no judder, no lag).\n"
		"Affects prediction only -- finger smoothing is unchanged.");
	ImGui::Spacing();

	openvr_pair::overlay::ui::DrawSectionHeading("Prediction");
	DrawPredictionTab();

	openvr_pair::overlay::ui::DrawSectionHeading("Finger smoothing");
	DrawFingersTab();
}

void SmoothingPlugin::DrawAdvancedTab()
{
	openvr_pair::overlay::ui::DrawTextWrapped(
		"No advanced smoothing knobs yet. Prediction and finger-smoothing "
		"controls live on Settings.");
}

void SmoothingPlugin::DrawLogsTab()
{
	openvr_pair::overlay::ui::DrawSectionHeading("File locations");
	const bool debugLogging = openvr_pair::common::IsDebugLoggingEnabled();
	openvr_pair::overlay::ui::DrawTextWrapped(
		debugLogging
			? "Debug logging is on. New Smoothing events append next to the umbrella's other logs."
			: "Debug logging is off. Enable it at the top of this Logs tab before reproducing an issue.");
	ImGui::Spacing();
	ImGui::TextWrapped("Overlay:  %%LocalAppDataLow%%\\WKOpenVR\\Logs\\smoothing_log.<ts>.txt");
	ImGui::TextWrapped("Driver:   %%LocalAppDataLow%%\\WKOpenVR\\Logs\\driver_log.<ts>.txt");
	ImGui::TextWrapped("Settings: %%LocalAppDataLow%%\\WKOpenVR\\profiles\\smoothing.txt");
}

void SmoothingPlugin::DrawLogsSection(openvr_pair::overlay::ShellContext &)
{
	// Same body as DrawLogsTab(); the umbrella's global Logs tab wraps this
	// in a collapsing header so the section heading + file paths render
	// cleanly alongside the other modules' log sections.
	DrawLogsTab();
}

void SmoothingPlugin::ProvidePresence(WKOpenVR::PresenceComposer &composer)
{
	WKOpenVR::PresenceUpdate u;

	if (externalSmoothingDetected_) {
		u.priority = 100;
		u.details  = "Smoothing trackers";
		u.state    = "external smoothing detected";
	} else {
		// Count trackers configured with prediction smoothness (non-zero map
		// entries). Count active finger curves: any bit in finger_mask where
		// the global smoothness or per-finger override is non-zero.
		const int trackers = static_cast<int>(cfg_.trackerSmoothness.size());

		int fingerCurves = 0;
		for (int i = 0; i < 10; ++i) {
			if ((cfg_.finger_mask >> i) & 1) {
				const int strength = cfg_.per_finger_smoothness[i] > 0
				    ? cfg_.per_finger_smoothness[i]
				    : cfg_.smoothness;
				if (strength > 0) ++fingerCurves;
			}
		}

		u.priority = 50;
		u.details  = "Smoothing trackers";
		u.state    = std::to_string(trackers) + " trackers | " +
		             std::to_string(fingerCurves) + " finger curves";
	}

	composer.Submit("Smoothing", std::move(u));
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateSmoothingPlugin()
{
	return std::make_unique<SmoothingPlugin>();
}

} // namespace openvr_pair::overlay
