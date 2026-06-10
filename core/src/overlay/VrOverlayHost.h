#pragma once

#include <openvr.h>

#include <string>

namespace openvr_pair::overlay {

// Owns the SteamVR dashboard-overlay lifecycle for the umbrella shell.
//
// The umbrella runs as a desktop GLFW + ImGui app first and an in-VR
// overlay second. VrOverlayHost is the bridge: it probes for SteamVR,
// upgrades to an overlay-mode VR session, creates a dashboard overlay
// keyed against the same `wk.wkopenvr` manifest the umbrella
// already registers, and translates OpenVR mouse / scroll / keyboard
// events into ImGuiIO so the in-VR overlay drives the same ImGui
// state that the desktop window does.
//
// The host does NOT own the ImGui context, the FBO, or the GL
// texture -- the umbrella main loop owns those and hands the texture
// name to SubmitTexture each frame the dashboard is visible. This
// keeps the host stateless about rendering and the shell stateless
// about OpenVR.
class VrOverlayHost
{
public:
	VrOverlayHost();
	~VrOverlayHost();

	VrOverlayHost(const VrOverlayHost&) = delete;
	VrOverlayHost& operator=(const VrOverlayHost&) = delete;

	// Per-frame pump. Retries VR_Init on a 1-second cadence until
	// SteamVR is up; creates the dashboard overlay once VR is
	// connected; drains overlay events into ImGuiIO; applies the safe
	// overlay visibility policy when DashboardInput is enabled. Returns
	// true when the real SteamVR dashboard overlay is currently visible.
	bool TickFrame(int overlayPixelWidth, int overlayPixelHeight);

	// Hand the umbrella's offscreen FBO texture to the VR overlays.
	// Safe to call every frame; no-ops when the overlay handle is
	// not yet created or no VR surface is visible.
	void SubmitTexture(unsigned int glTextureId, int width, int height);

	// SteamVR sent VREvent_Quit -- the umbrella should break its
	// main loop.
	bool QuitRequested() const { return quitRequested_; }

	void SetSafeOverlayEnabled(bool enabled);

	// User asked to show/hide the safe overlay (module tab button).
	// The flip is applied by the visibility policy on the next
	// TickFrame; the overlay auto-hides while the real dashboard is
	// open regardless of the requested state.
	void RequestSafeOverlayToggle();

	// True once we have a live Overlay-mode VR session. The
	// desktop window keeps working when this is false.
	bool VrConnected() const { return vrReady_; }
	bool AnyDashboardVisible() const { return anyDashboardVisible_; }
	uint32_t PrimaryDashboardDevice() const { return primaryDashboardDevice_; }
	int PrimaryDashboardHand() const { return primaryDashboardHand_; }
	bool SafeOverlayVisible() const { return safeOverlayVisible_; }
	const std::string& SafeOverlayStatus() const { return safeOverlayStatus_; }

private:
	bool TryInitVrStack();
	void TryCreateOverlay();
	void TryCreateSafeOverlay(int overlayPixelWidth, int overlayPixelHeight);
	void TickSafeOverlay(int overlayPixelWidth, int overlayPixelHeight);
	void UpdateSafeOverlayVisibility(bool visible);
	void DrainOverlayEvents();
	void DrainEventsForHandle(vr::VROverlayHandle_t handle);
	void RefreshDashboardState();
	void CleanupGlobalActionSetPrioritySetting();
	bool IsActiveDashboardOverlay() const;

	std::string ResolveIconPath() const;

	vr::VROverlayHandle_t mainHandle_ = 0;
	vr::VROverlayHandle_t thumbHandle_ = 0;
	vr::VROverlayHandle_t safeHandle_ = 0;

	bool vrReady_ = false;
	bool overlayCreated_ = false;
	bool safeOverlayCreated_ = false;
	bool safeOverlayFeatureEnabled_ = false;
	bool safeOverlayVisible_ = false;
	bool safeOverlayUserRequestedVisible_ = false;
	bool quitRequested_ = false;
	bool anyDashboardVisible_ = false;
	uint32_t primaryDashboardDevice_ = vr::k_unTrackedDeviceIndexInvalid;
	int primaryDashboardHand_ = 0;
	std::string safeOverlayStatus_ = "disabled";

	// Wall-clock seconds of the last TryInitVrStack attempt. Drives
	// the 1-second retry cadence so we don't hammer SteamVR while
	// it boots.
	double lastInitRetry_ = -1.0e9;
};

} // namespace openvr_pair::overlay
