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
	// connected; drains overlay events into ImGuiIO; pumps the safe
	// overlay input path when DashboardInput is enabled. Returns true
	// when the real SteamVR dashboard overlay is currently visible.
	bool TickFrame(int overlayPixelWidth, int overlayPixelHeight);

	// Hand the umbrella's offscreen FBO texture to the VR overlays.
	// Safe to call every frame; no-ops when the overlay handle is
	// not yet created or no VR surface is visible.
	void SubmitTexture(unsigned int glTextureId, int width, int height);

	// SteamVR sent VREvent_Quit -- the umbrella should break its
	// main loop.
	bool QuitRequested() const { return quitRequested_; }

	void SetSafeOverlayEnabled(bool enabled);

	// True once we have a live Overlay-mode VR session. The
	// desktop window keeps working when this is false.
	bool VrConnected() const { return vrReady_; }
	bool AnyDashboardVisible() const { return anyDashboardVisible_; }
	uint32_t PrimaryDashboardDevice() const { return primaryDashboardDevice_; }
	int PrimaryDashboardHand() const { return primaryDashboardHand_; }
	bool SafeOverlayVisible() const { return safeOverlayVisible_; }
	bool SafeOverlayInputReady() const { return safeInputReady_; }
	bool SafeOverlayGlobalPriorityEnabled() const { return safeGlobalPriorityEnabled_; }
	const std::string& SafeOverlayStatus() const { return safeOverlayStatus_; }

private:
	bool TryInitVrStack();
	void TryCreateOverlay();
	void TryCreateSafeOverlay();
	void TryInitSafeOverlayInput();
	void TickSafeOverlayInput(int overlayPixelWidth, int overlayPixelHeight);
	void UpdateSafeOverlayVisibility(bool visible);
	void PumpSafeOverlayPointerEvents(int width, int height);
	void DrainOverlayEvents();
	void RefreshDashboardState();
	bool IsActiveDashboardOverlay() const;

	std::string ResolveIconPath() const;
	std::string ResolveActionManifestPath() const;

	vr::VROverlayHandle_t mainHandle_ = 0;
	vr::VROverlayHandle_t thumbHandle_ = 0;
	vr::VROverlayHandle_t safeHandle_ = 0;

	vr::VRActionSetHandle_t safeToggleSet_ = vr::k_ulInvalidActionSetHandle;
	vr::VRActionSetHandle_t safePointerSet_ = vr::k_ulInvalidActionSetHandle;
	vr::VRActionHandle_t safeToggleAction_ = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t safePointerAction_ = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t safeClickAction_ = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t safeScrollAction_ = vr::k_ulInvalidActionHandle;
	vr::VRInputValueHandle_t leftHandSource_ = vr::k_ulInvalidInputValueHandle;
	vr::VRInputValueHandle_t rightHandSource_ = vr::k_ulInvalidInputValueHandle;

	bool vrReady_ = false;
	bool overlayCreated_ = false;
	bool safeOverlayCreated_ = false;
	bool safeOverlayFeatureEnabled_ = false;
	bool safeOverlayVisible_ = false;
	bool safeInputManifestLoaded_ = false;
	bool safeInputReady_ = false;
	bool safeGlobalPriorityEnabled_ = false;
	bool safeLeftToggleDown_ = false;
	bool safeRightToggleDown_ = false;
	bool safeClickDown_ = false;
	bool safeInputUnavailableLogged_ = false;
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
