#include "VrOverlayHost.h"

#include "DashboardInputSafeOverlayLogic.h"
#include "DiagnosticsLog.h"
#include "Win32Paths.h"

#include <imgui.h>

#include <cstdio>
#include <filesystem>
#include <fstream>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace openvr_pair::overlay {

namespace {

// Must match the manifest in core/src/overlay/manifest.vrmanifest and the
// constant in ManifestRegistration.cpp. SteamVR keys overlays by app key,
// so re-using the manifest key makes the dashboard tile and the autolaunch
// registration point at the same entity.
constexpr const char* kAppKey = "wk.wkopenvr";
constexpr const char* kFriendlyName = "WKOpenVR";
constexpr const char* kSafeOverlayKey = "wk.wkopenvr.dashboardinput.safe";
constexpr const char* kSafeOverlayName = "WKOpenVR";

// Width of the dashboard panel in metres. SC used 3.0; same value
// renders comfortably at typical viewing distance.
constexpr float kOverlayWidthMeters = 3.0f;
constexpr uint32_t kSafeOverlaySortOrder = 100;

// Retry the VR session probe at most once per second. Matches SC.
constexpr double kInitRetrySeconds = 1.0;

// Discrete-scroll scale factor for VREvent_ScrollDiscrete.
//
// xdelta/ydelta from VREvent_Scroll_t are in "ticks" -- one full
// scroll-wheel notch reports 1.0. ImGui's MouseWheel input expects the
// same units (1.0 = one notch -> ~5 lines of scroll). On Quest controllers
// reaching the overlay through Virtual Desktop the input is already
// 1.0-per-click, so any multiplier larger than ~1 makes a single thumbstick
// nudge scroll the whole window. The previous value (360*8 = 2880) was a
// holdover from a SteamVR-Input-driven config that delivered sub-tick
// fractional deltas and needed amplification; the modern overlay-event
// path doesn't.
//
// Leaving a small >1 amplification (2.0) so a single click moves ~10 lines
// rather than 5 -- closer to desktop scroll-wheel feel without overshooting
// the page.
constexpr float kScrollScale = 2.0f;

#ifdef _WIN32
std::filesystem::path ExeDir()
{
	char buf[MAX_PATH] = {};
	const DWORD len = GetModuleFileNameA(nullptr, buf, MAX_PATH);
	if (len == 0 || len == MAX_PATH) return {};
	std::filesystem::path p(buf);
	return p.parent_path();
}
#else
std::filesystem::path ExeDir()
{
	return {};
}
#endif

double NowSeconds()
{
#ifdef _WIN32
	LARGE_INTEGER freq{};
	LARGE_INTEGER counter{};
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&counter);
	return double(counter.QuadPart) / double(freq.QuadPart);
#else
	return 0.0;
#endif
}

vr::HmdMatrix34_t SafeOverlayHmdRelativeTransform()
{
	vr::HmdMatrix34_t mat{};
	mat.m[0][0] = 1.0f;
	mat.m[1][1] = 1.0f;
	mat.m[2][2] = 1.0f;
	mat.m[0][3] = 0.0f;
	mat.m[1][3] = -0.05f;
	mat.m[2][3] = -1.25f;
	return mat;
}

} // namespace

VrOverlayHost::VrOverlayHost() = default;

VrOverlayHost::~VrOverlayHost()
{
	openvr_pair::common::DiagnosticLog(
	    "vr-overlay", "destroy overlay_created=%d safe_created=%d vr_ready=%d main=%llu thumb=%llu safe=%llu",
	    overlayCreated_ ? 1 : 0, safeOverlayCreated_ ? 1 : 0, vrReady_ ? 1 : 0, (unsigned long long)mainHandle_,
	    (unsigned long long)thumbHandle_, (unsigned long long)safeHandle_);
	if (overlayCreated_ && vr::VROverlay()) {
		if (mainHandle_) vr::VROverlay()->DestroyOverlay(mainHandle_);
		if (thumbHandle_) vr::VROverlay()->DestroyOverlay(thumbHandle_);
	}
	if (safeOverlayCreated_ && vr::VROverlay() && safeHandle_) {
		vr::VROverlay()->DestroyOverlay(safeHandle_);
	}
	if (vrReady_) {
		vr::VR_Shutdown();
	}
}

std::string VrOverlayHost::ResolveIconPath() const
{
	std::filesystem::path p = ExeDir();
	if (p.empty()) return {};
	p /= "dashboard_icon.png";
	return p.string();
}

void VrOverlayHost::SetSafeOverlayEnabled(bool enabled)
{
	if (safeOverlayFeatureEnabled_ == enabled) return;
	safeOverlayFeatureEnabled_ = enabled;
	if (!enabled) {
		UpdateSafeOverlayVisibility(false);
		safeOverlayStatus_ = "disabled";
		safeOverlayUserRequestedVisible_ = false;
	}
	else {
		safeOverlayStatus_ = safeOverlayCreated_ ? "ready" : "starting";
	}
	openvr_pair::common::DiagnosticLog("vr-overlay", "safe_overlay_feature_enabled enabled=%d", enabled ? 1 : 0);
}

void VrOverlayHost::RequestSafeOverlayToggle()
{
	safeOverlayUserRequestedVisible_ = !safeOverlayUserRequestedVisible_;
	openvr_pair::common::DiagnosticLog("vr-overlay", "safe_overlay_toggle_requested visible=%d",
	                                   safeOverlayUserRequestedVisible_ ? 1 : 0);
}

bool VrOverlayHost::TryInitVrStack()
{
	if (vrReady_) return true;

	// Phase 1: probe with Background. Background-mode VR_Init fails fast
	// when vrserver isn't running instead of auto-launching SteamVR, which
	// is what we want -- the umbrella should not bring SteamVR up by
	// merely existing.
	vr::EVRInitError err = vr::VRInitError_None;
	vr::VR_Init(&err, vr::VRApplication_Background);
	if (err != vr::VRInitError_None) {
		// Not fatal. The most common error here is
		// VRInitError_Init_NoServerForBackgroundApp, which just means
		// SteamVR is not running yet.
		static vr::EVRInitError s_lastBackgroundErr = vr::VRInitError_None;
		static double s_lastBackgroundLog = -1e9;
		const double now = NowSeconds();
		if (err != s_lastBackgroundErr || now - s_lastBackgroundLog >= 30.0) {
			openvr_pair::common::DiagnosticLog("vr-overlay", "vr_init_background_failed error=%d description='%s'",
			                                   (int)err, vr::VR_GetVRInitErrorAsEnglishDescription(err));
			s_lastBackgroundErr = err;
			s_lastBackgroundLog = now;
		}
		return false;
	}
	openvr_pair::common::DiagnosticLog("vr-overlay", "vr_init_background_ok");

	// Phase 2: tear the probe down and re-init as Overlay so we can
	// create dashboard overlays.
	vr::VR_Shutdown();

	vr::VR_Init(&err, vr::VRApplication_Overlay);
	if (err != vr::VRInitError_None) {
		fprintf(stderr, "[VrOverlayHost] VR_Init(Overlay) failed: %s\n",
		        vr::VR_GetVRInitErrorAsEnglishDescription(err));
		openvr_pair::common::DiagnosticLog("vr-overlay", "vr_init_overlay_failed error=%d description='%s'", (int)err,
		                                   vr::VR_GetVRInitErrorAsEnglishDescription(err));
		return false;
	}

	if (!vr::VR_IsInterfaceVersionValid(vr::IVRSystem_Version) ||
	    !vr::VR_IsInterfaceVersionValid(vr::IVROverlay_Version)) {
		fprintf(stderr, "[VrOverlayHost] OpenVR interface version mismatch; "
		                "runtime DLL out of date.\n");
		openvr_pair::common::DiagnosticLog("vr-overlay", "interface_version_mismatch");
		vr::VR_Shutdown();
		return false;
	}

	vrReady_ = true;
	openvr_pair::common::DiagnosticLog("vr-overlay", "vr_init_overlay_ok");
	CleanupGlobalActionSetPrioritySetting();
	return true;
}

void VrOverlayHost::CleanupGlobalActionSetPrioritySetting()
{
	// Earlier dashboardinput builds force-enabled SteamVR's experimental
	// "global input from overlays" setting and left it behind. The input
	// path no longer uses action-set priority, so remove the key once if
	// it is still set. The sentinel keeps this from re-running, so a user
	// who later enables the setting for another overlay app keeps their
	// choice.
	const std::wstring root = openvr_pair::common::LocalAppDataLowPath();
	if (root.empty()) return;
	std::filesystem::path sentinel(root);
	sentinel /= L"WKOpenVR";
	sentinel /= L"dashboardinput_globalpriority_cleanup.flag";
	std::error_code ec;
	if (std::filesystem::exists(sentinel, ec)) return;

	bool wasEnabled = false;
	bool removed = false;
	if (vr::VRSettings()) {
		vr::EVRSettingsError settingsErr = vr::VRSettingsError_None;
		wasEnabled = vr::VRSettings()->GetBool(vr::k_pch_SteamVR_Section,
		                                       vr::k_pch_SteamVR_AllowGlobalActionSetPriority, &settingsErr);
		if (settingsErr != vr::VRSettingsError_None) wasEnabled = false;
		if (wasEnabled) {
			settingsErr = vr::VRSettingsError_None;
			vr::VRSettings()->RemoveKeyInSection(vr::k_pch_SteamVR_Section,
			                                     vr::k_pch_SteamVR_AllowGlobalActionSetPriority, &settingsErr);
			removed = settingsErr == vr::VRSettingsError_None;
		}
	}

	std::filesystem::create_directories(sentinel.parent_path(), ec);
	std::ofstream out(sentinel);
	out << "cleanup=done\n";
	openvr_pair::common::DiagnosticLog("vr-overlay", "global_priority_setting_cleanup was_enabled=%d removed=%d",
	                                   wasEnabled ? 1 : 0, removed ? 1 : 0);
}

void VrOverlayHost::TryCreateOverlay()
{
	if (overlayCreated_ || !vr::VROverlay()) return;

	const vr::EVROverlayError err =
	    vr::VROverlay()->CreateDashboardOverlay(kAppKey, kFriendlyName, &mainHandle_, &thumbHandle_);

	if (err == vr::VROverlayError_KeyInUse) {
		// Another instance of the umbrella is already registered with
		// vrserver. Log but don't crash -- the user can fix this by
		// closing the other instance.
		fprintf(stderr, "[VrOverlayHost] CreateDashboardOverlay: key in use "
		                "(another WKOpenVR instance is registered)\n");
		openvr_pair::common::DiagnosticLog("vr-overlay", "create_dashboard_overlay_key_in_use");
		return;
	}
	if (err != vr::VROverlayError_None) {
		fprintf(stderr, "[VrOverlayHost] CreateDashboardOverlay failed: %s\n",
		        vr::VROverlay()->GetOverlayErrorNameFromEnum(err));
		openvr_pair::common::DiagnosticLog("vr-overlay", "create_dashboard_overlay_failed error=%d name='%s'", (int)err,
		                                   vr::VROverlay()->GetOverlayErrorNameFromEnum(err));
		return;
	}

	vr::VROverlay()->SetOverlayWidthInMeters(mainHandle_, kOverlayWidthMeters);
	vr::VROverlay()->SetOverlayInputMethod(mainHandle_, vr::VROverlayInputMethod_Mouse);
	vr::VROverlay()->SetOverlayFlag(mainHandle_, vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);

	const std::string iconPath = ResolveIconPath();
	if (!iconPath.empty() && std::filesystem::exists(iconPath)) {
		vr::VROverlay()->SetOverlayFromFile(thumbHandle_, iconPath.c_str());
		openvr_pair::common::DiagnosticLog("vr-overlay", "dashboard_icon_set path='%s'", iconPath.c_str());
	}
	else {
		openvr_pair::common::DiagnosticLog("vr-overlay", "dashboard_icon_missing path='%s'", iconPath.c_str());
	}

	overlayCreated_ = true;
	fprintf(stderr, "[VrOverlayHost] dashboard overlay created (main=%llu thumb=%llu)\n",
	        (unsigned long long)mainHandle_, (unsigned long long)thumbHandle_);
	openvr_pair::common::DiagnosticLog("vr-overlay", "dashboard_overlay_created main=%llu thumb=%llu width_m=%.2f",
	                                   (unsigned long long)mainHandle_, (unsigned long long)thumbHandle_,
	                                   kOverlayWidthMeters);
}

void VrOverlayHost::TryCreateSafeOverlay(int overlayPixelWidth, int overlayPixelHeight)
{
	if (safeOverlayCreated_ || !vr::VROverlay()) return;

	const vr::EVROverlayError err = vr::VROverlay()->CreateOverlay(kSafeOverlayKey, kSafeOverlayName, &safeHandle_);
	if (err == vr::VROverlayError_KeyInUse) {
		safeOverlayStatus_ = "overlay key busy";
		openvr_pair::common::DiagnosticLog("vr-overlay", "create_safe_overlay_key_in_use");
		return;
	}
	if (err != vr::VROverlayError_None) {
		safeOverlayStatus_ = "overlay create failed";
		openvr_pair::common::DiagnosticLog("vr-overlay", "create_safe_overlay_failed error=%d name='%s'", (int)err,
		                                   vr::VROverlay()->GetOverlayErrorNameFromEnum(err));
		return;
	}

	// Input comes from the runtime laser mouse: Mouse input method plus
	// MakeOverlaysInteractiveIfVisible activates the system laser whenever
	// the overlay is shown, and the events arrive through
	// PollNextOverlayEvent like the main dashboard overlay. No action
	// manifest, no action-set priority, no reserved-button bindings.
	vr::VROverlay()->SetOverlayWidthInMeters(safeHandle_, kOverlayWidthMeters);
	vr::VROverlay()->SetOverlayInputMethod(safeHandle_, vr::VROverlayInputMethod_Mouse);
	vr::VROverlay()->SetOverlayFlag(safeHandle_, vr::VROverlayFlags_MakeOverlaysInteractiveIfVisible, true);
	vr::VROverlay()->SetOverlayFlag(safeHandle_, vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);
	vr::VROverlay()->SetOverlaySortOrder(safeHandle_, kSafeOverlaySortOrder);
	const vr::HmdMatrix34_t transform = SafeOverlayHmdRelativeTransform();
	vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(safeHandle_, vr::k_unTrackedDeviceIndex_Hmd, &transform);
	if (overlayPixelWidth > 0 && overlayPixelHeight > 0) {
		// Seed the mouse scale at creation so the first events already
		// arrive in texture-pixel space; SubmitTexture refreshes it.
		vr::HmdVector2_t mouseScale{};
		mouseScale.v[0] = static_cast<float>(overlayPixelWidth);
		mouseScale.v[1] = static_cast<float>(overlayPixelHeight);
		vr::VROverlay()->SetOverlayMouseScale(safeHandle_, &mouseScale);
	}

	safeOverlayCreated_ = true;
	if (safeOverlayFeatureEnabled_) safeOverlayStatus_ = "ready";
	openvr_pair::common::DiagnosticLog(
	    "vr-overlay", "safe_overlay_created handle=%llu width_m=%.2f hmd_relative_pos=(%.2f,%.2f,%.2f) sort=%u",
	    (unsigned long long)safeHandle_, kOverlayWidthMeters, transform.m[0][3], transform.m[1][3], transform.m[2][3],
	    kSafeOverlaySortOrder);
}

bool VrOverlayHost::IsActiveDashboardOverlay() const
{
	if (!overlayCreated_ || !vr::VROverlay()) return false;
	return vr::VROverlay()->IsActiveDashboardOverlay(mainHandle_);
}

void VrOverlayHost::UpdateSafeOverlayVisibility(bool visible)
{
	if (!safeOverlayCreated_ || !vr::VROverlay()) {
		safeOverlayVisible_ = false;
		if (visible) safeOverlayStatus_ = "overlay unavailable";
		return;
	}
	if (visible == safeOverlayVisible_) return;

	vr::EVROverlayError err = vr::VROverlayError_None;
	if (visible) {
		const vr::HmdMatrix34_t transform = SafeOverlayHmdRelativeTransform();
		err = vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(safeHandle_, vr::k_unTrackedDeviceIndex_Hmd,
		                                                                &transform);
		if (err == vr::VROverlayError_None) {
			err = vr::VROverlay()->ShowOverlay(safeHandle_);
		}
	}
	else {
		err = vr::VROverlay()->HideOverlay(safeHandle_);
		if (ImGui::GetCurrentContext()) {
			// A laser click can be mid-press when the overlay hides;
			// release so ImGui's button state can't get stuck down.
			ImGui::GetIO().AddMouseButtonEvent(0, false);
		}
	}

	if (err != vr::VROverlayError_None) {
		safeOverlayStatus_ = visible ? "show failed" : "hide failed";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_overlay_visibility_failed visible=%d error=%d name='%s'",
		                                   visible ? 1 : 0, (int)err,
		                                   vr::VROverlay()->GetOverlayErrorNameFromEnum(err));
		return;
	}

	safeOverlayVisible_ = visible;
	safeOverlayStatus_ = visible ? "visible" : (safeOverlayFeatureEnabled_ ? "ready" : "disabled");
	openvr_pair::common::DiagnosticLog("vr-overlay", "safe_overlay_visible visible=%d", visible ? 1 : 0);
}

void VrOverlayHost::TickSafeOverlay(int overlayPixelWidth, int overlayPixelHeight)
{
	if (!safeOverlayFeatureEnabled_) {
		UpdateSafeOverlayVisibility(false);
		return;
	}

	TryCreateSafeOverlay(overlayPixelWidth, overlayPixelHeight);
	UpdateSafeOverlayVisibility(DashboardInputSafeOverlayShouldBeVisible(
	    safeOverlayFeatureEnabled_, safeOverlayUserRequestedVisible_, anyDashboardVisible_));
}

void VrOverlayHost::RefreshDashboardState()
{
	anyDashboardVisible_ = false;
	primaryDashboardDevice_ = vr::k_unTrackedDeviceIndexInvalid;
	primaryDashboardHand_ = 0;

	if (!vrReady_ || !vr::VROverlay()) return;
	anyDashboardVisible_ = vr::VROverlay()->IsDashboardVisible();
	if (!anyDashboardVisible_) return;

	primaryDashboardDevice_ = vr::VROverlay()->GetPrimaryDashboardDevice();
	if (primaryDashboardDevice_ == vr::k_unTrackedDeviceIndexInvalid || !vr::VRSystem()) return;

	const vr::ETrackedControllerRole role =
	    vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(primaryDashboardDevice_);
	if (role == vr::TrackedControllerRole_LeftHand) {
		primaryDashboardHand_ = 1;
	}
	else if (role == vr::TrackedControllerRole_RightHand) {
		primaryDashboardHand_ = 2;
	}
}

void VrOverlayHost::DrainOverlayEvents()
{
	if (!vr::VROverlay()) return;
	if (overlayCreated_) DrainEventsForHandle(mainHandle_);
	if (safeOverlayCreated_) DrainEventsForHandle(safeHandle_);
}

void VrOverlayHost::DrainEventsForHandle(vr::VROverlayHandle_t handle)
{
	ImGuiIO& io = ImGui::GetIO();
	vr::VREvent_t ev{};
	while (vr::VROverlay()->PollNextOverlayEvent(handle, &ev, sizeof(ev))) {
		switch (ev.eventType) {
			case vr::VREvent_MouseMove:
				io.AddMousePosEvent(ev.data.mouse.x, ev.data.mouse.y);
				break;
			case vr::VREvent_MouseButtonDown:
				io.AddMouseButtonEvent(
				    (ev.data.mouse.button & vr::VRMouseButton_Left) == vr::VRMouseButton_Left ? 0 : 1, true);
				break;
			case vr::VREvent_MouseButtonUp:
				io.AddMouseButtonEvent(
				    (ev.data.mouse.button & vr::VRMouseButton_Left) == vr::VRMouseButton_Left ? 0 : 1, false);
				break;
			case vr::VREvent_ScrollDiscrete: {
				const float x = ev.data.scroll.xdelta * kScrollScale;
				const float y = ev.data.scroll.ydelta * kScrollScale;
				io.AddMouseWheelEvent(x, y);
				break;
			}
			case vr::VREvent_Quit:
				quitRequested_ = true;
				openvr_pair::common::DiagnosticLog("vr-overlay", "quit_event");
				break;
			default:
				break;
		}
	}
}

bool VrOverlayHost::TickFrame(int overlayPixelWidth, int overlayPixelHeight)
{
	// Stage 1: bring the VR session up. While SteamVR is down the rest
	// of this function is a no-op and the umbrella continues to render
	// to its desktop window only.
	if (!vrReady_) {
		const double now = NowSeconds();
		if (now - lastInitRetry_ >= kInitRetrySeconds) {
			lastInitRetry_ = now;
			TryInitVrStack();
		}
		if (!vrReady_) {
			RefreshDashboardState();
			return false;
		}
	}

	// Stage 2: register the dashboard overlay (one-shot), then refresh
	// dashboard state BEFORE the safe-overlay policy so the auto-hide
	// decision sees the current frame's visibility.
	TryCreateOverlay();
	RefreshDashboardState();
	TickSafeOverlay(overlayPixelWidth, overlayPixelHeight);

	// Stage 3: drain OpenVR events into ImGui IO. Virtual-keyboard
	// support is intentionally deferred -- replacing an active
	// ImGui InputText buffer needs imgui_internal API that varies
	// across the pin range and is best landed after the dashboard
	// render path is verified end-to-end.
	DrainOverlayEvents();

	return IsActiveDashboardOverlay();
}

void VrOverlayHost::SubmitTexture(unsigned int glTextureId, int width, int height)
{
	if (!vr::VROverlay()) return;
	if (glTextureId == 0 || width <= 0 || height <= 0) return;

	vr::Texture_t tex{};
	tex.handle = reinterpret_cast<void*>(static_cast<uintptr_t>(glTextureId));
	tex.eType = vr::TextureType_OpenGL;
	tex.eColorSpace = vr::ColorSpace_Auto;

	vr::HmdVector2_t mouseScale{};
	mouseScale.v[0] = static_cast<float>(width);
	mouseScale.v[1] = static_cast<float>(height);

	if (overlayCreated_ && IsActiveDashboardOverlay()) {
		vr::VROverlay()->SetOverlayTexture(mainHandle_, &tex);
		vr::VROverlay()->SetOverlayMouseScale(mainHandle_, &mouseScale);
	}
	if (safeOverlayCreated_ && safeOverlayVisible_) {
		vr::VROverlay()->SetOverlayTexture(safeHandle_, &tex);
		vr::VROverlay()->SetOverlayMouseScale(safeHandle_, &mouseScale);
	}
}

} // namespace openvr_pair::overlay
