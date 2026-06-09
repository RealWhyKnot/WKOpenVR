#include "VrOverlayHost.h"

#include "DashboardInputSafeOverlayLogic.h"
#include "DiagnosticsLog.h"

#include <imgui.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>

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

constexpr const char* kSafeToggleActionSet = "/actions/dashboardinput_toggle";
constexpr const char* kSafePointerActionSet = "/actions/dashboardinput_pointer";
constexpr const char* kSafeToggleAction = "/actions/dashboardinput_toggle/in/toggle";
constexpr const char* kSafePointerAction = "/actions/dashboardinput_pointer/in/pointer";
constexpr const char* kSafeClickAction = "/actions/dashboardinput_pointer/in/left_click";
constexpr const char* kSafeScrollAction = "/actions/dashboardinput_pointer/in/scroll";
constexpr const char* kLeftHandSource = "/user/hand/left";
constexpr const char* kRightHandSource = "/user/hand/right";

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
constexpr float kSafeOverlayScrollScale = 2.0f;

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

vr::VRActiveActionSet_t ActiveSet(vr::VRActionSetHandle_t actionSet, vr::VRInputValueHandle_t device)
{
	vr::VRActiveActionSet_t set{};
	set.ulActionSet = actionSet;
	set.ulRestrictedToDevice = device;
	set.ulSecondaryActionSet = vr::k_ulInvalidActionSetHandle;
	set.nPriority = DashboardInputSafeOverlayPriority();
	return set;
}

struct SafeOverlayPointerHit
{
	bool hit = false;
	float x = 0.0f;
	float y = 0.0f;
	float distance = 0.0f;
	vr::VRInputValueHandle_t source = vr::k_ulInvalidInputValueHandle;
};

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

std::string VrOverlayHost::ResolveActionManifestPath() const
{
	std::filesystem::path p = ExeDir();
	if (p.empty()) return {};
	p /= "action_manifest.json";
	return p.string();
}

void VrOverlayHost::SetSafeOverlayEnabled(bool enabled)
{
	if (safeOverlayFeatureEnabled_ == enabled) return;
	safeOverlayFeatureEnabled_ = enabled;
	if (!enabled) {
		UpdateSafeOverlayVisibility(false);
		safeOverlayStatus_ = "disabled";
		safeLeftToggleDown_ = false;
		safeRightToggleDown_ = false;
		safeClickDown_ = false;
	}
	else {
		safeOverlayStatus_ = safeInputReady_ ? "ready" : "starting";
	}
	openvr_pair::common::DiagnosticLog("vr-overlay", "safe_overlay_feature_enabled enabled=%d", enabled ? 1 : 0);
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
	return true;
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

void VrOverlayHost::TryCreateSafeOverlay()
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

	vr::VROverlay()->SetOverlayWidthInMeters(safeHandle_, kOverlayWidthMeters);
	vr::VROverlay()->SetOverlayInputMethod(safeHandle_, vr::VROverlayInputMethod_None);
	vr::VROverlay()->SetOverlaySortOrder(safeHandle_, kSafeOverlaySortOrder);
	const vr::HmdMatrix34_t transform = SafeOverlayHmdRelativeTransform();
	vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(safeHandle_, vr::k_unTrackedDeviceIndex_Hmd, &transform);

	safeOverlayCreated_ = true;
	openvr_pair::common::DiagnosticLog(
	    "vr-overlay", "safe_overlay_created handle=%llu width_m=%.2f hmd_relative_pos=(%.2f,%.2f,%.2f) sort=%u",
	    (unsigned long long)safeHandle_, kOverlayWidthMeters, transform.m[0][3], transform.m[1][3], transform.m[2][3],
	    kSafeOverlaySortOrder);
}

void VrOverlayHost::TryInitSafeOverlayInput()
{
	if (safeInputReady_ || !vr::VRInput()) return;

	const std::string manifestPath = ResolveActionManifestPath();
	if (manifestPath.empty() || !std::filesystem::exists(manifestPath)) {
		safeOverlayStatus_ = "missing action manifest";
		if (!safeInputUnavailableLogged_) {
			openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_manifest_missing path='%s'",
			                                   manifestPath.c_str());
			safeInputUnavailableLogged_ = true;
		}
		return;
	}

	if (!safeInputManifestLoaded_) {
		const vr::EVRInputError manifestErr = vr::VRInput()->SetActionManifestPath(manifestPath.c_str());
		if (manifestErr != vr::VRInputError_None && manifestErr != vr::VRInputError_MismatchedActionManifest) {
			safeOverlayStatus_ = "action manifest rejected";
			openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_manifest_failed error=%d path='%s'",
			                                   (int)manifestErr, manifestPath.c_str());
			return;
		}
		safeInputManifestLoaded_ = true;
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_manifest_loaded error=%d path='%s'",
		                                   (int)manifestErr, manifestPath.c_str());
	}

	const auto getSet = [](const char* path, vr::VRActionSetHandle_t& out) {
		return vr::VRInput()->GetActionSetHandle(path, &out);
	};
	const auto getAction = [](const char* path, vr::VRActionHandle_t& out) {
		return vr::VRInput()->GetActionHandle(path, &out);
	};
	const auto getSource = [](const char* path, vr::VRInputValueHandle_t& out) {
		return vr::VRInput()->GetInputSourceHandle(path, &out);
	};

	vr::EVRInputError err = getSet(kSafeToggleActionSet, safeToggleSet_);
	if (err != vr::VRInputError_None) {
		safeOverlayStatus_ = "toggle set missing";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_get_toggle_set_failed error=%d", (int)err);
		return;
	}
	err = getSet(kSafePointerActionSet, safePointerSet_);
	if (err != vr::VRInputError_None) {
		safeOverlayStatus_ = "pointer set missing";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_get_pointer_set_failed error=%d", (int)err);
		return;
	}
	err = getAction(kSafeToggleAction, safeToggleAction_);
	if (err != vr::VRInputError_None) {
		safeOverlayStatus_ = "toggle action missing";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_get_toggle_action_failed error=%d", (int)err);
		return;
	}
	err = getAction(kSafePointerAction, safePointerAction_);
	if (err != vr::VRInputError_None) {
		safeOverlayStatus_ = "pointer action missing";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_get_pointer_action_failed error=%d", (int)err);
		return;
	}
	err = getAction(kSafeClickAction, safeClickAction_);
	if (err != vr::VRInputError_None) {
		safeOverlayStatus_ = "click action missing";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_get_click_action_failed error=%d", (int)err);
		return;
	}
	err = getAction(kSafeScrollAction, safeScrollAction_);
	if (err != vr::VRInputError_None) {
		safeOverlayStatus_ = "scroll action missing";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_get_scroll_action_failed error=%d", (int)err);
		return;
	}
	err = getSource(kLeftHandSource, leftHandSource_);
	if (err != vr::VRInputError_None) {
		safeOverlayStatus_ = "left hand missing";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_get_left_source_failed error=%d", (int)err);
		return;
	}
	err = getSource(kRightHandSource, rightHandSource_);
	if (err != vr::VRInputError_None) {
		safeOverlayStatus_ = "right hand missing";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_get_right_source_failed error=%d", (int)err);
		return;
	}

	if (vr::VRSettings()) {
		vr::EVRSettingsError settingsErr = vr::VRSettingsError_None;
		const bool alreadyEnabled = vr::VRSettings()->GetBool(
		    vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_AllowGlobalActionSetPriority, &settingsErr);
		if (settingsErr == vr::VRSettingsError_None && !alreadyEnabled) {
			vr::VRSettings()->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_AllowGlobalActionSetPriority, true,
			                          &settingsErr);
		}
		settingsErr = vr::VRSettingsError_None;
		safeGlobalPriorityEnabled_ = vr::VRSettings()->GetBool(
		    vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_AllowGlobalActionSetPriority, &settingsErr);
		if (settingsErr != vr::VRSettingsError_None) {
			safeGlobalPriorityEnabled_ = false;
			openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_global_priority_read_failed error=%d",
			                                   (int)settingsErr);
		}
	}

	safeInputReady_ = true;
	safeOverlayStatus_ = safeGlobalPriorityEnabled_ ? "ready" : "priority unavailable";
	openvr_pair::common::DiagnosticLog(
	    "vr-overlay",
	    "safe_input_ready global_priority=%d toggle_set=%llu pointer_set=%llu toggle_action=%llu pointer_action=%llu "
	    "click_action=%llu scroll_action=%llu left_source=%llu right_source=%llu",
	    safeGlobalPriorityEnabled_ ? 1 : 0, (unsigned long long)safeToggleSet_, (unsigned long long)safePointerSet_,
	    (unsigned long long)safeToggleAction_, (unsigned long long)safePointerAction_,
	    (unsigned long long)safeClickAction_, (unsigned long long)safeScrollAction_,
	    (unsigned long long)leftHandSource_, (unsigned long long)rightHandSource_);
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
		if (safeClickDown_ && ImGui::GetCurrentContext()) {
			ImGui::GetIO().AddMouseButtonEvent(0, false);
		}
		safeClickDown_ = false;
	}

	if (err != vr::VROverlayError_None) {
		safeOverlayStatus_ = visible ? "show failed" : "hide failed";
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_overlay_visibility_failed visible=%d error=%d name='%s'",
		                                   visible ? 1 : 0, (int)err,
		                                   vr::VROverlay()->GetOverlayErrorNameFromEnum(err));
		return;
	}

	safeOverlayVisible_ = visible;
	safeOverlayStatus_ =
	    visible ? "visible" : (safeOverlayFeatureEnabled_ ? (safeInputReady_ ? "ready" : "starting") : "disabled");
	openvr_pair::common::DiagnosticLog("vr-overlay", "safe_overlay_visible visible=%d", visible ? 1 : 0);
}

void VrOverlayHost::TickSafeOverlayInput(int overlayPixelWidth, int overlayPixelHeight)
{
	if (!safeOverlayFeatureEnabled_) {
		UpdateSafeOverlayVisibility(false);
		return;
	}

	TryCreateSafeOverlay();
	TryInitSafeOverlayInput();
	if (!safeInputReady_ || !vr::VRInput()) {
		UpdateSafeOverlayVisibility(false);
		return;
	}

	const bool activateToggle = DashboardInputSafeOverlayToggleActive(safeOverlayFeatureEnabled_, safeInputReady_);
	const bool activatePointer =
	    DashboardInputSafeOverlayPointerActive(safeOverlayFeatureEnabled_, safeInputReady_, safeOverlayVisible_);
	const uint32_t expectedSetCount =
	    DashboardInputSafeOverlayActionSetCount(safeOverlayFeatureEnabled_, safeInputReady_, safeOverlayVisible_);

	std::array<vr::VRActiveActionSet_t, 4> sets{};
	uint32_t setCount = 0;
	if (activateToggle) {
		sets[setCount++] = ActiveSet(safeToggleSet_, leftHandSource_);
		sets[setCount++] = ActiveSet(safeToggleSet_, rightHandSource_);
	}
	if (activatePointer) {
		sets[setCount++] = ActiveSet(safePointerSet_, leftHandSource_);
		sets[setCount++] = ActiveSet(safePointerSet_, rightHandSource_);
	}
	if (setCount == 0) return;
	if (setCount != expectedSetCount) {
		openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_set_count_mismatch expected=%u actual=%u",
		                                   expectedSetCount, setCount);
	}

	const vr::EVRInputError updateErr =
	    vr::VRInput()->UpdateActionState(sets.data(), sizeof(vr::VRActiveActionSet_t), setCount);
	if (updateErr != vr::VRInputError_None) {
		if (safeOverlayStatus_ != "action update failed") {
			openvr_pair::common::DiagnosticLog("vr-overlay", "safe_input_update_failed error=%d", (int)updateErr);
		}
		safeOverlayStatus_ = "action update failed";
		return;
	}
	if (!safeOverlayVisible_) safeOverlayStatus_ = safeGlobalPriorityEnabled_ ? "ready" : "priority unavailable";

	auto readToggle = [&](vr::VRInputValueHandle_t source, bool& wasDown) {
		vr::InputDigitalActionData_t data{};
		const vr::EVRInputError err =
		    vr::VRInput()->GetDigitalActionData(safeToggleAction_, &data, sizeof(data), source);
		if (err != vr::VRInputError_None) {
			wasDown = false;
			return false;
		}
		const bool down = data.bActive && data.bState;
		const bool pressed = down && !wasDown;
		wasDown = down;
		return pressed;
	};

	const bool leftTogglePressed = readToggle(leftHandSource_, safeLeftToggleDown_);
	const bool rightTogglePressed = readToggle(rightHandSource_, safeRightToggleDown_);
	const bool togglePressed = leftTogglePressed || rightTogglePressed;
	if (togglePressed) {
		UpdateSafeOverlayVisibility(!safeOverlayVisible_);
	}

	if (safeOverlayVisible_) {
		PumpSafeOverlayPointerEvents(overlayPixelWidth, overlayPixelHeight);
	}
	else if (safeClickDown_ && ImGui::GetCurrentContext()) {
		ImGui::GetIO().AddMouseButtonEvent(0, false);
		safeClickDown_ = false;
	}
}

void VrOverlayHost::PumpSafeOverlayPointerEvents(int width, int height)
{
	if (!safeOverlayCreated_ || !safeOverlayVisible_ || !vr::VRInput() || !vr::VROverlay()) return;
	if (width <= 0 || height <= 0) return;

	auto readHit = [&](vr::VRInputValueHandle_t source) {
		SafeOverlayPointerHit hit{};
		vr::InputPoseActionData_t pose{};
		const vr::EVRInputError poseErr = vr::VRInput()->GetPoseActionDataRelativeToNow(
		    safePointerAction_, vr::TrackingUniverseStanding, 0.0f, &pose, sizeof(pose), source);
		if (poseErr != vr::VRInputError_None || !pose.bActive || !pose.pose.bPoseIsValid) return hit;

		const vr::HmdMatrix34_t& m = pose.pose.mDeviceToAbsoluteTracking;
		vr::VROverlayIntersectionParams_t params{};
		params.eOrigin = vr::TrackingUniverseStanding;
		params.vSource.v[0] = m.m[0][3];
		params.vSource.v[1] = m.m[1][3];
		params.vSource.v[2] = m.m[2][3];
		params.vDirection.v[0] = -m.m[0][2];
		params.vDirection.v[1] = -m.m[1][2];
		params.vDirection.v[2] = -m.m[2][2];

		vr::VROverlayIntersectionResults_t result{};
		if (!vr::VROverlay()->ComputeOverlayIntersection(safeHandle_, &params, &result)) return hit;

		hit.hit = true;
		hit.x = result.vUVs.v[0] * static_cast<float>(width);
		hit.y = (1.0f - result.vUVs.v[1]) * static_cast<float>(height);
		hit.distance = result.fDistance;
		hit.source = source;
		return hit;
	};

	SafeOverlayPointerHit best = readHit(leftHandSource_);
	const SafeOverlayPointerHit right = readHit(rightHandSource_);
	if (right.hit && (!best.hit || right.distance < best.distance)) {
		best = right;
	}

	ImGuiIO& io = ImGui::GetIO();
	if (!best.hit) {
		io.AddMousePosEvent(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		if (safeClickDown_) {
			io.AddMouseButtonEvent(0, false);
			safeClickDown_ = false;
		}
		return;
	}

	io.AddMousePosEvent(best.x, best.y);

	vr::InputDigitalActionData_t click{};
	const vr::EVRInputError clickErr =
	    vr::VRInput()->GetDigitalActionData(safeClickAction_, &click, sizeof(click), best.source);
	const bool clickDown = clickErr == vr::VRInputError_None && click.bActive && click.bState;
	if (clickDown != safeClickDown_) {
		io.AddMouseButtonEvent(0, clickDown);
		safeClickDown_ = clickDown;
	}

	vr::InputAnalogActionData_t scroll{};
	const vr::EVRInputError scrollErr =
	    vr::VRInput()->GetAnalogActionData(safeScrollAction_, &scroll, sizeof(scroll), best.source);
	if (scrollErr == vr::VRInputError_None && scroll.bActive) {
		const float wheelX = scroll.deltaX * kSafeOverlayScrollScale;
		const float wheelY = scroll.deltaY * kSafeOverlayScrollScale;
		if (std::abs(wheelX) > 0.0001f || std::abs(wheelY) > 0.0001f) {
			io.AddMouseWheelEvent(wheelX, wheelY);
		}
	}
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
	if (!overlayCreated_ || !vr::VROverlay()) return;

	ImGuiIO& io = ImGui::GetIO();
	vr::VREvent_t ev{};
	while (vr::VROverlay()->PollNextOverlayEvent(mainHandle_, &ev, sizeof(ev))) {
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

	// Stage 2: register the dashboard overlay (one-shot).
	TryCreateOverlay();
	TickSafeOverlayInput(overlayPixelWidth, overlayPixelHeight);

	// Stage 3: drain OpenVR events into ImGui IO. Virtual-keyboard
	// support is intentionally deferred -- replacing an active
	// ImGui InputText buffer needs imgui_internal API that varies
	// across the pin range and is best landed after the dashboard
	// render path is verified end-to-end.
	DrainOverlayEvents();
	RefreshDashboardState();

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
