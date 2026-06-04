#include "HeadMountPreview.h"

#include "DiagnosticsLog.h"

#include <openvr.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace wkopenvr::headmount {

// ---------------------------------------------------------------------------
// Module-level state
// ---------------------------------------------------------------------------

namespace {

struct PreviewState
{
	vr::VROverlayHandle_t handle = vr::k_ulOverlayHandleInvalid;
	vr::VROverlayHandle_t referenceHandle = vr::k_ulOverlayHandleInvalid;
	bool created = false;
	bool referenceCreated = false;
	bool visible = false;
	bool referenceVisible = false;
	bool textureReady = false;
	bool referenceTextureReady = false;
	vr::EVROverlayError lastError = vr::VROverlayError_None;
	vr::EVROverlayError referenceLastError = vr::VROverlayError_None;
	const char* lastSource = "none";
	vr::ETrackingUniverseOrigin trackingOrigin = vr::TrackingUniverseStanding;
};

PreviewState& PS()
{
	static PreviewState s;
	return s;
}

constexpr const char* kPreviewKey = "wkopenvr.headmount.preview";
constexpr const char* kPreviewName = "Head-mount eye-position marker";
constexpr const char* kReferenceKey = "wkopenvr.headmount.preview.reference";
constexpr const char* kReferenceName = "Head-mount visible reference marker";
constexpr float kMarkerWidthM = 0.075f;
constexpr float kReferenceMarkerWidthM = 0.055f;
constexpr double kMarkerForwardMeters = 0.45;

#ifdef _WIN32
std::filesystem::path ExeDir()
{
	char buf[MAX_PATH] = {};
	const DWORD len = GetModuleFileNameA(nullptr, buf, MAX_PATH);
	if (len == 0 || len == MAX_PATH) return {};
	return std::filesystem::path(buf).parent_path();
}
#else
std::filesystem::path ExeDir()
{
	return {};
}
#endif

std::string ResolveMarkerTexturePath()
{
	const std::filesystem::path exeDir = ExeDir();
	if (!exeDir.empty()) {
		const std::filesystem::path deployed = exeDir / "dashboard_icon.png";
		if (std::filesystem::exists(deployed)) return deployed.string();
	}

	const std::filesystem::path source =
	    std::filesystem::current_path() / "core" / "src" / "overlay" / "dashboard_icon.png";
	if (std::filesystem::exists(source)) return source.string();
	return {};
}

const char* OverlayErrorName(vr::EVROverlayError err)
{
	switch (err) {
		case vr::VROverlayError_None:
			return "None";
		case vr::VROverlayError_UnknownOverlay:
			return "UnknownOverlay";
		case vr::VROverlayError_InvalidHandle:
			return "InvalidHandle";
		case vr::VROverlayError_PermissionDenied:
			return "PermissionDenied";
		case vr::VROverlayError_OverlayLimitExceeded:
			return "OverlayLimitExceeded";
		case vr::VROverlayError_WrongVisibilityType:
			return "WrongVisibilityType";
		case vr::VROverlayError_KeyTooLong:
			return "KeyTooLong";
		case vr::VROverlayError_NameTooLong:
			return "NameTooLong";
		case vr::VROverlayError_KeyInUse:
			return "KeyInUse";
		case vr::VROverlayError_WrongTransformType:
			return "WrongTransformType";
		case vr::VROverlayError_InvalidTrackedDevice:
			return "InvalidTrackedDevice";
		case vr::VROverlayError_InvalidParameter:
			return "InvalidParameter";
		case vr::VROverlayError_ThumbnailCantBeDestroyed:
			return "ThumbnailCantBeDestroyed";
		case vr::VROverlayError_ArrayTooSmall:
			return "ArrayTooSmall";
		case vr::VROverlayError_RequestFailed:
			return "RequestFailed";
		case vr::VROverlayError_InvalidTexture:
			return "InvalidTexture";
		case vr::VROverlayError_UnableToLoadFile:
			return "UnableToLoadFile";
		case vr::VROverlayError_KeyboardAlreadyInUse:
			return "KeyboardAlreadyInUse";
		case vr::VROverlayError_NoNeighbor:
			return "NoNeighbor";
		case vr::VROverlayError_TooManyMaskPrimitives:
			return "TooManyMaskPrimitives";
		case vr::VROverlayError_BadMaskPrimitive:
			return "BadMaskPrimitive";
		default:
			return "Unknown";
	}
}

static vr::HmdMatrix34_t ToHmdMatrix34(const Eigen::Matrix4d& m)
{
	vr::HmdMatrix34_t out;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 4; c++) {
			out.m[r][c] = static_cast<float>(m(r, c));
		}
	}
	return out;
}

bool EnsureCreated(const char* source)
{
	auto& s = PS();
	if (s.created) return s.textureReady;
	if (!vr::VROverlay()) return false;

	vr::EVROverlayError err = vr::VROverlay()->CreateOverlay(kPreviewKey, kPreviewName, &s.handle);
	if (err == vr::VROverlayError_KeyInUse) {
		err = vr::VROverlay()->FindOverlay(kPreviewKey, &s.handle);
	}
	if (err != vr::VROverlayError_None) {
		s.lastError = err;
		std::fprintf(stderr, "[HeadMountPreview] CreateOverlay failed: %d\n", (int)err);
		openvr_pair::common::DiagnosticLog("head_mount_preview_status",
		                                   "created=0 visible=0 texture_ready=0 error=%d error_name=%s source=%s",
		                                   static_cast<int>(err), OverlayErrorName(err), source ? source : "create");
		return false;
	}

	vr::VROverlay()->SetOverlayWidthInMeters(s.handle, kMarkerWidthM);
	vr::VROverlay()->SetOverlaySortOrder(s.handle, 18);
	vr::VROverlay()->SetOverlayAlpha(s.handle, 1.0f);
	vr::VROverlay()->SetOverlayColor(s.handle, 0.0f, 0.9f, 1.0f);

	const std::string texturePath = ResolveMarkerTexturePath();
	if (texturePath.empty()) {
		s.created = true;
		s.visible = false;
		s.textureReady = false;
		s.lastError = vr::VROverlayError_UnableToLoadFile;
		openvr_pair::common::DiagnosticLog(
		    "head_mount_preview_status",
		    "created=1 visible=0 texture_ready=0 error=%d error_name=%s source=%s texture_missing=1",
		    static_cast<int>(s.lastError), OverlayErrorName(s.lastError), source ? source : "create");
		return false;
	}

	err = vr::VROverlay()->SetOverlayFromFile(s.handle, texturePath.c_str());
	s.created = true;
	s.visible = false;
	s.textureReady = (err == vr::VROverlayError_None);
	s.lastError = err;
	openvr_pair::common::DiagnosticLog(
	    "head_mount_preview_status",
	    "created=1 visible=0 texture_ready=%d error=%d error_name=%s source=%s texture='%s'", s.textureReady ? 1 : 0,
	    static_cast<int>(err), OverlayErrorName(err), source ? source : "create", texturePath.c_str());
	return s.textureReady;
}

bool EnsureReferenceCreated(const char* source)
{
	auto& s = PS();
	if (s.referenceCreated) return s.referenceTextureReady;
	if (!vr::VROverlay()) return false;

	vr::EVROverlayError err = vr::VROverlay()->CreateOverlay(kReferenceKey, kReferenceName, &s.referenceHandle);
	if (err == vr::VROverlayError_KeyInUse) {
		err = vr::VROverlay()->FindOverlay(kReferenceKey, &s.referenceHandle);
	}
	if (err != vr::VROverlayError_None) {
		s.referenceLastError = err;
		openvr_pair::common::DiagnosticLog("head_mount_preview_status",
		                                   "reference_created=0 reference_visible=0 reference_texture_ready=0 "
		                                   "reference_error=%d reference_error_name=%s source=%s",
		                                   static_cast<int>(err), OverlayErrorName(err),
		                                   source ? source : "reference_create");
		return false;
	}

	vr::VROverlay()->SetOverlayWidthInMeters(s.referenceHandle, kReferenceMarkerWidthM);
	vr::VROverlay()->SetOverlaySortOrder(s.referenceHandle, 19);
	vr::VROverlay()->SetOverlayAlpha(s.referenceHandle, 0.90f);
	vr::VROverlay()->SetOverlayColor(s.referenceHandle, 1.0f, 1.0f, 1.0f);

	const std::string texturePath = ResolveMarkerTexturePath();
	if (texturePath.empty()) {
		s.referenceCreated = true;
		s.referenceVisible = false;
		s.referenceTextureReady = false;
		s.referenceLastError = vr::VROverlayError_UnableToLoadFile;
		openvr_pair::common::DiagnosticLog("head_mount_preview_status",
		                                   "reference_created=1 reference_visible=0 reference_texture_ready=0 "
		                                   "reference_error=%d reference_error_name=%s source=%s texture_missing=1",
		                                   static_cast<int>(s.referenceLastError),
		                                   OverlayErrorName(s.referenceLastError),
		                                   source ? source : "reference_create");
		return false;
	}

	err = vr::VROverlay()->SetOverlayFromFile(s.referenceHandle, texturePath.c_str());
	s.referenceCreated = true;
	s.referenceVisible = false;
	s.referenceTextureReady = (err == vr::VROverlayError_None);
	s.referenceLastError = err;
	openvr_pair::common::DiagnosticLog("head_mount_preview_status",
	                                   "reference_created=1 reference_visible=0 reference_texture_ready=%d "
	                                   "reference_error=%d reference_error_name=%s source=%s texture='%s'",
	                                   s.referenceTextureReady ? 1 : 0, static_cast<int>(err), OverlayErrorName(err),
	                                   source ? source : "reference_create", texturePath.c_str());
	return s.referenceTextureReady;
}

void HideOnly(const char* source)
{
	auto& s = PS();
	if (!s.created && !s.referenceCreated) return;
	if (vr::VROverlay() && s.handle != vr::k_ulOverlayHandleInvalid) {
		vr::VROverlay()->HideOverlay(s.handle);
	}
	if (vr::VROverlay() && s.referenceHandle != vr::k_ulOverlayHandleInvalid) {
		vr::VROverlay()->HideOverlay(s.referenceHandle);
	}
	if (s.visible) {
		openvr_pair::common::DiagnosticLog("head_mount_preview_status",
		                                   "created=1 visible=0 texture_ready=%d error=%d error_name=%s source=%s",
		                                   s.textureReady ? 1 : 0, static_cast<int>(s.lastError),
		                                   OverlayErrorName(s.lastError), source ? source : s.lastSource);
	}
	if (s.referenceVisible) {
		openvr_pair::common::DiagnosticLog("head_mount_preview_status",
		                                   "reference_created=1 reference_visible=0 reference_texture_ready=%d "
		                                   "reference_error=%d reference_error_name=%s source=%s hmd_relative=1",
		                                   s.referenceTextureReady ? 1 : 0, static_cast<int>(s.referenceLastError),
		                                   OverlayErrorName(s.referenceLastError), source ? source : s.lastSource);
	}
	s.visible = false;
	s.referenceVisible = false;
}

} // namespace

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

double HeadMountPreviewForwardMeters()
{
	return kMarkerForwardMeters;
}

vr::ETrackingUniverseOrigin HeadMountPreviewTrackingOrigin()
{
	return vr::TrackingUniverseStanding;
}

vr::HmdMatrix34_t HeadMountPreviewTransform(const Eigen::Affine3d& headTrackerPose,
                                            const Eigen::AffineCompact3d& headFromTracker, double forwardMeters)
{
	const double clampedForward = std::clamp(forwardMeters, 0.05, 2.0);
	const Eigen::Affine3d headWorld = headTrackerPose * Eigen::Affine3d(headFromTracker);
	const Eigen::Affine3d markerWorld = headWorld * Eigen::Translation3d(0.0, 0.0, -clampedForward);
	return ToHmdMatrix34(markerWorld.matrix());
}

vr::HmdMatrix34_t HeadMountPreviewHmdReferenceTransform()
{
	vr::HmdMatrix34_t mat{};
	mat.m[0][0] = 1.0f;
	mat.m[1][1] = 1.0f;
	mat.m[2][2] = 1.0f;
	mat.m[0][3] = 0.12f;
	mat.m[1][3] = 0.08f;
	mat.m[2][3] = -0.45f;
	return mat;
}

HeadMountPreviewStatus GetHeadMountPreviewStatus()
{
	const auto& s = PS();
	HeadMountPreviewStatus status;
	status.created = s.created;
	status.visible = s.visible;
	status.textureReady = s.textureReady;
	status.referenceVisible = s.referenceVisible;
	status.referenceTextureReady = s.referenceTextureReady;
	status.lastError = static_cast<int>(s.lastError);
	status.referenceLastError = static_cast<int>(s.referenceLastError);
	status.lastErrorName = OverlayErrorName(s.lastError);
	status.referenceLastErrorName = OverlayErrorName(s.referenceLastError);
	status.lastSource = s.lastSource ? s.lastSource : "none";
	status.markerForwardMeters = kMarkerForwardMeters;
	status.trackingOrigin = s.trackingOrigin;
	return status;
}

void TickPreview(bool wantVisible, const Eigen::Affine3d& headTrackerPose,
                 const Eigen::AffineCompact3d& headFromTracker, vr::ETrackingUniverseOrigin trackingOrigin,
                 const char* source)
{
	auto& s = PS();
	s.lastSource = source ? source : "unknown";
	s.trackingOrigin = trackingOrigin;
	if (!wantVisible) {
		HideOnly(source ? source : "hide");
		return;
	}

	if (!EnsureCreated(source)) return;
	const bool referenceReady = EnsureReferenceCreated(source);

	vr::HmdMatrix34_t mat = HeadMountPreviewTransform(headTrackerPose, headFromTracker, kMarkerForwardMeters);

	vr::EVROverlayError err = vr::VROverlay()->SetOverlayTransformAbsolute(s.handle, trackingOrigin, &mat);
	if (err != vr::VROverlayError_None) {
		s.lastError = err;
		openvr_pair::common::DiagnosticLog(
		    "head_mount_preview_status",
		    "created=1 visible=%d texture_ready=%d transform_failed=1 error=%d error_name=%s origin=%d source=%s",
		    s.visible ? 1 : 0, s.textureReady ? 1 : 0, static_cast<int>(err), OverlayErrorName(err),
		    static_cast<int>(trackingOrigin), s.lastSource);
		return;
	}

	err = vr::VROverlay()->ShowOverlay(s.handle);
	if (err != vr::VROverlayError_None) {
		s.lastError = err;
		openvr_pair::common::DiagnosticLog(
		    "head_mount_preview_status",
		    "created=1 visible=%d texture_ready=%d show_failed=1 error=%d error_name=%s origin=%d source=%s",
		    s.visible ? 1 : 0, s.textureReady ? 1 : 0, static_cast<int>(err), OverlayErrorName(err),
		    static_cast<int>(trackingOrigin), s.lastSource);
		return;
	}

	if (!s.visible || s.lastError != vr::VROverlayError_None) {
		openvr_pair::common::DiagnosticLog("head_mount_preview_status",
		                                   "created=1 visible=1 texture_ready=%d error=0 error_name=None origin=%d "
		                                   "source=%s forward_m=%.3f pos=(%.3f,%.3f,%.3f)",
		                                   s.textureReady ? 1 : 0, static_cast<int>(trackingOrigin), s.lastSource,
		                                   kMarkerForwardMeters, mat.m[0][3], mat.m[1][3], mat.m[2][3]);
	}
	s.lastError = vr::VROverlayError_None;
	s.visible = true;

	if (referenceReady) {
		vr::HmdMatrix34_t refMat = HeadMountPreviewHmdReferenceTransform();
		err = vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(s.referenceHandle,
		                                                                vr::k_unTrackedDeviceIndex_Hmd, &refMat);
		if (err == vr::VROverlayError_None) {
			err = vr::VROverlay()->ShowOverlay(s.referenceHandle);
		}
		if (err != vr::VROverlayError_None) {
			s.referenceLastError = err;
			openvr_pair::common::DiagnosticLog("head_mount_preview_status",
			                                   "reference_created=1 reference_visible=%d reference_texture_ready=%d "
			                                   "reference_error=%d reference_error_name=%s source=%s hmd_relative=1",
			                                   s.referenceVisible ? 1 : 0, s.referenceTextureReady ? 1 : 0,
			                                   static_cast<int>(err), OverlayErrorName(err), s.lastSource);
			return;
		}
		if (!s.referenceVisible || s.referenceLastError != vr::VROverlayError_None) {
			openvr_pair::common::DiagnosticLog(
			    "head_mount_preview_status",
			    "reference_created=1 reference_visible=1 reference_texture_ready=%d reference_error=0 "
			    "reference_error_name=None source=%s hmd_relative=1 pos=(%.3f,%.3f,%.3f)",
			    s.referenceTextureReady ? 1 : 0, s.lastSource, refMat.m[0][3], refMat.m[1][3], refMat.m[2][3]);
		}
		s.referenceLastError = vr::VROverlayError_None;
		s.referenceVisible = true;
	}
}

} // namespace wkopenvr::headmount
