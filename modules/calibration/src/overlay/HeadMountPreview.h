#pragma once

// Live eye-position marker overlay.
//
// When the offset modal is open or nudge sliders are being adjusted, a small
// colored dot is rendered in the VR world at the computed eye position so the
// user can verify the offset visually.

#include <openvr.h>
#include <Eigen/Geometry>

namespace wkopenvr::headmount {

struct HeadMountPreviewStatus
{
	bool created = false;
	bool visible = false;
	bool textureReady = false;
	bool referenceVisible = false;
	bool referenceTextureReady = false;
	int lastError = 0;
	int referenceLastError = 0;
	const char* lastErrorName = "None";
	const char* referenceLastErrorName = "None";
	const char* lastSource = "none";
	double markerForwardMeters = 0.0;
	vr::ETrackingUniverseOrigin trackingOrigin = vr::TrackingUniverseStanding;
};

double HeadMountPreviewForwardMeters();

vr::ETrackingUniverseOrigin HeadMountPreviewTrackingOrigin();

vr::HmdMatrix34_t HeadMountPreviewTransform(const Eigen::Affine3d& headTrackerPose,
                                            const Eigen::AffineCompact3d& headFromTracker,
                                            double forwardMeters = HeadMountPreviewForwardMeters());

vr::HmdMatrix34_t HeadMountPreviewHmdReferenceTransform();

HeadMountPreviewStatus GetHeadMountPreviewStatus();

// Call each frame while the preview should be visible (modal open or sliders
// active). Pass wantVisible=false to hide the overlay.
//
// headTrackerPose: tracker pose in the supplied tracking origin
// headFromTracker: offset transform to compose onto tracker pose
void TickPreview(bool wantVisible, const Eigen::Affine3d& headTrackerPose,
                 const Eigen::AffineCompact3d& headFromTracker,
                 vr::ETrackingUniverseOrigin trackingOrigin = HeadMountPreviewTrackingOrigin(),
                 const char* source = nullptr);

} // namespace wkopenvr::headmount
