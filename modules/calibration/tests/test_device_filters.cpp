#include "DeviceFilters.h"

#include <gtest/gtest.h>

TEST(DeviceFiltersTest, HidesFaceTrackingSink)
{
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInCalibrationDeviceList(
	    vr::TrackedDeviceClass_GenericTracker, "OpenVRPair-FaceTracking-Sink", "OpenVRPair FaceTracking"));
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInSmoothingPredictionList(
	    vr::TrackedDeviceClass_GenericTracker, "OpenVRPair-FaceTracking-Sink", "generic_tracker"));
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInSmoothingPredictionList(
	    vr::TrackedDeviceClass_GenericTracker, "WKOpenVR-FaceTracking-Sink", "WKOpenVR FaceTracking"));
}

TEST(DeviceFiltersTest, HidesNonUserPoseClasses)
{
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInSmoothingPredictionList(
	    vr::TrackedDeviceClass_TrackingReference, "LHB-12345678", "lh_basestation_valve_gen2"));
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInCalibrationDeviceList(vr::TrackedDeviceClass_DisplayRedirect,
	                                                                     "redirect", "redirect"));
}

TEST(DeviceFiltersTest, KeepsUserPoseDevices)
{
	EXPECT_TRUE(
	    openvr_pair::overlay::ShouldShowInSmoothingPredictionList(vr::TrackedDeviceClass_HMD, "hmd-serial", "hmd"));
	EXPECT_TRUE(openvr_pair::overlay::ShouldShowInSmoothingPredictionList(vr::TrackedDeviceClass_Controller,
	                                                                      "LHR-controller", "valve_controller"));
	EXPECT_TRUE(openvr_pair::overlay::ShouldShowInCalibrationDeviceList(vr::TrackedDeviceClass_GenericTracker,
	                                                                    "LHR-tracker", "vive_tracker"));
}

TEST(DeviceFiltersTest, HidesWKOpenVREmittedDevices)
{
	// Phantom-emitted virtual tracker: filtered from calibration picker.
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInCalibrationDeviceList(vr::TrackedDeviceClass_GenericTracker,
	                                                                     "WKOPENVR-left_foot-abcd1234abcd1234",
	                                                                     "WKOpenVR Virtual Tracker", "wkopenvr"));
	// Same device should also be filtered from smoothing prediction list.
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInSmoothingPredictionList(vr::TrackedDeviceClass_GenericTracker,
	                                                                       "WKOPENVR-waist-1234abcd1234abcd",
	                                                                       "WKOpenVR Virtual Tracker", "wkopenvr"));
	// Case-insensitive match: a future capitalization of the system name
	// should also be filtered.
	EXPECT_FALSE(openvr_pair::overlay::ShouldShowInCalibrationDeviceList(
	    vr::TrackedDeviceClass_GenericTracker, "WKOPENVR-chest-0000", "WKOpenVR Virtual Tracker", "WKOpenVR"));
}

TEST(DeviceFiltersTest, KeepsLighthouseWhenTrackingSystemSupplied)
{
	// A real lighthouse-tracked device with the trackingSystem argument
	// supplied: must still pass through.
	EXPECT_TRUE(openvr_pair::overlay::ShouldShowInCalibrationDeviceList(
	    vr::TrackedDeviceClass_GenericTracker, "LHR-ABCDEF01", "vive_tracker_3.0", "lighthouse"));
	EXPECT_TRUE(openvr_pair::overlay::ShouldShowInSmoothingPredictionList(
	    vr::TrackedDeviceClass_GenericTracker, "LHR-ABCDEF01", "vive_tracker_3.0", "lighthouse"));
}

TEST(DeviceFiltersTest, EmptyTrackingSystemLegacyCallersStillWork)
{
	// Existing callers that haven't been updated to supply a trackingSystem
	// must continue to see real devices.
	EXPECT_TRUE(openvr_pair::overlay::ShouldShowInCalibrationDeviceList(vr::TrackedDeviceClass_GenericTracker,
	                                                                    "LHR-tracker", "vive_tracker"));
}
