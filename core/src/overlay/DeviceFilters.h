#pragma once

#include <openvr.h>

#include <string>

namespace openvr_pair::overlay {

bool IsInternalAuxiliaryTrackedDevice(const std::string& serial, const std::string& model);

bool ShouldShowInCalibrationDeviceList(vr::TrackedDeviceClass deviceClass, const std::string& serial,
                                       const std::string& model, const std::string& trackingSystem = {});

bool ShouldShowInSmoothingPredictionList(vr::TrackedDeviceClass deviceClass, const std::string& serial,
                                         const std::string& model, const std::string& trackingSystem = {});

} // namespace openvr_pair::overlay
