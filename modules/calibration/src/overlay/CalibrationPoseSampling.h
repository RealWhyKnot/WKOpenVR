#pragma once

#include "Calibration.h"

#include <Eigen/Dense>

vr::HmdQuaternion_t VRRotationQuat(const Eigen::Quaterniond& rotQuat);
vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg);
vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm);

Pose ConvertPose(const vr::DriverPose_t& driverPose);
bool CollectSample(const CalibrationContext& ctx);
bool AssignTargets();
