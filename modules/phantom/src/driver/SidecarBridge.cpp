#include "SidecarBridge.h"

#include "Protocol.h"

#include <cstddef>
#include <cstring>

namespace phantom {

bool SidecarBridge::TryOpen()
{
    if (ready_) return true;
    ready_ = shmem_.Open(OPENVR_PAIRDRIVER_PHANTOM_INFERENCE_OUT_SHMEM_NAME);
    return ready_;
}

void SidecarBridge::Close()
{
    shmem_.Close();
    ready_ = false;
}

float SidecarBridge::GlobalConfidence() const
{
    if (!ready_ || !shmem_.layout()) return 0.0f;
    return shmem_.layout()->global_confidence;
}

bool SidecarBridge::FetchPose(BodyRole role,
                              float min_global,
                              vr::DriverPose_t& out_pose) const
{
    if (!ready_ || !shmem_.layout()) return false;
    const auto* L = shmem_.layout();
    if (L->global_confidence < min_global) return false;
    const auto idx = static_cast<size_t>(role);
    if (idx >= kBodyRoleCount) return false;
    const auto& t = L->trackers[idx];
    if (!t.valid) return false;

    std::memset(&out_pose, 0, sizeof(out_pose));
    out_pose.qWorldFromDriverRotation = {1, 0, 0, 0};
    out_pose.qDriverFromHeadRotation  = {1, 0, 0, 0};
    out_pose.vecPosition[0] = t.position[0];
    out_pose.vecPosition[1] = t.position[1];
    out_pose.vecPosition[2] = t.position[2];
    out_pose.qRotation.w = t.rotation[0];
    out_pose.qRotation.x = t.rotation[1];
    out_pose.qRotation.y = t.rotation[2];
    out_pose.qRotation.z = t.rotation[3];
    out_pose.poseIsValid = true;
    out_pose.deviceIsConnected = true;
    out_pose.result = vr::TrackingResult_Running_OK;
    return true;
}

} // namespace phantom
