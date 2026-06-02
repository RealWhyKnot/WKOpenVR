#pragma once

#include "Protocol.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstdint>

namespace facetracking {

class FaceSignalProcessor
{
public:
    void Apply(protocol::FaceTrackingFrameBody &frame,
               const protocol::FaceTrackingConfig &config);
    void Reset();

private:
    struct ScalarFilter
    {
        bool initialized = false;
        float value = 0.0f;
    };

    struct Vec3Filter
    {
        bool initialized = false;
        float value[3] = { 0.0f, 0.0f, -1.0f };
    };

    ScalarFilter openness_l_;
    ScalarFilter openness_r_;
    Vec3Filter gaze_l_;
    Vec3Filter gaze_r_;

    uint64_t last_qpc_sample_time_ = 0;
    uint64_t idle_mouth_open_since_qpc_ = 0;
    uint64_t last_source_hash_ = 0;
    mutable LARGE_INTEGER qpc_freq_{ 0 };

    float FrameDeltaSeconds(const protocol::FaceTrackingFrameBody &frame);
    LARGE_INTEGER QpcFreq() const;
    static void SmoothScalar(float &value,
                             ScalarFilter &state,
                             uint8_t strength,
                             float dt_sec);
    static void SmoothVec3(float value[3],
                           Vec3Filter &state,
                           uint8_t strength,
                           float dt_sec);
};

} // namespace facetracking
