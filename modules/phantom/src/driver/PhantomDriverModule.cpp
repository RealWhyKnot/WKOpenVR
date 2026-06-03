#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "DriverModule.h"
#include "FeatureFlags.h"
#include "Logging.h"
#include "PhantomStateShmem.h"

#include "BlendController.h"
#include "BlendCurves.h"
#include "BodyCompletionSolver.h"
#include "DeadReckoner.h"
#include "DropoutState.h"
#include "IkFallback.h"
#include "PoseHistory.h"
#include "RoleCatalog.h"
#include "VirtualTrackerManager.h"

#include <openvr_driver.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace phantom {

namespace {

// Hash a serial string to a 64-bit FNV-1a id. Matches the convention used
// elsewhere in the umbrella (inputhealth) so the overlay's per-device
// state map keys line up with what other modules produce.
uint64_t Fnv1a64(const char* s)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (; s && *s; ++s) {
        h ^= static_cast<unsigned char>(*s);
        h *= 0x100000001b3ull;
    }
    return h;
}

bool PoseIsTrackedOk(const vr::DriverPose_t& pose)
{
    return pose.poseIsValid
        && pose.deviceIsConnected
        && pose.result == vr::TrackingResult_Running_OK;
}

uint32_t AgeMsFromQpc(int64_t now_qpc, int64_t sample_qpc, int64_t qpc_freq)
{
    if (qpc_freq <= 0 || sample_qpc <= 0 || now_qpc <= sample_qpc) return 0;
    const int64_t age = ((now_qpc - sample_qpc) * 1000) / qpc_freq;
    return static_cast<uint32_t>(std::max<int64_t>(0, age));
}

BodyCompletionPose ToBodyCompletionPose(const vr::DriverPose_t& pose)
{
    BodyCompletionPose out;
    out.position[0] = pose.vecPosition[0];
    out.position[1] = pose.vecPosition[1];
    out.position[2] = pose.vecPosition[2];
    out.rotation[0] = pose.qRotation.w;
    out.rotation[1] = pose.qRotation.x;
    out.rotation[2] = pose.qRotation.y;
    out.rotation[3] = pose.qRotation.z;
    out.velocity[0] = pose.vecVelocity[0];
    out.velocity[1] = pose.vecVelocity[1];
    out.velocity[2] = pose.vecVelocity[2];
    return out;
}

BodyCompletionSensorPose ToBodyCompletionSensorPose(const vr::DriverPose_t& pose,
                                                    uint32_t age_ms)
{
    BodyCompletionSensorPose out;
    out.pose = ToBodyCompletionPose(pose);
    out.valid = PoseIsTrackedOk(pose);
    out.age_ms = age_ms;
    return out;
}

vr::DriverPose_t PoseFromBodyCompletion(const vr::DriverPose_t& base,
                                        const BodyCompletionRoleOutput& role)
{
    vr::DriverPose_t out = base;
    out.poseTimeOffset = 0.0;
    out.vecPosition[0] = role.pose.position[0];
    out.vecPosition[1] = role.pose.position[1];
    out.vecPosition[2] = role.pose.position[2];
    out.vecVelocity[0] = role.pose.velocity[0];
    out.vecVelocity[1] = role.pose.velocity[1];
    out.vecVelocity[2] = role.pose.velocity[2];
    out.vecAcceleration[0] = 0.0;
    out.vecAcceleration[1] = 0.0;
    out.vecAcceleration[2] = 0.0;
    out.qRotation = {
        role.pose.rotation[0],
        role.pose.rotation[1],
        role.pose.rotation[2],
        role.pose.rotation[3],
    };
    out.vecAngularVelocity[0] = 0.0;
    out.vecAngularVelocity[1] = 0.0;
    out.vecAngularVelocity[2] = 0.0;
    out.vecAngularAcceleration[0] = 0.0;
    out.vecAngularAcceleration[1] = 0.0;
    out.vecAngularAcceleration[2] = 0.0;
    out.poseIsValid = true;
    out.deviceIsConnected = true;
    out.shouldApplyHeadModel = false;
    out.result = vr::TrackingResult_Running_OK;
    return out;
}

// Per-device state owned by PhantomDriverModule. Keyed by OpenVR device id
// (small fixed range, allows a vector for O(1) access). Holds the ring,
// the state machine, and the cached opt-in flag.
struct DeviceSlot
{
    PoseHistory  history;
    DropoutState ladder;
    std::string  serial;
    uint64_t     serial_hash = 0;
    bool         opted_in    = false;

    // Body role assigned by the calibration UI. The completion solver treats
    // fresh role-assigned trackers as measured anchors.
    BodyRole     role = BodyRole::None;

    // Tracks whether this device is the HMD; the IK solver needs the
    // HMD's live pose as its reference frame. Set the first time
    // ResolveSerialIfMissing identifies a Class_HMD device.
    bool         is_hmd = false;

    vr::ETrackedDeviceClass    device_class = vr::TrackedDeviceClass_Invalid;
    vr::ETrackedControllerRole controller_role = vr::TrackedControllerRole_Invalid;

    vr::DriverPose_t last_observed{};
    int64_t          last_observed_qpc = 0;
    bool             last_observed_valid = false;

    // Cached "latest published pose" so BLEND_IN has a starting point that
    // matches what SteamVR is currently seeing. Updated whenever phantom
    // emits a pose (synth or passthrough).
    vr::DriverPose_t last_published{};
    bool             last_published_valid = false;
};

class PhantomModule final : public DriverModule
{
public:
    const char* Name() const override          { return "Phantom"; }
    uint32_t    FeatureMask() const override   { return pairdriver::kFeaturePhantom; }
    const char* PipeName() const override      { return OPENVR_PAIRDRIVER_PHANTOM_PIPE_NAME; }

    bool Init(DriverModuleContext& context) override;
    void Shutdown() override;

    bool HandleRequest(const protocol::Request& request,
                       protocol::Response&      response) override;

    // Hot path -- called from ServerTrackedDeviceProvider via the
    // phantom:: namespace free functions in PhantomHotPath.h.
    void OnRealPoseObserved(uint32_t openVRID,
                            int64_t qpc_ns,
                            const vr::DriverPose_t& pose);
    bool MaybeOverridePose(uint32_t openVRID,
                           int64_t qpc_ns,
                           int64_t qpc_freq,
                           vr::DriverPose_t& pose);

private:
    DeviceSlot& slot(uint32_t openVRID);
    void ResolveSerialIfMissing(uint32_t openVRID, DeviceSlot& s);
    BodyCompletionInput BuildBodyCompletionInput(int64_t now_qpc) const;
    void UpdateBodyCompletion(int64_t now_qpc);
    bool TryBodyCompletionPose(BodyRole role, vr::DriverPose_t& synth) const;
    void PublishStateSnapshot();

    LadderTimings timings_ = LadderTimings::Defaults();
    std::atomic<bool> master_enabled_{false};

    // Per-serial-hash opt-in cache. The overlay sends a stream of
    // PhantomDeviceOptIn messages and the hot path checks against this map.
    std::unordered_map<uint64_t, bool> opt_in_by_serial_hash_;

    // Per-serial-hash body-role cache. Populated by RequestSetPhantomDeviceRole;
    // applied to the per-device slot when the device first arrives in
    // OnRealPoseObserved (or immediately if the slot already resolved).
    std::unordered_map<uint64_t, BodyRole> role_by_serial_hash_;

    // Live HMD pose cache (post-calibration / smoothing). The IK fallback
    // applies role-relative offsets to this pose. Updated on every
    // OnRealPoseObserved for the device flagged is_hmd.
    vr::DriverPose_t last_hmd_pose_{};
    bool             last_hmd_valid_ = false;

    IkFallback ik_fallback_;
    BodyCompletionSolver body_solver_;
    BodyCompletionCalibration body_calibration_;
    BodyCompletionResult last_body_completion_{};
    int64_t last_body_completion_qpc_ = 0;
    int64_t qpc_freq_ = 0;
    VirtualTrackerManager virtual_trackers_;
    bool virtual_master_disabled_logged_ = false;

    // Per-openVRID device state. The hot path runs without locking these
    // because openVRID assignments are stable for the lifetime of the
    // device and the hook is single-threaded per device; the IPC handler
    // takes state_mutex_ briefly when applying opt-in changes.
    std::array<DeviceSlot, vr::k_unMaxTrackedDeviceCount> slots_{};

    std::mutex state_mutex_;

    DeadReckoner    reckoner_;
    PhantomStateShmem shmem_;
    std::atomic<int64_t> last_snapshot_qpc_{0};

    DriverModuleContext context_{};
};

PhantomModule* g_active = nullptr;

void PhantomModule::ResolveSerialIfMissing(uint32_t openVRID, DeviceSlot& s)
{
    if (!s.serial.empty() || !vr::VRProperties()) return;
    const auto handle = vr::VRProperties()->TrackedDeviceToPropertyContainer(openVRID);
    if (handle == vr::k_ulInvalidPropertyContainer) return;
    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    const std::string serial = vr::VRProperties()->GetStringProperty(
        handle, vr::Prop_SerialNumber_String, &err);
    if (err != vr::TrackedProp_Success || serial.empty()) return;
    s.serial = serial;
    s.serial_hash = Fnv1a64(serial.c_str());

    // Tag HMD via the property system rather than assuming openVRID == 0;
    // SteamVR's convention is stable but some custom multi-HMD setups
    // re-order. The IK fallback's reference frame depends on this being
    // right, so query directly.
    vr::ETrackedPropertyError classErr = vr::TrackedProp_Success;
    const int32_t deviceClass = vr::VRProperties()->GetInt32Property(
        handle, vr::Prop_DeviceClass_Int32, &classErr);
    if (classErr == vr::TrackedProp_Success) {
        s.device_class = static_cast<vr::ETrackedDeviceClass>(deviceClass);
        if (deviceClass == vr::TrackedDeviceClass_HMD) {
            s.is_hmd = true;
        }
    }

    if (s.device_class == vr::TrackedDeviceClass_Controller) {
        vr::ETrackedPropertyError roleErr = vr::TrackedProp_Success;
        const int32_t controllerRole = vr::VRProperties()->GetInt32Property(
            handle, vr::Prop_ControllerRoleHint_Int32, &roleErr);
        if (roleErr == vr::TrackedProp_Success) {
            s.controller_role = static_cast<vr::ETrackedControllerRole>(controllerRole);
        }
    }

    std::lock_guard<std::mutex> lk(state_mutex_);
    if (auto it = opt_in_by_serial_hash_.find(s.serial_hash);
        it != opt_in_by_serial_hash_.end()) {
        s.opted_in = it->second;
    }
    if (auto it = role_by_serial_hash_.find(s.serial_hash);
        it != role_by_serial_hash_.end()) {
        s.role = it->second;
    }
}

DeviceSlot& PhantomModule::slot(uint32_t openVRID)
{
    if (openVRID >= slots_.size()) {
        static DeviceSlot sink;
        return sink;
    }
    return slots_[openVRID];
}

BodyCompletionInput PhantomModule::BuildBodyCompletionInput(int64_t now_qpc) const
{
    BodyCompletionInput input;
    input.calibration = body_calibration_;
    if (qpc_freq_ > 0 && last_body_completion_qpc_ > 0 && now_qpc > last_body_completion_qpc_) {
        input.dt_seconds = std::clamp(
            static_cast<double>(now_qpc - last_body_completion_qpc_) /
                static_cast<double>(qpc_freq_),
            1.0 / 240.0,
            0.10);
    }

    if (last_hmd_valid_) {
        input.hmd = ToBodyCompletionSensorPose(last_hmd_pose_, 0);
        input.enabled_roles[static_cast<uint8_t>(BodyRole::Hmd)] = true;
    }

    for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
        const auto role = static_cast<BodyRole>(i);
        if (virtual_trackers_.IsEnabled(role)) {
            input.enabled_roles[i] = true;
        }
    }

    for (const auto& s : slots_) {
        if (s.role != BodyRole::None) {
            input.enabled_roles[static_cast<uint8_t>(s.role)] = true;
        }
        if (!s.last_observed_valid) continue;

        const uint32_t age_ms = AgeMsFromQpc(now_qpc, s.last_observed_qpc, qpc_freq_);
        const auto sensor = ToBodyCompletionSensorPose(s.last_observed, age_ms);

        if (s.device_class == vr::TrackedDeviceClass_Controller && age_ms <= 250u) {
            if (s.controller_role == vr::TrackedControllerRole_LeftHand) {
                input.left_controller = sensor;
            } else if (s.controller_role == vr::TrackedControllerRole_RightHand) {
                input.right_controller = sensor;
            }
        }

        if (s.role == BodyRole::None) continue;
        if (static_cast<uint8_t>(s.role) >= input.real_roles.size()) continue;
        if (age_ms > std::max<uint32_t>(timings_.dropout_silence_ms, 200u)) continue;
        if (s.ladder.state() != TrackerState::REAL
            && s.ladder.state() != TrackerState::BLEND_IN) {
            continue;
        }
        input.real_roles[static_cast<uint8_t>(s.role)] = sensor;
    }

    return input;
}

void PhantomModule::UpdateBodyCompletion(int64_t now_qpc)
{
    last_body_completion_ = body_solver_.Solve(BuildBodyCompletionInput(now_qpc));
    last_body_completion_qpc_ = now_qpc;
}

bool PhantomModule::TryBodyCompletionPose(BodyRole role, vr::DriverPose_t& synth) const
{
    const auto idx = static_cast<uint8_t>(role);
    if (!last_hmd_valid_ || idx >= last_body_completion_.roles.size()) return false;
    const auto& solved = last_body_completion_.roles[idx];
    if (!solved.valid || solved.confidence < body_calibration_.virtual_min_confidence) {
        return false;
    }
    synth = PoseFromBodyCompletion(last_hmd_pose_, solved);
    return true;
}

bool PhantomModule::Init(DriverModuleContext& context)
{
    context_ = context;
    g_active = this;
    virtual_trackers_.OnDriverInit();
    LARGE_INTEGER freqLI{};
    if (QueryPerformanceFrequency(&freqLI)) {
        qpc_freq_ = freqLI.QuadPart;
    }
    if (!shmem_.Create(OPENVR_PAIRDRIVER_PHANTOM_STATE_SHMEM_NAME)) {
        // Non-fatal: badge readout is a nice-to-have. Driver still synthesises.
        LOG("[phantom] PhantomStateShmem.Create('%s') failed; overlay badges disabled",
            OPENVR_PAIRDRIVER_PHANTOM_STATE_SHMEM_NAME);
    }
    LOG("[phantom] PhantomModule initialised; ladder defaults silence=%u blend_out=%u blend_in=%u reckon=%u synth=%u lost=%u (ms)",
        (unsigned)timings_.dropout_silence_ms,
        (unsigned)timings_.blend_out_ms,
        (unsigned)timings_.blend_in_ms,
        (unsigned)timings_.reckon_hold_ms,
        (unsigned)timings_.synth_hold_ms,
        (unsigned)timings_.lost_hold_ms);
    return true;
}

void PhantomModule::Shutdown()
{
    g_active = nullptr;
    shmem_.Close();
    LOG("[phantom] PhantomModule shutdown");
}

bool PhantomModule::HandleRequest(const protocol::Request& request,
                                   protocol::Response&     response)
{
    switch (request.type) {
        case protocol::RequestSetPhantomConfig: {
            const auto& c = request.setPhantomConfig;
            timings_ = LadderTimings{
                /*dropout_silence_ms=*/ DefaultTimings::kDropoutSilenceMs,
                /*blend_out_ms=*/       c.blend_out_ms ? c.blend_out_ms : DefaultTimings::kBlendOutMs,
                /*blend_in_ms=*/        c.blend_in_ms  ? c.blend_in_ms  : DefaultTimings::kBlendInMs,
                /*reckon_hold_ms=*/     c.reckon_hold_ms ? c.reckon_hold_ms : DefaultTimings::kReckonHoldMs,
                /*synth_hold_ms=*/      c.synth_hold_ms ? c.synth_hold_ms : DefaultTimings::kSynthHoldMs,
                /*lost_hold_ms=*/       c.lost_hold_ms  ? c.lost_hold_ms  : DefaultTimings::kLostHoldMs,
            };
            const bool master_enabled = c.master_enabled != 0;
            master_enabled_.store(master_enabled, std::memory_order_release);
            virtual_trackers_.SetMasterEnabled(master_enabled);
            virtual_master_disabled_logged_ = false;
            // Apply new timings to every active slot. Cheap: just a copy.
            std::lock_guard<std::mutex> lk(state_mutex_);
            for (auto& s : slots_) s.ladder.SetTimings(timings_);
            response.type = protocol::ResponseSuccess;
            LOG("[phantom] config applied: master_enabled=%d blend_out=%u blend_in=%u reckon=%u synth=%u lost=%u",
                (int)c.master_enabled,
                (unsigned)timings_.blend_out_ms, (unsigned)timings_.blend_in_ms,
                (unsigned)timings_.reckon_hold_ms, (unsigned)timings_.synth_hold_ms,
                (unsigned)timings_.lost_hold_ms);
            return true;
        }
        case protocol::RequestSetPhantomSolverConfig: {
            const auto& c = request.setPhantomSolverConfig;
            body_calibration_.floor_y_m = c.floor_y_m;
            body_calibration_.height_m = c.height_m;
            body_calibration_.forward_yaw_rad = c.forward_yaw_rad;
            body_calibration_.stance_width_m = c.stance_width_m;
            body_calibration_.shoulder_width_m = c.shoulder_width_m;
            body_calibration_.pelvis_width_m = c.pelvis_width_m;
            body_calibration_.upper_arm_m = c.upper_arm_m;
            body_calibration_.lower_arm_m = c.lower_arm_m;
            body_calibration_.upper_leg_m = c.upper_leg_m;
            body_calibration_.lower_leg_m = c.lower_leg_m;
            body_calibration_.virtual_min_confidence = c.virtual_min_confidence;
            body_calibration_.forward_calibrated = (c.calibrated != 0);
            body_solver_.Reset();
            response.type = protocol::ResponseSuccess;
            LOG("[phantom] solver config applied: calibrated=%u height=%.2f floor=%.2f virtual_min=%.2f",
                (unsigned)c.calibrated,
                body_calibration_.height_m,
                body_calibration_.floor_y_m,
                body_calibration_.virtual_min_confidence);
            return true;
        }
        case protocol::RequestSetPhantomDeviceOptIn: {
            const auto& e = request.setPhantomDeviceOptIn;
            std::lock_guard<std::mutex> lk(state_mutex_);
            opt_in_by_serial_hash_[e.device_serial_hash] = (e.dropout_enabled != 0);
            // Push the change into any currently-active slot whose serial hash
            // matches. Devices not yet seen will pick it up on their first
            // OnRealPoseObserved.
            for (auto& s : slots_) {
                if (s.serial_hash == e.device_serial_hash) {
                    s.opted_in = (e.dropout_enabled != 0);
                }
            }
            response.type = protocol::ResponseSuccess;
            return true;
        }
        case protocol::RequestSetPhantomDeviceRole: {
            const auto& e = request.setPhantomDeviceRole;
            const BodyRole role = static_cast<BodyRole>(e.body_role);
            std::lock_guard<std::mutex> lk(state_mutex_);
            if (role == BodyRole::None) {
                role_by_serial_hash_.erase(e.device_serial_hash);
            } else {
                role_by_serial_hash_[e.device_serial_hash] = role;
            }
            for (auto& s : slots_) {
                if (s.serial_hash == e.device_serial_hash) {
                    s.role = role;
                    s.ladder.SetIkAvailable(s.role != BodyRole::None);
                }
            }
            response.type = protocol::ResponseSuccess;
            return true;
        }
        case protocol::RequestSetPhantomTrackerOffset: {
            const auto& e = request.setPhantomTrackerOffset;
            const BodyRole role = static_cast<BodyRole>(e.body_role);
            std::lock_guard<std::mutex> lk(state_mutex_);
            if (e.calibrated == 0) {
                ik_fallback_.ClearOffset(role);
            } else {
                ik_fallback_.SetOffset(role, e.rel_position, e.rel_rotation);
            }
            // Update every slot's ik_available flag in case this newly
            // calibrated role applies to a device that already has its
            // role assigned.
            for (auto& s : slots_) {
                s.ladder.SetIkAvailable(s.role != BodyRole::None);
            }
            response.type = protocol::ResponseSuccess;
            return true;
        }
        case protocol::RequestSetPhantomVirtualEnabled: {
            const auto& e = request.setPhantomVirtualEnabled;
            const BodyRole role = static_cast<BodyRole>(e.body_role);
            virtual_trackers_.SetEnabled(role, e.enabled != 0);
            response.type = protocol::ResponseSuccess;
            return true;
        }
        default:
            return false;
    }
}

void PhantomModule::OnRealPoseObserved(uint32_t openVRID,
                                       int64_t qpc_ns,
                                       const vr::DriverPose_t& pose)
{
    if (openVRID >= slots_.size()) return;
    DeviceSlot& s = slots_[openVRID];
    ResolveSerialIfMissing(openVRID, s);
    s.last_observed = pose;
    s.last_observed_qpc = qpc_ns;
    s.last_observed_valid = PoseIsTrackedOk(pose);
    s.history.Push(qpc_ns, pose);
    s.ladder.SetTimings(timings_);
    s.ladder.SetIkAvailable(s.role != BodyRole::None);
    s.ladder.OnRealPoseObserved(qpc_ns, s.history, pose);
    if (s.is_hmd && pose.poseIsValid && pose.deviceIsConnected
        && pose.result == vr::TrackingResult_Running_OK) {
        last_hmd_pose_ = pose;
        last_hmd_valid_ = true;
        UpdateBodyCompletion(qpc_ns);
        // Drive virtual trackers off the HMD pose cadence. Each HMD update
        // produces one completion frame; the manager only publishes roles
        // whose confidence passes the policy threshold.
        const bool master_enabled = master_enabled_.load(std::memory_order_acquire);
        if (PhantomVirtualTrackersShouldRun(master_enabled)) {
            virtual_trackers_.Tick(
                last_hmd_pose_,
                last_body_completion_,
                body_calibration_.virtual_min_confidence);
            virtual_master_disabled_logged_ = false;
        } else if (!virtual_master_disabled_logged_
                   && virtual_trackers_.EnabledCount() > 0) {
            virtual_master_disabled_logged_ = true;
            LOG("[phantom] virtual trackers gated off: master_enabled=0 enabled_roles=%d",
                virtual_trackers_.EnabledCount());
        }
    }
}

bool PhantomModule::MaybeOverridePose(uint32_t openVRID,
                                      int64_t qpc_ns,
                                      int64_t qpc_freq,
                                      vr::DriverPose_t& pose)
{
    if (openVRID >= slots_.size()) return true;
    DeviceSlot& s = slots_[openVRID];

    s.ladder.Tick(qpc_ns, qpc_freq);

    const bool enabled = master_enabled_.load(std::memory_order_acquire)
        && s.opted_in;

    // Always keep last_published current so BLEND_IN can match-and-fade
    // from whatever SteamVR most recently saw, even on the very next pose
    // after a recovery.
    auto cachePublished = [&](const vr::DriverPose_t& published) {
        s.last_published = published;
        s.last_published_valid = true;
    };

    // Periodic state snapshot for the overlay badge (rate-limited).
    const int64_t snap_window = (qpc_freq > 0) ? (qpc_freq / 10) : 0; // ~10 Hz
    if (snap_window > 0 &&
        qpc_ns - last_snapshot_qpc_.load(std::memory_order_relaxed) >= snap_window) {
        last_snapshot_qpc_.store(qpc_ns, std::memory_order_relaxed);
        PublishStateSnapshot();
    }

    if (!enabled) {
        cachePublished(pose);
        return true;
    }

    switch (s.ladder.state()) {
        case TrackerState::REAL:
            cachePublished(pose);
            return true;

        case TrackerState::BLEND_OUT: {
            vr::DriverPose_t synth{};
            if (reckoner_.Project(s.history, qpc_freq, qpc_ns, synth)) {
                vr::DriverPose_t blended{};
                BlendController::Lerp(
                    pose, synth, s.ladder.blend_alpha(qpc_ns, qpc_freq), blended);
                blended.result = s.ladder.tracking_result_override();
                pose = blended;
            }
            cachePublished(pose);
            return true;
        }

        case TrackerState::SYNTH_RECKON:
        case TrackerState::SYNTH_IK:
        case TrackerState::SYNTH_ML:
        case TrackerState::OUT_OF_RANGE: {
            // Cascade: in-process completion -> rigid-offset IK -> dead
            // reckoner. Each layer is optional; the cascade falls through
            // to the next when its input is missing or below confidence.
            // The ladder state is mostly a diagnostics hint -- the dispatch
            // picks the best source available.
            vr::DriverPose_t synth{};
            bool produced = false;
            if (!produced
                && s.role != BodyRole::None
                && TryBodyCompletionPose(s.role, synth)) {
                produced = true;
            }
            if (!produced
                && last_hmd_valid_
                && s.role != BodyRole::None
                && ik_fallback_.Solve(s.role, last_hmd_pose_, synth)) {
                produced = true;
            }
            if (!produced) {
                produced = reckoner_.Project(s.history, qpc_freq, qpc_ns, synth);
            }
            if (produced) {
                synth.result = s.ladder.tracking_result_override();
                pose = synth;
            } else {
                // No source produced a pose -- keep whatever the upstream
                // driver gave us but stamp the override result so consumers
                // see the degradation signal.
                pose.result = s.ladder.tracking_result_override();
            }
            cachePublished(pose);
            return true;
        }

        case TrackerState::BLEND_IN: {
            // Lerp from the synthesised "anchor" (the last thing we
            // published) to the freshly-arrived real pose.
            if (s.last_published_valid) {
                vr::DriverPose_t blended{};
                BlendController::Lerp(
                    s.last_published, pose,
                    s.ladder.blend_alpha(qpc_ns, qpc_freq),
                    blended);
                pose = blended;
            }
            cachePublished(pose);
            return true;
        }

        case TrackerState::LOST:
        default:
            // Caller skips the downstream pose update entirely. SteamVR
            // treats absence as disconnect after its own short timeout.
            return false;
    }
}

void PhantomModule::PublishStateSnapshot()
{
    auto* layout = shmem_.layout();
    if (!layout) return;

    const int64_t now_qpc = last_snapshot_qpc_.load(std::memory_order_relaxed);
    LARGE_INTEGER freqLI{};
    QueryPerformanceFrequency(&freqLI);
    const int64_t freq = freqLI.QuadPart;

    for (uint32_t i = 0; i < slots_.size() && i < kMaxPhantomDevices; ++i) {
        const auto& s = slots_[i];
        auto& dst = layout->devices[i];
        // Bump epoch into odd (writing) state.
        dst.epoch = dst.epoch + 1;
        MemoryBarrier();

        dst.state           = static_cast<uint8_t>(s.ladder.state());
        dst.opted_in        = s.opted_in ? 1u : 0u;
        dst.dropout_count   = s.ladder.dropout_count();
        dst.dropout_age_ms  = s.ladder.dropout_age_ms(now_qpc, freq);
        dst.longest_dropout_ms = s.ladder.longest_dropout_ms();

        const uint32_t copy_len = static_cast<uint32_t>(
            std::min<size_t>(s.serial.size(),
                              PhantomDeviceState::kMaxSerialLen - 1));
        std::memset(dst.serial, 0, sizeof(dst.serial));
        if (copy_len > 0) {
            std::memcpy(dst.serial, s.serial.data(), copy_len);
        }
        dst.serial_len = copy_len;

        MemoryBarrier();
        // Bump epoch back to even (stable) state.
        dst.epoch = dst.epoch + 1;
    }

    for (uint8_t i = 0; i < kBodyRoleCount; ++i) {
        const auto& src = last_body_completion_.roles[i];
        auto& dst = layout->roles[i];
        dst.epoch = dst.epoch + 1;
        MemoryBarrier();

        dst.role = i;
        dst.valid = src.valid ? 1u : 0u;
        dst.solver_mode = static_cast<uint8_t>(src.mode);
        dst._pad0 = 0;
        dst.source_mask = src.source_mask;
        dst._pad1 = 0;
        dst.confidence = src.confidence;
        dst.age_ms = src.age_ms;
        dst.position[0] = src.pose.position[0];
        dst.position[1] = src.pose.position[1];
        dst.position[2] = src.pose.position[2];

        MemoryBarrier();
        dst.epoch = dst.epoch + 1;
    }
}

} // namespace

std::unique_ptr<DriverModule> CreateDriverModule()
{
    return std::make_unique<PhantomModule>();
}

void OnRealPoseObserved(uint32_t openVRID,
                        int64_t qpc_ns,
                        const vr::DriverPose_t& pose)
{
    if (auto* m = g_active) m->OnRealPoseObserved(openVRID, qpc_ns, pose);
}

bool MaybeOverridePose(uint32_t openVRID,
                       int64_t qpc_ns,
                       int64_t qpc_freq,
                       vr::DriverPose_t& pose)
{
    if (auto* m = g_active) return m->MaybeOverridePose(openVRID, qpc_ns, qpc_freq, pose);
    return true;
}

} // namespace phantom
