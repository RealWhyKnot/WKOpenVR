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
#include "DeadReckoner.h"
#include "DropoutState.h"
#include "IkFallback.h"
#include "PoseHistory.h"
#include "RoleCatalog.h"
#include "VirtualTrackerManager.h"

#include <openvr_driver.h>

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

    // Body role assigned via the T-pose calibration wizard. The IK
    // fallback (Phase 1.5+) consults this to decide which rigid offset
    // to apply when the device drops out.
    BodyRole     role = BodyRole::None;

    // Tracks whether this device is the HMD; the IK solver needs the
    // HMD's live pose as its reference frame. Set the first time
    // ResolveSerialIfMissing identifies a Class_HMD device.
    bool         is_hmd = false;

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
    VirtualTrackerManager virtual_trackers_;

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
    if (classErr == vr::TrackedProp_Success
        && deviceClass == vr::TrackedDeviceClass_HMD) {
        s.is_hmd = true;
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

bool PhantomModule::Init(DriverModuleContext& context)
{
    context_ = context;
    g_active = this;
    virtual_trackers_.OnDriverInit();
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
            master_enabled_.store(c.master_enabled != 0, std::memory_order_release);
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
                    s.ladder.SetIkAvailable(ik_fallback_.HasOffset(s.role));
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
                s.ladder.SetIkAvailable(s.role != BodyRole::None
                    && ik_fallback_.HasOffset(s.role));
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
    s.history.Push(qpc_ns, pose);
    s.ladder.SetTimings(timings_);
    s.ladder.SetIkAvailable(s.role != BodyRole::None
        && ik_fallback_.HasOffset(s.role));
    s.ladder.OnRealPoseObserved(qpc_ns, s.history, pose);
    if (s.is_hmd && pose.poseIsValid && pose.deviceIsConnected
        && pose.result == vr::TrackingResult_Running_OK) {
        last_hmd_pose_ = pose;
        last_hmd_valid_ = true;
        // Drive any absent-mode virtual trackers off the HMD's pose
        // cadence. Each HMD update -> one virtual-pose update per
        // enabled role. Activation is deferred inside the manager until
        // the openvr#1536 settle window has elapsed.
        virtual_trackers_.Tick(last_hmd_pose_, ik_fallback_);
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

        case TrackerState::SYNTH_IK: {
            // Phase 1.5: rigid-offset IK. Requires a live HMD pose and a
            // calibration on file for this device's role; if either is
            // missing, fall through to dead reckoning.
            vr::DriverPose_t synth{};
            if (last_hmd_valid_
                && s.role != BodyRole::None
                && ik_fallback_.Solve(s.role, last_hmd_pose_, synth)) {
                synth.result = s.ladder.tracking_result_override();
                pose = synth;
                cachePublished(pose);
                return true;
            }
            // Falls through to reckoner below.
            [[fallthrough]];
        }
        case TrackerState::SYNTH_RECKON:
        case TrackerState::SYNTH_ML:    // Phase 3 hook stays in reckoner today.
        case TrackerState::OUT_OF_RANGE: {
            vr::DriverPose_t synth{};
            if (reckoner_.Project(s.history, qpc_freq, qpc_ns, synth)) {
                synth.result = s.ladder.tracking_result_override();
                pose = synth;
            } else {
                // No real history to project from -- keep whatever the source
                // gave us but stamp the override result so consumers still
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
