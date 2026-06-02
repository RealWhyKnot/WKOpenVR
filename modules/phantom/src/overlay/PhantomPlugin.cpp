// openvr.h must precede anything that may pull in openvr_driver.h via the
// IPCClient -> Protocol include chain, matching the smoothing plugin's
// convention. Defining _OPENVR_API early makes Protocol.h skip the driver
// header so we don't redefine vr:: symbols.
#include <openvr.h>

#include "PhantomPlugin.h"

#include "DeviceFilters.h"
#include "Protocol.h"
#include "ShellContext.h"
#include "UiHelpers.h"

#include <imgui.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <cmath>
#include <string>

namespace {

const char* SolverModeLabel(uint8_t mode);
std::string SourceMaskLabel(uint16_t mask);

} // namespace

void PhantomPlugin::OnStart(openvr_pair::overlay::ShellContext&)
{
    (void)ConnectIfNeeded();
    SendConfig();
    SendSolverConfig();
    // Push the persisted per-device opt-in map + calibration so the driver
    // picks up the user's prior choices without needing to wait for them
    // to re-toggle / re-calibrate.
    for (const auto& kv : cfg_.dropout_enabled) {
        if (kv.second) SendDeviceOptIn(kv.first, true);
    }
    ReplayCalibration();
    seededDriver_ = ipc_.IsConnected();
}

void PhantomPlugin::Tick(openvr_pair::overlay::ShellContext&)
{
    if (!ipc_.IsConnected() && ConnectIfNeeded()) {
        // Reconnected after a drop -- replay full state so the driver is in
        // sync with what the overlay believes.
        SendConfig();
        SendSolverConfig();
        for (const auto& kv : cfg_.dropout_enabled) {
            if (kv.second) SendDeviceOptIn(kv.first, true);
        }
        ReplayCalibration();
        seededDriver_ = true;
    }

    if (!stateShmemReady_) {
        if (stateShmem_.Open(OPENVR_PAIRDRIVER_PHANTOM_STATE_SHMEM_NAME)) {
            stateShmemReady_ = true;
        }
    }
}

bool PhantomPlugin::ConnectIfNeeded()
{
    if (ipc_.IsConnected()) return false;
    const auto now = std::chrono::steady_clock::now();
    if (nextConnectAttempt_.time_since_epoch().count() != 0 && now < nextConnectAttempt_) {
        return false;
    }
    nextConnectAttempt_ = now + std::chrono::seconds(1);

    try {
        ipc_.Connect();
        connectError_.clear();
        nextConnectAttempt_ = {};
        return true;
    } catch (const std::exception& e) {
        connectError_ = e.what();
    }
    return false;
}

void PhantomPlugin::SendConfig()
{
    if (!ipc_.IsConnected()) return;
    protocol::Request req(protocol::RequestSetPhantomConfig);
    auto& c = req.setPhantomConfig;
    c.master_enabled = cfg_.master_enabled ? 1u : 0u;
    c._pad0[0] = c._pad0[1] = c._pad0[2] = 0;
    c.blend_out_ms   = cfg_.blend_out_ms;
    c.blend_in_ms    = cfg_.blend_in_ms;
    c.reckon_hold_ms = cfg_.reckon_hold_ms;
    c.synth_hold_ms  = cfg_.synth_hold_ms;
    c.lost_hold_ms   = cfg_.lost_hold_ms;
    try {
        ipc_.SendBlocking(req);
    } catch (const std::exception& e) {
        connectError_ = e.what();
    }
}

void PhantomPlugin::SendSolverConfig()
{
    if (!ipc_.IsConnected()) return;
    protocol::Request req(protocol::RequestSetPhantomSolverConfig);
    auto& s = req.setPhantomSolverConfig;
    std::memset(&s, 0, sizeof(s));
    s.calibrated = cfg_.solver.calibrated ? 1u : 0u;
    s.floor_y_m = cfg_.solver.floor_y_m;
    s.height_m = cfg_.solver.height_m;
    s.forward_yaw_rad = cfg_.solver.forward_yaw_rad;
    s.stance_width_m = cfg_.solver.stance_width_m;
    s.shoulder_width_m = cfg_.solver.shoulder_width_m;
    s.pelvis_width_m = cfg_.solver.pelvis_width_m;
    s.upper_arm_m = cfg_.solver.upper_arm_m;
    s.lower_arm_m = cfg_.solver.lower_arm_m;
    s.upper_leg_m = cfg_.solver.upper_leg_m;
    s.lower_leg_m = cfg_.solver.lower_leg_m;
    s.virtual_min_confidence = cfg_.solver.virtual_min_confidence;
    try {
        ipc_.SendBlocking(req);
    } catch (const std::exception& e) {
        connectError_ = e.what();
    }
}

void PhantomPlugin::SendDeviceOptIn(const std::string& serial, bool enabled)
{
    if (!ipc_.IsConnected()) return;
    // FNV-1a 64-bit of the serial string; matches the driver-side hash so
    // the slot lookup lands on the right device.
    uint64_t h = 0xcbf29ce484222325ull;
    for (char c : serial) {
        h ^= static_cast<unsigned char>(c);
        h *= 0x100000001b3ull;
    }
    protocol::Request req(protocol::RequestSetPhantomDeviceOptIn);
    req.setPhantomDeviceOptIn.device_serial_hash = h;
    req.setPhantomDeviceOptIn.dropout_enabled    = enabled ? 1u : 0u;
    std::memset(req.setPhantomDeviceOptIn._reserved, 0,
                sizeof(req.setPhantomDeviceOptIn._reserved));
    try {
        ipc_.SendBlocking(req);
    } catch (const std::exception& e) {
        connectError_ = e.what();
    }
}

void PhantomPlugin::DrawTab(openvr_pair::overlay::ShellContext&)
{
    openvr_pair::overlay::ui::TabBarScope tabs("PhantomTabs");
    if (tabs) {
        openvr_pair::overlay::ui::DrawTabItem("Dropouts", [&] { DrawDropoutsTab(); });
        openvr_pair::overlay::ui::DrawTabItem("Calibration", [&] { DrawCalibrationTab(); });
        openvr_pair::overlay::ui::DrawTabItem("Absent", [&] { DrawAbsentTab(); });
        openvr_pair::overlay::ui::DrawTabItem("Diagnostics", [&] { DrawDiagnosticsTab(); });
        openvr_pair::overlay::ui::DrawTabItem("Advanced", [&] { DrawAdvancedTab(); });
    }

    if (!connectError_.empty() && !ipc_.IsConnected()) {
        ImGui::Spacing();
        ImGui::TextColored(openvr_pair::overlay::ui::GetPalette().statusError,
            "IPC: %s", connectError_.c_str());
    }
}

void PhantomPlugin::DrawDropoutsTab()
{
    ImGui::Spacing();
    if (ImGui::Checkbox("Bridge dropped trackers", &cfg_.master_enabled)) {
        SendConfig();
        SavePhantomConfig(cfg_);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Master switch. With this on, the driver fills in plausible poses\n"
            "for any tracker you opted in below when its real pose goes silent.\n"
            "Past the synth-hold window the tracker is marked OutOfRange so\n"
            "VRChat / Resonite drop it from the IK chain cleanly.");
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Per-tracker opt-in");
    ImGui::Spacing();

    auto* vrSystem = vr::VRSystem();
    if (!vrSystem) {
        ImGui::TextDisabled("(VR system not available)");
        return;
    }

    char buffer[vr::k_unMaxPropertyStringSize];
    bool anyShown = false;
    for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
        const auto deviceClass = vrSystem->GetTrackedDeviceClass(id);
        if (deviceClass == vr::TrackedDeviceClass_Invalid) continue;

        vr::ETrackedPropertyError err = vr::TrackedProp_Success;
        vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String,
            buffer, sizeof(buffer), &err);
        if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
        const std::string serial = buffer;

        vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_RenderModelName_String,
            buffer, sizeof(buffer), &err);
        const std::string model = (err == vr::TrackedProp_Success) ? buffer : "";

        vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String,
            buffer, sizeof(buffer), &err);
        const std::string trackingSystem = (err == vr::TrackedProp_Success) ? buffer : "";

        if (!openvr_pair::overlay::ShouldShowInSmoothingPredictionList(
                deviceClass, serial, model, trackingSystem)) {
            continue;
        }

        anyShown = true;
        bool enabled = cfg_.dropout_enabled.count(serial) ? cfg_.dropout_enabled[serial] : false;
        ImGui::PushID(("trk_" + serial).c_str());
        if (ImGui::Checkbox("##en", &enabled)) {
            cfg_.dropout_enabled[serial] = enabled;
            SendDeviceOptIn(serial, enabled);
            SavePhantomConfig(cfg_);
        }
        ImGui::SameLine();
        ImGui::TextWrapped("%s  [%s]",
            model.empty() ? "(unknown model)" : model.c_str(),
            serial.c_str());
        ImGui::PopID();
    }
    if (!anyShown) {
        ImGui::TextDisabled("No bridgeable trackers detected. HMD, controllers, and "
                            "generic body trackers will appear here once SteamVR is "
                            "running and the devices are on.");
    }
}

void PhantomPlugin::DrawDiagnosticsTab()
{
    ImGui::Spacing();
    if (!stateShmemReady_ || !stateShmem_.layout()) {
        ImGui::TextDisabled("Driver state not yet available. The driver must be loaded "
                            "and the phantom feature flag enabled.");
        return;
    }
    const auto* layout = stateShmem_.layout();
    if (layout->magic != phantom::kPhantomStateShmemMagic) {
        ImGui::TextDisabled("Driver state shmem has unexpected magic; mismatched install?");
        return;
    }
    if (layout->version != phantom::kPhantomStateShmemVersion) {
        ImGui::TextDisabled("Driver state shmem has unexpected version; mismatched install?");
        return;
    }

    openvr_pair::overlay::ui::TableScope table("PhantomDiag", 5,
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH);
    if (table) {
        ImGui::TableSetupColumn("Serial");
        ImGui::TableSetupColumn("State");
        ImGui::TableSetupColumn("Drops");
        ImGui::TableSetupColumn("Now (ms)");
        ImGui::TableSetupColumn("Longest (ms)");
        ImGui::TableHeadersRow();
        for (uint32_t i = 0; i < layout->device_count; ++i) {
            const auto& d = layout->devices[i];
            if (d.serial_len == 0) continue;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%.*s", (int)d.serial_len, d.serial);
            ImGui::TableNextColumn();
            ImGui::Text("%s", phantom::TrackerStateLabel(
                static_cast<phantom::TrackerState>(d.state)));
            ImGui::TableNextColumn();
            ImGui::Text("%u", d.dropout_count);
            ImGui::TableNextColumn();
            ImGui::Text("%u", d.dropout_age_ms);
            ImGui::TableNextColumn();
            ImGui::Text("%u", d.longest_dropout_ms);
        }
    }

    if (layout->version >= phantom::kPhantomStateShmemVersion) {
        ImGui::Spacing();
        ImGui::SeparatorText("Role completion");
        openvr_pair::overlay::ui::TableScope roleTable("PhantomRoleDiag", 6,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH);
        if (roleTable) {
            ImGui::TableSetupColumn("Role");
            ImGui::TableSetupColumn("Confidence");
            ImGui::TableSetupColumn("Mode");
            ImGui::TableSetupColumn("Source");
            ImGui::TableSetupColumn("Age");
            ImGui::TableSetupColumn("Position");
            ImGui::TableHeadersRow();
            for (uint8_t i = 0; i < phantom::kBodyRoleCount; ++i) {
                const auto& r = layout->roles[i];
                if (!r.valid) continue;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%s", phantom::BodyRoleLabel(
                    static_cast<phantom::BodyRole>(r.role)));
                ImGui::TableNextColumn();
                ImGui::Text("%.2f", r.confidence);
                ImGui::TableNextColumn();
                ImGui::Text("%s", SolverModeLabel(r.solver_mode));
                ImGui::TableNextColumn();
                const std::string sources = SourceMaskLabel(r.source_mask);
                ImGui::TextWrapped("%s", sources.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%u ms", r.age_ms);
                ImGui::TableNextColumn();
                ImGui::Text("%.2f %.2f %.2f",
                    r.position[0], r.position[1], r.position[2]);
            }
        }
    }
}

void PhantomPlugin::DrawAdvancedTab()
{
    ImGui::Spacing();
    ImGui::TextWrapped(
        "Timing ladder. Lower the synth-hold to recover trackers faster after "
        "a stuck pose; raise it to bridge longer outages. Out-of-range happens "
        "at synth-hold; the device stops publishing at lost-hold.");
    ImGui::Spacing();

    auto sliderMs = [&](const char* label, uint32_t& v, uint32_t lo, uint32_t hi,
                        const char* tip) {
        int tmp = static_cast<int>(v);
        if (ImGui::SliderInt(label, &tmp, (int)lo, (int)hi, "%d ms")) {
            v = static_cast<uint32_t>(std::clamp(tmp, (int)lo, (int)hi));
            SendConfig();
            SavePhantomConfig(cfg_);
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
    };

    sliderMs("Blend out",  cfg_.blend_out_ms,  0, 500,
        "Real to synth fade duration on dropout start.");
    sliderMs("Blend in",   cfg_.blend_in_ms,   0, 1000,
        "Synth to real fade duration when the real signal returns.");
    sliderMs("Reckon hold", cfg_.reckon_hold_ms, 0, 1000,
        "How long dead reckoning is the primary synthesis source before the "
        "ladder escalates (to IK / ML in later phases).");
    sliderMs("Synth hold", cfg_.synth_hold_ms, 100, 10000,
        "Total time after dropout before ETrackingResult flips to OutOfRange.");
    sliderMs("Lost hold",  cfg_.lost_hold_ms,  500, 60000,
        "Total time after dropout before the device stops publishing entirely.");

    ImGui::Spacing();
    if (ImGui::Button("Reset to defaults")) {
        cfg_.blend_out_ms   = phantom::DefaultTimings::kBlendOutMs;
        cfg_.blend_in_ms    = phantom::DefaultTimings::kBlendInMs;
        cfg_.reckon_hold_ms = phantom::DefaultTimings::kReckonHoldMs;
        cfg_.synth_hold_ms  = phantom::DefaultTimings::kSynthHoldMs;
        cfg_.lost_hold_ms   = phantom::DefaultTimings::kLostHoldMs;
        SendConfig();
        SavePhantomConfig(cfg_);
    }
}

namespace {

// FNV-1a 64-bit over a serial string. Matches the driver-side convention
// so per-serial-hash maps line up on both ends.
uint64_t Fnv1a64Local(const std::string& s)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (char c : s) {
        h ^= static_cast<unsigned char>(c);
        h *= 0x100000001b3ull;
    }
    return h;
}

// Quaternion conjugate (== inverse for a unit quaternion).
vr::HmdQuaternion_t QConj(const vr::HmdQuaternion_t& q)
{
    return { q.w, -q.x, -q.y, -q.z };
}

vr::HmdQuaternion_t QMul(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
{
    vr::HmdQuaternion_t r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

// HmdMatrix34_t (column-major 3x4 with column 3 = translation) -> position
// + quaternion. Matches the convention used by GetDeviceToAbsoluteTrackingPose.
void DecomposeMatrix34(const vr::HmdMatrix34_t& m,
                       double pos[3],
                       vr::HmdQuaternion_t& q)
{
    pos[0] = m.m[0][3];
    pos[1] = m.m[1][3];
    pos[2] = m.m[2][3];
    // Standard rotation-matrix to quaternion. The matrix's upper-left 3x3
    // is the rotation; we read it as row-major-3x3 directly.
    const double m00 = m.m[0][0], m01 = m.m[0][1], m02 = m.m[0][2];
    const double m10 = m.m[1][0], m11 = m.m[1][1], m12 = m.m[1][2];
    const double m20 = m.m[2][0], m21 = m.m[2][1], m22 = m.m[2][2];
    const double trace = m00 + m11 + m22;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q.w = (m21 - m12) / s;
        q.x = 0.25 * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25 * s;
        q.z = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25 * s;
    }
}

void QRotateInverse(const vr::HmdQuaternion_t& q, const double v[3], double out[3])
{
    // Rotate v by q^-1 (= conjugate for unit q). Standard formula via
    // q^-1 * v * q expressed without intermediate quaternions.
    const auto qi = QConj(q);
    const double ux = qi.x, uy = qi.y, uz = qi.z, s = qi.w;
    const double tx = 2.0 * (uy * v[2] - uz * v[1]);
    const double ty = 2.0 * (uz * v[0] - ux * v[2]);
    const double tz = 2.0 * (ux * v[1] - uy * v[0]);
    out[0] = v[0] + s * tx + (uy * tz - uz * ty);
    out[1] = v[1] + s * ty + (uz * tx - ux * tz);
    out[2] = v[2] + s * tz + (ux * ty - uy * tx);
}

double YawRadiansFromQuat(const vr::HmdQuaternion_t& q)
{
    const double siny = 2.0 * (q.w * q.y + q.x * q.z);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

const char* SolverModeLabel(uint8_t mode)
{
    switch (mode) {
    case 0: return "none";
    case 1: return "measured";
    case 2: return "hmd_root";
    case 3: return "controller_ik";
    case 4: return "floor_contact";
    case 5: return "held_contact";
    case 6: return "low_confidence";
    }
    return "unknown";
}

std::string SourceMaskLabel(uint16_t mask)
{
    std::string out;
    auto add = [&](uint16_t bit, const char* label) {
        if ((mask & bit) == 0) return;
        if (!out.empty()) out += ",";
        out += label;
    };
    add(1u << 0, "measured");
    add(1u << 1, "hmd");
    add(1u << 2, "controller");
    add(1u << 3, "floor");
    add(1u << 4, "contact");
    add(1u << 5, "predicted");
    add(1u << 6, "held");
    if (out.empty()) out = "none";
    return out;
}

} // namespace

void PhantomPlugin::SendDeviceRole(const std::string& serial, phantom::BodyRole role)
{
    if (!ipc_.IsConnected()) return;
    protocol::Request req(protocol::RequestSetPhantomDeviceRole);
    req.setPhantomDeviceRole.device_serial_hash = Fnv1a64Local(serial);
    req.setPhantomDeviceRole.body_role = static_cast<uint8_t>(role);
    std::memset(req.setPhantomDeviceRole._reserved, 0,
                sizeof(req.setPhantomDeviceRole._reserved));
    try { ipc_.SendBlocking(req); }
    catch (const std::exception& e) { connectError_ = e.what(); }
}

void PhantomPlugin::SendTrackerOffset(phantom::BodyRole role,
                                      const PhantomRoleOffset& offset)
{
    if (!ipc_.IsConnected()) return;
    protocol::Request req(protocol::RequestSetPhantomTrackerOffset);
    auto& o = req.setPhantomTrackerOffset;
    o.body_role = static_cast<uint8_t>(role);
    o.calibrated = offset.calibrated ? 1u : 0u;
    std::memset(o._pad, 0, sizeof(o._pad));
    o.rel_position[0] = offset.rel_position_x;
    o.rel_position[1] = offset.rel_position_y;
    o.rel_position[2] = offset.rel_position_z;
    o.rel_rotation.w = offset.rel_rotation_w;
    o.rel_rotation.x = offset.rel_rotation_x;
    o.rel_rotation.y = offset.rel_rotation_y;
    o.rel_rotation.z = offset.rel_rotation_z;
    try { ipc_.SendBlocking(req); }
    catch (const std::exception& e) { connectError_ = e.what(); }
}

void PhantomPlugin::SendVirtualEnabled(phantom::BodyRole role, bool enabled)
{
    if (!ipc_.IsConnected()) return;
    protocol::Request req(protocol::RequestSetPhantomVirtualEnabled);
    req.setPhantomVirtualEnabled.body_role = static_cast<uint8_t>(role);
    req.setPhantomVirtualEnabled.enabled = enabled ? 1u : 0u;
    std::memset(req.setPhantomVirtualEnabled._reserved, 0,
                sizeof(req.setPhantomVirtualEnabled._reserved));
    try { ipc_.SendBlocking(req); }
    catch (const std::exception& e) { connectError_ = e.what(); }
}

void PhantomPlugin::ReplayCalibration()
{
    if (!ipc_.IsConnected()) return;
    SendSolverConfig();
    for (const auto& kv : cfg_.device_role) {
        SendDeviceRole(kv.first, kv.second);
    }
    for (const auto& kv : cfg_.role_offset) {
        if (kv.second.calibrated) SendTrackerOffset(kv.first, kv.second);
    }
    for (const auto& kv : cfg_.virtual_enabled) {
        if (kv.second) SendVirtualEnabled(kv.first, true);
    }
}

void PhantomPlugin::CaptureNeutralStanding()
{
    auto* vrSystem = vr::VRSystem();
    if (!vrSystem) {
        lastSolverCalibrationSummary_ = "VR system not available; neutral pose not captured.";
        return;
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
    vrSystem->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        /*predictedSecondsToPhotonsFromNow=*/0.0f,
        poses,
        vr::k_unMaxTrackedDeviceCount);

    uint32_t hmdId = vr::k_unTrackedDeviceIndexInvalid;
    for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
        if (vrSystem->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_HMD
            && poses[id].bPoseIsValid
            && poses[id].eTrackingResult == vr::TrackingResult_Running_OK) {
            hmdId = id;
            break;
        }
    }
    if (hmdId == vr::k_unTrackedDeviceIndexInvalid) {
        lastSolverCalibrationSummary_ = "HMD not tracked; neutral pose not captured.";
        return;
    }

    double hmdPos[3];
    vr::HmdQuaternion_t hmdRot;
    DecomposeMatrix34(poses[hmdId].mDeviceToAbsoluteTracking, hmdPos, hmdRot);

    cfg_.solver.calibrated = true;
    cfg_.solver.floor_y_m = 0.0;
    cfg_.solver.height_m = std::clamp(hmdPos[1] - cfg_.solver.floor_y_m, 1.0, 2.4);
    cfg_.solver.forward_yaw_rad = YawRadiansFromQuat(hmdRot);

    const double h = cfg_.solver.height_m;
    cfg_.solver.stance_width_m = std::clamp(h * 0.165, 0.10, 0.70);
    cfg_.solver.shoulder_width_m = std::clamp(h * 0.225, 0.20, 0.70);
    cfg_.solver.pelvis_width_m = std::clamp(h * 0.165, 0.15, 0.60);
    cfg_.solver.upper_arm_m = std::clamp(h * 0.176, 0.15, 0.55);
    cfg_.solver.lower_arm_m = std::clamp(h * 0.159, 0.15, 0.55);
    cfg_.solver.upper_leg_m = std::clamp(h * 0.265, 0.20, 0.70);
    cfg_.solver.lower_leg_m = std::clamp(h * 0.265, 0.20, 0.70);

    SendSolverConfig();
    SavePhantomConfig(cfg_);

    char tmp[128];
    std::snprintf(tmp, sizeof(tmp),
        "Neutral standing captured: height %.2f m, forward %.2f rad.",
        cfg_.solver.height_m,
        cfg_.solver.forward_yaw_rad);
    lastSolverCalibrationSummary_ = tmp;
}

void PhantomPlugin::CaptureTPose()
{
    auto* vrSystem = vr::VRSystem();
    if (!vrSystem) {
        lastCalibrationSummary_ = "VR system not available; tracking poses not captured.";
        return;
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
    vrSystem->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        /*predictedSecondsToPhotonsFromNow=*/0.0f,
        poses,
        vr::k_unMaxTrackedDeviceCount);

    // Locate the HMD (Class_HMD). Without it we have no reference frame for
    // the IK fallback's rigid offsets.
    uint32_t hmdId = vr::k_unTrackedDeviceIndexInvalid;
    for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
        if (vrSystem->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_HMD
            && poses[id].bPoseIsValid
            && poses[id].eTrackingResult == vr::TrackingResult_Running_OK) {
            hmdId = id;
            break;
        }
    }
    if (hmdId == vr::k_unTrackedDeviceIndexInvalid) {
        lastCalibrationSummary_ = "HMD not tracked; calibration aborted.";
        return;
    }

    double hmdPos[3];
    vr::HmdQuaternion_t hmdRot;
    DecomposeMatrix34(poses[hmdId].mDeviceToAbsoluteTracking, hmdPos, hmdRot);

    int captured = 0;
    int skipped  = 0;
    char buffer[vr::k_unMaxPropertyStringSize];
    for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
        const auto cls = vrSystem->GetTrackedDeviceClass(id);
        if (cls == vr::TrackedDeviceClass_Invalid || cls == vr::TrackedDeviceClass_HMD) {
            continue;
        }
        if (!poses[id].bPoseIsValid
            || poses[id].eTrackingResult != vr::TrackingResult_Running_OK) {
            continue;
        }
        vr::ETrackedPropertyError err = vr::TrackedProp_Success;
        vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String,
            buffer, sizeof(buffer), &err);
        if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
        const std::string serial = buffer;

        const auto roleIt = cfg_.device_role.find(serial);
        if (roleIt == cfg_.device_role.end() || roleIt->second == phantom::BodyRole::None) {
            ++skipped;
            continue;
        }

        double trkPos[3];
        vr::HmdQuaternion_t trkRot;
        DecomposeMatrix34(poses[id].mDeviceToAbsoluteTracking, trkPos, trkRot);

        const double delta[3] = {
            trkPos[0] - hmdPos[0],
            trkPos[1] - hmdPos[1],
            trkPos[2] - hmdPos[2],
        };
        double rel[3];
        QRotateInverse(hmdRot, delta, rel);
        const vr::HmdQuaternion_t relRot = QMul(QConj(hmdRot), trkRot);

        PhantomRoleOffset off{};
        off.calibrated = true;
        off.rel_position_x = rel[0];
        off.rel_position_y = rel[1];
        off.rel_position_z = rel[2];
        off.rel_rotation_w = relRot.w;
        off.rel_rotation_x = relRot.x;
        off.rel_rotation_y = relRot.y;
        off.rel_rotation_z = relRot.z;

        cfg_.role_offset[roleIt->second] = off;
        SendTrackerOffset(roleIt->second, off);
        ++captured;
    }

    SavePhantomConfig(cfg_);
    char tmp[128];
    std::snprintf(tmp, sizeof(tmp),
        "Captured %d role%s; skipped %d unassigned tracker%s.",
        captured, captured == 1 ? "" : "s",
        skipped,  skipped  == 1 ? "" : "s");
    lastCalibrationSummary_ = tmp;
}

void PhantomPlugin::DrawCalibrationTab()
{
    ImGui::Spacing();
    ImGui::TextWrapped(
        "Capture neutral standing first for headset/controller body completion. "
        "Assign physical trackers below only when a real tracker should anchor "
        "or disambiguate a body role.");
    ImGui::Spacing();

    ImGui::SeparatorText("Neutral standing");
    if (ImGui::Button("Capture neutral standing")) {
        CaptureNeutralStanding();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Stand upright, face forward, and click. Uses the current HMD pose "
            "to set height, floor, forward direction, and starting proportions.");
    }
    if (!lastSolverCalibrationSummary_.empty()) {
        ImGui::Spacing();
        ImGui::TextWrapped("%s", lastSolverCalibrationSummary_.c_str());
    }

    auto solverSlider = [&](const char* label, double& v, float lo, float hi,
                            const char* fmt) {
        float tmp = static_cast<float>(v);
        if (ImGui::SliderFloat(label, &tmp, lo, hi, fmt)) {
            v = static_cast<double>(std::clamp(tmp, lo, hi));
            cfg_.solver.calibrated = true;
            SendSolverConfig();
            SavePhantomConfig(cfg_);
        }
    };

    ImGui::Spacing();
    solverSlider("Height", cfg_.solver.height_m, 1.0f, 2.4f, "%.2f m");
    solverSlider("Floor Y", cfg_.solver.floor_y_m, -1.0f, 1.0f, "%.2f m");
    solverSlider("Stance width", cfg_.solver.stance_width_m, 0.10f, 0.70f, "%.2f m");
    solverSlider("Shoulder width", cfg_.solver.shoulder_width_m, 0.20f, 0.70f, "%.2f m");
    solverSlider("Pelvis width", cfg_.solver.pelvis_width_m, 0.15f, 0.60f, "%.2f m");
    solverSlider("Upper arm", cfg_.solver.upper_arm_m, 0.15f, 0.55f, "%.2f m");
    solverSlider("Lower arm", cfg_.solver.lower_arm_m, 0.15f, 0.55f, "%.2f m");
    solverSlider("Upper leg", cfg_.solver.upper_leg_m, 0.20f, 0.70f, "%.2f m");
    solverSlider("Lower leg", cfg_.solver.lower_leg_m, 0.20f, 0.70f, "%.2f m");
    solverSlider("Minimum virtual confidence",
        cfg_.solver.virtual_min_confidence, 0.0f, 1.0f, "%.2f");

    ImGui::Spacing();
    auto* vrSystem = vr::VRSystem();
    if (!vrSystem) {
        ImGui::TextDisabled("(VR system not available)");
        return;
    }

    ImGui::SeparatorText("Physical tracker roles");
    char buffer[vr::k_unMaxPropertyStringSize];
    for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
        const auto cls = vrSystem->GetTrackedDeviceClass(id);
        if (cls == vr::TrackedDeviceClass_Invalid) continue;

        vr::ETrackedPropertyError err = vr::TrackedProp_Success;
        vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String,
            buffer, sizeof(buffer), &err);
        if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
        const std::string serial = buffer;

        vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_RenderModelName_String,
            buffer, sizeof(buffer), &err);
        const std::string model = (err == vr::TrackedProp_Success) ? buffer : "";

        vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String,
            buffer, sizeof(buffer), &err);
        const std::string trackingSystem = (err == vr::TrackedProp_Success) ? buffer : "";

        if (!openvr_pair::overlay::ShouldShowInSmoothingPredictionList(cls, serial, model, trackingSystem)) {
            continue;
        }

        const auto roleIt = cfg_.device_role.find(serial);
        phantom::BodyRole cur = (roleIt != cfg_.device_role.end())
            ? roleIt->second
            : phantom::BodyRole::None;

        ImGui::PushID(("cal_" + serial).c_str());
        ImGui::TextWrapped("%s  [%s]",
            model.empty() ? "(unknown model)" : model.c_str(),
            serial.c_str());

        const char* curLabel = phantom::BodyRoleLabel(cur);
        if (ImGui::BeginCombo("Role", curLabel)) {
            for (uint8_t i = 0; i < phantom::kBodyRoleCount; ++i) {
                const auto r = static_cast<phantom::BodyRole>(i);
                // HMD / hand roles are not assignable on a body tracker;
                // skip them in the dropdown to avoid invalid combinations.
                if (r == phantom::BodyRole::Hmd
                    || r == phantom::BodyRole::LeftHand
                    || r == phantom::BodyRole::RightHand) continue;
                const bool selected = (r == cur);
                if (ImGui::Selectable(phantom::BodyRoleLabel(r), selected)) {
                    if (r == phantom::BodyRole::None) {
                        cfg_.device_role.erase(serial);
                    } else {
                        cfg_.device_role[serial] = r;
                    }
                    SendDeviceRole(serial, r);
                    SavePhantomConfig(cfg_);
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::PopID();
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Tracker mounting offsets");
    if (ImGui::Button("Capture T-pose now")) {
        CaptureTPose();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Stand in a T-pose (arms out, body upright, head level) and click.\n"
            "Captures the rigid mounting offset of every assigned physical\n"
            "tracker. Reassigning a role or moving a tracker means re-capturing.");
    }
    if (!lastCalibrationSummary_.empty()) {
        ImGui::Spacing();
        ImGui::TextWrapped("%s", lastCalibrationSummary_.c_str());
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Calibration status");
    if (cfg_.role_offset.empty()) {
        ImGui::TextDisabled("No roles calibrated yet.");
    } else {
        for (const auto& kv : cfg_.role_offset) {
            ImGui::Text("%-18s %s",
                phantom::BodyRoleLabel(kv.first),
                kv.second.calibrated ? "calibrated" : "not captured");
        }
    }
}

void PhantomPlugin::DrawAbsentTab()
{
    ImGui::Spacing();
    ImGui::TextWrapped(
        "Add body trackers you do not physically own. The driver creates a "
        "virtual SteamVR tracker for each enabled role and feeds it from "
        "headset/controller/physical-tracker body completion when confidence "
        "is high enough. The list follows VRChat's eight supported extra "
        "tracker points.");
    ImGui::Spacing();
    ImGui::TextDisabled(
        "Note: SteamVR does not allow live retraction. Disabling a role here "
        "stops pose publishing immediately but the SteamVR device entry stays "
        "until the next vrserver restart.");
    ImGui::Spacing();

    ImGui::SeparatorText("Per-role virtual trackers");
    const phantom::BodyRole roles[] = {
        phantom::BodyRole::Waist,
        phantom::BodyRole::Chest,
        phantom::BodyRole::LeftFoot,  phantom::BodyRole::RightFoot,
        phantom::BodyRole::LeftKnee,  phantom::BodyRole::RightKnee,
        phantom::BodyRole::LeftElbow, phantom::BodyRole::RightElbow,
    };
    for (auto role : roles) {
        bool enabled = cfg_.virtual_enabled.count(role)
            ? cfg_.virtual_enabled[role]
            : false;

        ImGui::PushID(static_cast<int>(role));
        if (ImGui::Checkbox("##en", &enabled)) {
            cfg_.virtual_enabled[role] = enabled;
            SendVirtualEnabled(role, enabled);
            SavePhantomConfig(cfg_);
        }
        ImGui::SameLine();
        ImGui::Text("%-18s", phantom::BodyRoleLabel(role));
        ImGui::PopID();
    }
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreatePhantomPlugin()
{
    return std::make_unique<PhantomPlugin>();
}

} // namespace openvr_pair::overlay

