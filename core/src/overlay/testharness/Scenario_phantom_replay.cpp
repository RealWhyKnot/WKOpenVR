#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "HarnessIpcClient.h"
#include "HarnessScenario.h"
#include "MockPoseSource.h"
#include "PhantomMetrics.h"
#include "PhantomReplay.h"
#include "PhantomStateShmem.h"
#include "Protocol.h"
#include "ProtocolNames.h"
#include "RoleCatalog.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>
#include <thread>
#include <unordered_map>

namespace openvr_pair::overlay::testharness {

namespace {

uint64_t Fnv1a64(const std::string& s)
{
	uint64_t h = 0xcbf29ce484222325ULL;
	for (char c : s) {
		h ^= (uint64_t)(uint8_t)c;
		h *= 0x100000001b3ULL;
	}
	return h;
}

double PositionErrorM(const vr::DriverPose_t& a, const vr::DriverPose_t& b)
{
	const double dx = a.vecPosition[0] - b.vecPosition[0];
	const double dy = a.vecPosition[1] - b.vecPosition[1];
	const double dz = a.vecPosition[2] - b.vecPosition[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

phantom::BodyCompletionPose ToMetricPose(const vr::DriverPose_t& pose)
{
	phantom::BodyCompletionPose out;
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

phantom::BodyCompletionPose ToMetricPose(const MockCall& call)
{
	phantom::BodyCompletionPose out;
	out.position[0] = call.pose_position[0];
	out.position[1] = call.pose_position[1];
	out.position[2] = call.pose_position[2];
	out.rotation[0] = call.pose_rotation[0];
	out.rotation[1] = call.pose_rotation[1];
	out.rotation[2] = call.pose_rotation[2];
	out.rotation[3] = call.pose_rotation[3];
	out.velocity[0] = call.pose_velocity[0];
	out.velocity[1] = call.pose_velocity[1];
	out.velocity[2] = call.pose_velocity[2];
	return out;
}

bool SendExpectSuccess(HarnessIpcClient& client, const protocol::Request& request, std::string& error)
{
	try {
		const auto response = client.SendBlocking(request);
		if (response.type != protocol::ResponseSuccess) {
			error = "expected ResponseSuccess, got type=" + std::to_string((int)response.type);
			return false;
		}
		return true;
	}
	catch (const std::exception& ex) {
		error = ex.what();
		return false;
	}
}

bool ConfigurePhantom(HarnessIpcClient& client, const PhantomReplayRecording& replay, std::string& error)
{
	{
		protocol::Request req(protocol::RequestSetPhantomConfig);
		auto& cfg = req.setPhantomConfig;
		std::memset(&cfg, 0, sizeof(cfg));
		cfg.master_enabled = 1;
		cfg.blend_out_ms = 80;
		cfg.blend_in_ms = 150;
		cfg.reckon_hold_ms = 250;
		cfg.synth_hold_ms = 2000;
		cfg.lost_hold_ms = 5000;
		if (!SendExpectSuccess(client, req, error)) return false;
	}

	{
		protocol::Request req(protocol::RequestSetPhantomSolverConfig);
		auto& cfg = req.setPhantomSolverConfig;
		std::memset(&cfg, 0, sizeof(cfg));
		cfg.calibrated = 1;
		cfg.floor_y_m = 0.0;
		cfg.height_m = 1.70;
		cfg.forward_yaw_rad = 0.0;
		cfg.stance_width_m = 0.30;
		cfg.shoulder_width_m = 0.40;
		cfg.pelvis_width_m = 0.30;
		cfg.upper_arm_m = 0.31;
		cfg.lower_arm_m = 0.27;
		cfg.upper_leg_m = 0.45;
		cfg.lower_leg_m = 0.45;
		cfg.virtual_min_confidence = 0.20;
		if (!SendExpectSuccess(client, req, error)) return false;
	}

	for (const auto& device : replay.devices) {
		const uint64_t hash = Fnv1a64(device.serial);
		if (device.body_role != phantom::BodyRole::None) {
			protocol::Request req(protocol::RequestSetPhantomDeviceRole);
			req.setPhantomDeviceRole.device_serial_hash = hash;
			req.setPhantomDeviceRole.body_role = static_cast<uint8_t>(device.body_role);
			std::memset(req.setPhantomDeviceRole._reserved, 0, sizeof(req.setPhantomDeviceRole._reserved));
			if (!SendExpectSuccess(client, req, error)) return false;
		}
		if (device.dropout_enabled || device.body_role != phantom::BodyRole::None) {
			protocol::Request req(protocol::RequestSetPhantomDeviceOptIn);
			req.setPhantomDeviceOptIn.device_serial_hash = hash;
			req.setPhantomDeviceOptIn.dropout_enabled = 1;
			std::memset(req.setPhantomDeviceOptIn._reserved, 0, sizeof(req.setPhantomDeviceOptIn._reserved));
			if (!SendExpectSuccess(client, req, error)) return false;
		}
	}
	return true;
}

std::vector<phantom::BodyRole> EnableAbsentVirtualRoles(HarnessIpcClient& client, const PhantomReplayRecording& replay,
                                                        std::string& error)
{
	std::array<bool, phantom::kBodyRoleCount> physical_roles{};
	for (const auto& device : replay.devices) {
		const auto idx = static_cast<size_t>(device.body_role);
		if (idx < physical_roles.size()) physical_roles[idx] = true;
	}

	std::vector<phantom::BodyRole> enabled;
	for (uint8_t i = 0; i < phantom::kBodyRoleCount; ++i) {
		const auto role = static_cast<phantom::BodyRole>(i);
		if (phantom::BodyRoleToControllerType(role) == nullptr) continue;
		if (physical_roles[i]) continue;

		protocol::Request req(protocol::RequestSetPhantomVirtualEnabled);
		req.setPhantomVirtualEnabled.body_role = i;
		req.setPhantomVirtualEnabled.enabled = 1;
		std::memset(req.setPhantomVirtualEnabled._reserved, 0, sizeof(req.setPhantomVirtualEnabled._reserved));
		if (!SendExpectSuccess(client, req, error)) {
			enabled.clear();
			return enabled;
		}
		enabled.push_back(role);
	}
	return enabled;
}

std::string JoinRoleKeys(const std::vector<std::string>& roles)
{
	std::ostringstream oss;
	for (size_t i = 0; i < roles.size(); ++i) {
		if (i > 0) oss << ';';
		oss << roles[i];
	}
	return oss.str();
}

std::string VirtualRoleFromSerial(const std::string& serial)
{
	const std::string prefix = "WKOPENVR-";
	if (serial.rfind(prefix, 0) != 0) return {};
	const size_t last_dash = serial.rfind('-');
	if (last_dash == std::string::npos || last_dash <= prefix.size()) return {};
	const std::string role = serial.substr(prefix.size(), last_dash - prefix.size());
	if (phantom::BodyRoleToControllerType(phantom::BodyRoleFromKey(role.c_str())) == nullptr) return {};
	return role;
}

bool ReadRoleSnapshot(std::array<phantom::PhantomRoleCompletionState, phantom::kBodyRoleCount>& roles)
{
	HANDLE h = ::OpenFileMappingA(FILE_MAP_READ, FALSE, OPENVR_PAIRDRIVER_PHANTOM_STATE_SHMEM_NAME);
	if (h == nullptr) return false;
	void* view = ::MapViewOfFile(h, FILE_MAP_READ, 0, 0, sizeof(phantom::PhantomStateShmemLayout));
	if (view == nullptr) {
		::CloseHandle(h);
		return false;
	}
	const auto* layout = static_cast<const phantom::PhantomStateShmemLayout*>(view);
	const bool ok =
	    layout->magic == phantom::kPhantomStateShmemMagic && layout->version == phantom::kPhantomStateShmemVersion;
	if (ok) {
		for (uint8_t i = 0; i < phantom::kBodyRoleCount; ++i) {
			roles[i] = layout->roles[i];
		}
	}
	::UnmapViewOfFile(view);
	::CloseHandle(h);
	return ok;
}

struct ReplayMetrics
{
	std::string hidden_role = "none";
	uint32_t frames = 0;
	uint32_t hidden_samples = 0;
	uint32_t hidden_outputs = 0;
	uint32_t hidden_out_of_range = 0;
	double hidden_error_sum_sq = 0.0;
	double hidden_error_max = 0.0;
	double hidden_recovery_latency_ms = -1.0;
	std::vector<phantom::BodyCompletionPose> hidden_truth_poses;
	std::vector<phantom::BodyCompletionPose> hidden_output_poses;
	uint32_t role_snapshots = 0;
	uint32_t role_valid_sum = 0;
	double role_confidence_sum = 0.0;
	std::vector<std::string> virtual_requested_roles;
	uint32_t virtual_activated_roles = 0;
	uint32_t virtual_pose_outputs = 0;
	std::map<std::string, uint32_t> virtual_outputs_by_role;
};

std::string FormatDouble(double v, int precision = 3)
{
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(precision) << v;
	return oss.str();
}

bool WriteReport(const std::filesystem::path& path, const PhantomReplayRecording& replay, const ReplayMetrics& m,
                 std::string& error)
{
	if (path.empty()) return true;
	std::ofstream out(path);
	if (!out) {
		error = "could not open replay report for writing";
		return false;
	}
	const double coverage = m.hidden_samples ? (double)m.hidden_outputs / (double)m.hidden_samples : 0.0;
	const double rms = m.hidden_outputs ? std::sqrt(m.hidden_error_sum_sq / (double)m.hidden_outputs) : 0.0;
	const auto hidden_error_stats =
	    phantom::analysis::ComputePoseErrorStats(m.hidden_output_poses, m.hidden_truth_poses);
	const auto hidden_continuity = phantom::analysis::ComputeContinuityStats(m.hidden_output_poses, 0.75, 120.0);
	const double valid_avg = m.role_snapshots ? (double)m.role_valid_sum / (double)m.role_snapshots : 0.0;
	const double conf_avg = m.role_snapshots ? m.role_confidence_sum / (double)m.role_snapshots : 0.0;
	out << "metric,value\n";
	out << "source_format," << replay.source_format << "\n";
	out << "devices," << replay.devices.size() << "\n";
	out << "samples," << replay.samples.size() << "\n";
	out << "duration_ms," << FormatDouble(replay.duration_ms) << "\n";
	out << "hidden_role," << m.hidden_role << "\n";
	out << "hidden_samples," << m.hidden_samples << "\n";
	out << "hidden_outputs," << m.hidden_outputs << "\n";
	out << "hidden_coverage," << FormatDouble(coverage, 4) << "\n";
	out << "hidden_rms_error_m," << FormatDouble(rms, 4) << "\n";
	out << "hidden_max_error_m," << FormatDouble(m.hidden_error_max, 4) << "\n";
	out << "hidden_orientation_rms_deg," << FormatDouble(hidden_error_stats.orientation_deg.rms, 3) << "\n";
	out << "hidden_orientation_max_deg," << FormatDouble(hidden_error_stats.orientation_deg.max, 3) << "\n";
	out << "hidden_continuity_max_step_m," << FormatDouble(hidden_continuity.max_step_m, 4) << "\n";
	out << "hidden_continuity_max_orientation_step_deg," << FormatDouble(hidden_continuity.max_orientation_step_deg, 3)
	    << "\n";
	out << "hidden_continuity_teleports," << hidden_continuity.teleport_count << "\n";
	out << "hidden_invalid_outputs," << hidden_continuity.invalid_count << "\n";
	out << "hidden_recovery_latency_ms," << FormatDouble(m.hidden_recovery_latency_ms, 3) << "\n";
	out << "hidden_out_of_range," << m.hidden_out_of_range << "\n";
	out << "completion_valid_roles_avg," << FormatDouble(valid_avg, 4) << "\n";
	out << "completion_confidence_avg," << FormatDouble(conf_avg, 4) << "\n";
	out << "virtual_requested_roles," << JoinRoleKeys(m.virtual_requested_roles) << "\n";
	out << "virtual_requested_count," << m.virtual_requested_roles.size() << "\n";
	out << "virtual_activated_roles," << m.virtual_activated_roles << "\n";
	out << "virtual_pose_outputs," << m.virtual_pose_outputs << "\n";
	for (const auto& kv : m.virtual_outputs_by_role) {
		out << "virtual_pose_outputs_" << kv.first << "," << kv.second << "\n";
	}
	return true;
}

} // namespace

ScenarioResult RunScenario_phantom_replay(ScenarioContext& ctx)
{
	const auto t0 = std::chrono::steady_clock::now();
	auto duration_now = [&]() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	};

	if (ctx.phantom_replay_path.empty()) {
		return Fail("phantom_replay", duration_now(), "pass --phantom-replay <path>");
	}

	const auto loaded = LoadPhantomReplay(ctx.phantom_replay_path);
	if (!loaded.ok) {
		return Fail("phantom_replay", duration_now(), loaded.error);
	}
	const auto& replay = loaded.recording;
	ctx.log.Info("loaded replay source=" + replay.source_format + " devices=" + std::to_string(replay.devices.size()) +
	             " samples=" + std::to_string(replay.samples.size()) +
	             " duration_ms=" + FormatDouble(replay.duration_ms));

	HarnessIpcClient client;
	try {
		client.OpenWithRetries(OPENVR_PAIRDRIVER_PHANTOM_PIPE_NAME);
	}
	catch (const std::exception& ex) {
		return Fail("phantom_replay", duration_now(), ex.what());
	}
	if (client.GetMismatchState() != HarnessIpcClient::MismatchState::Matching) {
		return Fail("phantom_replay", duration_now(),
		            "protocol version mismatch driver=" + std::to_string(client.GetDriverVersion()));
	}

	std::unordered_map<uint32_t, const PhantomReplayDevice*> devices_by_replay_id;
	std::unordered_map<uint32_t, uint32_t> openvr_by_replay_id;
	for (const auto& device : replay.devices) {
		devices_by_replay_id.emplace(device.replay_id, &device);
		const uint32_t openvr_id = ctx.pose_source.AddDevice(device.serial, device.device_class);
		if (openvr_id == UINT32_MAX) {
			return Fail("phantom_replay", duration_now(), "could not add replay device " + device.serial);
		}
		openvr_by_replay_id.emplace(device.replay_id, openvr_id);
		ctx.mock.properties().SeedString(openvr_id, vr::Prop_SerialNumber_String, device.serial);
		ctx.mock.properties().SeedInt32(openvr_id, vr::Prop_DeviceClass_Int32, (int32_t)device.device_class);
		ctx.mock.properties().SeedInt32(openvr_id, vr::Prop_ControllerRoleHint_Int32, (int32_t)device.controller_role);
	}

	std::string ipc_error;
	if (!ConfigurePhantom(client, replay, ipc_error)) {
		return Fail("phantom_replay", duration_now(), "phantom config failed: " + ipc_error);
	}

	phantom::BodyRole hidden_role = phantom::BodyRoleFromKey(ctx.phantom_replay_dropout_role.c_str());
	if (hidden_role == phantom::BodyRole::None) {
		for (const auto& device : replay.devices) {
			if (device.device_class == vr::TrackedDeviceClass_GenericTracker &&
			    device.body_role != phantom::BodyRole::None) {
				hidden_role = device.body_role;
				break;
			}
		}
	}

	double dropout_start = ctx.phantom_replay_dropout_start_ms;
	double dropout_end = ctx.phantom_replay_dropout_end_ms;
	if (dropout_start < 0.0) dropout_start = replay.duration_ms * 0.33;
	if (dropout_end < 0.0) dropout_end = replay.duration_ms * 0.75;
	if (dropout_end <= dropout_start) dropout_end = dropout_start + 250.0;

	ReplayMetrics metrics;
	metrics.hidden_role = phantom::BodyRoleToKey(hidden_role);
	const auto virtual_roles = EnableAbsentVirtualRoles(client, replay, ipc_error);
	if (!ipc_error.empty()) {
		return Fail("phantom_replay", duration_now(), "phantom virtual config failed: " + ipc_error);
	}
	for (const auto role : virtual_roles) {
		metrics.virtual_requested_roles.push_back(phantom::BodyRoleToKey(role));
	}
	if (!virtual_roles.empty()) {
		ctx.log.Info("enabled absent virtual roles=" + JoinRoleKeys(metrics.virtual_requested_roles));
		std::this_thread::sleep_for(std::chrono::milliseconds(3200));
	}
	if (hidden_role != phantom::BodyRole::None) {
		ctx.log.Info("withholding role=" + metrics.hidden_role + " start_ms=" + FormatDouble(dropout_start) +
		             " end_ms=" + FormatDouble(dropout_end));
	}
	else {
		ctx.log.Warn("no role-labeled GenericTracker found; replay will run without intentional dropout metrics");
	}

	size_t index = 0;
	double last_time = 0.0;
	uint32_t hidden_openvr_device_id = UINT32_MAX;
	for (const auto& device : replay.devices) {
		if (device.body_role != hidden_role) continue;
		auto it = openvr_by_replay_id.find(device.replay_id);
		if (it != openvr_by_replay_id.end()) {
			hidden_openvr_device_id = it->second;
			break;
		}
	}
	while (index < replay.samples.size()) {
		const double frame_time = replay.samples[index].time_ms;
		const double sleep_ms = std::max(0.0, (frame_time - last_time) / ctx.phantom_replay_speed);
		if (sleep_ms > 0.0) {
			std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
		}
		last_time = frame_time;
		++metrics.frames;

		size_t frame_end = index;
		while (frame_end < replay.samples.size() && std::abs(replay.samples[frame_end].time_ms - frame_time) < 0.001) {
			++frame_end;
		}

		uint64_t frame_cursor = ctx.mock.recorder().LatestSeq();
		bool hidden_this_frame = false;
		vr::DriverPose_t hidden_truth{};
		for (size_t i = index; i < frame_end; ++i) {
			const auto& sample = replay.samples[i];
			auto dev_it = devices_by_replay_id.find(sample.replay_device_id);
			auto id_it = openvr_by_replay_id.find(sample.replay_device_id);
			if (dev_it == devices_by_replay_id.end() || id_it == openvr_by_replay_id.end()) continue;
			const auto& device = *dev_it->second;
			const bool withhold = hidden_role != phantom::BodyRole::None && device.body_role == hidden_role &&
			                      frame_time >= dropout_start && frame_time <= dropout_end;
			if (withhold) {
				hidden_this_frame = true;
				hidden_truth = sample.pose;
				hidden_openvr_device_id = id_it->second;
				continue;
			}
			ctx.pose_source.PushPose(id_it->second, sample.pose);
		}

		if (hidden_this_frame) {
			++metrics.hidden_samples;
			const auto calls = ctx.mock.recorder().Snapshot();
			const MockCall* latest = nullptr;
			for (const auto& call : calls) {
				if (call.seq <= frame_cursor) continue;
				if (call.kind == MockCallKind::TrackedDevicePoseUpdated && call.device_id == hidden_openvr_device_id &&
				    call.has_pose) {
					latest = &call;
				}
			}
			if (latest) {
				++metrics.hidden_outputs;
				vr::DriverPose_t output = hidden_truth;
				output.vecPosition[0] = latest->pose_position[0];
				output.vecPosition[1] = latest->pose_position[1];
				output.vecPosition[2] = latest->pose_position[2];
				output.result = static_cast<vr::ETrackingResult>(latest->pose_tracking_result);
				const double err = PositionErrorM(output, hidden_truth);
				metrics.hidden_error_sum_sq += err * err;
				metrics.hidden_error_max = std::max(metrics.hidden_error_max, err);
				metrics.hidden_truth_poses.push_back(ToMetricPose(hidden_truth));
				metrics.hidden_output_poses.push_back(ToMetricPose(*latest));
				if (output.result == vr::TrackingResult_Running_OutOfRange) {
					++metrics.hidden_out_of_range;
				}
			}
		}
		else if (hidden_openvr_device_id != UINT32_MAX && metrics.hidden_recovery_latency_ms < 0.0 &&
		         frame_time > dropout_end) {
			const auto calls = ctx.mock.recorder().Snapshot();
			for (const auto& call : calls) {
				if (call.seq <= frame_cursor) continue;
				if (call.kind != MockCallKind::TrackedDevicePoseUpdated || call.device_id != hidden_openvr_device_id ||
				    !call.has_pose) {
					continue;
				}
				metrics.hidden_recovery_latency_ms = frame_time - dropout_end;
				break;
			}
		}

		std::array<phantom::PhantomRoleCompletionState, phantom::kBodyRoleCount> roles{};
		if (ReadRoleSnapshot(roles)) {
			uint32_t valid_roles = 0;
			double confidence = 0.0;
			for (const auto& role : roles) {
				if (role.valid) {
					++valid_roles;
					confidence += role.confidence;
				}
			}
			++metrics.role_snapshots;
			metrics.role_valid_sum += valid_roles;
			if (valid_roles > 0) {
				metrics.role_confidence_sum += confidence / static_cast<double>(valid_roles);
			}
		}

		index = frame_end;
	}

	{
		std::unordered_map<uint32_t, std::string> virtual_role_by_id;
		const auto calls = ctx.mock.recorder().Snapshot();
		for (const auto& call : calls) {
			if (call.kind != MockCallKind::TrackedDeviceAdded) continue;
			const std::string role = VirtualRoleFromSerial(call.text);
			if (role.empty()) continue;
			virtual_role_by_id[call.device_id] = role;
		}
		metrics.virtual_activated_roles = static_cast<uint32_t>(virtual_role_by_id.size());
		for (const auto& call : calls) {
			if (call.kind != MockCallKind::TrackedDevicePoseUpdated || !call.has_pose) continue;
			const auto it = virtual_role_by_id.find(call.device_id);
			if (it == virtual_role_by_id.end()) continue;
			++metrics.virtual_pose_outputs;
			++metrics.virtual_outputs_by_role[it->second];
		}
	}

	const double coverage =
	    metrics.hidden_samples ? (double)metrics.hidden_outputs / (double)metrics.hidden_samples : 0.0;
	const double rms =
	    metrics.hidden_outputs ? std::sqrt(metrics.hidden_error_sum_sq / (double)metrics.hidden_outputs) : 0.0;
	const auto hidden_error_stats =
	    phantom::analysis::ComputePoseErrorStats(metrics.hidden_output_poses, metrics.hidden_truth_poses);
	const auto hidden_continuity = phantom::analysis::ComputeContinuityStats(metrics.hidden_output_poses, 0.75, 120.0);
	const double valid_avg =
	    metrics.role_snapshots ? (double)metrics.role_valid_sum / (double)metrics.role_snapshots : 0.0;
	const double conf_avg = metrics.role_snapshots ? metrics.role_confidence_sum / (double)metrics.role_snapshots : 0.0;

	ctx.log.Info("dropout hidden_role=" + metrics.hidden_role + " samples=" + std::to_string(metrics.hidden_samples) +
	             " outputs=" + std::to_string(metrics.hidden_outputs) + " coverage=" + FormatDouble(coverage, 3) +
	             " rms_error_m=" + FormatDouble(rms, 4) + " max_error_m=" + FormatDouble(metrics.hidden_error_max, 4) +
	             " orient_rms_deg=" + FormatDouble(hidden_error_stats.orientation_deg.rms, 2) +
	             " teleports=" + std::to_string(hidden_continuity.teleport_count) +
	             " recovery_ms=" + FormatDouble(metrics.hidden_recovery_latency_ms, 1) +
	             " out_of_range=" + std::to_string(metrics.hidden_out_of_range));
	ctx.log.Info("completion snapshots=" + std::to_string(metrics.role_snapshots) +
	             " valid_roles_avg=" + FormatDouble(valid_avg, 3) + " confidence_avg=" + FormatDouble(conf_avg, 3));
	ctx.log.Info("virtual requested=" + std::to_string(metrics.virtual_requested_roles.size()) +
	             " activated=" + std::to_string(metrics.virtual_activated_roles) +
	             " pose_outputs=" + std::to_string(metrics.virtual_pose_outputs));

	std::string report_error;
	if (!WriteReport(ctx.phantom_replay_report_path, replay, metrics, report_error)) {
		return Fail("phantom_replay", duration_now(), report_error);
	}
	if (!ctx.phantom_replay_report_path.empty()) {
		ctx.log.Info("wrote report " + ctx.phantom_replay_report_path.string());
	}

	client.Close();

	if (metrics.hidden_samples > 0) {
		if (metrics.hidden_outputs == 0) {
			return Fail("phantom_replay", duration_now(), "dropout window produced no Phantom replacement poses");
		}
		if (coverage < 0.70) {
			return Fail("phantom_replay", duration_now(), "dropout replacement coverage below 70%");
		}
		if (rms > 0.35) {
			return Fail("phantom_replay", duration_now(), "dropout replacement RMS error above 0.35 m");
		}
		if (hidden_continuity.invalid_count > 0) {
			return Fail("phantom_replay", duration_now(), "dropout replacement produced non-finite output");
		}
		if (hidden_continuity.teleport_count > 0) {
			return Fail("phantom_replay", duration_now(), "dropout replacement continuity exceeded teleport threshold");
		}
	}
	if (!metrics.virtual_requested_roles.empty()) {
		if (metrics.virtual_activated_roles < metrics.virtual_requested_roles.size()) {
			return Fail("phantom_replay", duration_now(), "absent virtual roles did not all activate");
		}
		for (const auto& role : metrics.virtual_requested_roles) {
			if (metrics.virtual_outputs_by_role[role] == 0) {
				return Fail("phantom_replay", duration_now(), "absent virtual role produced no pose output: " + role);
			}
		}
	}

	return Pass("phantom_replay", duration_now());
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
