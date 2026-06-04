#include "LearningEngine.h"

#include "IPCClient.h"
#include "Logging.h"
#include "Profiles.h"
#include "SnapshotReader.h"

#include "inputhealth/PathPolicy.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>

namespace {

uint64_t SteadyMicros()
{
	const auto now = std::chrono::steady_clock::now().time_since_epoch();
	return static_cast<uint64_t>(
		std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

uint64_t UnixSeconds()
{
	const auto now = std::chrono::system_clock::now().time_since_epoch();
	return static_cast<uint64_t>(
		std::chrono::duration_cast<std::chrono::seconds>(now).count());
}

std::string PathFromBody(const protocol::InputHealthSnapshotBody &b)
{
	size_t n = 0;
	while (n < protocol::INPUTHEALTH_PATH_LEN && b.path[n] != '\0') ++n;
	return std::string(b.path, b.path + n);
}

std::string KeyFor(uint64_t serial_hash, const std::string &path)
{
	char buf[32];
	snprintf(buf, sizeof(buf), "%016llx:", (unsigned long long)serial_hash);
	return std::string(buf) + path;
}

bool IsStickKind(uint8_t kind)
{
	return kind == protocol::InputHealthCompStickX ||
		kind == protocol::InputHealthCompStickY;
}

bool IsTriggerKind(uint8_t kind, inputhealth::PathFamily family)
{
	return kind == protocol::InputHealthCompScalarSingle
		&& inputhealth::IsTriggerRemapFamily(family);
}

uint8_t KindForBody(const protocol::InputHealthSnapshotBody &b)
{
	if (b.is_boolean) return protocol::InputHealthCompBoolean;
	if (b.axis_role == protocol::InputHealthCompStickX) return protocol::InputHealthCompStickX;
	if (b.axis_role == protocol::InputHealthCompStickY) return protocol::InputHealthCompStickY;
	return protocol::InputHealthCompScalarSingle;
}

const char *KindString(uint8_t kind)
{
	switch (kind) {
		case protocol::InputHealthCompStickX: return "stick_x";
		case protocol::InputHealthCompStickY: return "stick_y";
		case protocol::InputHealthCompBoolean: return "boolean";
		default: return "scalar_single";
	}
}

uint8_t KindFromString(const std::string &kind)
{
	if (kind == "stick_x") return protocol::InputHealthCompStickX;
	if (kind == "stick_y") return protocol::InputHealthCompStickY;
	if (kind == "boolean") return protocol::InputHealthCompBoolean;
	return protocol::InputHealthCompScalarSingle;
}

uint64_t ReadyThreshold(uint8_t kind)
{
	return kind == protocol::InputHealthCompBoolean ? 500ULL : 1024ULL;
}

std::string InputStem(const std::string &path)
{
	const size_t value = path.rfind("/value");
	if (value != std::string::npos && value + 6 == path.size()) {
		return path.substr(0, value);
	}
	const size_t force = path.rfind("/force");
	if (force != std::string::npos && force + 6 == path.size()) {
		return path.substr(0, force);
	}
	const size_t pressure = path.rfind("/pressure");
	if (pressure != std::string::npos && pressure + 9 == path.size()) {
		return path.substr(0, pressure);
	}
	const size_t slash = path.rfind('/');
	return slash == std::string::npos ? path : path.substr(0, slash);
}

bool IsPeerPathForStem(const std::string &path, const std::string &stem)
{
	return !stem.empty() && path.rfind(stem + "/", 0) == 0;
}

struct IdlePeerEvidence
{
	bool peer_present = false;
	bool peer_idle = true;
};

IdlePeerEvidence CheckIdleFloorPeers(
	const std::unordered_map<uint64_t, SnapshotReader::Entry> &entries,
	const protocol::InputHealthSnapshotBody &current,
	const std::string &currentPath)
{
	IdlePeerEvidence evidence;
	const std::string stem = InputStem(currentPath);
	for (const auto &kv : entries) {
		const auto &peer = kv.second.body;
		if (peer.device_serial_hash != current.device_serial_hash) continue;
		const std::string peerPath = PathFromBody(peer);
		if (peerPath.empty() || peerPath == currentPath) continue;
		if (!IsPeerPathForStem(peerPath, stem)) continue;

		const inputhealth::PathFamily peerFamily =
			inputhealth::ClassifyPathFamily(peerPath);
		if (peer.is_boolean &&
			(peerPath.find("/click") != std::string::npos ||
			 peerPath.find("/touch") != std::string::npos))
		{
			evidence.peer_present = true;
			if (peer.last_boolean) evidence.peer_idle = false;
		} else if (peer.is_scalar &&
			inputhealth::IsIdleFloorFamily(peerFamily))
		{
			evidence.peer_present = true;
			if (std::fabs(peer.last_value) > inputhealth::kStrictRestThreshold) {
				evidence.peer_idle = false;
			}
		}
	}
	return evidence;
}

double ClampDouble(double value, double lo, double hi)
{
	return std::max(lo, std::min(value, hi));
}

LearnedPathRecord *FindRecord(DeviceProfile &profile, const std::string &path)
{
	for (auto &record : profile.learned_paths) {
		if (record.path == path) return &record;
	}
	return nullptr;
}

const LearnedPathRecord *FindRecord(const DeviceProfile &profile, const std::string &path)
{
	for (const auto &record : profile.learned_paths) {
		if (record.path == path) return &record;
	}
	return nullptr;
}

std::string FormatCorrection(uint8_t kind, const std::string &path,
	double offset, double triggerMin, double triggerMax, double deadzone,
	uint32_t debounceUs, bool ready)
{
	if (!ready) return "-";
	char buf[96];
	const inputhealth::PathFamily family = inputhealth::ClassifyPathFamily(path);
	if (kind == protocol::InputHealthCompBoolean) {
		snprintf(buf, sizeof(buf), "debounce=%.1fms", debounceUs / 1000.0);
	} else if (IsTriggerKind(kind, family)) {
		snprintf(buf, sizeof(buf), "min=%.3f max=%.3f", triggerMin, triggerMax);
	} else if (inputhealth::IsIdleFloorFamily(family)) {
		snprintf(buf, sizeof(buf), "floor=%.4f", offset);
	} else if (IsStickKind(kind)) {
		snprintf(buf, sizeof(buf), "rest=%.4f dead=%.3f", offset, deadzone);
	} else {
		snprintf(buf, sizeof(buf), "rest=%.4f", offset);
	}
	return buf;
}

void CopyPath(char (&dst)[protocol::INPUTHEALTH_PATH_LEN], const std::string &path)
{
	std::memset(dst, 0, sizeof(dst));
	const size_t n = std::min<size_t>(path.size(), protocol::INPUTHEALTH_PATH_LEN - 1);
	if (n > 0) std::memcpy(dst, path.data(), n);
}

} // namespace

LearningEngine::LearningEngine(IPCClient &ipc, ProfileStore &profiles)
	: ipc_(ipc), profiles_(profiles)
{
}

LearningEngine::PathState &LearningEngine::StateFor(uint64_t serial_hash, const std::string &path)
{
	auto &state = states_[KeyFor(serial_hash, path)];
	if (state.serial_hash == 0) {
		state.serial_hash = serial_hash;
		state.path = path;
		auto &profile = profiles_.GetOrCreate(serial_hash);
		if (auto *record = FindRecord(profile, path)) {
			state.kind = KindFromString(record->kind);
			state.sample_count = record->sample_count;
			state.ready = record->ready;
			state.learned_rest_offset = record->learned_rest_offset;
			state.learned_stddev = record->learned_stddev;
			state.learned_trigger_min = record->learned_trigger_min;
			state.learned_trigger_max = record->learned_trigger_max;
			state.learned_deadzone_radius = record->learned_deadzone_radius;
			state.learned_debounce_us = record->learned_debounce_us;
			state.last_updated_unix = record->last_updated_unix;
			state.drift_shift_resets = record->drift_shift_resets;
			if (record->sample_count > 1) {
				state.rest.count = record->sample_count;
				state.rest.mean = record->learned_rest_offset;
				state.rest.m2 = record->learned_stddev * record->learned_stddev *
					static_cast<double>(record->sample_count - 1);
			}
		}
	}
	return state;
}

const LearningEngine::PathState *LearningEngine::FindState(uint64_t serial_hash, const std::string &path) const
{
	auto it = states_.find(KeyFor(serial_hash, path));
	return it == states_.end() ? nullptr : &it->second;
}

void LearningEngine::SetRestBurstActive(bool active)
{
	rest_burst_active_ = active;
}

void LearningEngine::Tick(const SnapshotReader &reader)
{
	const auto &entries = reader.EntriesByHandle();
	if (entries.empty()) return;

	const uint64_t now_us = SteadyMicros();
	const uint64_t quiet_window_us = 500000;

	for (const auto &kv : entries) {
		const auto &b = kv.second.body;
		if (!b.is_boolean || b.device_serial_hash == 0) continue;
		const std::string path = PathFromBody(b);
		if (path.empty()) continue;

		const inputhealth::PathFamily family = inputhealth::ClassifyPathFamily(path);
		if (family == inputhealth::PathFamily::Unsupported) {
			WarnUnsupportedOnce("boolean", path);
			continue;
		}
		if (!inputhealth::AllowsPersistentBooleanLearning(family)) {
			// DiagnosticsOnly: observable in the UI but not learned into compensation.
			++stats_.diagnostic_only_samples;
			++stats_.diagnostic_path_counts[path];
			continue;
		}

		auto &state = StateFor(b.device_serial_hash, path);
		state.kind = protocol::InputHealthCompBoolean;

		if (!inputhealth::IsSystemButtonPath(path) &&
			b.bounce_transition_count >= inputhealth::kButtonBounceReadyTransitions &&
			b.bounce_max_interval_us > 0)
		{
			const uint32_t learned =
				inputhealth::DebounceFromBounceInterval(b.bounce_max_interval_us);
			if (!state.ready || learned > state.learned_debounce_us) {
				state.learned_debounce_us = learned;
				state.ready = true;
				state.last_updated_unix = UnixSeconds();
				state.sample_count = std::max<uint64_t>(
					state.sample_count,
					std::max<uint64_t>(b.press_count, b.bounce_transition_count));
				SyncProfile(b.device_serial_hash, state, true, "button_ready");
				PushCompensation(b.device_serial_hash, state, true);
			}
		}

		if (b.press_count > state.last_press_count) {
			if (state.last_press_time_us != 0) {
				const uint64_t interval = now_us - state.last_press_time_us;
				if (state.inter_press_us.size() >= 1024) {
					state.inter_press_us.erase(state.inter_press_us.begin());
				}
				state.inter_press_us.push_back(interval);
			}
			state.last_press_time_us = now_us;
			state.last_press_count = b.press_count;
			state.sample_count = std::max<uint64_t>(state.sample_count, b.press_count);
			device_button_quiet_until_us_[b.device_serial_hash] = now_us + quiet_window_us;

			if (!state.ready && state.sample_count >= ReadyThreshold(state.kind) && !state.inter_press_us.empty()) {
				auto samples = state.inter_press_us;
				std::sort(samples.begin(), samples.end());
				const size_t idx = samples.size() > 1
					? static_cast<size_t>((samples.size() - 1) * 0.01)
					: 0;
				const uint64_t learned = std::max<uint64_t>(1000,
					std::min<uint64_t>(20000, samples[idx]));
				state.learned_debounce_us = static_cast<uint32_t>(learned);
				state.ready = true;
				state.last_updated_unix = UnixSeconds();
				SyncProfile(b.device_serial_hash, state, true, "button_ready");
				PushCompensation(b.device_serial_hash, state, true);
			} else {
				SyncProfile(b.device_serial_hash, state, false, "periodic_material");
			}
		}
	}

	for (const auto &kv : entries) {
		const auto &b = kv.second.body;
		if (!b.is_scalar || b.device_serial_hash == 0) continue;
		const std::string path = PathFromBody(b);
		if (path.empty()) continue;

		const inputhealth::PathFamily family = inputhealth::ClassifyPathFamily(path);
		if (family == inputhealth::PathFamily::Unsupported) {
			WarnUnsupportedOnce("scalar", path);
			continue;
		}
		if (!inputhealth::AllowsPersistentScalarLearning(family)) {
			// DiagnosticsOnly: visible in diagnostics UI but not pushed into compensation.
			if (const auto *oldState = FindState(b.device_serial_hash, path)) {
				if (oldState->ready) {
					++stats_.drift_suppressed_policy;
					LOG("[inputhealth] diagnostic-only scalar ignored: serial=0x%016llx path='%s' family=%s",
						(unsigned long long)b.device_serial_hash,
						path.c_str(),
						inputhealth::PathFamilyName(family));
				}
			}
			++stats_.diagnostic_only_samples;
			++stats_.diagnostic_path_counts[path];
			continue;
		}

		const bool isIdleFloor = inputhealth::IsIdleFloorFamily(family);
		IdlePeerEvidence idlePeers;
		if (isIdleFloor) {
			idlePeers = CheckIdleFloorPeers(entries, b, path);
			if (!idlePeers.peer_present) {
				++stats_.diagnostic_only_samples;
				++stats_.diagnostic_path_counts[path];
				continue;
			}
		}

		auto &state = StateFor(b.device_serial_hash, path);
		state.kind = KindForBody(b);
		const bool isTrigger = IsTriggerKind(state.kind, family);
		const bool isStick = inputhealth::IsThumbstickAxisFamily(family) &&
			IsStickKind(state.kind);
		if (!inputhealth::ScalarMetadataAllowsLearning(
				family, path, state.kind, b.scalar_type, b.scalar_units)) {
			continue;
		}

		if (isTrigger) {
			state.trigger_peak = std::max<double>(state.trigger_peak, b.last_value);
			if (b.scalar_range_initialized) {
				state.trigger_peak = std::max<double>(state.trigger_peak, b.observed_max);
			}
		}

		bool rest = false;
		const auto quietIt = device_button_quiet_until_us_.find(b.device_serial_hash);
		const bool buttonsQuiet = quietIt == device_button_quiet_until_us_.end() ||
			now_us >= quietIt->second;
		if (isStick) {
			auto partnerIt = entries.find(b.partner_handle);
			if (partnerIt != entries.end()) {
				const float partnerValue = partnerIt->second.body.last_value;
				const bool strictRest = inputhealth::IsStrictStickRest(
					b.last_value, partnerValue, buttonsQuiet);
				const bool stableRest = inputhealth::UpdateStableRestWindow(
					state.stable_rest,
					inputhealth::IsStableStickRestCandidate(
						b.last_value, partnerValue, buttonsQuiet),
					b.last_value, partnerValue, now_us);
				rest = strictRest || stableRest;
			} else {
				inputhealth::ResetStableRestWindow(state.stable_rest);
			}
		} else if (isTrigger || isIdleFloor) {
			const bool strictRest = inputhealth::IsStrictTriggerRest(b.last_value);
			const bool stableRest = inputhealth::UpdateStableRestWindow(
				state.stable_rest,
				inputhealth::IsStableTriggerRestCandidate(
					b.last_value, buttonsQuiet && (!isIdleFloor || idlePeers.peer_idle)),
				b.last_value, 0.0f, now_us);
			rest = isIdleFloor
				? (stableRest && idlePeers.peer_idle)
				: (strictRest || stableRest);
		} else {
			rest = std::fabs(b.last_value) < 0.05f && buttonsQuiet;
		}

		if (!rest) continue;

		state.rest_credit += rest_burst_active_ ? 1.0 : 0.5;
		while (state.rest_credit >= 1.0) {
			inputhealth::WelfordUpdate(state.rest, static_cast<double>(b.last_value));
			state.rest_credit -= 1.0;
		}
		state.sample_count = state.rest.count;

		if (!state.short_mean_initialized) {
			state.short_mean = b.last_value;
			state.short_mean_initialized = true;
		} else {
			state.short_mean += 0.05 * (static_cast<double>(b.last_value) - state.short_mean);
		}

		if (state.ready) {
			const double gate = 3.0 * std::max(0.001, state.learned_stddev);
			if (now_us >= state.drift_cooldown_until_us &&
				std::fabs(state.short_mean - state.learned_rest_offset) > gate) {
				if (state.drift_exceeded_since_us == 0) state.drift_exceeded_since_us = now_us;
				if (now_us - state.drift_exceeded_since_us >= 10000000ULL) {
					state.ready = false;
					state.drift_shift_pending = true;
					state.drift_exceeded_since_us = 0;
					state.drift_cooldown_until_us = now_us + 60000000ULL;
					++state.drift_shift_resets;
					++stats_.drift_resets;
					state.sample_count = 0;
					state.rest_credit = 0.0;
					inputhealth::WelfordReset(state.rest);
					state.short_mean_initialized = false;
					state.learned_rest_offset = 0.0;
					state.learned_stddev = 0.0;
					state.learned_trigger_min = 0.0;
					state.learned_trigger_max = 0.0;
					state.learned_deadzone_radius = 0.0;
					state.learned_debounce_us = 0;
					state.last_updated_unix = UnixSeconds();
					SyncProfile(b.device_serial_hash, state, true, "drift_relearn");
					PushCompensation(b.device_serial_hash, state, false);
					LOG("[inputhealth] drift-shift detected on serial=0x%016llx path='%s' -> relearning",
						(unsigned long long)b.device_serial_hash, path.c_str());
				}
			} else {
				state.drift_exceeded_since_us = 0;
				state.drift_shift_pending = false;
			}
			continue;
		}

		const double stddev = inputhealth::SampleStdDev(state.rest);
		const double limit = isTrigger ? 0.05 : 0.02;
		if (state.rest.count >= ReadyThreshold(state.kind) && stddev < limit) {
			// Only treat this as an immediate-save trigger on the actual
			// not-ready -> ready transition. Without this guard the branch
			// re-fires every tick once the threshold holds, bypassing
			// SyncProfile's 5 s throttle and writing the profile to disk
			// per-tick per-path (one session log showed 25k saves).
			const bool was_not_ready = !state.ready;
			state.ready = true;
			state.drift_shift_pending = false;
			state.learned_rest_offset = state.rest.mean;
			state.learned_stddev = stddev;
			if (isTrigger) {
				state.learned_trigger_min = ClampDouble(state.rest.mean, 0.0, 0.10);
				state.learned_trigger_max = state.trigger_peak > 0.10 ? state.trigger_peak : 1.0;
			} else if (isIdleFloor) {
				state.learned_rest_offset = ClampDouble(state.rest.mean, 0.0, 0.05);
				state.learned_trigger_min = 0.0;
				state.learned_trigger_max = 0.0;
				state.learned_deadzone_radius = 0.0;
			}
			if (isStick) {
				state.learned_deadzone_radius = ClampDouble(stddev * 3.0, 0.01, 0.08);
			}
			state.last_updated_unix = UnixSeconds();
			if (was_not_ready) ++stats_.ready_transitions;
			SyncProfile(b.device_serial_hash, state, was_not_ready, "ready_transition");
			PushCompensation(b.device_serial_hash, state, true);
		} else {
			SyncProfile(b.device_serial_hash, state, false, "periodic_material");
		}
	}
}

void LearningEngine::SyncProfile(
	uint64_t serial_hash,
	PathState &state,
	bool immediate,
	const char *reason)
{
	++stats_.profile_sync_attempts;
	const uint64_t now_us = SteadyMicros();
	auto &profile = profiles_.GetOrCreate(serial_hash);
	auto *record = FindRecord(profile, state.path);
	bool materialChange = false;
	if (!record) {
		LearnedPathRecord fresh;
		fresh.path = state.path;
		profile.learned_paths.push_back(std::move(fresh));
		record = &profile.learned_paths.back();
		materialChange = true;
	} else {
		LearnedPathRecord updated = *record;
		updated.kind = KindString(state.kind);
		updated.ready = state.ready;
		updated.learned_rest_offset = state.learned_rest_offset;
		updated.learned_stddev = state.learned_stddev;
		updated.learned_trigger_min = state.learned_trigger_min;
		updated.learned_trigger_max = state.learned_trigger_max;
		updated.learned_deadzone_radius = state.learned_deadzone_radius;
		updated.learned_debounce_us = state.learned_debounce_us;
		updated.last_updated_unix = state.last_updated_unix;
		updated.drift_shift_resets = state.drift_shift_resets;
		materialChange =
			!LearnedPathMaterialEqual(*record, updated);
	}

	record->kind = KindString(state.kind);
	record->sample_count = state.sample_count;
	record->ready = state.ready;
	record->learned_rest_offset = state.learned_rest_offset;
	record->learned_stddev = state.learned_stddev;
	record->learned_trigger_min = state.learned_trigger_min;
	record->learned_trigger_max = state.learned_trigger_max;
	record->learned_deadzone_radius = state.learned_deadzone_radius;
	record->learned_debounce_us = state.learned_debounce_us;
	record->last_updated_unix = state.last_updated_unix;
	record->drift_shift_resets = state.drift_shift_resets;

	if (!immediate && !materialChange) {
		++stats_.profile_sync_skipped_sample_churn;
		return;
	}

	if (!immediate) {
		auto &lastDeviceSave = device_last_periodic_save_us_[serial_hash];
		if (lastDeviceSave != 0 && now_us - lastDeviceSave < 60000000ULL) {
			++stats_.profile_sync_skipped_periodic_throttle;
			return;
		}
	}

	if (TrySaveProfile(profile, reason)) {
		state.last_persist_us = now_us;
		if (!immediate) {
			device_last_periodic_save_us_[serial_hash] = now_us;
		}
	}
}

void LearningEngine::PushCompensation(uint64_t serial_hash, const PathState &state, bool enabled)
{
	const inputhealth::PathFamily family = inputhealth::ClassifyPathFamily(state.path);
	if (enabled &&
		(!inputhealth::AllowsDriverCompensation(family) ||
		 (state.kind == protocol::InputHealthCompBoolean &&
		  inputhealth::IsSystemButtonPath(state.path))))
	{
		++stats_.compensation_push_rejected;
		LOG("[inputhealth] compensation push suppressed by policy: serial=0x%016llx path='%s' family=%s",
			(unsigned long long)serial_hash,
			state.path.c_str(),
			inputhealth::PathFamilyName(family));
		return;
	}

	if (!ipc_.IsConnected()) return;
	++stats_.compensation_push_attempts;
	bool actualEnabled = enabled;
	if (actualEnabled) {
		const auto profilesIt = profiles_.All().find(serial_hash);
		if (profilesIt != profiles_.All().end() && !profilesIt->second.corrections_enabled) {
			actualEnabled = false;
		}
	}

	protocol::InputHealthCompensationEntry entry{};
	entry.device_serial_hash = serial_hash;
	CopyPath(entry.path, state.path);
	entry.kind = state.kind;
	entry.enabled = actualEnabled ? 1 : 0;
	entry.learned_rest_offset = static_cast<float>(state.learned_rest_offset);
	entry.learned_trigger_min = static_cast<float>(state.learned_trigger_min);
	entry.learned_trigger_max = static_cast<float>(state.learned_trigger_max);
	entry.learned_deadzone_radius = static_cast<float>(state.learned_deadzone_radius);
	entry.learned_debounce_us = static_cast<uint16_t>(
		std::min<uint32_t>(65535, state.learned_debounce_us));

	try {
		auto resp = ipc_.SendCompensationEntry(entry);
		if (resp.type != protocol::ResponseSuccess) {
			++stats_.compensation_push_rejected;
			LOG("[inputhealth] compensation push rejected: serial=0x%016llx path='%s' type=%d",
				(unsigned long long)serial_hash, state.path.c_str(), (int)resp.type);
		} else {
			++stats_.compensation_push_success;
		}
	} catch (const std::exception &e) {
		++stats_.compensation_push_failed;
		LOG("[inputhealth] compensation push failed: serial=0x%016llx path='%s' err=%s",
			(unsigned long long)serial_hash, state.path.c_str(), e.what());
	}
}

void LearningEngine::SetDeviceCorrectionsEnabled(uint64_t serial_hash, bool enabled)
{
	if (serial_hash == 0) return;

	auto &profile = profiles_.GetOrCreate(serial_hash);
	if (profile.corrections_enabled == enabled) return;

	profile.corrections_enabled = enabled;
	TrySaveProfile(profile, "corrections_toggle");

	for (const auto &record : profile.learned_paths) {
		if (enabled && !record.ready) continue;
		const inputhealth::PathFamily family =
			inputhealth::ClassifyPathFamily(record.path);
		if (enabled && !inputhealth::AllowsDriverCompensation(family)) continue;

		PathState state;
		state.serial_hash = profile.serial_hash;
		state.path = record.path;
		state.kind = KindFromString(record.kind);
		state.ready = record.ready;
		state.learned_rest_offset = record.learned_rest_offset;
		state.learned_stddev = record.learned_stddev;
		state.learned_trigger_min = record.learned_trigger_min;
		state.learned_trigger_max = record.learned_trigger_max;
		state.learned_deadzone_radius = record.learned_deadzone_radius;
		state.learned_debounce_us = record.learned_debounce_us;
		PushCompensation(profile.serial_hash, state, enabled && record.ready);
	}
}

void LearningEngine::PushReadyCompensations()
{
	for (const auto &profileKv : profiles_.All()) {
		const auto &profile = profileKv.second;
		if (!profile.corrections_enabled) continue;
		for (const auto &record : profile.learned_paths) {
			if (!record.ready) continue;
			const inputhealth::PathFamily family =
				inputhealth::ClassifyPathFamily(record.path);
			if (!inputhealth::AllowsDriverCompensation(family)) continue;
			PathState state;
			state.serial_hash = profile.serial_hash;
			state.path = record.path;
			state.kind = KindFromString(record.kind);
			state.ready = true;
			state.learned_rest_offset = record.learned_rest_offset;
			state.learned_stddev = record.learned_stddev;
			state.learned_trigger_min = record.learned_trigger_min;
			state.learned_trigger_max = record.learned_trigger_max;
			state.learned_deadzone_radius = record.learned_deadzone_radius;
			state.learned_debounce_us = record.learned_debounce_us;
			PushCompensation(profile.serial_hash, state, true);
		}
	}
}

LearningPathView LearningEngine::GetPathView(uint64_t serial_hash, const char *pathRaw) const
{
	const std::string path = pathRaw ? pathRaw : "";
	LearningPathView view;

	const auto profilesIt = profiles_.All().find(serial_hash);
	if (profilesIt != profiles_.All().end()) {
		view.corrections_enabled = profilesIt->second.corrections_enabled;
	}

	if (const auto *state = FindState(serial_hash, path)) {
		view.sample_count = state->sample_count;
		view.threshold = ReadyThreshold(state->kind);
		view.ready = state->ready;
		view.drift_shift_pending = state->drift_shift_pending;
		view.last_updated_unix = state->last_updated_unix;
		view.correction = FormatCorrection(state->kind, state->path,
			state->learned_rest_offset, state->learned_trigger_min,
			state->learned_trigger_max, state->learned_deadzone_radius,
			state->learned_debounce_us, state->ready);
	} else if (profilesIt != profiles_.All().end()) {
		if (const auto *record = FindRecord(profilesIt->second, path)) {
			const uint8_t kind = KindFromString(record->kind);
			view.sample_count = record->sample_count;
			view.threshold = ReadyThreshold(kind);
			view.ready = record->ready;
			view.last_updated_unix = record->last_updated_unix;
			view.correction = FormatCorrection(kind, record->path,
				record->learned_rest_offset, record->learned_trigger_min,
				record->learned_trigger_max, record->learned_deadzone_radius,
				record->learned_debounce_us, record->ready);
		}
	}

	if (!view.corrections_enabled) view.status = "off";
	else if (view.drift_shift_pending) view.status = "drift-shift";
	else if (view.ready) view.status = "ready";
	else view.status = "learning";
	if (view.threshold == 0) view.threshold = 1;
	if (view.correction.empty()) view.correction = "-";
	return view;
}

void LearningEngine::UnlearnPath(uint64_t serial_hash, const char *pathRaw)
{
	const std::string path = pathRaw ? pathRaw : "";
	if (serial_hash == 0 || path.empty()) return;
	PathState clearState;
	clearState.serial_hash = serial_hash;
	clearState.path = path;

	auto &profile = profiles_.GetOrCreate(serial_hash);
	profile.learned_paths.erase(
		std::remove_if(profile.learned_paths.begin(), profile.learned_paths.end(),
			[&](const LearnedPathRecord &record) {
				if (record.path != path) return false;
				clearState.kind = KindFromString(record.kind);
				return true;
			}),
		profile.learned_paths.end());
	TrySaveProfile(profile, "manual_unlearn");
	states_.erase(KeyFor(serial_hash, path));
	PushCompensation(serial_hash, clearState, false);
}

void LearningEngine::UnlearnDevice(uint64_t serial_hash)
{
	if (serial_hash == 0) return;
	auto &profile = profiles_.GetOrCreate(serial_hash);
	for (const auto &record : profile.learned_paths) {
		PathState clearState;
		clearState.serial_hash = serial_hash;
		clearState.path = record.path;
		clearState.kind = KindFromString(record.kind);
		PushCompensation(serial_hash, clearState, false);
	}
	profile.learned_paths.clear();
	TrySaveProfile(profile, "manual_unlearn");

	for (auto it = states_.begin(); it != states_.end(); ) {
		if (it->second.serial_hash == serial_hash) it = states_.erase(it);
		else ++it;
	}
}

bool LearningEngine::TrySaveProfile(const DeviceProfile &profile, const char *reason)
{
	stats_.last_profile_save_reason = reason ? reason : "";
	try {
		return profiles_.Save(profile, reason);
	} catch (const std::exception &e) {
		LOG("[inputhealth] profile save failed for serial_hash=0x%016llx: %s",
			(unsigned long long)profile.serial_hash, e.what());
	} catch (...) {
		LOG("[inputhealth] profile save failed for serial_hash=0x%016llx: unknown exception",
			(unsigned long long)profile.serial_hash);
	}
	return false;
}

void LearningEngine::WarnUnsupportedOnce(const char *kind, const std::string &path)
{
	if (warned_unsupported_paths_.insert(path).second) {
		LOG("[inputhealth] unsupported path ignored (%s, logged once): '%s'",
			kind, path.c_str());
	}
}

void LearningEngine::Flush()
{
	for (auto &kv : states_) {
		if (kv.second.serial_hash != 0) {
			SyncProfile(kv.second.serial_hash, kv.second, true, "shutdown_flush");
		}
	}
}
