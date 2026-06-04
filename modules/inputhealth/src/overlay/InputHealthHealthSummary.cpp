#include "InputHealthHealthSummary.h"

#include "Win32Paths.h"
#include "Win32Text.h"

#include "inputhealth/PathPolicy.h"
#include "picojson.h"

#include <algorithm>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace {

std::string PathFromBody(const protocol::InputHealthSnapshotBody &b)
{
	size_t n = 0;
	while (n < protocol::INPUTHEALTH_PATH_LEN && b.path[n] != '\0') ++n;
	return std::string(b.path, b.path + n);
}

void IncrementFamily(InputHealthPathFamilyCounters &c, inputhealth::PathFamily family)
{
	switch (family) {
		case inputhealth::PathFamily::TriggerValue:     ++c.trigger_value; break;
		case inputhealth::PathFamily::ThumbstickAxis:   ++c.thumbstick_axis; break;
		case inputhealth::PathFamily::TrackpadAxis:     ++c.trackpad_axis; break;
		case inputhealth::PathFamily::ForceSensor:      ++c.force_sensor; break;
		case inputhealth::PathFamily::GripValue:        ++c.grip_value; break;
		case inputhealth::PathFamily::FingerCapsense:   ++c.finger_capsense; break;
		case inputhealth::PathFamily::ControllerButton: ++c.controller_button; break;
		case inputhealth::PathFamily::DiagnosticsOnly:  ++c.diagnostics_only; break;
		case inputhealth::PathFamily::Unsupported:      ++c.unsupported; break;
	}
}

picojson::value CountValue(uint64_t value)
{
	return picojson::value(static_cast<double>(value));
}

picojson::object PathFamilyObject(const InputHealthPathFamilyCounters &c)
{
	picojson::object obj;
	obj["trigger_value"] = CountValue(c.trigger_value);
	obj["thumbstick_axis"] = CountValue(c.thumbstick_axis);
	obj["trackpad_axis"] = CountValue(c.trackpad_axis);
	obj["force_sensor"] = CountValue(c.force_sensor);
	obj["grip_value"] = CountValue(c.grip_value);
	obj["finger_capsense"] = CountValue(c.finger_capsense);
	obj["controller_button"] = CountValue(c.controller_button);
	obj["diagnostics_only"] = CountValue(c.diagnostics_only);
	obj["unsupported"] = CountValue(c.unsupported);
	return obj;
}

picojson::array TopPaths(const std::unordered_map<std::string, uint64_t> &counts)
{
	std::vector<std::pair<std::string, uint64_t>> items;
	items.reserve(counts.size());
	for (const auto &kv : counts) {
		items.emplace_back(kv.first, kv.second);
	}
	std::sort(items.begin(), items.end(),
		[](const auto &a, const auto &b) {
			if (a.second != b.second) return a.second > b.second;
			return a.first < b.first;
		});

	picojson::array out;
	const size_t limit = std::min<size_t>(items.size(), 8);
	for (size_t i = 0; i < limit; ++i) {
		picojson::object item;
		item["path"] = picojson::value(items[i].first);
		item["count"] = CountValue(items[i].second);
		out.push_back(picojson::value(item));
	}
	return out;
}

} // namespace

InputHealthPathFamilyCounters CountInputHealthPathFamilies(
	const std::unordered_map<uint64_t, SnapshotReader::Entry> &entries)
{
	InputHealthPathFamilyCounters counters;
	for (const auto &kv : entries) {
		const std::string path = PathFromBody(kv.second.body);
		IncrementFamily(counters, inputhealth::ClassifyPathFamily(path));
	}
	return counters;
}

std::string BuildInputHealthHealthSummaryJson(
	const InputHealthHealthSummarySnapshot &snapshot)
{
	picojson::object root;
	root["schema"] = picojson::value(std::string("inputhealth_health_summary.v1"));

	picojson::object pipeline;
	pipeline["overlay_started"] = picojson::value(snapshot.overlay_started);
	pipeline["ipc_connected"] = picojson::value(snapshot.ipc_connected);
	pipeline["shmem_opened"] = picojson::value(snapshot.shmem_opened);
	pipeline["publish_tick"] = CountValue(snapshot.publish_tick);
	pipeline["live_components"] = CountValue(snapshot.live_components);
	root["pipeline"] = picojson::value(pipeline);

	root["path_families"] = picojson::value(PathFamilyObject(snapshot.path_families));

	picojson::object learning;
	learning["diagnostic_only_samples"] = CountValue(snapshot.learning.diagnostic_only_samples);
	learning["drift_suppressed_policy"] = CountValue(snapshot.learning.drift_suppressed_policy);
	learning["drift_resets"] = CountValue(snapshot.learning.drift_resets);
	learning["ready_transitions"] = CountValue(snapshot.learning.ready_transitions);
	learning["compensation_push_attempts"] = CountValue(snapshot.learning.compensation_push_attempts);
	learning["compensation_push_success"] = CountValue(snapshot.learning.compensation_push_success);
	learning["compensation_push_rejected"] = CountValue(snapshot.learning.compensation_push_rejected);
	learning["compensation_push_failed"] = CountValue(snapshot.learning.compensation_push_failed);
	learning["profile_sync_attempts"] = CountValue(snapshot.learning.profile_sync_attempts);
	learning["profile_sync_skipped_sample_churn"] =
		CountValue(snapshot.learning.profile_sync_skipped_sample_churn);
	learning["profile_sync_skipped_periodic_throttle"] =
		CountValue(snapshot.learning.profile_sync_skipped_periodic_throttle);
	learning["last_profile_save_reason"] =
		picojson::value(snapshot.learning.last_profile_save_reason);
	learning["top_drift_paths"] =
		picojson::value(TopPaths(snapshot.learning.diagnostic_path_counts));
	root["learning"] = picojson::value(learning);

	picojson::object profileIo;
	profileIo["profiles_loaded"] = CountValue(snapshot.profiles_loaded);
	profileIo["attempted_saves"] = CountValue(snapshot.profile_io.attempted_saves);
	profileIo["skipped_unchanged"] = CountValue(snapshot.profile_io.skipped_unchanged);
	profileIo["actual_writes"] = CountValue(snapshot.profile_io.actual_writes);
	profileIo["failed_writes"] = CountValue(snapshot.profile_io.failed_writes);
	profileIo["last_save_reason"] = picojson::value(snapshot.profile_io.last_save_reason);
	root["profile_io"] = picojson::value(profileIo);

	return picojson::value(root).serialize(true);
}

bool WriteInputHealthHealthSummary(
	const InputHealthHealthSummarySnapshot &snapshot)
{
	const std::wstring logs = openvr_pair::common::WkOpenVrLogsPath(true);
	if (logs.empty()) return false;
	const std::wstring path = logs + L"\\inputhealth_health_summary.json";
	std::ofstream out(path, std::ios::out | std::ios::trunc);
	if (!out.is_open()) return false;
	out << BuildInputHealthHealthSummaryJson(snapshot);
	out << "\n";
	return out.good();
}
