#pragma once

#include "LearningEngine.h"
#include "Profiles.h"
#include "SnapshotReader.h"

#include <cstdint>
#include <string>
#include <unordered_map>

struct InputHealthPathFamilyCounters
{
	uint64_t trigger_value = 0;
	uint64_t thumbstick_axis = 0;
	uint64_t trackpad_axis = 0;
	uint64_t force_sensor = 0;
	uint64_t grip_value = 0;
	uint64_t finger_capsense = 0;
	uint64_t controller_button = 0;
	uint64_t diagnostics_only = 0;
	uint64_t unsupported = 0;
};

struct InputHealthHealthSummarySnapshot
{
	bool overlay_started = false;
	bool ipc_connected = false;
	bool shmem_opened = false;
	uint64_t publish_tick = 0;
	uint64_t live_components = 0;
	uint64_t profiles_loaded = 0;
	InputHealthPathFamilyCounters path_families;
	LearningEngineStats learning;
	ProfileIoStats profile_io;
};

InputHealthPathFamilyCounters CountInputHealthPathFamilies(
	const std::unordered_map<uint64_t, SnapshotReader::Entry> &entries);

std::string BuildInputHealthHealthSummaryJson(
	const InputHealthHealthSummarySnapshot &snapshot);

bool WriteInputHealthHealthSummary(
	const InputHealthHealthSummarySnapshot &snapshot);
