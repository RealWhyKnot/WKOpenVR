#include "DynamicResolutionProfile.h"

#include "Win32Paths.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace wkopenvr::dynamicres {
namespace {

bool ParseBool(const std::string& value)
{
	return value == "1" || value == "true" || value == "on";
}

double ParseDouble(const std::string& value, double fallback)
{
	try {
		size_t end = 0;
		const double parsed = std::stod(value, &end);
		if (end == 0 || !std::isfinite(parsed)) return fallback;
		return parsed;
	}
	catch (...) {
		return fallback;
	}
}

int ParseInt(const std::string& value, int fallback)
{
	try {
		size_t end = 0;
		const int parsed = std::stoi(value, &end);
		if (end == 0) return fallback;
		return parsed;
	}
	catch (...) {
		return fallback;
	}
}

std::string Trim(std::string value)
{
	size_t begin = 0;
	while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
		++begin;
	}
	size_t end = value.size();
	while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
		--end;
	}
	return value.substr(begin, end - begin);
}

void WritePair(std::ostream& out, const char* key, const std::string& value)
{
	out << key << "=" << value << "\n";
}

void WritePair(std::ostream& out, const char* key, double value)
{
	out << key << "=" << value << "\n";
}

void WritePair(std::ostream& out, const char* key, int value)
{
	out << key << "=" << value << "\n";
}

void WritePair(std::ostream& out, const char* key, bool value)
{
	out << key << "=" << (value ? 1 : 0) << "\n";
}

} // namespace

std::wstring DynamicResolutionProfilePath()
{
	const std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	if (dir.empty()) return {};
	return dir + L"\\dynamicres.txt";
}

DynamicResolutionProfile LoadDynamicResolutionProfile()
{
	const std::wstring path = DynamicResolutionProfilePath();
	if (path.empty()) return {};
	std::ifstream in(path);
	if (!in) return {};
	DynamicResolutionProfile profile = ParseDynamicResolutionProfile(in);
	ReconcilePresetSettings(profile.settings);
	return profile;
}

void SaveDynamicResolutionProfile(const DynamicResolutionProfile& profile)
{
	const std::wstring path = DynamicResolutionProfilePath();
	if (path.empty()) return;
	std::ofstream out(path, std::ios::trunc);
	if (!out) return;
	WriteDynamicResolutionProfile(profile, out);
}

DynamicResolutionProfile ParseDynamicResolutionProfile(std::istream& in)
{
	DynamicResolutionProfile profile;
	std::string line;
	while (std::getline(in, line)) {
		const size_t eq = line.find('=');
		if (eq == std::string::npos) continue;
		const std::string key = Trim(line.substr(0, eq));
		const std::string value = Trim(line.substr(eq + 1));

		if (key == "min_scale_fraction")
			profile.settings.minScaleFraction =
			    ClampScaleFraction(ParseDouble(value, profile.settings.minScaleFraction));
		else if (key == "step_fraction")
			profile.settings.stepFraction = std::clamp(ParseDouble(value, profile.settings.stepFraction), 0.01, 0.25);
		else if (key == "max_scale_fraction")
			profile.settings.maxScaleFraction =
			    std::clamp(ParseDouble(value, profile.settings.maxScaleFraction), 1.0, 2.0);
		else if (key == "gpu_harm_rate_fraction")
			profile.settings.gpuHarmRateFraction =
			    std::clamp(ParseDouble(value, profile.settings.gpuHarmRateFraction), 0.001, 0.20);
		else if (key == "raise_harm_rate_fraction")
			profile.settings.raiseHarmRateFraction =
			    std::clamp(ParseDouble(value, profile.settings.raiseHarmRateFraction), 0.0, 0.05);
		else if (key == "motion_rate_fraction")
			profile.settings.motionRateFraction =
			    std::clamp(ParseDouble(value, profile.settings.motionRateFraction), 0.0, 1.0);
		else if (key == "lower_target_fraction")
			profile.settings.lowerTargetFraction =
			    std::clamp(ParseDouble(value, profile.settings.lowerTargetFraction), 0.70, 0.98);
		else if (key == "raise_safety_fraction")
			profile.settings.raiseSafetyFraction =
			    std::clamp(ParseDouble(value, profile.settings.raiseSafetyFraction), 0.70, 0.98);
		else if (key == "burned_decay_ticks")
			profile.settings.burnedDecayTicks =
			    std::clamp(ParseInt(value, profile.settings.burnedDecayTicks), 10, 600);
		else if (key == "raise_above_baseline_fraction")
			profile.settings.raiseAboveBaselineFraction =
			    std::clamp(ParseDouble(value, profile.settings.raiseAboveBaselineFraction), 0.30, 0.95);
		else if (key == "cpu_stall_fraction")
			profile.settings.cpuStallFraction =
			    std::clamp(ParseDouble(value, profile.settings.cpuStallFraction), 1.0, 2.0);
		else if (key == "lower_required_ticks")
			profile.settings.lowerRequiredTicks =
			    std::clamp(ParseInt(value, profile.settings.lowerRequiredTicks), 1, 30);
		else if (key == "raise_required_ticks")
			profile.settings.raiseRequiredTicks =
			    std::clamp(ParseInt(value, profile.settings.raiseRequiredTicks), 1, 30);
		else if (key == "raise_above_baseline_ticks")
			profile.settings.raiseAboveBaselineTicks =
			    std::clamp(ParseInt(value, profile.settings.raiseAboveBaselineTicks), 1, 60);
		else if (key == "quality_preset")
			profile.settings.qualityPreset =
			    static_cast<QualityPreset>(std::clamp(ParseInt(value, static_cast<int>(profile.settings.qualityPreset)),
			                                          0, static_cast<int>(QualityPreset::Custom)));
		else if (key == "allow_raise_back")
			profile.settings.allowRaiseBack = ParseBool(value);
		else if (key == "release_on_cpu_bound")
			profile.settings.releaseOnCpuBound = ParseBool(value);
		else if (key == "cpu_release_ticks")
			profile.settings.cpuReleaseTicks = std::clamp(ParseInt(value, profile.settings.cpuReleaseTicks), 1, 60);
		else if (key == "restore_pending")
			profile.restore.restorePending = ParseBool(value);
		else if (key == "baseline_scale")
			profile.restore.baselineScale = std::max(0.1, ParseDouble(value, profile.restore.baselineScale));
		else if (key == "baseline_manual_override")
			profile.restore.baselineManualOverride = ParseBool(value);
		else if (key == "active_scene_pid")
			profile.restore.sceneProcessId = static_cast<uint32_t>(std::max(0, ParseInt(value, 0)));
		else if (key == "last_written_scale")
			profile.restore.lastWrittenScale = std::max(0.0, ParseDouble(value, profile.restore.lastWrittenScale));
	}
	return profile;
}

void WriteDynamicResolutionProfile(const DynamicResolutionProfile& profile, std::ostream& out)
{
	WritePair(out, "quality_preset", static_cast<int>(profile.settings.qualityPreset));
	WritePair(out, "min_scale_fraction", profile.settings.minScaleFraction);
	WritePair(out, "max_scale_fraction", profile.settings.maxScaleFraction);
	WritePair(out, "step_fraction", profile.settings.stepFraction);
	WritePair(out, "gpu_harm_rate_fraction", profile.settings.gpuHarmRateFraction);
	WritePair(out, "raise_harm_rate_fraction", profile.settings.raiseHarmRateFraction);
	WritePair(out, "motion_rate_fraction", profile.settings.motionRateFraction);
	WritePair(out, "lower_target_fraction", profile.settings.lowerTargetFraction);
	WritePair(out, "raise_safety_fraction", profile.settings.raiseSafetyFraction);
	WritePair(out, "burned_decay_ticks", profile.settings.burnedDecayTicks);
	WritePair(out, "raise_above_baseline_fraction", profile.settings.raiseAboveBaselineFraction);
	WritePair(out, "cpu_stall_fraction", profile.settings.cpuStallFraction);
	WritePair(out, "lower_required_ticks", profile.settings.lowerRequiredTicks);
	WritePair(out, "raise_required_ticks", profile.settings.raiseRequiredTicks);
	WritePair(out, "raise_above_baseline_ticks", profile.settings.raiseAboveBaselineTicks);
	WritePair(out, "allow_raise_back", profile.settings.allowRaiseBack);
	WritePair(out, "release_on_cpu_bound", profile.settings.releaseOnCpuBound);
	WritePair(out, "cpu_release_ticks", profile.settings.cpuReleaseTicks);
	WritePair(out, "restore_pending", profile.restore.restorePending);
	WritePair(out, "baseline_scale", profile.restore.baselineScale);
	WritePair(out, "baseline_manual_override", profile.restore.baselineManualOverride);
	WritePair(out, "active_scene_pid", static_cast<int>(profile.restore.sceneProcessId));
	WritePair(out, "last_written_scale", profile.restore.lastWrittenScale);
}

} // namespace wkopenvr::dynamicres
