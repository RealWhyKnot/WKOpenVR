#include "RuntimeHealthSummary.h"

#include "Win32Paths.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace openvr_pair::common {
namespace {

constexpr size_t kMetricBuckets = 202;
constexpr double kBucketWidth = 0.5;

double BytesToMb(uint64_t bytes)
{
	return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

double CleanNumber(double value)
{
	return std::isfinite(value) ? value : 0.0;
}

std::string JsonString(const std::string& value)
{
	std::string out;
	out.reserve(value.size() + 2);
	out.push_back('"');
	for (const char ch : value) {
		switch (ch) {
		case '\\': out += "\\\\"; break;
		case '"': out += "\\\""; break;
		case '\n': out += "\\n"; break;
		case '\r': out += "\\r"; break;
		case '\t': out += "\\t"; break;
		default:
			if (static_cast<unsigned char>(ch) < 0x20) {
				char escaped[7]{};
				std::snprintf(escaped, sizeof escaped, "\\u%04x", static_cast<unsigned char>(ch));
				out += escaped;
			} else {
				out.push_back(ch);
			}
			break;
		}
	}
	out.push_back('"');
	return out;
}

struct MetricSummary {
	uint64_t count = 0;
	double min = 0.0;
	double mean = 0.0;
	double p50 = 0.0;
	double p95 = 0.0;
	double p99 = 0.0;
	double max = 0.0;
};

class RuntimeMetric {
public:
	void Observe(double value)
	{
		if (!std::isfinite(value) || value < 0.0) return;

		if (count_ == 0) {
			min_ = value;
			max_ = value;
		} else {
			min_ = std::min(min_, value);
			max_ = std::max(max_, value);
		}
		++count_;
		sum_ += value;

		const size_t bucket = BucketFor(value);
		++buckets_[bucket];
	}

	MetricSummary Summary() const
	{
		MetricSummary summary{};
		summary.count = count_;
		if (count_ == 0) return summary;

		summary.min = min_;
		summary.mean = sum_ / static_cast<double>(count_);
		summary.p50 = Percentile(0.50);
		summary.p95 = Percentile(0.95);
		summary.p99 = Percentile(0.99);
		summary.max = max_;
		return summary;
	}

	void Reset()
	{
		count_ = 0;
		sum_ = 0.0;
		min_ = 0.0;
		max_ = 0.0;
		buckets_.fill(0);
	}

private:
	static size_t BucketFor(double value)
	{
		if (value <= 0.0) return 0;
		const double scaled = value / kBucketWidth;
		if (scaled >= static_cast<double>(kMetricBuckets - 1)) {
			return kMetricBuckets - 1;
		}
		return static_cast<size_t>(scaled);
	}

	static double BucketUpper(size_t bucket)
	{
		if (bucket >= kMetricBuckets - 1) {
			return (kMetricBuckets - 1) * kBucketWidth;
		}
		return static_cast<double>(bucket + 1) * kBucketWidth;
	}

	double Percentile(double fraction) const
	{
		if (count_ == 0) return 0.0;
		const uint64_t target = std::max<uint64_t>(
			1,
			static_cast<uint64_t>(std::ceil(static_cast<double>(count_) * fraction)));
		uint64_t seen = 0;
		for (size_t i = 0; i < buckets_.size(); ++i) {
			seen += buckets_[i];
			if (seen >= target) return BucketUpper(i);
		}
		return max_;
	}

	uint64_t count_ = 0;
	double sum_ = 0.0;
	double min_ = 0.0;
	double max_ = 0.0;
	std::array<uint64_t, kMetricBuckets> buckets_{};
};

struct ProcessHealth {
	bool valid = false;
	std::string role;
	ProcessPerfSample latest{};
	uint64_t samples = 0;
	double maxCpuPctTotal = 0.0;
	double maxCpuPctOneCore = 0.0;
	uint64_t maxWorkingSetBytes = 0;
	uint64_t maxPrivateBytes = 0;
	uint32_t maxHandleCount = 0;
};

struct CompositorHealth {
	uint64_t frameSamples = 0;
	uint64_t droppedFrames = 0;
	uint64_t mispresentedFrames = 0;
	uint64_t reprojectedFrames = 0;
	uint64_t hmdPoseInvalidFrames = 0;
	uint32_t lastFrameIndex = 0;
	RuntimeMetric clientFrameIntervalMs;
	RuntimeMetric totalRenderGpuMs;
	RuntimeMetric compositorRenderGpuMs;
	RuntimeMetric compositorRenderCpuMs;
	RuntimeMetric submitFrameMs;
};

struct PoseHealth {
	uint64_t samples = 0;
	uint64_t invalidSamples = 0;
	uint64_t staleSamples = 0;
	uint64_t jumpSamples = 0;
	int lastRefTrackingResult = 0;
	int lastTargetTrackingResult = 0;
	RuntimeMetric poseAgeMs;
	RuntimeMetric poseGapMs;
};

struct CalibrationHealth {
	bool valid = false;
	RuntimeCalibrationHealthSample latest{};
};

struct RuntimeHealthState {
	std::mutex mutex;
	ProcessHealth process;
	CompositorHealth compositor;
	PoseHealth pose;
	CalibrationHealth calibration;
	uint64_t lastWriteWallMs = 0;
};

RuntimeHealthState& State()
{
	static RuntimeHealthState state;
	return state;
}

void AppendMetric(std::ostringstream& out, const MetricSummary& summary)
{
	out << "{\"count\":" << summary.count
		<< ",\"min\":" << CleanNumber(summary.min)
		<< ",\"mean\":" << CleanNumber(summary.mean)
		<< ",\"p50\":" << CleanNumber(summary.p50)
		<< ",\"p95\":" << CleanNumber(summary.p95)
		<< ",\"p99\":" << CleanNumber(summary.p99)
		<< ",\"max\":" << CleanNumber(summary.max)
		<< "}";
}

std::string BuildJsonLocked(const RuntimeHealthState& state)
{
	std::ostringstream out;
	out << std::fixed << std::setprecision(3);
	out << "{\n";
	out << "  \"schema\": 1,\n";

	out << "  \"process\": {";
	if (state.process.valid) {
		const ProcessPerfSample& p = state.process.latest;
		const ProcessPerfSnapshot& s = p.snapshot;
		out << "\"valid\": true"
			<< ", \"role\": " << JsonString(state.process.role)
			<< ", \"samples\": " << state.process.samples
			<< ", \"pid\": " << s.processId
			<< ", \"logical_cpus\": " << s.logicalProcessors
			<< ", \"cpu_valid\": " << (p.cpuValid ? "true" : "false")
			<< ", \"cpu_pct_total\": " << CleanNumber(p.cpuPctTotal)
			<< ", \"cpu_pct_one_core\": " << CleanNumber(p.cpuPctOneCore)
			<< ", \"cpu_pct_total_max\": " << CleanNumber(state.process.maxCpuPctTotal)
			<< ", \"cpu_pct_one_core_max\": " << CleanNumber(state.process.maxCpuPctOneCore)
			<< ", \"memory_valid\": " << (s.memoryValid ? "true" : "false")
			<< ", \"working_set_mb\": " << CleanNumber(BytesToMb(s.workingSetBytes))
			<< ", \"private_mb\": " << CleanNumber(BytesToMb(s.privateBytes))
			<< ", \"peak_working_set_mb\": " << CleanNumber(BytesToMb(s.peakWorkingSetBytes))
			<< ", \"working_set_mb_max\": " << CleanNumber(BytesToMb(state.process.maxWorkingSetBytes))
			<< ", \"private_mb_max\": " << CleanNumber(BytesToMb(state.process.maxPrivateBytes))
			<< ", \"handle_valid\": " << (s.handleCountValid ? "true" : "false")
			<< ", \"handles\": " << s.handleCount
			<< ", \"handles_max\": " << state.process.maxHandleCount;
	} else {
		out << "\"valid\": false";
	}
	out << "},\n";

	out << "  \"compositor\": {"
		<< "\"frames\": " << state.compositor.frameSamples
		<< ", \"last_frame_index\": " << state.compositor.lastFrameIndex
		<< ", \"dropped_frames\": " << state.compositor.droppedFrames
		<< ", \"mispresented_frames\": " << state.compositor.mispresentedFrames
		<< ", \"reprojected_frames\": " << state.compositor.reprojectedFrames
		<< ", \"hmd_pose_invalid_frames\": " << state.compositor.hmdPoseInvalidFrames
		<< ", \"client_frame_interval_ms\": ";
	AppendMetric(out, state.compositor.clientFrameIntervalMs.Summary());
	out << ", \"total_render_gpu_ms\": ";
	AppendMetric(out, state.compositor.totalRenderGpuMs.Summary());
	out << ", \"compositor_render_gpu_ms\": ";
	AppendMetric(out, state.compositor.compositorRenderGpuMs.Summary());
	out << ", \"compositor_render_cpu_ms\": ";
	AppendMetric(out, state.compositor.compositorRenderCpuMs.Summary());
	out << ", \"submit_frame_ms\": ";
	AppendMetric(out, state.compositor.submitFrameMs.Summary());
	out << "},\n";

	out << "  \"tracking\": {"
		<< "\"samples\": " << state.pose.samples
		<< ", \"invalid_samples\": " << state.pose.invalidSamples
		<< ", \"stale_samples\": " << state.pose.staleSamples
		<< ", \"jump_samples\": " << state.pose.jumpSamples
		<< ", \"last_ref_tracking_result\": " << state.pose.lastRefTrackingResult
		<< ", \"last_target_tracking_result\": " << state.pose.lastTargetTrackingResult
		<< ", \"pose_age_ms\": ";
	AppendMetric(out, state.pose.poseAgeMs.Summary());
	out << ", \"pose_gap_ms\": ";
	AppendMetric(out, state.pose.poseGapMs.Summary());
	out << "},\n";

	out << "  \"calibration\": {";
	if (state.calibration.valid) {
		const RuntimeCalibrationHealthSample& c = state.calibration.latest;
		out << "\"valid\": true"
			<< ", \"sample_count\": " << c.sampleCount
			<< ", \"valid_sample_count\": " << c.validSampleCount
			<< ", \"paired_sample_count\": " << c.pairedSampleCount
			<< ", \"tracking_health_pass\": " << (c.trackingHealthPass ? "true" : "false")
			<< ", \"shadow_dynamic_pass\": " << (c.shadowDynamicPass ? "true" : "false")
			<< ", \"residual_rms_mm\": " << CleanNumber(c.residualRmsMm)
			<< ", \"residual_p95_mm\": " << CleanNumber(c.residualP95Mm)
			<< ", \"holdout_rms_mm\": " << CleanNumber(c.holdoutRmsMm);
	} else {
		out << "\"valid\": false";
	}
	out << "}\n";
	out << "}\n";
	return out.str();
}

bool WriteUtf8File(const std::wstring& path, const std::string& body)
{
	FILE* file = nullptr;
	if (_wfopen_s(&file, path.c_str(), L"wb") != 0 || !file) {
		return false;
	}
	const bool ok = fwrite(body.data(), 1, body.size(), file) == body.size();
	fclose(file);
	return ok;
}

} // namespace

void RecordRuntimeProcessSample(const char* role, const ProcessPerfSample& sample)
{
	RuntimeHealthState& state = State();
	std::lock_guard<std::mutex> lock(state.mutex);
	state.process.valid = true;
	state.process.role = role ? role : "process";
	state.process.latest = sample;
	++state.process.samples;
	state.process.maxCpuPctTotal = std::max(state.process.maxCpuPctTotal, sample.cpuPctTotal);
	state.process.maxCpuPctOneCore = std::max(state.process.maxCpuPctOneCore, sample.cpuPctOneCore);
	state.process.maxWorkingSetBytes = std::max(
		state.process.maxWorkingSetBytes,
		sample.snapshot.workingSetBytes);
	state.process.maxPrivateBytes = std::max(
		state.process.maxPrivateBytes,
		sample.snapshot.privateBytes);
	state.process.maxHandleCount = std::max(
		state.process.maxHandleCount,
		sample.snapshot.handleCount);
}

void RecordRuntimeCompositorTiming(const RuntimeCompositorTimingSample& sample)
{
	RuntimeHealthState& state = State();
	std::lock_guard<std::mutex> lock(state.mutex);

	++state.compositor.frameSamples;
	state.compositor.lastFrameIndex = std::max(state.compositor.lastFrameIndex, sample.frameIndex);
	state.compositor.droppedFrames += sample.droppedFrames;
	state.compositor.mispresentedFrames += sample.mispresentedFrames;
	if (sample.reprojectionFlags != 0 || sample.framePresents > 1) {
		++state.compositor.reprojectedFrames;
	}
	if (!sample.hmdPoseValid) {
		++state.compositor.hmdPoseInvalidFrames;
	}
	state.compositor.clientFrameIntervalMs.Observe(sample.clientFrameIntervalMs);
	state.compositor.totalRenderGpuMs.Observe(sample.totalRenderGpuMs);
	state.compositor.compositorRenderGpuMs.Observe(sample.compositorRenderGpuMs);
	state.compositor.compositorRenderCpuMs.Observe(sample.compositorRenderCpuMs);
	state.compositor.submitFrameMs.Observe(sample.submitFrameMs);
}

void RecordRuntimePoseHealth(const RuntimePoseHealthSample& sample)
{
	RuntimeHealthState& state = State();
	std::lock_guard<std::mutex> lock(state.mutex);

	++state.pose.samples;
	if (sample.invalid) ++state.pose.invalidSamples;
	if (sample.stale) ++state.pose.staleSamples;
	if (sample.jump) ++state.pose.jumpSamples;
	state.pose.lastRefTrackingResult = sample.refTrackingResult;
	state.pose.lastTargetTrackingResult = sample.targetTrackingResult;
	state.pose.poseAgeMs.Observe(std::max(sample.refPoseAgeMs, sample.targetPoseAgeMs));
	state.pose.poseGapMs.Observe(std::max(sample.refPoseGapMs, sample.targetPoseGapMs));
}

void RecordRuntimeCalibrationHealth(const RuntimeCalibrationHealthSample& sample)
{
	RuntimeHealthState& state = State();
	std::lock_guard<std::mutex> lock(state.mutex);
	state.calibration.valid = sample.valid;
	state.calibration.latest = sample;
}

bool WriteRuntimeHealthSummary(const wchar_t* fileName)
{
	std::string body;
	{
		RuntimeHealthState& state = State();
		std::lock_guard<std::mutex> lock(state.mutex);
		body = BuildJsonLocked(state);
	}

	std::wstring path = WkOpenVrLogsPath(true);
	if (path.empty()) return false;
	path += L"\\";
	path += (fileName && fileName[0]) ? fileName : L"runtime_health_summary.json";
	return WriteUtf8File(path, body);
}

bool MaybeWriteRuntimeHealthSummary(uint64_t intervalMs, const wchar_t* fileName)
{
	const uint64_t nowMs = GetTickCount64();
	{
		RuntimeHealthState& state = State();
		std::lock_guard<std::mutex> lock(state.mutex);
		if (state.lastWriteWallMs != 0 && nowMs >= state.lastWriteWallMs
			&& nowMs - state.lastWriteWallMs < intervalMs) {
			return false;
		}
		state.lastWriteWallMs = nowMs;
	}
	return WriteRuntimeHealthSummary(fileName);
}

void ResetRuntimeHealthSummaryForTests()
{
	RuntimeHealthState& state = State();
	std::lock_guard<std::mutex> lock(state.mutex);
	state.process = ProcessHealth{};
	state.compositor = CompositorHealth{};
	state.pose = PoseHealth{};
	state.calibration = CalibrationHealth{};
	state.lastWriteWallMs = 0;
}

std::string FormatRuntimeHealthSummaryForTests()
{
	RuntimeHealthState& state = State();
	std::lock_guard<std::mutex> lock(state.mutex);
	return BuildJsonLocked(state);
}

} // namespace openvr_pair::common
