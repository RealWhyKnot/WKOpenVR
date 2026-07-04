#pragma once

#include "SkeletalSmoothingMath.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// On-disk schema for finger-smoothing captures plus the offline scorer that
// replays them. One CSV carries two row kinds:
//   kind=summary -- once per second per hand: live params (alpha per finger,
//     mask) and the worst-case frame stats accumulated since the previous
//     summary. Cheap enough to run all session.
//   kind=frame -- raw INPUT bones for one smoothing call, recorded in short
//     sampled windows. Windows keep the volume bounded while still giving the
//     scorer real input sequences to re-run with candidate parameters.
// Both kinds share one column header; summary rows simply end at the stats
// columns and frame rows leave the stats columns empty.

namespace skeletal::recording {

constexpr const char* kSchemaBanner = "smoothing_rec_v1";
constexpr int kWindowFrames = 90;
constexpr double kWindowPeriodMs = 60000.0;
constexpr double kSummaryIntervalMs = 1000.0;

inline std::string BuildColumnHeader()
{
	std::string header =
	    "time_ms,hand,motion_range,kind,window_id,frame_idx,alpha0,alpha1,alpha2,alpha3,alpha4,finger_mask,frames,"
	    "smoothed_frames,win_max_pos_delta,win_max_pos_bone,win_min_quat_dot,win_min_quat_bone";
	for (uint32_t b = 0; b < math::kFingerBoneCount; ++b) {
		char part[80] = {};
		snprintf(part, sizeof(part), ",b%u_px,b%u_py,b%u_pz,b%u_qw,b%u_qx,b%u_qy,b%u_qz", b, b, b, b, b, b, b);
		header += part;
	}
	return header;
}

struct SmoothingFrameRow
{
	double timeMs = 0.0;
	int hand = 0;
	int motionRange = 0;
	uint64_t windowId = 0;
	int frameIdx = 0;
	vr::VRBoneTransform_t bones[math::kFingerBoneCount] = {};
};

struct SmoothingSummaryRow
{
	double timeMs = 0.0;
	int hand = 0;
	float alpha[math::kFingersPerHand] = {};
	uint16_t fingerMask = 0;
	uint64_t frames = 0;
	uint64_t smoothedFrames = 0;
	float winMaxPosDelta = 0.0f;
	int winMaxPosBone = -1;
	float winMinQuatDot = 1.0f;
	int winMinQuatBone = -1;
};

inline std::string FormatFrameRow(const SmoothingFrameRow& row)
{
	char head[96] = {};
	snprintf(head, sizeof(head), "%.1f,%d,%d,frame,%llu,%d,,,,,,,,,,,,", row.timeMs, row.hand, row.motionRange,
	         (unsigned long long)row.windowId, row.frameIdx);
	std::string out = head;
	for (uint32_t b = 0; b < math::kFingerBoneCount; ++b) {
		const auto& bone = row.bones[b];
		char part[128] = {};
		snprintf(part, sizeof(part), ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", bone.position.v[0], bone.position.v[1],
		         bone.position.v[2], bone.orientation.w, bone.orientation.x, bone.orientation.y, bone.orientation.z);
		out += part;
	}
	return out;
}

inline std::string FormatSummaryRow(const SmoothingSummaryRow& row)
{
	char buf[256] = {};
	snprintf(buf, sizeof(buf), "%.1f,%d,,summary,,,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%llu,%llu,%.4f,%d,%.4f,%d", row.timeMs,
	         row.hand, row.alpha[0], row.alpha[1], row.alpha[2], row.alpha[3], row.alpha[4], (unsigned)row.fingerMask,
	         (unsigned long long)row.frames, (unsigned long long)row.smoothedFrames, row.winMaxPosDelta,
	         row.winMaxPosBone, row.winMinQuatDot, row.winMinQuatBone);
	return buf;
}

// Frame-window sampler: one contiguous run of kWindowFrames calls per hand,
// starting when the period since the previous window start has elapsed. The
// first call opens the first window so short sessions still capture one.
class WindowScheduler
{
public:
	struct Decision
	{
		bool record = false;
		uint64_t windowId = 0;
		int frameIdx = 0;
	};

	explicit WindowScheduler(double periodMs = kWindowPeriodMs, int windowFrames = kWindowFrames)
	    : periodMs_(periodMs), windowFrames_(windowFrames)
	{
	}

	Decision OnFrame(double nowMs)
	{
		Decision decision;
		if (framesLeft_ == 0 && (!started_ || nowMs - windowStartMs_ >= periodMs_)) {
			started_ = true;
			++windowId_;
			framesLeft_ = windowFrames_;
			frameIdx_ = 0;
			windowStartMs_ = nowMs;
		}
		if (framesLeft_ > 0) {
			decision.record = true;
			decision.windowId = windowId_;
			decision.frameIdx = frameIdx_++;
			--framesLeft_;
		}
		return decision;
	}

private:
	double periodMs_;
	int windowFrames_;
	bool started_ = false;
	uint64_t windowId_ = 0;
	int framesLeft_ = 0;
	int frameIdx_ = 0;
	double windowStartMs_ = 0.0;
};

struct LoadedSmoothingRecording
{
	std::vector<SmoothingFrameRow> frames;
	std::vector<SmoothingSummaryRow> summaries;
	std::string error;
	bool ok = false;
};

namespace detail {

inline std::vector<std::string> SplitCsv(const std::string& line)
{
	std::vector<std::string> fields;
	std::string current;
	for (char ch : line) {
		if (ch == ',') {
			fields.push_back(current);
			current.clear();
		}
		else {
			current += ch;
		}
	}
	fields.push_back(current);
	return fields;
}

inline int ColumnIndex(const std::vector<std::string>& header, const char* name)
{
	for (size_t i = 0; i < header.size(); ++i) {
		if (header[i] == name) return static_cast<int>(i);
	}
	return -1;
}

inline double FieldAsDouble(const std::vector<std::string>& fields, int index, double fallback = 0.0)
{
	if (index < 0 || index >= static_cast<int>(fields.size()) || fields[index].empty()) return fallback;
	return std::strtod(fields[index].c_str(), nullptr);
}

} // namespace detail

// Column-name based, comment-skipping loader. Unknown columns are ignored and
// appended columns keep old readers working -- same contract as the other
// recording formats.
inline LoadedSmoothingRecording ParseSmoothingRecording(const std::vector<std::string>& lines)
{
	LoadedSmoothingRecording out;

	bool sawBanner = false;
	std::vector<std::string> header;
	int colTime = -1, colHand = -1, colMotionRange = -1, colKind = -1, colWindowId = -1, colFrameIdx = -1;
	int colAlpha0 = -1, colFingerMask = -1, colFrames = -1, colSmoothedFrames = -1;
	int colWinMaxPos = -1, colWinMaxPosBone = -1, colWinMinQuat = -1, colWinMinQuatBone = -1;
	int colFirstBone = -1;

	for (const std::string& line : lines) {
		if (line.empty()) continue;
		if (line[0] == '#') {
			if (line.find(kSchemaBanner) != std::string::npos) sawBanner = true;
			continue;
		}
		const std::vector<std::string> fields = detail::SplitCsv(line);
		if (header.empty()) {
			if (!sawBanner) {
				out.error = "missing schema banner";
				return out;
			}
			header = fields;
			colTime = detail::ColumnIndex(header, "time_ms");
			colHand = detail::ColumnIndex(header, "hand");
			colMotionRange = detail::ColumnIndex(header, "motion_range");
			colKind = detail::ColumnIndex(header, "kind");
			colWindowId = detail::ColumnIndex(header, "window_id");
			colFrameIdx = detail::ColumnIndex(header, "frame_idx");
			colAlpha0 = detail::ColumnIndex(header, "alpha0");
			colFingerMask = detail::ColumnIndex(header, "finger_mask");
			colFrames = detail::ColumnIndex(header, "frames");
			colSmoothedFrames = detail::ColumnIndex(header, "smoothed_frames");
			colWinMaxPos = detail::ColumnIndex(header, "win_max_pos_delta");
			colWinMaxPosBone = detail::ColumnIndex(header, "win_max_pos_bone");
			colWinMinQuat = detail::ColumnIndex(header, "win_min_quat_dot");
			colWinMinQuatBone = detail::ColumnIndex(header, "win_min_quat_bone");
			colFirstBone = detail::ColumnIndex(header, "b0_px");
			if (colTime < 0 || colHand < 0 || colKind < 0) {
				out.error = "missing required columns";
				return out;
			}
			continue;
		}

		if (colKind >= static_cast<int>(fields.size())) continue;
		const std::string& kind = fields[colKind];
		if (kind == "frame") {
			if (colFirstBone < 0 ||
			    static_cast<int>(fields.size()) < colFirstBone + static_cast<int>(math::kFingerBoneCount) * 7) {
				continue; // truncated row (mid-write tail)
			}
			SmoothingFrameRow row;
			row.timeMs = detail::FieldAsDouble(fields, colTime);
			row.hand = static_cast<int>(detail::FieldAsDouble(fields, colHand));
			row.motionRange = static_cast<int>(detail::FieldAsDouble(fields, colMotionRange));
			row.windowId = static_cast<uint64_t>(detail::FieldAsDouble(fields, colWindowId));
			row.frameIdx = static_cast<int>(detail::FieldAsDouble(fields, colFrameIdx));
			for (uint32_t b = 0; b < math::kFingerBoneCount; ++b) {
				const int base = colFirstBone + static_cast<int>(b) * 7;
				row.bones[b].position.v[0] = static_cast<float>(detail::FieldAsDouble(fields, base));
				row.bones[b].position.v[1] = static_cast<float>(detail::FieldAsDouble(fields, base + 1));
				row.bones[b].position.v[2] = static_cast<float>(detail::FieldAsDouble(fields, base + 2));
				row.bones[b].orientation.w = static_cast<float>(detail::FieldAsDouble(fields, base + 3));
				row.bones[b].orientation.x = static_cast<float>(detail::FieldAsDouble(fields, base + 4));
				row.bones[b].orientation.y = static_cast<float>(detail::FieldAsDouble(fields, base + 5));
				row.bones[b].orientation.z = static_cast<float>(detail::FieldAsDouble(fields, base + 6));
			}
			out.frames.push_back(row);
		}
		else if (kind == "summary") {
			SmoothingSummaryRow row;
			row.timeMs = detail::FieldAsDouble(fields, colTime);
			row.hand = static_cast<int>(detail::FieldAsDouble(fields, colHand));
			for (int f = 0; f < math::kFingersPerHand; ++f) {
				row.alpha[f] = static_cast<float>(detail::FieldAsDouble(fields, colAlpha0 < 0 ? -1 : colAlpha0 + f));
			}
			row.fingerMask = static_cast<uint16_t>(detail::FieldAsDouble(fields, colFingerMask));
			row.frames = static_cast<uint64_t>(detail::FieldAsDouble(fields, colFrames));
			row.smoothedFrames = static_cast<uint64_t>(detail::FieldAsDouble(fields, colSmoothedFrames));
			row.winMaxPosDelta = static_cast<float>(detail::FieldAsDouble(fields, colWinMaxPos));
			row.winMaxPosBone = static_cast<int>(detail::FieldAsDouble(fields, colWinMaxPosBone, -1.0));
			row.winMinQuatDot = static_cast<float>(detail::FieldAsDouble(fields, colWinMinQuat, 1.0));
			row.winMinQuatBone = static_cast<int>(detail::FieldAsDouble(fields, colWinMinQuatBone, -1.0));
			out.summaries.push_back(row);
		}
	}

	if (header.empty()) {
		out.error = "no column header";
		return out;
	}
	out.ok = true;
	return out;
}

struct SmoothingReplayMetrics
{
	float maxInterFrameOutPosDelta = 0.0f; // jerk: worst bone step between consecutive outputs
	float meanInterFrameOutPosDelta = 0.0f;
	float minQuatDot = 1.0f;
	float meanLagM = 0.0f; // responsiveness: mean |output - input| position
	int framesScored = 0;
};

// Re-runs one recorded window through the live smoothing math with candidate
// parameters. Comparing candidate metrics against the recorded parameters on
// the same input answers "did this change help": lower jerk at equal-or-lower
// lag is a win.
inline SmoothingReplayMetrics ScoreWindow(const std::vector<SmoothingFrameRow>& frames,
                                          const float alphaPerFinger[math::kFingersPerHand], uint16_t fingerMask,
                                          int handBase)
{
	SmoothingReplayMetrics metrics;
	if (frames.empty()) return metrics;

	math::FingerFrameState state;
	vr::VRBoneTransform_t output[math::kFingerBoneCount] = {};
	vr::VRBoneTransform_t previousOutput[math::kFingerBoneCount] = {};
	bool hasPrevious = false;
	double sumInterFrame = 0.0;
	double sumLag = 0.0;
	uint64_t lagSamples = 0;

	for (const SmoothingFrameRow& frame : frames) {
		math::SmoothFingerFrame(state, frame.bones, math::kFingerBoneCount, handBase, fingerMask, alphaPerFinger,
		                        output);
		++metrics.framesScored;

		for (uint32_t b = 0; b < math::kFingerBoneCount; ++b) {
			const float dx = output[b].position.v[0] - frame.bones[b].position.v[0];
			const float dy = output[b].position.v[1] - frame.bones[b].position.v[1];
			const float dz = output[b].position.v[2] - frame.bones[b].position.v[2];
			sumLag += std::sqrt(double(dx) * dx + double(dy) * dy + double(dz) * dz);
			++lagSamples;
		}

		if (hasPrevious) {
			float framePeak = 0.0f;
			for (uint32_t b = 0; b < math::kFingerBoneCount; ++b) {
				const float dx = output[b].position.v[0] - previousOutput[b].position.v[0];
				const float dy = output[b].position.v[1] - previousOutput[b].position.v[1];
				const float dz = output[b].position.v[2] - previousOutput[b].position.v[2];
				const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
				if (dist > framePeak) framePeak = dist;

				const float dot = output[b].orientation.w * previousOutput[b].orientation.w +
				                  output[b].orientation.x * previousOutput[b].orientation.x +
				                  output[b].orientation.y * previousOutput[b].orientation.y +
				                  output[b].orientation.z * previousOutput[b].orientation.z;
				const float absDot = dot < 0.0f ? -dot : dot;
				if (absDot < metrics.minQuatDot) metrics.minQuatDot = absDot;
			}
			if (framePeak > metrics.maxInterFrameOutPosDelta) metrics.maxInterFrameOutPosDelta = framePeak;
			sumInterFrame += framePeak;
		}

		for (uint32_t b = 0; b < math::kFingerBoneCount; ++b) {
			previousOutput[b] = output[b];
		}
		hasPrevious = true;
	}

	if (metrics.framesScored > 1) {
		metrics.meanInterFrameOutPosDelta = static_cast<float>(sumInterFrame / (metrics.framesScored - 1));
	}
	if (lagSamples > 0) {
		metrics.meanLagM = static_cast<float>(sumLag / static_cast<double>(lagSamples));
	}
	return metrics;
}

} // namespace skeletal::recording
