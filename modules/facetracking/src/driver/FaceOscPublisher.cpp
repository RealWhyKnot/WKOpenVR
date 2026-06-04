#define _CRT_SECURE_NO_DEPRECATE
#include "FaceOscPublisher.h"

#include "RouterPublishApi.h"
#include "facetracking/UpstreamShapeMap.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <unordered_set>

namespace facetracking {

namespace {

struct OscCounts
{
	uint32_t attempted = 0;
	uint32_t sent = 0;
	uint32_t dropped = 0;
	uint32_t filtered = 0;
	uint32_t deduped = 0;
	uint32_t remapped = 0;

	void Add(const OscCounts& other)
	{
		attempted += other.attempted;
		sent += other.sent;
		dropped += other.dropped;
		filtered += other.filtered;
		deduped += other.deduped;
		remapped += other.remapped;
	}

	FaceOscPublishCounts Public() const
	{
		return FaceOscPublishCounts{attempted, sent, dropped, filtered, deduped, remapped};
	}
};

static thread_local const FaceOscAddressFilter* t_addressFilter = nullptr;
static thread_local std::unordered_set<std::string>* t_filteredDestinations = nullptr;

static inline void OscPublishFloat(OscCounts& counts, const char* address, float value)
{
	++counts.attempted;
	if (t_addressFilter && t_addressFilter->Active()) {
		const std::string* compatibleAddress = t_addressFilter->CompatibleAddress(address);
		if (!compatibleAddress) {
			++counts.filtered;
			return;
		}
		if (t_filteredDestinations && !t_filteredDestinations->insert(*compatibleAddress).second) {
			++counts.deduped;
			return;
		}
		if (*compatibleAddress != address) ++counts.remapped;
		address = compatibleAddress->c_str();
	}

	uint32_t bits;
	std::memcpy(&bits, &value, sizeof(bits));
	uint8_t arg_bytes[4] = {
	    static_cast<uint8_t>(bits >> 24),
	    static_cast<uint8_t>(bits >> 16),
	    static_cast<uint8_t>(bits >> 8),
	    static_cast<uint8_t>(bits),
	};
	if (pairdriver::oscrouter::PublishOsc("facetracking", address, ",f", arg_bytes, 4)) {
		++counts.sent;
	}
	else {
		++counts.dropped;
	}
}

static const char* const kExprParamNames[protocol::FACETRACKING_EXPRESSION_COUNT] = {
    "EyeLookOutLeft",
    "EyeLookInLeft",
    "EyeLookUpLeft",
    "EyeLookDownLeft",
    "EyeLookOutRight",
    "EyeLookInRight",
    "EyeLookUpRight",
    "EyeLookDownRight",
    "EyeWideLeft",
    "EyeWideRight",
    "EyeSquintLeft",
    "EyeSquintRight",
    "BrowLowererLeft",
    "BrowLowererRight",
    "BrowInnerUpLeft",
    "BrowInnerUpRight",
    "BrowOuterUpLeft",
    "BrowOuterUpRight",
    "BrowPinchLeft",
    "BrowPinchRight",
    "CheekPuffLeft",
    "CheekPuffRight",
    "CheekSuckLeft",
    "CheekSuckRight",
    "NoseSneerLeft",
    "NoseSneerRight",
    "JawOpen",
    "JawForward",
    "JawLeft",
    "JawRight",
    "LipSuckUpperLeft",
    "LipSuckUpperRight",
    "LipSuckLowerLeft",
    "LipSuckLowerRight",
    "LipFunnelUpperLeft",
    "LipFunnelUpperRight",
    "LipFunnelLowerLeft",
    "LipFunnelLowerRight",
    "LipPuckerUpperLeft",
    "LipPuckerUpperRight",
    "MouthClose",
    "MouthUpperLeft",
    "MouthUpperRight",
    "MouthLowerLeft",
    "MouthLowerRight",
    "MouthSmileLeft",
    "MouthSmileRight",
    "MouthSadLeft",
    "MouthSadRight",
    "MouthStretchLeft",
    "MouthStretchRight",
    "MouthDimpleLeft",
    "MouthDimpleRight",
    "MouthRaiserUpper",
    "MouthRaiserLower",
    "MouthPressLeft",
    "MouthPressRight",
    "MouthTightenerLeft",
    "MouthTightenerRight",
    "TongueOut",
    "TongueUp",
    "TongueDown",
    "TongueLeft",
};

static_assert(sizeof(kExprParamNames) / sizeof(kExprParamNames[0]) == protocol::FACETRACKING_EXPRESSION_COUNT,
              "kExprParamNames length must match FACETRACKING_EXPRESSION_COUNT");

// Upstream VRCFaceTracking-v5 alias names for slots where our enum kept
// the pre-rename label (MouthSmile, MouthSad, MouthClose). Emitting the
// upstream name in parallel lets avatars built against modern VRCFT
// receive the same value as legacy avatars without the avatar author
// having to support both naming conventions. nullptr = no alias.
static const char* const kExprParamUpstreamAliases[protocol::FACETRACKING_EXPRESSION_COUNT] = {
    nullptr,
    nullptr,
    nullptr,
    nullptr, // EyeLook* (no upstream equivalents)
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr, // EyeWide/Squint
    nullptr,
    nullptr,
    nullptr,
    nullptr, // Brow Lowerer/InnerUp
    nullptr,
    nullptr,
    nullptr,
    nullptr, // Brow OuterUp/Pinch
    nullptr,
    nullptr,
    nullptr,
    nullptr, // CheekPuff/CheekSuck
    nullptr,
    nullptr, // NoseSneer (name unchanged)
    nullptr,
    nullptr,
    nullptr,
    nullptr, // Jaw Open/Forward/Left/Right
    nullptr,
    nullptr,
    nullptr,
    nullptr, // LipSuckUpper/Lower
    nullptr,
    nullptr,
    nullptr,
    nullptr, // LipFunnelUpper/Lower
    nullptr,
    nullptr,       // LipPuckerUpper
    "MouthClosed", // [40] MouthClose <-> MouthClosed (upstream v5 trailing 'd')
    nullptr,
    nullptr,
    nullptr,
    nullptr,                // MouthUpper/Lower direction
    "MouthCornerPullLeft",  // [45] MouthSmileLeft  <-> MouthCornerPullLeft
    "MouthCornerPullRight", // [46] MouthSmileRight <-> MouthCornerPullRight
    "MouthFrownLeft",       // [47] MouthSadLeft    <-> MouthFrownLeft
    "MouthFrownRight",      // [48] MouthSadRight   <-> MouthFrownRight
    nullptr,
    nullptr,
    nullptr,
    nullptr, // MouthStretch/Dimple
    nullptr,
    nullptr, // MouthRaiser
    nullptr,
    nullptr,
    nullptr,
    nullptr, // MouthPress/Tightener
    nullptr,
    nullptr,
    nullptr,
    nullptr, // TongueOut/Up/Down/Left
};

static_assert(sizeof(kExprParamUpstreamAliases) / sizeof(kExprParamUpstreamAliases[0]) ==
                  protocol::FACETRACKING_EXPRESSION_COUNT,
              "kExprParamUpstreamAliases length must match FACETRACKING_EXPRESSION_COUNT");

// Upstream VRCFaceTracking UnifiedExpressions order. This mirrors
// modules/facetracking/src/host/WKOpenVR.FaceTracking.UpstreamRuntime/
// Params/Expressions/UnifiedExpressions.cs and lets us publish the current
// VRCFT v2 address family from the raw 88-slot wire data.
static const char* const kUpstreamExpressionNames[protocol::FACETRACKING_UPSTREAM_EXPRESSION_COUNT] = {
    "EyeSquintRight",
    "EyeSquintLeft",
    "EyeWideRight",
    "EyeWideLeft",
    "BrowPinchRight",
    "BrowPinchLeft",
    "BrowLowererRight",
    "BrowLowererLeft",
    "BrowInnerUpRight",
    "BrowInnerUpLeft",
    "BrowOuterUpRight",
    "BrowOuterUpLeft",
    "NasalDilationRight",
    "NasalDilationLeft",
    "NasalConstrictRight",
    "NasalConstrictLeft",
    "CheekSquintRight",
    "CheekSquintLeft",
    "CheekPuffRight",
    "CheekPuffLeft",
    "CheekSuckRight",
    "CheekSuckLeft",
    "JawOpen",
    "JawRight",
    "JawLeft",
    "JawForward",
    "JawBackward",
    "JawClench",
    "JawMandibleRaise",
    "MouthClosed",
    "LipSuckUpperRight",
    "LipSuckUpperLeft",
    "LipSuckLowerRight",
    "LipSuckLowerLeft",
    "LipSuckCornerRight",
    "LipSuckCornerLeft",
    "LipFunnelUpperRight",
    "LipFunnelUpperLeft",
    "LipFunnelLowerRight",
    "LipFunnelLowerLeft",
    "LipPuckerUpperRight",
    "LipPuckerUpperLeft",
    "LipPuckerLowerRight",
    "LipPuckerLowerLeft",
    "MouthUpperUpRight",
    "MouthUpperUpLeft",
    "MouthUpperDeepenRight",
    "MouthUpperDeepenLeft",
    "NoseSneerRight",
    "NoseSneerLeft",
    "MouthLowerDownRight",
    "MouthLowerDownLeft",
    "MouthUpperRight",
    "MouthUpperLeft",
    "MouthLowerRight",
    "MouthLowerLeft",
    "MouthCornerPullRight",
    "MouthCornerPullLeft",
    "MouthCornerSlantRight",
    "MouthCornerSlantLeft",
    "MouthFrownRight",
    "MouthFrownLeft",
    "MouthStretchRight",
    "MouthStretchLeft",
    "MouthDimpleRight",
    "MouthDimpleLeft",
    "MouthRaiserUpper",
    "MouthRaiserLower",
    "MouthPressRight",
    "MouthPressLeft",
    "MouthTightenerRight",
    "MouthTightenerLeft",
    "TongueOut",
    "TongueUp",
    "TongueDown",
    "TongueRight",
    "TongueLeft",
    "TongueRoll",
    "TongueBendDown",
    "TongueCurlUp",
    "TongueSquish",
    "TongueFlat",
    "TongueTwistRight",
    "TongueTwistLeft",
    "SoftPalateClose",
    "ThroatSwallow",
    "NeckFlexRight",
    "NeckFlexLeft",
};

static_assert(sizeof(kUpstreamExpressionNames) / sizeof(kUpstreamExpressionNames[0]) ==
                  protocol::FACETRACKING_UPSTREAM_EXPRESSION_COUNT,
              "kUpstreamExpressionNames length must match FACETRACKING_UPSTREAM_EXPRESSION_COUNT");

enum UpstreamShape : uint32_t
{
	U_EyeSquintRight = 0,
	U_EyeSquintLeft,
	U_EyeWideRight,
	U_EyeWideLeft,
	U_BrowPinchRight,
	U_BrowPinchLeft,
	U_BrowLowererRight,
	U_BrowLowererLeft,
	U_BrowInnerUpRight,
	U_BrowInnerUpLeft,
	U_BrowOuterUpRight,
	U_BrowOuterUpLeft,
	U_NasalDilationRight,
	U_NasalDilationLeft,
	U_NasalConstrictRight,
	U_NasalConstrictLeft,
	U_CheekSquintRight,
	U_CheekSquintLeft,
	U_CheekPuffRight,
	U_CheekPuffLeft,
	U_CheekSuckRight,
	U_CheekSuckLeft,
	U_JawOpen,
	U_JawRight,
	U_JawLeft,
	U_JawForward,
	U_JawBackward,
	U_JawClench,
	U_JawMandibleRaise,
	U_MouthClosed,
	U_LipSuckUpperRight,
	U_LipSuckUpperLeft,
	U_LipSuckLowerRight,
	U_LipSuckLowerLeft,
	U_LipSuckCornerRight,
	U_LipSuckCornerLeft,
	U_LipFunnelUpperRight,
	U_LipFunnelUpperLeft,
	U_LipFunnelLowerRight,
	U_LipFunnelLowerLeft,
	U_LipPuckerUpperRight,
	U_LipPuckerUpperLeft,
	U_LipPuckerLowerRight,
	U_LipPuckerLowerLeft,
	U_MouthUpperUpRight,
	U_MouthUpperUpLeft,
	U_MouthUpperDeepenRight,
	U_MouthUpperDeepenLeft,
	U_NoseSneerRight,
	U_NoseSneerLeft,
	U_MouthLowerDownRight,
	U_MouthLowerDownLeft,
	U_MouthUpperRight,
	U_MouthUpperLeft,
	U_MouthLowerRight,
	U_MouthLowerLeft,
	U_MouthCornerPullRight,
	U_MouthCornerPullLeft,
	U_MouthCornerSlantRight,
	U_MouthCornerSlantLeft,
	U_MouthFrownRight,
	U_MouthFrownLeft,
	U_MouthStretchRight,
	U_MouthStretchLeft,
	U_MouthDimpleRight,
	U_MouthDimpleLeft,
	U_MouthRaiserUpper,
	U_MouthRaiserLower,
	U_MouthPressRight,
	U_MouthPressLeft,
	U_MouthTightenerRight,
	U_MouthTightenerLeft,
	U_TongueOut,
	U_TongueUp,
	U_TongueDown,
	U_TongueRight,
	U_TongueLeft,
	U_TongueRoll,
	U_TongueBendDown,
	U_TongueCurlUp,
	U_TongueSquish,
	U_TongueFlat,
	U_TongueTwistRight,
	U_TongueTwistLeft,
	U_SoftPalateClose,
	U_ThroatSwallow,
	U_NeckFlexRight,
	U_NeckFlexLeft,
};

static inline float FiniteOrZero(float v)
{
	return facetracking::IsInvalidUpstreamSignal(v) ? 0.0f : v;
}

static inline float Upstream(const protocol::FaceTrackingFrameBody& frame, uint32_t index)
{
	if (index >= protocol::FACETRACKING_UPSTREAM_EXPRESSION_COUNT) return 0.0f;
	return facetracking::ClampUpstreamUnitSignal(frame.upstream_expressions[index]);
}

static inline float Avg2(float a, float b)
{
	return (a + b) * 0.5f;
}

static bool PublishFtV2Float(OscCounts& counts, const char* name, float value)
{
	static const char kPrefix[] = "/avatar/parameters/v2/";
	char address[96];
	int written = std::snprintf(address, sizeof(address), "%s%s", kPrefix, name);
	if (written <= 0 || static_cast<size_t>(written) >= sizeof(address)) return false;
	OscPublishFloat(counts, address, value);
	return true;
}

static float BrowUpRight(const protocol::FaceTrackingFrameBody& frame)
{
	return Upstream(frame, U_BrowOuterUpRight) * 0.60f + Upstream(frame, U_BrowInnerUpRight) * 0.40f;
}

static float BrowUpLeft(const protocol::FaceTrackingFrameBody& frame)
{
	return Upstream(frame, U_BrowOuterUpLeft) * 0.60f + Upstream(frame, U_BrowInnerUpLeft) * 0.40f;
}

static float BrowDownRight(const protocol::FaceTrackingFrameBody& frame)
{
	return Upstream(frame, U_BrowLowererRight) * 0.75f + Upstream(frame, U_BrowPinchRight) * 0.25f;
}

static float BrowDownLeft(const protocol::FaceTrackingFrameBody& frame)
{
	return Upstream(frame, U_BrowLowererLeft) * 0.75f + Upstream(frame, U_BrowPinchLeft) * 0.25f;
}

static float MouthSmileRight(const protocol::FaceTrackingFrameBody& frame)
{
	return Upstream(frame, U_MouthCornerPullRight) * 0.80f + Upstream(frame, U_MouthCornerSlantRight) * 0.20f;
}

static float MouthSmileLeft(const protocol::FaceTrackingFrameBody& frame)
{
	return Upstream(frame, U_MouthCornerPullLeft) * 0.80f + Upstream(frame, U_MouthCornerSlantLeft) * 0.20f;
}

static float MouthSadRight(const protocol::FaceTrackingFrameBody& frame)
{
	return std::max(Upstream(frame, U_MouthFrownRight), Upstream(frame, U_MouthStretchRight));
}

static float MouthSadLeft(const protocol::FaceTrackingFrameBody& frame)
{
	return std::max(Upstream(frame, U_MouthFrownLeft), Upstream(frame, U_MouthStretchLeft));
}

static OscCounts PublishEye(const protocol::FaceTrackingFrameBody& frame)
{
	OscCounts counts;
	const float gx_l = FiniteOrZero(frame.eye_gaze_l[0]);
	const float gy_l = FiniteOrZero(frame.eye_gaze_l[1]);
	const float gx_r = FiniteOrZero(frame.eye_gaze_r[0]);
	const float gy_r = FiniteOrZero(frame.eye_gaze_r[1]);
	const float open_l = FiniteOrZero(frame.eye_openness_l);
	const float open_r = FiniteOrZero(frame.eye_openness_r);
	const float pupil = (FiniteOrZero(frame.pupil_dilation_l) + FiniteOrZero(frame.pupil_dilation_r)) * 0.5f;

	OscPublishFloat(counts, "/avatar/parameters/LeftEyeX", gx_l);
	OscPublishFloat(counts, "/avatar/parameters/LeftEyeY", gy_l);
	OscPublishFloat(counts, "/avatar/parameters/RightEyeX", gx_r);
	OscPublishFloat(counts, "/avatar/parameters/RightEyeY", gy_r);
	OscPublishFloat(counts, "/avatar/parameters/LeftEyeLid", open_l);
	OscPublishFloat(counts, "/avatar/parameters/RightEyeLid", open_r);
	OscPublishFloat(counts, "/avatar/parameters/EyesDilation", pupil);

	OscPublishFloat(counts, "/avatar/parameters/v2/EyeLeftX", gx_l);
	OscPublishFloat(counts, "/avatar/parameters/v2/EyeLeftY", gy_l);
	OscPublishFloat(counts, "/avatar/parameters/v2/EyeRightX", gx_r);
	OscPublishFloat(counts, "/avatar/parameters/v2/EyeRightY", gy_r);
	OscPublishFloat(counts, "/avatar/parameters/v2/EyeOpenLeft", open_l);
	OscPublishFloat(counts, "/avatar/parameters/v2/EyeOpenRight", open_r);
	OscPublishFloat(counts, "/avatar/parameters/v2/PupilDilation", pupil);
	return counts;
}

static OscCounts PublishExpressions(const protocol::FaceTrackingFrameBody& frame)
{
	static const char kLegacyPrefix[] = "/avatar/parameters/";
	static const char kV2Prefix[] = "/avatar/parameters/v2/";
	static const size_t kLegacyPrefixLen = sizeof(kLegacyPrefix) - 1;
	static const size_t kV2PrefixLen = sizeof(kV2Prefix) - 1;

	OscCounts counts;
	char legacy[64];
	char v2addr[64];

	auto emitBoth = [&](const char* name, float value) {
		const size_t nameLen = std::strlen(name);
		if (kLegacyPrefixLen + nameLen + 1 > sizeof(legacy)) return;
		if (kV2PrefixLen + nameLen + 1 > sizeof(v2addr)) return;
		std::memcpy(legacy, kLegacyPrefix, kLegacyPrefixLen);
		std::memcpy(legacy + kLegacyPrefixLen, name, nameLen + 1);
		std::memcpy(v2addr, kV2Prefix, kV2PrefixLen);
		std::memcpy(v2addr + kV2PrefixLen, name, nameLen + 1);
		OscPublishFloat(counts, legacy, value);
		OscPublishFloat(counts, v2addr, value);
	};

	for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
		const float value = FiniteOrZero(frame.expressions[i]);
		emitBoth(kExprParamNames[i], value);
		// Modern VRCFaceTracking-v5 avatars bind to the renamed parameter
		// names (MouthClosed, MouthCornerPull*, MouthFrown*) instead of
		// the pre-rename ones we keep in our internal enum. Publish the
		// upstream alias side-by-side so avatars built against either
		// naming convention receive data.
		if (kExprParamUpstreamAliases[i] != nullptr) {
			emitBoth(kExprParamUpstreamAliases[i], value);
		}
	}
	return counts;
}

static OscCounts PublishCurrentVrcft(const protocol::FaceTrackingFrameBody& frame)
{
	OscCounts counts;

	for (uint32_t i = 0; i < protocol::FACETRACKING_UPSTREAM_EXPRESSION_COUNT; ++i) {
		PublishFtV2Float(counts, kUpstreamExpressionNames[i], Upstream(frame, i));
	}

	const float gx_l = FiniteOrZero(frame.eye_gaze_l[0]);
	const float gy_l = FiniteOrZero(frame.eye_gaze_l[1]);
	const float gx_r = FiniteOrZero(frame.eye_gaze_r[0]);
	const float gy_r = FiniteOrZero(frame.eye_gaze_r[1]);
	const float open_l = FiniteOrZero(frame.eye_openness_l);
	const float open_r = FiniteOrZero(frame.eye_openness_r);
	const float pupil_l = FiniteOrZero(frame.pupil_dilation_l);
	const float pupil_r = FiniteOrZero(frame.pupil_dilation_r);
	const float pupil = Avg2(pupil_l, pupil_r);

	PublishFtV2Float(counts, "EyeX", Avg2(gx_l, gx_r));
	PublishFtV2Float(counts, "EyeY", Avg2(gy_l, gy_r));
	PublishFtV2Float(counts, "EyeLeftX", gx_l);
	PublishFtV2Float(counts, "EyeLeftY", gy_l);
	PublishFtV2Float(counts, "EyeRightX", gx_r);
	PublishFtV2Float(counts, "EyeRightY", gy_r);

	PublishFtV2Float(counts, "PupilDilation", pupil);
	PublishFtV2Float(counts, "PupilDiameterLeft", pupil_l);
	PublishFtV2Float(counts, "PupilDiameterRight", pupil_r);
	PublishFtV2Float(counts, "PupilDiameter", pupil);

	PublishFtV2Float(counts, "EyeOpenLeft", open_l);
	PublishFtV2Float(counts, "EyeOpenRight", open_r);
	PublishFtV2Float(counts, "EyeOpen", Avg2(open_l, open_r));
	PublishFtV2Float(counts, "EyeClosedLeft", 1.0f - open_l);
	PublishFtV2Float(counts, "EyeClosedRight", 1.0f - open_r);
	PublishFtV2Float(counts, "EyeClosed", 1.0f - Avg2(open_l, open_r));
	PublishFtV2Float(counts, "EyeLidLeft", open_l * 0.75f + Upstream(frame, U_EyeWideLeft) * 0.25f);
	PublishFtV2Float(counts, "EyeLidRight", open_r * 0.75f + Upstream(frame, U_EyeWideRight) * 0.25f);
	PublishFtV2Float(counts, "EyeLid",
	                 Avg2(open_l, open_r) * 0.75f +
	                     Avg2(Upstream(frame, U_EyeWideLeft), Upstream(frame, U_EyeWideRight)) * 0.25f);
	PublishFtV2Float(counts, "EyeWide", std::max(Upstream(frame, U_EyeWideLeft), Upstream(frame, U_EyeWideRight)));
	PublishFtV2Float(counts, "EyeSquint",
	                 std::max(Upstream(frame, U_EyeSquintLeft), Upstream(frame, U_EyeSquintRight)));
	PublishFtV2Float(counts, "EyesSquint",
	                 std::max(Upstream(frame, U_EyeSquintLeft), Upstream(frame, U_EyeSquintRight)));

	const float browUpRight = BrowUpRight(frame);
	const float browUpLeft = BrowUpLeft(frame);
	const float browDownRight = BrowDownRight(frame);
	const float browDownLeft = BrowDownLeft(frame);
	PublishFtV2Float(counts, "BrowUpRight", browUpRight);
	PublishFtV2Float(counts, "BrowUpLeft", browUpLeft);
	PublishFtV2Float(counts, "BrowDownRight", browDownRight);
	PublishFtV2Float(counts, "BrowDownLeft", browDownLeft);
	PublishFtV2Float(counts, "BrowUp", Avg2(browUpRight, browUpLeft));
	PublishFtV2Float(counts, "BrowDown", Avg2(browDownRight, browDownLeft));
	PublishFtV2Float(counts, "BrowInnerUp",
	                 Avg2(Upstream(frame, U_BrowInnerUpLeft), Upstream(frame, U_BrowInnerUpRight)));
	PublishFtV2Float(counts, "BrowOuterUp",
	                 Avg2(Upstream(frame, U_BrowOuterUpLeft), Upstream(frame, U_BrowOuterUpRight)));
	const float browExpressionRight =
	    std::min(1.0f, Avg2(Upstream(frame, U_BrowInnerUpRight), Upstream(frame, U_BrowOuterUpRight))) - browDownRight;
	const float browExpressionLeft =
	    std::min(1.0f, Avg2(Upstream(frame, U_BrowInnerUpLeft), Upstream(frame, U_BrowOuterUpLeft))) - browDownLeft;
	PublishFtV2Float(counts, "BrowExpressionRight", browExpressionRight);
	PublishFtV2Float(counts, "BrowExpressionLeft", browExpressionLeft);
	PublishFtV2Float(counts, "BrowExpression", Avg2(browExpressionRight, browExpressionLeft));

	PublishFtV2Float(counts, "JawX", Upstream(frame, U_JawRight) - Upstream(frame, U_JawLeft));
	PublishFtV2Float(counts, "JawZ", Upstream(frame, U_JawForward) - Upstream(frame, U_JawBackward));

	PublishFtV2Float(counts, "CheekSquint",
	                 Avg2(Upstream(frame, U_CheekSquintLeft), Upstream(frame, U_CheekSquintRight)));
	PublishFtV2Float(counts, "CheekPuffSuckLeft", Upstream(frame, U_CheekPuffLeft) - Upstream(frame, U_CheekSuckLeft));
	PublishFtV2Float(counts, "CheekPuffSuckRight",
	                 Upstream(frame, U_CheekPuffRight) - Upstream(frame, U_CheekSuckRight));
	PublishFtV2Float(counts, "CheekPuffSuck",
	                 Avg2(Upstream(frame, U_CheekPuffLeft), Upstream(frame, U_CheekPuffRight)) -
	                     Avg2(Upstream(frame, U_CheekSuckLeft), Upstream(frame, U_CheekSuckRight)));
	PublishFtV2Float(counts, "CheekSuck", Avg2(Upstream(frame, U_CheekSuckLeft), Upstream(frame, U_CheekSuckRight)));

	PublishFtV2Float(counts, "MouthUpperX", Upstream(frame, U_MouthUpperRight) - Upstream(frame, U_MouthUpperLeft));
	PublishFtV2Float(counts, "MouthLowerX", Upstream(frame, U_MouthLowerRight) - Upstream(frame, U_MouthLowerLeft));
	PublishFtV2Float(counts, "MouthX",
	                 Avg2(Upstream(frame, U_MouthUpperRight), Upstream(frame, U_MouthLowerRight)) -
	                     Avg2(Upstream(frame, U_MouthUpperLeft), Upstream(frame, U_MouthLowerLeft)));

	const float lipSuckUpper = Avg2(Upstream(frame, U_LipSuckUpperRight), Upstream(frame, U_LipSuckUpperLeft));
	const float lipSuckLower = Avg2(Upstream(frame, U_LipSuckLowerRight), Upstream(frame, U_LipSuckLowerLeft));
	const float lipFunnelUpper = Avg2(Upstream(frame, U_LipFunnelUpperRight), Upstream(frame, U_LipFunnelUpperLeft));
	const float lipFunnelLower = Avg2(Upstream(frame, U_LipFunnelLowerRight), Upstream(frame, U_LipFunnelLowerLeft));
	const float lipPuckerUpper = Avg2(Upstream(frame, U_LipPuckerUpperRight), Upstream(frame, U_LipPuckerUpperLeft));
	const float lipPuckerLower = Avg2(Upstream(frame, U_LipPuckerLowerRight), Upstream(frame, U_LipPuckerLowerLeft));
	PublishFtV2Float(counts, "LipSuckUpper", lipSuckUpper);
	PublishFtV2Float(counts, "LipSuckLower", lipSuckLower);
	PublishFtV2Float(counts, "LipSuck", Avg2(lipSuckUpper, lipSuckLower));
	PublishFtV2Float(counts, "LipFunnelUpper", lipFunnelUpper);
	PublishFtV2Float(counts, "LipFunnelLower", lipFunnelLower);
	PublishFtV2Float(counts, "LipFunnel", Avg2(lipFunnelUpper, lipFunnelLower));
	PublishFtV2Float(counts, "LipPuckerUpper", lipPuckerUpper);
	PublishFtV2Float(counts, "LipPuckerLower", lipPuckerLower);
	PublishFtV2Float(counts, "LipPuckerRight",
	                 Avg2(Upstream(frame, U_LipPuckerUpperRight), Upstream(frame, U_LipPuckerLowerRight)));
	PublishFtV2Float(counts, "LipPuckerLeft",
	                 Avg2(Upstream(frame, U_LipPuckerUpperLeft), Upstream(frame, U_LipPuckerLowerLeft)));
	PublishFtV2Float(counts, "LipPucker", Avg2(lipPuckerUpper, lipPuckerLower));
	PublishFtV2Float(counts, "LipSuckFunnelUpper", lipSuckUpper - lipFunnelUpper);
	PublishFtV2Float(counts, "LipSuckFunnelLower", lipSuckLower - lipFunnelLower);
	PublishFtV2Float(counts, "LipSuckFunnelLowerLeft",
	                 Upstream(frame, U_LipSuckLowerLeft) - Upstream(frame, U_LipFunnelLowerLeft));
	PublishFtV2Float(counts, "LipSuckFunnelLowerRight",
	                 Upstream(frame, U_LipSuckLowerRight) - Upstream(frame, U_LipFunnelLowerRight));
	PublishFtV2Float(counts, "LipSuckFunnelUpperLeft",
	                 Upstream(frame, U_LipSuckUpperLeft) - Upstream(frame, U_LipFunnelUpperLeft));
	PublishFtV2Float(counts, "LipSuckFunnelUpperRight",
	                 Upstream(frame, U_LipSuckUpperRight) - Upstream(frame, U_LipFunnelUpperRight));

	const float mouthUpperUp = Avg2(Upstream(frame, U_MouthUpperUpRight), Upstream(frame, U_MouthUpperUpLeft));
	const float mouthLowerDown = Avg2(Upstream(frame, U_MouthLowerDownRight), Upstream(frame, U_MouthLowerDownLeft));
	const float mouthStretch = Avg2(Upstream(frame, U_MouthStretchRight), Upstream(frame, U_MouthStretchLeft));
	const float mouthTightener = Avg2(Upstream(frame, U_MouthTightenerRight), Upstream(frame, U_MouthTightenerLeft));
	PublishFtV2Float(counts, "MouthUpperUp", mouthUpperUp);
	PublishFtV2Float(counts, "MouthLowerDown", mouthLowerDown);
	PublishFtV2Float(counts, "MouthOpen", Avg2(mouthUpperUp, mouthLowerDown));
	PublishFtV2Float(counts, "MouthStretch", mouthStretch);
	PublishFtV2Float(counts, "MouthTightener", mouthTightener);
	PublishFtV2Float(counts, "MouthPress", Avg2(Upstream(frame, U_MouthPressRight), Upstream(frame, U_MouthPressLeft)));
	PublishFtV2Float(counts, "MouthDimple",
	                 Avg2(Upstream(frame, U_MouthDimpleRight), Upstream(frame, U_MouthDimpleLeft)));
	PublishFtV2Float(counts, "NoseSneer", Avg2(Upstream(frame, U_NoseSneerRight), Upstream(frame, U_NoseSneerLeft)));
	PublishFtV2Float(counts, "MouthTightenerStretch", mouthTightener - mouthStretch);
	PublishFtV2Float(counts, "MouthTightenerStretchLeft",
	                 Upstream(frame, U_MouthTightenerLeft) - Upstream(frame, U_MouthStretchLeft));
	PublishFtV2Float(counts, "MouthTightenerStretchRight",
	                 Upstream(frame, U_MouthTightenerRight) - Upstream(frame, U_MouthStretchRight));

	PublishFtV2Float(counts, "MouthCornerYLeft",
	                 Upstream(frame, U_MouthCornerSlantLeft) - Upstream(frame, U_MouthFrownLeft));
	PublishFtV2Float(counts, "MouthCornerYRight",
	                 Upstream(frame, U_MouthCornerSlantRight) - Upstream(frame, U_MouthFrownRight));
	PublishFtV2Float(counts, "MouthCornerY",
	                 Avg2(Upstream(frame, U_MouthCornerSlantLeft) - Upstream(frame, U_MouthFrownLeft),
	                      Upstream(frame, U_MouthCornerSlantRight) - Upstream(frame, U_MouthFrownRight)));

	const float smileRight = MouthSmileRight(frame);
	const float smileLeft = MouthSmileLeft(frame);
	const float sadRight = MouthSadRight(frame);
	const float sadLeft = MouthSadLeft(frame);
	PublishFtV2Float(counts, "MouthSmileRight", smileRight);
	PublishFtV2Float(counts, "MouthSmileLeft", smileLeft);
	PublishFtV2Float(counts, "MouthSadRight", sadRight);
	PublishFtV2Float(counts, "MouthSadLeft", sadLeft);
	PublishFtV2Float(counts, "SmileFrownRight", smileRight - Upstream(frame, U_MouthFrownRight));
	PublishFtV2Float(counts, "SmileFrownLeft", smileLeft - Upstream(frame, U_MouthFrownLeft));
	PublishFtV2Float(counts, "SmileFrown",
	                 Avg2(smileRight, smileLeft) -
	                     Avg2(Upstream(frame, U_MouthFrownRight), Upstream(frame, U_MouthFrownLeft)));
	PublishFtV2Float(counts, "SmileSadRight", smileRight - sadRight);
	PublishFtV2Float(counts, "SmileSadLeft", smileLeft - sadLeft);
	PublishFtV2Float(counts, "SmileSad", Avg2(smileRight, smileLeft) - Avg2(sadRight, sadLeft));

	PublishFtV2Float(counts, "TongueX", Upstream(frame, U_TongueRight) - Upstream(frame, U_TongueLeft));
	PublishFtV2Float(counts, "TongueY", Upstream(frame, U_TongueUp) - Upstream(frame, U_TongueDown));
	PublishFtV2Float(counts, "TongueArchY", Upstream(frame, U_TongueCurlUp) - Upstream(frame, U_TongueBendDown));
	PublishFtV2Float(counts, "TongueShape", Upstream(frame, U_TongueFlat) - Upstream(frame, U_TongueSquish));

	return counts;
}

} // namespace

FaceOscPublishCounts PublishFaceFrameOsc(const protocol::FaceTrackingFrameBody& frame,
                                         const FaceOscAddressFilter* filter)
{
	const FaceOscAddressFilter* previousFilter = t_addressFilter;
	std::unordered_set<std::string> filteredDestinations;
	std::unordered_set<std::string>* previousFilteredDestinations = t_filteredDestinations;
	t_addressFilter = filter;
	t_filteredDestinations = (filter && filter->Active()) ? &filteredDestinations : nullptr;

	FaceOscPublishCounts counts;
	if ((frame.flags & 0x1u) != 0) counts.Add(PublishEye(frame).Public());
	if ((frame.flags & 0x2u) != 0) {
		if (filter && filter->Active()) {
			counts.Add(PublishCurrentVrcft(frame).Public());
			counts.Add(PublishExpressions(frame).Public());
		}
		else {
			counts.Add(PublishExpressions(frame).Public());
			counts.Add(PublishCurrentVrcft(frame).Public());
		}
	}

	t_filteredDestinations = previousFilteredDestinations;
	t_addressFilter = previousFilter;
	return counts;
}

const char* FaceExpressionOscName(uint32_t index)
{
	if (index >= protocol::FACETRACKING_EXPRESSION_COUNT) return "";
	return kExprParamNames[index];
}

} // namespace facetracking
