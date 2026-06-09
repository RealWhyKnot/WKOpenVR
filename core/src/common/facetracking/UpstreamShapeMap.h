#pragma once

#include "Protocol.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace facetracking {

// Number of shape slots in upstream VRCFaceTracking's UnifiedExpressions
// enum, excluding the trailing Max sentinel. The host writes a dense
// array of this many floats into FaceTrackingFrameBody::expressions;
// the driver remaps them into our protocol::FACETRACKING_EXPRESSION_COUNT
// slots via the table below.
inline constexpr int kUpstreamShapeCount = 88;
inline constexpr float kInvalidSignalMin = 1000000.0f;

inline bool IsInvalidUpstreamSignal(float v)
{
	return !std::isfinite(v) || v >= kInvalidSignalMin;
}

inline float ClampUpstreamUnitSignal(float v)
{
	if (IsInvalidUpstreamSignal(v)) return 0.0f;
	return std::clamp(v, 0.0f, 1.0f);
}

inline constexpr float kExpressionOutputMax = 2.0f;

inline float ClampExpressionOutputSignal(float v)
{
	if (IsInvalidUpstreamSignal(v)) return 0.0f;
	return std::clamp(v, 0.0f, kExpressionOutputMax);
}

// Index into protocol::FACETRACKING_EXPRESSION_COUNT slots for each
// upstream shape, or -1 if the upstream shape has no equivalent in our
// enum (silently dropped).
//
// Sources of truth in lockstep with this table:
//   upstream: VRCFaceTracking.Core.Params.Expressions.UnifiedExpressions
//             vendored at
//             modules/facetracking/src/host/WKOpenVR.FaceTracking.UpstreamRuntime/
//             Params/Expressions/UnifiedExpressions.cs
//             (mirrors upstream v5.1.1.0 order including NoseSneer at 48-49).
//   ours:     protocol::FACETRACKING_EXPRESSION_COUNT order, published by
//             modules/facetracking/src/driver/FaceOscPublisher.cpp
//
// Most entries are direct case-insensitive name matches. Five entries are
// semantic aliases bridging upstream's later-renamed shapes to our
// pre-rename enum names so VRChat avatars built against either naming
// convention receive data:
//   - MouthClosed (upstream) -> MouthClose (ours)
//   - MouthCornerPull* (upstream) -> MouthSmile* (ours)
//   - MouthFrown*     (upstream) -> MouthSad*   (ours)
// These five are flagged in the comments below. The remaining -1 entries
// are upstream shapes with no equivalent in our 63-slot enum (drop
// silently; the avatar parameter, if bound, stays at zero).
inline constexpr int kUpstreamToOurs[kUpstreamShapeCount] = {
    // 0..3 Eye Squint/Wide
    11, // [ 0] EyeSquintRight       -> EyeSquintRight (11)
    10, // [ 1] EyeSquintLeft        -> EyeSquintLeft (10)
    9,  // [ 2] EyeWideRight         -> EyeWideRight (9)
    8,  // [ 3] EyeWideLeft          -> EyeWideLeft (8)
    // 4..11 Brow
    19, // [ 4] BrowPinchRight       -> BrowPinchRight (19)
    18, // [ 5] BrowPinchLeft        -> BrowPinchLeft (18)
    13, // [ 6] BrowLowererRight     -> BrowLowererRight (13)
    12, // [ 7] BrowLowererLeft      -> BrowLowererLeft (12)
    15, // [ 8] BrowInnerUpRight     -> BrowInnerUpRight (15)
    14, // [ 9] BrowInnerUpLeft      -> BrowInnerUpLeft (14)
    17, // [10] BrowOuterUpRight     -> BrowOuterUpRight (17)
    16, // [11] BrowOuterUpLeft      -> BrowOuterUpLeft (16)
    // 12..15 Nose (no equivalents: we have NoseSneer at 24/25, not Nasal*)
    -1, // [12] NasalDilationRight
    -1, // [13] NasalDilationLeft
    -1, // [14] NasalConstrictRight
    -1, // [15] NasalConstrictLeft
    // 16..21 Cheek
    -1, // [16] CheekSquintRight     (no cheek-squint in ours)
    -1, // [17] CheekSquintLeft
    21, // [18] CheekPuffRight       -> CheekPuffRight (21)
    20, // [19] CheekPuffLeft        -> CheekPuffLeft (20)
    23, // [20] CheekSuckRight       -> CheekSuckRight (23)
    22, // [21] CheekSuckLeft        -> CheekSuckLeft (22)
    // 22..28 Jaw
    26, // [22] JawOpen              -> JawOpen (26)
    29, // [23] JawRight             -> JawRight (29)
    28, // [24] JawLeft              -> JawLeft (28)
    27, // [25] JawForward           -> JawForward (27)
    -1, // [26] JawBackward
    -1, // [27] JawClench
    -1, // [28] JawMandibleRaise
    // 29 MouthClosed -- SEMANTIC ALIAS to ours.MouthClose (40). Upstream
    //    added the trailing 'd' in v5.x; legacy avatars use MouthClose.
    40, // [29] MouthClosed          -> MouthClose (40) (alias)
    // 30..35 Lip Suck Upper/Lower/Corner
    31, // [30] LipSuckUpperRight    -> LipSuckUpperRight (31)
    30, // [31] LipSuckUpperLeft     -> LipSuckUpperLeft (30)
    33, // [32] LipSuckLowerRight    -> LipSuckLowerRight (33)
    32, // [33] LipSuckLowerLeft     -> LipSuckLowerLeft (32)
    -1, // [34] LipSuckCornerRight   (no corner-suck in ours)
    -1, // [35] LipSuckCornerLeft
    // 36..43 Lip Funnel/Pucker
    35, // [36] LipFunnelUpperRight  -> LipFunnelUpperRight (35)
    34, // [37] LipFunnelUpperLeft   -> LipFunnelUpperLeft (34)
    37, // [38] LipFunnelLowerRight  -> LipFunnelLowerRight (37)
    36, // [39] LipFunnelLowerLeft   -> LipFunnelLowerLeft (36)
    39, // [40] LipPuckerUpperRight  -> LipPuckerUpperRight (39)
    38, // [41] LipPuckerUpperLeft   -> LipPuckerUpperLeft (38)
    -1, // [42] LipPuckerLowerRight  (ours has Upper only)
    -1, // [43] LipPuckerLowerLeft
    // 44..47 Mouth Upper Up + Deepen (no equivalents in ours)
    -1, // [44] MouthUpperUpRight
    -1, // [45] MouthUpperUpLeft
    -1, // [46] MouthUpperDeepenRight
    -1, // [47] MouthUpperDeepenLeft
    // 48..49 NoseSneer (upstream v5.x position; ours has NoseSneer at 24/25)
    25, // [48] NoseSneerRight       -> NoseSneerRight (25)
    24, // [49] NoseSneerLeft        -> NoseSneerLeft (24)
    // 50..51 Mouth Lower Down (no equivalents in ours)
    -1, // [50] MouthLowerDownRight
    -1, // [51] MouthLowerDownLeft
    // 52..55 Mouth Upper/Lower Direction (upstream pairs Right-then-Left)
    42, // [52] MouthUpperRight      -> MouthUpperRight (42)
    41, // [53] MouthUpperLeft       -> MouthUpperLeft (41)
    44, // [54] MouthLowerRight      -> MouthLowerRight (44)
    43, // [55] MouthLowerLeft       -> MouthLowerLeft (43)
    // 56..57 MouthCornerPull -- SEMANTIC ALIAS to ours.MouthSmile
    //    Upstream renamed Smile to CornerPull in v5.x. Legacy avatars
    //    use MouthSmile; route the same value into our slot so OSC
    //    publishes /avatar/parameters/MouthSmile* for them.
    46, // [56] MouthCornerPullRight -> MouthSmileRight (46) (alias)
    45, // [57] MouthCornerPullLeft  -> MouthSmileLeft  (45) (alias)
    // 58..59 MouthCornerSlant (no equivalent in ours)
    -1, // [58] MouthCornerSlantRight
    -1, // [59] MouthCornerSlantLeft
    // 60..61 MouthFrown -- SEMANTIC ALIAS to ours.MouthSad
    //    Same story: upstream renamed Sad to Frown. Legacy avatars
    //    use MouthSad.
    48, // [60] MouthFrownRight      -> MouthSadRight (48) (alias)
    47, // [61] MouthFrownLeft       -> MouthSadLeft  (47) (alias)
    // 62..71 Stretch / Dimple / Raiser / Press / Tightener
    50, // [62] MouthStretchRight    -> MouthStretchRight (50)
    49, // [63] MouthStretchLeft     -> MouthStretchLeft (49)
    52, // [64] MouthDimpleRight     -> MouthDimpleRight (52)
    51, // [65] MouthDimpleLeft      -> MouthDimpleLeft (51)
    53, // [66] MouthRaiserUpper     -> MouthRaiserUpper (53)
    54, // [67] MouthRaiserLower     -> MouthRaiserLower (54)
    56, // [68] MouthPressRight      -> MouthPressRight (56)
    55, // [69] MouthPressLeft       -> MouthPressLeft (55)
    58, // [70] MouthTightenerRight  -> MouthTightenerRight (58)
    57, // [71] MouthTightenerLeft   -> MouthTightenerLeft (57)
    // 72..83 Tongue
    59, // [72] TongueOut            -> TongueOut (59)
    60, // [73] TongueUp             -> TongueUp (60)
    61, // [74] TongueDown           -> TongueDown (61)
    -1, // [75] TongueRight          (ours has TongueLeft only)
    62, // [76] TongueLeft           -> TongueLeft (62)
    -1, // [77] TongueRoll
    -1, // [78] TongueBendDown
    -1, // [79] TongueCurlUp
    -1, // [80] TongueSquish
    -1, // [81] TongueFlat
    -1, // [82] TongueTwistRight
    -1, // [83] TongueTwistLeft
    // 84..87 Soft Palate / Throat / Neck (no equivalents)
    -1, // [84] SoftPalateClose
    -1, // [85] ThroatSwallow
    -1, // [86] NeckFlexRight
    -1, // [87] NeckFlexLeft
};

// Remap an upstream dense expression array into our protocol-ordered
// dense array. Indices unmapped on the upstream side are silently
// dropped (the corresponding slot in `dst` stays at whatever the
// caller initialised it to, typically zero). NaN / Inf and VRCFT's
// 0xFFFFFFFF invalid sentinel are also dropped. Each output is clamped
// to [0, 1].
//
// `src` must have at least kUpstreamShapeCount entries; `dst` must
// have at least protocol::FACETRACKING_EXPRESSION_COUNT entries.
inline void RemapUpstreamShapes(const float* src, float* dst)
{
	constexpr int kOurCount = static_cast<int>(protocol::FACETRACKING_EXPRESSION_COUNT);
	for (int u = 0; u < kUpstreamShapeCount; ++u) {
		const int o = kUpstreamToOurs[u];
		if (o < 0 || o >= kOurCount) continue;
		const float v = src[u];
		if (IsInvalidUpstreamSignal(v)) continue;
		dst[o] = std::clamp(v, 0.0f, 1.0f);
	}
}

} // namespace facetracking
