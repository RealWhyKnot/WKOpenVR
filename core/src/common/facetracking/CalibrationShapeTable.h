#pragma once

#include "../Protocol.h"

#include <cstdint>

namespace facetracking {
namespace calib_table {

// Slot layout shared by the calibration engine, overlay, and tests:
//   [0..62]  expressions[0..62]  (see ExpressionNames.h for the index map)
//   [63]     eye_openness_l      (calibrated as closedness = 1 - openness)
//   [64]     eye_openness_r
//   [65]     pupil_dilation_l
//   [66]     pupil_dilation_r
constexpr int kTotalShapes = (int)protocol::FACETRACKING_EXPRESSION_COUNT + 4;
constexpr int kIdxOpenL = (int)protocol::FACETRACKING_EXPRESSION_COUNT;
constexpr int kIdxOpenR = kIdxOpenL + 1;
constexpr int kIdxPupilL = kIdxOpenL + 2;
constexpr int kIdxPupilR = kIdxOpenL + 3;

// Per-shape auto-calibration gain ceiling. The learned mapping may never
// amplify a shape by more than this factor, no matter how small its observed
// range is -- unbounded gain is what turns sensor noise on a barely-moved
// shape into a visible false expression.
//   3.0 -- high-amplitude shapes the user exercises constantly during normal
//          play (talking, blinking, smiling); their true range is observed
//          quickly, so a stronger stretch is safe.
//   1.5 -- moderate-use shapes prone to cross-talk; stretch gently.
//   1.0 -- excluded: shapes whose true maximum is rarely or never observed
//          passively (tongue, cheek puff) or whose scale is meaningful as-is
//          (gaze directions, pupil). Calibration passes these through
//          untouched.
constexpr float kGainCap[kTotalShapes] = {
    1.0f, // 0  EyeLookOutLeft   (gaze-derived; vergence lock owns these)
    1.0f, // 1  EyeLookInLeft
    1.0f, // 2  EyeLookUpLeft
    1.0f, // 3  EyeLookDownLeft
    1.0f, // 4  EyeLookOutRight
    1.0f, // 5  EyeLookInRight
    1.0f, // 6  EyeLookUpRight
    1.0f, // 7  EyeLookDownRight
    1.5f, // 8  EyeWideLeft
    1.5f, // 9  EyeWideRight
    1.5f, // 10 EyeSquintLeft
    1.5f, // 11 EyeSquintRight
    1.5f, // 12 BrowLowererLeft
    1.5f, // 13 BrowLowererRight
    1.5f, // 14 BrowInnerUpLeft
    1.5f, // 15 BrowInnerUpRight
    1.5f, // 16 BrowOuterUpLeft
    1.5f, // 17 BrowOuterUpRight
    1.5f, // 18 BrowPinchLeft
    1.5f, // 19 BrowPinchRight
    1.0f, // 20 CheekPuffLeft    (rarely maxed passively)
    1.0f, // 21 CheekPuffRight
    1.0f, // 22 CheekSuckLeft
    1.0f, // 23 CheekSuckRight
    1.5f, // 24 NoseSneerLeft
    1.5f, // 25 NoseSneerRight
    3.0f, // 26 JawOpen
    1.5f, // 27 JawForward
    3.0f, // 28 JawLeft
    3.0f, // 29 JawRight
    1.5f, // 30 LipSuckUpperLeft
    1.5f, // 31 LipSuckUpperRight
    1.5f, // 32 LipSuckLowerLeft
    1.5f, // 33 LipSuckLowerRight
    1.5f, // 34 LipFunnelUpperLeft
    1.5f, // 35 LipFunnelUpperRight
    1.5f, // 36 LipFunnelLowerLeft
    1.5f, // 37 LipFunnelLowerRight
    1.5f, // 38 LipPuckerUpperLeft
    1.5f, // 39 LipPuckerUpperRight
    1.5f, // 40 MouthClose
    1.5f, // 41 MouthUpperLeft
    1.5f, // 42 MouthUpperRight
    1.5f, // 43 MouthLowerLeft
    1.5f, // 44 MouthLowerRight
    3.0f, // 45 MouthSmileLeft
    3.0f, // 46 MouthSmileRight
    3.0f, // 47 MouthSadLeft
    3.0f, // 48 MouthSadRight
    1.5f, // 49 MouthStretchLeft
    1.5f, // 50 MouthStretchRight
    1.5f, // 51 MouthDimpleLeft
    1.5f, // 52 MouthDimpleRight
    1.5f, // 53 MouthRaiserUpper
    1.5f, // 54 MouthRaiserLower
    1.5f, // 55 MouthPressLeft
    1.5f, // 56 MouthPressRight
    1.5f, // 57 MouthTightenerLeft
    1.5f, // 58 MouthTightenerRight
    1.0f, // 59 TongueOut        (never maxed passively)
    1.0f, // 60 TongueUp
    1.0f, // 61 TongueDown
    1.0f, // 62 TongueLeft
    3.0f, // 63 eye_openness_l   (blinks exercise the full range constantly)
    3.0f, // 64 eye_openness_r
    1.0f, // 65 pupil_dilation_l (absolute scale; do not stretch)
    1.0f, // 66 pupil_dilation_r
};

// Shapes whose raw signal rests HIGH and activates downward. The engine
// learns and normalizes these on the inverted signal (1 - raw) so the rest
// anchor / deadband model (rest near 0, activation grows upward) holds.
constexpr bool kInverted[kTotalShapes] = {
    false, false, false, false, false, false, false, false, // 0-7
    false, false, false, false, false, false, false, false, // 8-15
    false, false, false, false, false, false, false, false, // 16-23
    false, false, false, false, false, false, false, false, // 24-31
    false, false, false, false, false, false, false, false, // 32-39
    false, false, false, false, false, false, false, false, // 40-47
    false, false, false, false, false, false, false, false, // 48-55
    false, false, false, false, false, false, false,        // 56-62
    true,  true,                                            // 63-64 openness
    false, false,                                           // 65-66 pupil
};

// Lower-face shapes: their rest baseline may only be learned while the whole
// face is judged still, so speech does not drag the baseline upward.
constexpr bool kLowerFace[kTotalShapes] = {
    false, false, false, false, false, false, false, false, // 0-7  eye look
    false, false, false, false, false, false, false, false, // 8-15 eye/brow
    false, false, false, false, false, false, false, false, // 16-23 brow/cheek
    true,  true,  true,  true,  true,  true,  true,  true,  // 24-31 nose/jaw/lip
    true,  true,  true,  true,  true,  true,  true,  true,  // 32-39 lip
    true,  true,  true,  true,  true,  true,  true,  true,  // 40-47 mouth
    true,  true,  true,  true,  true,  true,  true,  true,  // 48-55 mouth
    true,  true,  true,  true,  true,  true,  true,         // 56-62 mouth/tongue
    false, false,                                           // 63-64 openness
    false, false,                                           // 65-66 pupil
};

// Left/right mirror pairs share ONE set of learned calibration parameters
// (rest / variance / max envelope / confidence), fed from the more active of
// the two sides each frame. Each side is then normalized against the shared
// parameters using its OWN raw value, so winks and deliberate asymmetry pass
// through while the two sides can never learn diverging ranges (independent
// per-side learning is what makes idle eyes and smiles drift asymmetric).
//
// kPairCanonical[i] = index of the slot that owns the learned state (the
// lower index of the pair; i itself when unpaired). JawLeft/JawRight and the
// EyeLook directions are antagonist directions, not mirrors -- unpaired.
constexpr uint8_t kPairCanonical[kTotalShapes] = {
    0,  1,  2,  3,  4, 5, 6, 7, // EyeLook* unpaired
    8,  8,                      // EyeWide L/R
    10, 10,                     // EyeSquint
    12, 12,                     // BrowLowerer
    14, 14,                     // BrowInnerUp
    16, 16,                     // BrowOuterUp
    18, 18,                     // BrowPinch
    20, 20,                     // CheekPuff
    22, 22,                     // CheekSuck
    24, 24,                     // NoseSneer
    26, 27, 28, 29,             // JawOpen/Forward/Left/Right unpaired
    30, 30,                     // LipSuckUpper
    32, 32,                     // LipSuckLower
    34, 34,                     // LipFunnelUpper
    36, 36,                     // LipFunnelLower
    38, 38,                     // LipPuckerUpper
    40,                         // MouthClose unpaired
    41, 41,                     // MouthUpper
    43, 43,                     // MouthLower
    45, 45,                     // MouthSmile
    47, 47,                     // MouthSad
    49, 49,                     // MouthStretch
    51, 51,                     // MouthDimple
    53, 54,                     // MouthRaiserUpper/Lower unpaired
    55, 55,                     // MouthPress
    57, 57,                     // MouthTightener
    59, 60, 61, 62,             // Tongue* unpaired
    63, 63,                     // eye openness L/R
    65, 65,                     // pupil L/R
};

// The other member of a slot's pair (self when unpaired). Derived from
// kPairCanonical for cheap iteration.
constexpr uint8_t PairOther(int i)
{
	const int c = kPairCanonical[i];
	if (c == i) {
		// i is canonical; scan for a mirror that points at it.
		for (int j = i + 1; j < kTotalShapes; ++j) {
			if (kPairCanonical[j] == c) return (uint8_t)j;
		}
		return (uint8_t)i;
	}
	return (uint8_t)c;
}

// Table consistency: every slot's canonical must be canonical itself and
// must not exceed the slot index (canonical = lower member of the pair).
constexpr bool PairTableConsistent()
{
	for (int i = 0; i < kTotalShapes; ++i) {
		const int c = kPairCanonical[i];
		if (c > i) return false;
		if (kPairCanonical[c] != c) return false;
		// Pair members must agree on gain cap, inversion, and face region --
		// they share learned state, so mismatched policy would be incoherent.
		if (kGainCap[i] != kGainCap[c]) return false;
		if (kInverted[i] != kInverted[c]) return false;
		if (kLowerFace[i] != kLowerFace[c]) return false;
	}
	return true;
}
static_assert(PairTableConsistent(), "calibration pair table is inconsistent");

// True when auto-calibration can modify this slot at all.
constexpr bool IsCalibratable(int i)
{
	return i >= 0 && i < kTotalShapes && kGainCap[i] > 1.0f;
}

} // namespace calib_table
} // namespace facetracking
