// Verify the hand-coded upstream -> ours expression remap table.
//
// The table lives in core/src/common/facetracking/UpstreamShapeMap.h and
// is maintained in lockstep with the upstream C# enum and our protocol
// expression order:
//   - VRCFaceTracking.Core.Params.Expressions.UnifiedExpressions (upstream
//     v5.1.1.0 order, vendored at
//     modules/facetracking/src/host/WKOpenVR.FaceTracking.UpstreamRuntime/
//     Params/Expressions/UnifiedExpressions.cs)
//   - protocol::FACETRACKING_EXPRESSION_COUNT order, published by
//     modules/facetracking/src/driver/FaceOscPublisher.cpp
//
// If either expression list gains entries, the table
// needs a matching pass. These tests anchor a handful of well-known
// shapes (the names a code reviewer would recognise from VRCFaceTracking
// docs) plus the five semantic aliases that bridge upstream's later
// renames to our pre-rename names. Any drift surfaces immediately
// rather than silently producing zero values.

#include <gtest/gtest.h>

#include "facetracking/UpstreamShapeMap.h"

#include <algorithm>
#include <limits>

using namespace facetracking;
namespace p = protocol;

TEST(UpstreamShapeMap, TableSizeIs88)
{
	EXPECT_EQ(kUpstreamShapeCount, 88);
	EXPECT_EQ(static_cast<int>(sizeof(kUpstreamToOurs) / sizeof(kUpstreamToOurs[0])), kUpstreamShapeCount);
}

TEST(UpstreamShapeMap, JawOpenMapsThroughCorrectly)
{
	// Upstream UnifiedExpressions.JawOpen lives at index 22.
	// Our protocol JawOpen slot lives at index 26.
	EXPECT_EQ(kUpstreamToOurs[22], 26);

	float src[kUpstreamShapeCount] = {};
	src[22] = 0.75f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_NEAR(dst[26], 0.75f, 1e-6f);
	for (int i = 0; i < static_cast<int>(p::FACETRACKING_EXPRESSION_COUNT); ++i) {
		if (i == 26) continue;
		EXPECT_FLOAT_EQ(dst[i], 0.0f) << "slot " << i << " unexpectedly nonzero";
	}
}

TEST(UpstreamShapeMap, EyeSquintWideMapsThroughCorrectly)
{
	EXPECT_EQ(kUpstreamToOurs[0], 11); // EyeSquintRight
	EXPECT_EQ(kUpstreamToOurs[1], 10); // EyeSquintLeft
	EXPECT_EQ(kUpstreamToOurs[2], 9);  // EyeWideRight
	EXPECT_EQ(kUpstreamToOurs[3], 8);  // EyeWideLeft

	float src[kUpstreamShapeCount] = {};
	src[0] = 0.4f;
	src[1] = 0.5f;
	src[2] = 0.6f;
	src[3] = 0.7f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_NEAR(dst[11], 0.4f, 1e-6f);
	EXPECT_NEAR(dst[10], 0.5f, 1e-6f);
	EXPECT_NEAR(dst[9], 0.6f, 1e-6f);
	EXPECT_NEAR(dst[8], 0.7f, 1e-6f);
}

TEST(UpstreamShapeMap, NoseSneerMapsThroughCorrectly)
{
	// Upstream v5.1.1.0 positions NoseSneer at slots 48 / 49.
	// Our enum has NoseSneer at slots 25 / 24.
	EXPECT_EQ(kUpstreamToOurs[48], 25); // NoseSneerRight
	EXPECT_EQ(kUpstreamToOurs[49], 24); // NoseSneerLeft

	float src[kUpstreamShapeCount] = {};
	src[48] = 0.65f;
	src[49] = 0.35f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_NEAR(dst[25], 0.65f, 1e-6f);
	EXPECT_NEAR(dst[24], 0.35f, 1e-6f);
}

TEST(UpstreamShapeMap, MouthDirectionRightLeftPairing)
{
	// Upstream v5.1.1.0 pairs Mouth Upper/Lower direction Right-then-Left.
	EXPECT_EQ(kUpstreamToOurs[52], 42); // MouthUpperRight  -> 42
	EXPECT_EQ(kUpstreamToOurs[53], 41); // MouthUpperLeft   -> 41
	EXPECT_EQ(kUpstreamToOurs[54], 44); // MouthLowerRight  -> 44
	EXPECT_EQ(kUpstreamToOurs[55], 43); // MouthLowerLeft   -> 43

	float src[kUpstreamShapeCount] = {};
	src[52] = 0.42f;
	src[53] = 0.41f;
	src[54] = 0.44f;
	src[55] = 0.43f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_NEAR(dst[42], 0.42f, 1e-6f);
	EXPECT_NEAR(dst[41], 0.41f, 1e-6f);
	EXPECT_NEAR(dst[44], 0.44f, 1e-6f);
	EXPECT_NEAR(dst[43], 0.43f, 1e-6f);
}

TEST(UpstreamShapeMap, SemanticAliasMouthClosedToMouthClose)
{
	// Upstream's MouthClosed (slot 29) bridges to our MouthClose (slot 40).
	// Without this alias, legacy VRChat avatars bound to MouthClose would
	// never receive jaw-closed input from a modern VRCFT module.
	EXPECT_EQ(kUpstreamToOurs[29], 40);

	float src[kUpstreamShapeCount] = {};
	src[29] = 0.83f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_NEAR(dst[40], 0.83f, 1e-6f);
}

TEST(UpstreamShapeMap, SemanticAliasMouthCornerPullToMouthSmile)
{
	// Upstream renamed MouthSmile to MouthCornerPull in v5.x. Legacy
	// avatars bind to MouthSmile{Left,Right}.
	EXPECT_EQ(kUpstreamToOurs[56], 46); // MouthCornerPullRight -> MouthSmileRight
	EXPECT_EQ(kUpstreamToOurs[57], 45); // MouthCornerPullLeft  -> MouthSmileLeft

	float src[kUpstreamShapeCount] = {};
	src[56] = 0.71f;
	src[57] = 0.29f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_NEAR(dst[46], 0.71f, 1e-6f);
	EXPECT_NEAR(dst[45], 0.29f, 1e-6f);
}

TEST(UpstreamShapeMap, SemanticAliasMouthFrownToMouthSad)
{
	// Upstream renamed MouthSad to MouthFrown in v5.x. Same rationale.
	EXPECT_EQ(kUpstreamToOurs[60], 48); // MouthFrownRight -> MouthSadRight
	EXPECT_EQ(kUpstreamToOurs[61], 47); // MouthFrownLeft  -> MouthSadLeft

	float src[kUpstreamShapeCount] = {};
	src[60] = 0.55f;
	src[61] = 0.45f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_NEAR(dst[48], 0.55f, 1e-6f);
	EXPECT_NEAR(dst[47], 0.45f, 1e-6f);
}

TEST(UpstreamShapeMap, NasalAndJawBackwardAreDropped)
{
	// Upstream slots without an ours equivalent must remain dropped
	// (table entry -1). Picking shapes the audit memo called out
	// explicitly: nasal dilation/constriction, jaw backward, mouth
	// corner-slant.
	EXPECT_EQ(kUpstreamToOurs[12], -1); // NasalDilationRight
	EXPECT_EQ(kUpstreamToOurs[13], -1); // NasalDilationLeft
	EXPECT_EQ(kUpstreamToOurs[14], -1); // NasalConstrictRight
	EXPECT_EQ(kUpstreamToOurs[15], -1); // NasalConstrictLeft
	EXPECT_EQ(kUpstreamToOurs[26], -1); // JawBackward
	EXPECT_EQ(kUpstreamToOurs[27], -1); // JawClench
	EXPECT_EQ(kUpstreamToOurs[58], -1); // MouthCornerSlantRight
	EXPECT_EQ(kUpstreamToOurs[59], -1); // MouthCornerSlantLeft
	EXPECT_EQ(kUpstreamToOurs[77], -1); // TongueRoll
	EXPECT_EQ(kUpstreamToOurs[85], -1); // ThroatSwallow

	float src[kUpstreamShapeCount] = {};
	src[12] = 0.9f;
	src[26] = 0.8f;
	src[58] = 0.7f;
	src[85] = 0.6f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	for (int i = 0; i < static_cast<int>(p::FACETRACKING_EXPRESSION_COUNT); ++i)
		dst[i] = 0.0f;
	RemapUpstreamShapes(src, dst);

	for (int i = 0; i < static_cast<int>(p::FACETRACKING_EXPRESSION_COUNT); ++i) {
		EXPECT_FLOAT_EQ(dst[i], 0.0f) << "slot " << i << " unexpectedly nonzero";
	}
}

TEST(UpstreamShapeMap, NaNAndInfAreDropped)
{
	float src[kUpstreamShapeCount] = {};
	src[22] = std::nanf("");
	src[0] = std::numeric_limits<float>::infinity();
	src[1] = -std::numeric_limits<float>::infinity();
	src[2] = 0.5f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_FLOAT_EQ(dst[26], 0.0f);   // JawOpen stays zero from NaN
	EXPECT_FLOAT_EQ(dst[11], 0.0f);   // EyeSquintRight stays zero from +Inf
	EXPECT_FLOAT_EQ(dst[10], 0.0f);   // EyeSquintLeft  stays zero from -Inf
	EXPECT_NEAR(dst[9], 0.5f, 1e-6f); // EyeWideRight took the OK 0.5
}

TEST(UpstreamShapeMap, VrcftInvalidSentinelIsDropped)
{
	float src[kUpstreamShapeCount] = {};
	src[22] = 4294967296.0f; // 0xFFFFFFFF as a float in VRCFT IPC
	src[57] = 0.25f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_FLOAT_EQ(dst[26], 0.0f);
	EXPECT_NEAR(dst[45], 0.25f, 1e-6f);
	EXPECT_FLOAT_EQ(ClampUpstreamUnitSignal(src[22]), 0.0f);
}

TEST(UpstreamShapeMap, ValuesAreClampedTo01)
{
	float src[kUpstreamShapeCount] = {};
	src[22] = 2.5f;
	src[0] = -0.5f;

	float dst[p::FACETRACKING_EXPRESSION_COUNT] = {};
	RemapUpstreamShapes(src, dst);

	EXPECT_FLOAT_EQ(dst[26], 1.0f);
	EXPECT_FLOAT_EQ(dst[11], 0.0f);
}

TEST(UpstreamShapeMap, MappedCountMatchesExpectation)
{
	// Updated total after the v5.1.1.0 vendoring + semantic aliases land.
	// 55 mapped covers:
	//   eye squint/wide (4), brow (8), cheek puff/suck (4),
	//   jaw open/right/left/forward (4),
	//   MouthClosed alias (1),
	//   lip suck upper/lower (4), lip funnel (4), lip pucker upper (2),
	//   NoseSneer (2),
	//   mouth upper/lower direction (4),
	//   MouthCornerPull alias x2 (2),
	//   MouthFrown alias x2 (2),
	//   stretch (2), dimple (2), raiser (2), press (2), tightener (2),
	//   tongue out/up/down/left (4).
	// 33 dropped are shapes upstream has but ours does not: nasal (4),
	// cheek squint (2), jaw backward/clench/mandible (3), lip suck corner
	// (2), lip pucker lower (2), mouth upper-up (2), upper-deepen (2),
	// lower-down (2), corner-slant (2), tongue right / roll / bend /
	// curl / squish / flat / twist (8), soft palate / throat / neck (4).
	int mapped = 0;
	int dropped = 0;
	for (int i = 0; i < kUpstreamShapeCount; ++i) {
		if (kUpstreamToOurs[i] < 0)
			++dropped;
		else
			++mapped;
	}
	EXPECT_EQ(mapped, 55);
	EXPECT_EQ(dropped, 33);
}

TEST(UpstreamShapeMap, NoTargetSlotOutOfRange)
{
	// Defence-in-depth: any entry that isn't -1 must point inside our
	// 63-slot output array. A typo that points at slot 99 would silently
	// overwrite memory past the buffer in the live remap.
	for (int u = 0; u < kUpstreamShapeCount; ++u) {
		const int o = kUpstreamToOurs[u];
		if (o == -1) continue;
		EXPECT_GE(o, 0);
		EXPECT_LT(o, static_cast<int>(p::FACETRACKING_EXPRESSION_COUNT))
		    << "upstream slot " << u << " maps to invalid ours slot " << o;
	}
}

TEST(UpstreamShapeMap, NoTwoUpstreamSlotsMapToTheSameOursSlot)
{
	// Each ours-slot should be the destination of at most one upstream
	// slot (otherwise the last-write-wins behaviour in the remap loop
	// becomes order-dependent). Aliased slots (MouthSmile, MouthSad,
	// MouthClose) get input from a SINGLE upstream slot each (the
	// renamed upstream name), not from both the legacy and renamed
	// upstream slot -- legacy MouthSmile* has no upstream slot anyway.
	bool used[p::FACETRACKING_EXPRESSION_COUNT] = {};
	for (int u = 0; u < kUpstreamShapeCount; ++u) {
		const int o = kUpstreamToOurs[u];
		if (o == -1) continue;
		EXPECT_FALSE(used[o]) << "ours slot " << o
		                      << " mapped to by multiple upstream slots; "
		                         "last is upstream "
		                      << u;
		used[o] = true;
	}
}
