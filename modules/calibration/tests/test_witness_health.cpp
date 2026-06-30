#include "WitnessHealth.h"

#include <gtest/gtest.h>

namespace wh = spacecal::witness_health;
using wh::WitnessHealth;

TEST(WitnessHealth, UnboundTicksAreNotCounted)
{
	WitnessHealth w;
	wh::TickWitness(w, /*bound=*/false, /*valid=*/false, 1.0);
	EXPECT_EQ(w.totalTicks, 0u);
	EXPECT_EQ(w.validTicks, 0u);
	EXPECT_DOUBLE_EQ(wh::ValidPct(w), 0.0);
	EXPECT_DOUBLE_EQ(wh::LastValidSec(w, 5.0), -1.0); // never valid
}

TEST(WitnessHealth, ValidAndInvalidTicksTrackPct)
{
	WitnessHealth w;
	wh::TickWitness(w, true, true, 1.0);
	wh::TickWitness(w, true, false, 2.0);
	wh::TickWitness(w, true, true, 3.0);
	wh::TickWitness(w, true, false, 4.0);
	EXPECT_EQ(w.totalTicks, 4u);
	EXPECT_EQ(w.validTicks, 2u);
	EXPECT_DOUBLE_EQ(wh::ValidPct(w), 50.0);
}

TEST(WitnessHealth, LastValidSecMeasuresFromLastValidTick)
{
	WitnessHealth w;
	wh::TickWitness(w, true, true, 10.0);
	wh::TickWitness(w, true, false, 11.0); // invalid does not move lastValidTime
	EXPECT_DOUBLE_EQ(w.lastValidTime, 10.0);
	EXPECT_DOUBLE_EQ(wh::LastValidSec(w, 25.0), 15.0);
}

TEST(WitnessHealth, NeverValidReportsSentinel)
{
	WitnessHealth w;
	wh::TickWitness(w, true, false, 1.0);
	wh::TickWitness(w, true, false, 2.0);
	EXPECT_DOUBLE_EQ(wh::LastValidSec(w, 9.0), -1.0);
	EXPECT_DOUBLE_EQ(wh::ValidPct(w), 0.0);
}

TEST(WitnessHealth, SubthresholdRelocCounts)
{
	WitnessHealth w;
	wh::NoteSubthresholdReloc(w);
	wh::NoteSubthresholdReloc(w);
	EXPECT_EQ(w.subthresholdRelocs, 2u);
}
