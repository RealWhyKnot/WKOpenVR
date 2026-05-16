#include <gtest/gtest.h>

#include "PhantomInferenceShmem.h"

#include <windows.h>

#include <cstdio>
#include <string>

namespace {

// Each test picks a unique shmem name so parallel CI runs and re-runs
// from a wedged previous binary do not collide.
std::string UniqueName(const char* tag)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "WKOpenVRPhantomTest_%s_%lu_%llu",
                  tag,
                  (unsigned long)GetCurrentProcessId(),
                  (unsigned long long)GetTickCount64());
    return buf;
}

} // namespace

TEST(PhantomInferenceShmemConstants, AreSet)
{
    EXPECT_NE(phantom::kPhantomInferenceShmemMagic, 0u);
    EXPECT_EQ(phantom::kPhantomInferenceShmemVersion, 1u);
    EXPECT_GT(phantom::kBodyRoleCount, 0u);
}

TEST(PhantomInferenceShmemConstants, LayoutSizesFitOnePage)
{
    EXPECT_GT(sizeof(phantom::PhantomInferenceInLayout),  0u);
    EXPECT_GT(sizeof(phantom::PhantomInferenceOutLayout), 0u);
    EXPECT_LT(sizeof(phantom::PhantomInferenceInLayout),  8192u);
    EXPECT_LT(sizeof(phantom::PhantomInferenceOutLayout), 8192u);
}

TEST(PhantomInferenceShmemIn, CreateInitialisesHeader)
{
    const std::string name = UniqueName("in_header");
    phantom::PhantomInferenceInShmem seg;
    ASSERT_TRUE(seg.Create(name.c_str()));
    auto* L = seg.layout();
    ASSERT_NE(L, nullptr);
    EXPECT_EQ(L->magic,   phantom::kPhantomInferenceShmemMagic);
    EXPECT_EQ(L->version, phantom::kPhantomInferenceShmemVersion);
    EXPECT_EQ(L->epoch,    0u);
    EXPECT_EQ(L->frame_id, 0u);
}

TEST(PhantomInferenceShmemOut, CreateInitialisesHeaderWithDistinctMagic)
{
    const std::string name = UniqueName("out_header");
    phantom::PhantomInferenceOutShmem seg;
    ASSERT_TRUE(seg.Create(name.c_str()));
    auto* L = seg.layout();
    ASSERT_NE(L, nullptr);
    // OUT magic is intentionally derived from IN magic so accidentally
    // opening one segment as the other type is detectable by header check.
    EXPECT_NE(L->magic, phantom::kPhantomInferenceShmemMagic);
    EXPECT_EQ(L->magic, phantom::kPhantomInferenceShmemMagic ^ 0xFFFFu);
    EXPECT_EQ(L->version, phantom::kPhantomInferenceShmemVersion);
}

TEST(PhantomInferenceShmemIn, CreateOpenRoundTripSeesSameWrites)
{
    const std::string name = UniqueName("in_roundtrip");
    phantom::PhantomInferenceInShmem writer;
    ASSERT_TRUE(writer.Create(name.c_str()));
    writer.layout()->frame_id = 4242u;
    writer.layout()->hmd_position[0] = 1.25;

    phantom::PhantomInferenceInShmem reader;
    ASSERT_TRUE(reader.Open(name.c_str()));
    auto* L = reader.layout();
    ASSERT_NE(L, nullptr);
    EXPECT_EQ(L->magic,    phantom::kPhantomInferenceShmemMagic);
    EXPECT_EQ(L->frame_id, 4242u);
    EXPECT_DOUBLE_EQ(L->hmd_position[0], 1.25);
}

TEST(PhantomInferenceShmemOut, CreateOpenRoundTripSeesSameWrites)
{
    const std::string name = UniqueName("out_roundtrip");
    phantom::PhantomInferenceOutShmem writer;
    ASSERT_TRUE(writer.Create(name.c_str()));
    writer.layout()->global_confidence = 0.75f;
    writer.layout()->frame_id          = 7u;
    writer.layout()->trackers[0].valid = 1;

    phantom::PhantomInferenceOutShmem reader;
    ASSERT_TRUE(reader.Open(name.c_str()));
    auto* L = reader.layout();
    ASSERT_NE(L, nullptr);
    EXPECT_FLOAT_EQ(L->global_confidence, 0.75f);
    EXPECT_EQ(L->frame_id, 7u);
    EXPECT_EQ(L->trackers[0].valid, 1u);
}

TEST(PhantomInferenceShmemIn, OpenWithoutCreateFails)
{
    const std::string name = UniqueName("in_no_create");
    phantom::PhantomInferenceInShmem seg;
    EXPECT_FALSE(seg.Open(name.c_str()));
    EXPECT_EQ(seg.layout(), nullptr);
}

TEST(PhantomInferenceShmemOut, OpenWithoutCreateFails)
{
    const std::string name = UniqueName("out_no_create");
    phantom::PhantomInferenceOutShmem seg;
    EXPECT_FALSE(seg.Open(name.c_str()));
    EXPECT_EQ(seg.layout(), nullptr);
}

TEST(PhantomInferenceShmemIn, CloseUnmaps)
{
    const std::string name = UniqueName("in_close");
    phantom::PhantomInferenceInShmem seg;
    ASSERT_TRUE(seg.Create(name.c_str()));
    ASSERT_NE(seg.layout(), nullptr);
    seg.Close();
    EXPECT_EQ(seg.layout(), nullptr);
}

TEST(PhantomInferenceShmemIn, RecreateOverridesPriorInstance)
{
    const std::string name = UniqueName("in_recreate");
    phantom::PhantomInferenceInShmem seg;
    ASSERT_TRUE(seg.Create(name.c_str()));
    seg.layout()->frame_id = 1u;
    ASSERT_TRUE(seg.Create(name.c_str()));            // implicit Close + re-Create
    ASSERT_NE(seg.layout(), nullptr);
    EXPECT_EQ(seg.layout()->frame_id, 0u);             // header reset
    EXPECT_EQ(seg.layout()->magic, phantom::kPhantomInferenceShmemMagic);
}
