// Tests for GuardianAutoApply.
//
// AdbController is stubbed by overriding Run; CalCtx is shared (CalibrationContext
// global from stubs.cpp). Each test resets the relevant CalCtx fields before use.

#include "GuardianAutoApply.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "AdbController.h"

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Stub AdbController
//
// Tracks call counts and injects canned results without spawning any subprocess.
// All virtual methods delegate to simple fields the test sets before calling.
// ---------------------------------------------------------------------------

namespace {

struct CallLog {
    int connectCalls       = 0;
    int setGuardianCalls   = 0;
    int getGuardianCalls   = 0;
    int connectedCalls     = 0;

    // Most-recent argument to Connect.
    std::string lastEndpoint;
    // Most-recent arguments to SetGuardianPaused.
    bool lastSetPaused = false;
    int  lastSetValue  = -1;
};

class StubAdb : public AdbController {
public:
    bool connectResult      = true;
    bool setGuardianResult  = true;
    int  getGuardianResult  = 1;    // value returned by GetGuardianPaused
    bool connectedResult    = true;

    CallLog calls;

    bool Connect(const std::string& endpoint) override {
        ++calls.connectCalls;
        calls.lastEndpoint = endpoint;
        return connectResult;
    }

    bool SetGuardianPaused(bool paused, int value) override {
        ++calls.setGuardianCalls;
        calls.lastSetPaused = paused;
        calls.lastSetValue  = value;
        return setGuardianResult;
    }

    int GetGuardianPaused() override {
        ++calls.getGuardianCalls;
        return getGuardianResult;
    }

    bool Connected() override {
        ++calls.connectedCalls;
        return connectedResult;
    }

    // Run is not directly tested here; override to avoid any subprocess spawn.
    Result Run(const std::vector<std::string>& /*args*/,
               std::chrono::milliseconds /*timeout*/) override
    {
        Result r;
        r.exitCode = 0;
        return r;
    }
};

// Helpers to set up a fully-valid CalCtx for the "all conditions met" path.
void SetupValidCalCtx(CalibrationContext& ctx, const std::string& endpoint = "192.168.1.2:5555")
{
    ctx.adb.guardianPauseEnabled = true;
    ctx.adb.autoApplyOnStart     = true;
    ctx.adb.guardianPauseValue   = 1;
    ctx.adb.savedEndpoint        = endpoint;

    ctx.boundary.enabled = true;
    ctx.boundary.vertices = { {0,0,0}, {1,0,0}, {1,1,0} }; // three vertices
}

void ResetCalCtx(CalibrationContext& ctx)
{
    ctx.adb = AdbConfig{};
    ctx.boundary = BoundaryConfig{};
}

} // namespace

// ---------------------------------------------------------------------------
// MaybeAutoApplyAtStart tests
// ---------------------------------------------------------------------------

TEST(GuardianAutoApply, MaybeAutoApply_skips_when_disabled)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);
    CalCtx.adb.guardianPauseEnabled = false;

    StubAdb adb;
    const bool result = wkopenvr::adb::MaybeAutoApplyAtStart(adb);

    EXPECT_FALSE(result);
    EXPECT_EQ(adb.calls.connectCalls,     0);
    EXPECT_EQ(adb.calls.setGuardianCalls, 0);
    EXPECT_EQ(adb.calls.getGuardianCalls, 0);
}

TEST(GuardianAutoApply, MaybeAutoApply_skips_when_no_boundary)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);
    CalCtx.boundary.enabled = false;

    StubAdb adb;
    const bool result = wkopenvr::adb::MaybeAutoApplyAtStart(adb);

    EXPECT_FALSE(result);
    EXPECT_EQ(adb.calls.connectCalls, 0);
    EXPECT_EQ(adb.calls.setGuardianCalls, 0);
}

TEST(GuardianAutoApply, MaybeAutoApply_skips_when_few_vertices)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);
    CalCtx.boundary.vertices = { {0,0,0}, {1,0,0} }; // only 2

    StubAdb adb;
    const bool result = wkopenvr::adb::MaybeAutoApplyAtStart(adb);

    EXPECT_FALSE(result);
    EXPECT_EQ(adb.calls.connectCalls, 0);
}

TEST(GuardianAutoApply, MaybeAutoApply_skips_when_no_endpoint)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);
    CalCtx.adb.savedEndpoint = "";

    StubAdb adb;
    const bool result = wkopenvr::adb::MaybeAutoApplyAtStart(adb);

    EXPECT_FALSE(result);
    EXPECT_EQ(adb.calls.connectCalls, 0);
}

TEST(GuardianAutoApply, MaybeAutoApply_skips_when_auto_apply_off)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);
    CalCtx.adb.autoApplyOnStart = false;

    StubAdb adb;
    const bool result = wkopenvr::adb::MaybeAutoApplyAtStart(adb);

    EXPECT_FALSE(result);
    EXPECT_EQ(adb.calls.connectCalls, 0);
}

TEST(GuardianAutoApply, MaybeAutoApply_pauses_when_all_conditions_met)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);

    StubAdb adb;
    // getGuardianResult matches guardianPauseValue (1) -- readback confirms.
    adb.getGuardianResult = 1;

    const bool result = wkopenvr::adb::MaybeAutoApplyAtStart(adb);

    EXPECT_TRUE(result);
    EXPECT_EQ(adb.calls.connectCalls,     1);
    EXPECT_EQ(adb.calls.setGuardianCalls, 1);
    EXPECT_EQ(adb.calls.getGuardianCalls, 1);
    EXPECT_TRUE(adb.calls.lastSetPaused);
    EXPECT_EQ(adb.calls.lastSetValue, 1);

    // Metric should reflect "paused".
    EXPECT_TRUE(Metrics::guardianPaused.last());
    EXPECT_TRUE(Metrics::adbConnected.last());
}

TEST(GuardianAutoApply, MaybeAutoApply_logs_mismatch_when_readback_disagrees)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);
    CalCtx.adb.guardianPauseValue = 1;

    StubAdb adb;
    // We wrote 1 but the headset reports 0 (stale reboot, or polarity mismatch).
    adb.getGuardianResult = 0;

    const bool result = wkopenvr::adb::MaybeAutoApplyAtStart(adb);

    // Mismatch -> returns false; metric reflects "not paused".
    EXPECT_FALSE(result);
    EXPECT_FALSE(Metrics::guardianPaused.last());

    // All three calls were still made (connect, set, get).
    EXPECT_EQ(adb.calls.connectCalls,     1);
    EXPECT_EQ(adb.calls.setGuardianCalls, 1);
    EXPECT_EQ(adb.calls.getGuardianCalls, 1);
}

// ---------------------------------------------------------------------------
// ApplyGuardianPauseSetting tests
// ---------------------------------------------------------------------------

TEST(GuardianAutoApply, ApplyGuardianPauseSetting_pauses_when_conditions_met)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);

    StubAdb adb;
    adb.getGuardianResult = 1;

    const bool result = wkopenvr::adb::ApplyGuardianPauseSetting(adb, true);
    EXPECT_TRUE(result);
    EXPECT_TRUE(adb.calls.lastSetPaused);
}

TEST(GuardianAutoApply, ApplyGuardianPauseSetting_unpauses_on_false)
{
    ResetCalCtx(CalCtx);
    // guardianPauseEnabled is irrelevant for desired=false, but savedEndpoint
    // must be present for the connect step.
    CalCtx.adb.savedEndpoint        = "192.168.1.2:5555";
    CalCtx.adb.guardianPauseEnabled = true; // already paused -- now turning off
    CalCtx.adb.guardianPauseValue   = 1;

    StubAdb adb;

    const bool result = wkopenvr::adb::ApplyGuardianPauseSetting(adb, false);
    EXPECT_TRUE(result);

    // SetGuardianPaused was called with paused=false and value=0.
    // The value matters: writing guardianPauseValue here would keep Guardian
    // paused on the headset, the opposite of what the user clicked.
    EXPECT_EQ(adb.calls.setGuardianCalls, 1);
    EXPECT_FALSE(adb.calls.lastSetPaused);
    EXPECT_EQ(adb.calls.lastSetValue, 0);

    // Metric should report "not paused".
    EXPECT_FALSE(Metrics::guardianPaused.last());
}

TEST(GuardianAutoApply, ApplyGuardianPauseSetting_false_skips_when_no_endpoint)
{
    ResetCalCtx(CalCtx);
    CalCtx.adb.savedEndpoint = "";

    StubAdb adb;
    const bool result = wkopenvr::adb::ApplyGuardianPauseSetting(adb, false);
    EXPECT_FALSE(result);
    EXPECT_EQ(adb.calls.connectCalls, 0);
}

TEST(GuardianAutoApply, RecordGuardianPausedConfirmation_marks_state_paused)
{
    ResetCalCtx(CalCtx);
    CalCtx.adb.guardianPauseEnabled = false;
    CalCtx.adb.guardianPauseValue = 1;
    Metrics::guardianPaused.Push(false);

    wkopenvr::adb::RecordGuardianPausedConfirmation("test");

    EXPECT_TRUE(CalCtx.adb.guardianPauseEnabled);
    EXPECT_TRUE(Metrics::guardianPaused.last());
}

TEST(GuardianAutoApply, SetGuardianPauseValueOverride_returns_confirmed_state)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);
    CalCtx.adb.guardianPauseEnabled = false;
    CalCtx.adb.guardianPauseValue = 1;
    CalCtx.boundary.enabled = false;
    CalCtx.boundary.vertices.clear();

    StubAdb adb;
    adb.getGuardianResult = 0;

    const bool result = wkopenvr::adb::SetGuardianPauseValueOverride(adb, 0);

    EXPECT_TRUE(result);
    EXPECT_EQ(CalCtx.adb.guardianPauseValue, 0);
    EXPECT_EQ(adb.calls.setGuardianCalls, 1);
    EXPECT_TRUE(adb.calls.lastSetPaused);
    EXPECT_EQ(adb.calls.lastSetValue, 0);
    EXPECT_TRUE(Metrics::guardianPaused.last());
}

// ---------------------------------------------------------------------------
// TickGuardianHealth cadence test
// ---------------------------------------------------------------------------

TEST(GuardianAutoApply, TickGuardianHealth_only_polls_at_cadence)
{
    ResetCalCtx(CalCtx);
    SetupValidCalCtx(CalCtx);
    CalCtx.adb.guardianPauseEnabled = false; // don't trigger re-apply

    StubAdb adb;

    // Call Tick 200 times rapidly (all within the 7 s cadence gate). The first
    // call fires immediately; subsequent ones are gated. We expect Connected()
    // to be called exactly once (the initial poll) for the entire burst
    // -- but because the cadence state is static, previous test runs may have
    // already fired it. What we can assert is that the burst does not call
    // Connected() 200 times.
    const int before = adb.calls.connectedCalls;
    for (int i = 0; i < 200; ++i) {
        wkopenvr::adb::TickGuardianHealth(adb);
    }
    const int delta = adb.calls.connectedCalls - before;

    // Within a tight loop, the cadence gate should limit calls to at most a
    // small constant. On a slow machine the loop might span the interval, but
    // 200 calls in a tight loop is always well under 7 s. Expect <= 2 polls
    // (one on entry, possibly one if a previous test left state near-ready).
    EXPECT_LE(delta, 2)
        << "Expected cadence gate to limit Connected() calls; got " << delta;
}
