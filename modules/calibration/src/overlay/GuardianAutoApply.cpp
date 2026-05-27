#include "GuardianAutoApply.h"

#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "DiagnosticsLog.h"

#include <chrono>
#include <cstdio>
#include <string>

// Total wall-clock budget for the entire auto-apply sequence.
static constexpr auto kAutoApplyBudget = std::chrono::seconds(30);

// Cadence gate for TickGuardianHealth. ADB round-trips are slow (~300 ms);
// polling every tick would saturate the subprocess pipe. 7 seconds is short
// enough to notice a headset reboot within one user-perceptible beat.
static constexpr auto kHealthTickInterval = std::chrono::seconds(7);

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace {

// Returns true if CalCtx satisfies all preconditions for pausing Guardian.
// Logs the first failing condition so the log shows exactly which gate blocked.
bool PreconditionsMet(
    const char* site,
    bool requirePauseEnabled = true,
    bool requireBoundary = true)
{
    const auto& a = CalCtx.adb;
    const auto& b = CalCtx.boundary;

    if (requirePauseEnabled && !a.guardianPauseEnabled) {
        fprintf(stderr, "[guardian-auto] %s: skip -- guardianPauseEnabled=false\n", site);
        openvr_pair::common::DiagnosticLog(
            "guardian-auto", "%s precondition_failed gate=guardian_pause_disabled", site);
        return false;
    }
    if (requireBoundary && !b.enabled) {
        fprintf(stderr, "[guardian-auto] %s: skip -- boundary.enabled=false\n", site);
        openvr_pair::common::DiagnosticLog(
            "guardian-auto", "%s precondition_failed gate=boundary_disabled", site);
        return false;
    }
    if (requireBoundary && b.vertices.size() < 3) {
        fprintf(stderr, "[guardian-auto] %s: skip -- boundary has %zu vertices (need >=3)\n",
                site, b.vertices.size());
        openvr_pair::common::DiagnosticLog(
            "guardian-auto", "%s precondition_failed gate=few_vertices vertices=%zu",
            site, b.vertices.size());
        return false;
    }
    if (a.savedEndpoint.empty()) {
        fprintf(stderr, "[guardian-auto] %s: skip -- savedEndpoint is empty\n", site);
        openvr_pair::common::DiagnosticLog(
            "guardian-auto", "%s precondition_failed gate=no_endpoint", site);
        return false;
    }
    return true;
}

// Run the connect -> set -> verify sequence within kAutoApplyBudget.
// paused=true tries to pause Guardian; paused=false re-enables it.
// Returns true if the headset reflects the desired state at the end.
// Sets *aborted=true if any step exceeded the total budget.
bool RunPauseSequence(AdbController& adb, bool paused, bool* aborted)
{
    *aborted = false;
    const auto deadline = std::chrono::steady_clock::now() + kAutoApplyBudget;
    openvr_pair::common::DiagnosticLog(
        "guardian-auto", "pause_sequence_start desired=%d endpoint_set=%d value=%d",
        paused ? 1 : 0,
        CalCtx.adb.savedEndpoint.empty() ? 0 : 1,
        paused ? CalCtx.adb.guardianPauseValue : 0);

    // Step 1: connect (idempotent; already-connected succeeds cheaply).
    {
        if (std::chrono::steady_clock::now() >= deadline) {
            fprintf(stderr, "[guardian-auto] connect: exceeded budget before starting\n");
            *aborted = true;
            Metrics::adbConnected.Push(false);
            return false;
        }

        const bool ok = adb.Connect(CalCtx.adb.savedEndpoint);
        if (!ok) {
            fprintf(stderr, "[guardian-auto] connect failed to %s\n",
                    CalCtx.adb.savedEndpoint.c_str());
            Metrics::adbConnected.Push(false);
            openvr_pair::common::DiagnosticLog(
                "guardian-auto", "pause_sequence_connect_failed");
            return false;
        }
        Metrics::adbConnected.Push(true);
        fprintf(stderr, "[guardian-auto] connected to %s\n",
                CalCtx.adb.savedEndpoint.c_str());
    }

    // Step 2: set the property.
    {
        if (std::chrono::steady_clock::now() >= deadline) {
            fprintf(stderr, "[guardian-auto] SetGuardianPaused: exceeded budget\n");
            *aborted = true;
            Metrics::guardianPaused.Push(false);
            return false;
        }

        const bool ok = adb.SetGuardianPaused(paused, CalCtx.adb.guardianPauseValue);
        if (!ok) {
            fprintf(stderr, "[guardian-auto] SetGuardianPaused(%s, %d) failed\n",
                    paused ? "true" : "false", CalCtx.adb.guardianPauseValue);
            Metrics::guardianPaused.Push(false);
            openvr_pair::common::DiagnosticLog(
                "guardian-auto", "pause_sequence_set_failed desired=%d value=%d",
                paused ? 1 : 0, CalCtx.adb.guardianPauseValue);
            return false;
        }
    }

    // Step 3: read back to confirm.
    {
        if (std::chrono::steady_clock::now() >= deadline) {
            fprintf(stderr, "[guardian-auto] GetGuardianPaused: exceeded budget\n");
            *aborted = true;
            Metrics::guardianPaused.Push(false);
            return false;
        }

        const int read = adb.GetGuardianPaused();
        const int want = paused ? CalCtx.adb.guardianPauseValue : 0;
        const bool matched = (read == want);

        if (!matched) {
            // Do not auto-revert: the value could be stale on a fresh Quest
            // reboot and the user needs to observe the real state.
            fprintf(stderr,
                    "[guardian-auto] readback mismatch: wrote %d, read %d -- not reverting\n",
                    want, read);
        } else {
            fprintf(stderr, "[guardian-auto] Guardian %s confirmed (value=%d)\n",
                    paused ? "paused" : "resumed", read);
        }
        openvr_pair::common::DiagnosticLog(
            "guardian-auto", "pause_sequence_readback desired=%d want=%d read=%d matched=%d",
            paused ? 1 : 0,
            want,
            read,
            matched ? 1 : 0);

        Metrics::guardianPaused.Push(matched && paused);
        return matched;
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

namespace wkopenvr::adb {

bool MaybeAutoApplyAtStart(AdbController& adb)
{
    if (!CalCtx.adb.autoApplyOnStart) {
        fprintf(stderr, "[guardian-auto] startup: skip -- autoApplyOnStart=false\n");
        return false;
    }
    if (!PreconditionsMet("startup")) {
        // PreconditionsMet already wrote to stderr. Add a Metrics annotation with
        // the first failing gate so the UI log surfaces it on startup.
        const auto& a = CalCtx.adb;
        const auto& b = CalCtx.boundary;
        const char* gate =
            !b.enabled              ? "boundary_disabled" :
            b.vertices.size() < 3  ? "few_vertices"      :
            a.savedEndpoint.empty() ? "no_endpoint"       :
                                     "pause_disabled";
        char lbuf[96];
        snprintf(lbuf, sizeof lbuf,
            "[guardian-auto] skipped at start: gate='%s'", gate);
        Metrics::WriteLogAnnotation(lbuf);
        return false;
    }

    bool aborted = false;
    const bool ok = RunPauseSequence(adb, true, &aborted);

    if (aborted) {
        fprintf(stderr, "[guardian-auto] startup: aborted -- exceeded 30 s budget\n");
    } else if (!ok) {
        fprintf(stderr, "[guardian-auto] startup: Guardian pause unsuccessful\n");
    }

    {
        const int written = CalCtx.adb.guardianPauseValue;
        const int readback = ok ? written : -1;
        char lbuf[128];
        snprintf(lbuf, sizeof lbuf,
            "[guardian-auto] applied at start: written=%d readback=%d",
            written, ok ? written : readback);
        Metrics::WriteLogAnnotation(lbuf);
    }

    return ok;
}

bool ApplyGuardianPauseSetting(AdbController& adb, bool desired)
{
    if (desired) {
        if (!PreconditionsMet("runtime-toggle-on")) {
            return false;
        }
        bool aborted = false;
        const bool ok = RunPauseSequence(adb, true, &aborted);
        if (aborted) {
            fprintf(stderr, "[guardian-auto] runtime-toggle-on: timed out\n");
        }
        {
            char lbuf[80];
            snprintf(lbuf, sizeof lbuf, "[guardian-auto] toggle on: success=%s",
                     ok ? "true" : "false");
            Metrics::WriteLogAnnotation(lbuf);
        }
        return ok;
    } else {
        // Re-enable Guardian. Boundary preconditions are not required.
        if (CalCtx.adb.savedEndpoint.empty()) {
            fprintf(stderr,
                    "[guardian-auto] runtime-toggle-off: savedEndpoint empty -- skip\n");
            return false;
        }

        if (!adb.Connect(CalCtx.adb.savedEndpoint)) {
            fprintf(stderr, "[guardian-auto] runtime-toggle-off: connect failed\n");
            Metrics::adbConnected.Push(false);
            return false;
        }
        Metrics::adbConnected.Push(true);

        // Always write 0 to re-enable; guardianPauseValue is the pause value, not the resume value.
        const bool ok = adb.SetGuardianPaused(false, 0);
        Metrics::guardianPaused.Push(false);
        if (!ok) {
            fprintf(stderr, "[guardian-auto] runtime-toggle-off: SetGuardianPaused failed\n");
        } else {
            fprintf(stderr, "[guardian-auto] Guardian re-enabled\n");
        }
        {
            char lbuf[80];
            snprintf(lbuf, sizeof lbuf, "[guardian-auto] toggle off: success=%s",
                     ok ? "true" : "false");
            Metrics::WriteLogAnnotation(lbuf);
        }
        return ok;
    }
}

void RecordGuardianPausedConfirmation(const char* site)
{
    CalCtx.adb.guardianPauseEnabled = true;
    Metrics::guardianPaused.Push(true);

    const char* label = site && site[0] ? site : "unknown";
    char lbuf[128];
    snprintf(lbuf, sizeof lbuf,
        "[guardian-auto] manual pause confirmed: site='%s' value=%d",
        label,
        CalCtx.adb.guardianPauseValue);
    Metrics::WriteLogAnnotation(lbuf);
    openvr_pair::common::DiagnosticLog(
        "guardian-auto", "manual_pause_confirmed site='%s' value=%d",
        label,
        CalCtx.adb.guardianPauseValue);
}

bool SetGuardianPauseValueOverride(AdbController& adb, int newValue)
{
    CalCtx.adb.guardianPauseValue = newValue;
    fprintf(stderr,
            "[guardian-auto] guardianPauseValue overridden to %d -- re-applying\n",
            newValue);
    {
        char lbuf[80];
        snprintf(lbuf, sizeof lbuf,
            "[guardian-auto] polarity override: new_value=%d", newValue);
        Metrics::WriteLogAnnotation(lbuf);
    }

    if (!PreconditionsMet("polarity-override",
                          /*requirePauseEnabled=*/false,
                          /*requireBoundary=*/false)) {
        return false;
    }
    bool aborted = false;
    const bool ok = RunPauseSequence(adb, true, &aborted);
    if (aborted) {
        fprintf(stderr, "[guardian-auto] polarity-override: timed out\n");
    }
    return ok && !aborted;
}

void TickGuardianHealth(AdbController& adb)
{
    // Enforce cadence: only poll adb every kHealthTickInterval.
    static std::chrono::steady_clock::time_point s_lastPoll =
        std::chrono::steady_clock::time_point{}; // epoch = "never polled"
    static bool s_wasConnected = false;
    static bool s_firstTick    = true;

    const auto now = std::chrono::steady_clock::now();
    if (!s_firstTick && now - s_lastPoll < kHealthTickInterval) {
        return;
    }
    s_firstTick = false;
    s_lastPoll = now;

    const bool connected = adb.Connected();
    Metrics::adbConnected.Push(connected);

    if (!connected && s_wasConnected && CalCtx.adb.guardianPauseEnabled) {
        fprintf(stderr,
                "[guardian-auto] ADB connection lost while guardianPauseEnabled=true\n");
        Metrics::WriteLogAnnotation("[guardian-auto] adb disconnected during session");
    }

    // Re-apply on connection restore. s_wasConnected starts false; the
    // transition false->true on the very first tick means we just connected
    // and should apply if conditions are met.
    if (connected && !s_wasConnected && CalCtx.adb.guardianPauseEnabled) {
        fprintf(stderr,
                "[guardian-auto] ADB reconnected -- re-applying Guardian pause\n");
        Metrics::WriteLogAnnotation("[guardian-auto] adb reconnected: re-applying");
        bool aborted = false;
        if (PreconditionsMet("reconnect")) {
            RunPauseSequence(adb, true, &aborted);
            if (aborted) {
                fprintf(stderr, "[guardian-auto] reconnect re-apply: timed out\n");
            }
        }
    }

    s_wasConnected = connected;
}

} // namespace wkopenvr::adb
