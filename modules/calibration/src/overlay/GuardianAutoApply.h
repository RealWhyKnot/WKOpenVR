#pragma once

// Auto-apply and runtime-sync helpers for the Guardian pause feature.
//
// All functions operate on the single AdbController instance owned by the
// SpaceCalibrator plugin. The caller holds that instance; these functions
// take it by reference so tests can substitute a stub subclass.

#include "AdbController.h"

namespace wkopenvr::adb {

// Run the gated auto-apply sequence on startup. Returns true if Guardian is
// paused on the headset at the end of the sequence. All preconditions (flags,
// boundary, endpoint) are checked internally; failures are logged and the
// function returns false without touching the headset.
// Total wall-clock budget: 30 seconds. Any step that times out aborts the
// sequence and logs the failure.
bool MaybeAutoApplyAtStart(AdbController& adb);

// UI-driven runtime toggle. desired=true tries to pause Guardian;
// desired=false re-enables Guardian. Boundary preconditions are required for
// desired=true. Returns true if the headset reflects the desired state at the
// end of the call.
bool ApplyGuardianPauseSetting(AdbController& adb, bool desired);

// Record an in-headset confirmation from the setup wizard that Guardian is
// currently paused. The wizard's polarity probe already wrote the property;
// this keeps the panel state and saved auto-apply preference in sync with the
// confirmed headset state.
void RecordGuardianPausedConfirmation(const char* site);

// UI-driven polarity override. Called when in-headset confirmation shows that
// the stored guardianPauseValue is wrong (e.g. the property semantics changed
// across a runtime update). Updates CalCtx.adb.guardianPauseValue and
// re-runs the auto-apply sequence. Returns true when readback confirms the
// new value.
bool SetGuardianPauseValueOverride(AdbController& adb, int newValue);

// Periodic connection health check. Call from a low-rate tick (5-10 s cadence
// is enforced internally). Updates Metrics::adbConnected. Logs a warning if
// the connection drops while guardianPauseEnabled is true. Re-attempts pause
// on connection restore; does not spam adb on every tick.
void TickGuardianHealth(AdbController& adb);

} // namespace wkopenvr::adb
