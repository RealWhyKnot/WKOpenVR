# calibration module

Aligns two tracking systems (for example a standalone headset plus lighthouse-tracked devices)
by solving the offset between their tracking universes and keeping it correct over time with
continuous calibration, drift recovery, and boundary/floor tools. The overlay piece in WKOpenVR.exe
owns the solver, state machine, and UI; the driver piece applies the transform in driver_wkopenvr.dll.

## Pieces

- src/overlay -- solver, continuous-calibration and recovery logic, boundary/floor
  capture, metrics logging, UI; builds openvr_pair_feature_calibration_overlay
- src/driver -- ModuleRegistry driver module that routes calibration requests and
  applies poses; builds openvr_pair_feature_calibration_driver
- src/common -- version header shared by both sides
- tests -- unit suite (spacecal_tests)

## Talks over

- \\.\pipe\WKOpenVR-Calibration -- overlay -> driver commands and queries as
  protocol::Request/Response, versioned by protocol::Version in core/src/common/Protocol.h;
  opened by the driver only when the calibration feature flag file is present
- WKOpenVRPoseMemoryV2 shared memory -- driver -> overlay pose snapshots and
  telemetry counters

## Build and test

- ./test.ps1 -Suite calibration builds and runs spacecal_tests.
- For tracking-math changes also run tools/Run-SessionReplayGate.ps1 -Quick -Baseline
  (session-replay counters vs tools/replay-baselines) and tools/Run-CalibrationReplayMatrix.ps1.

## Logs

- spacecal_log.<timestamp>.txt under %LocalAppDataLow%\WKOpenVR\Logs -- per-sample
  metrics rows plus event annotations
- the shared diagnostics_log.<timestamp>.txt in the same folder carries [spacecal] lines
