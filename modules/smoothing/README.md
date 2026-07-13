# smoothing module

Smooths Index-controller finger curls and applies per-device pose prediction
inside the SteamVR driver, configured from a Smoothing tab in the WKOpenVR
overlay. The driver piece patches the IVRDriverInput skeleton-update vtable
slots via MinHook and filters bone data in place before SteamVR sees it;
there is no sidecar process.

## Pieces

- `src/driver` -- static library linked into driver_wkopenvr.dll: skeletal
  hook install on the public IVRDriverInput interface, per-finger smoothing
  math, hook diagnostics, and a skeletal-frame recorder for offline replay.
- `src/overlay` -- static library linked into WKOpenVR.exe: the Smoothing
  tab UI (finger smoothing and pose-prediction controls), config
  persistence, pipe client, and per-plugin logging.
- `tests` -- GoogleTest suite (`smoothing_tests`) covering the smoothing
  math, the recording schema, and prediction UI logic.

## Talks over

- `\\.\pipe\WKOpenVR-Smoothing` (overlay -> driver): the module command
  channel. Opened by the driver when `enable_smoothing.flag` is present;
  carries `protocol::Request`/`Response` structs (finger-smoothing and
  device-prediction config; version from `protocol::Version` in
  `core/src/common/Protocol.h`).

No shared-memory segments or OSC ports are specific to this module.

## Build and test

Built as part of the umbrella CMake tree. Run the module suite with
`./test.ps1 -Suite smoothing`.

## Logs

With debug logging enabled, under `%LocalAppDataLow%\WKOpenVR\Logs`:
overlay events in `smoothing_log.<ts>.txt`; driver-side events go to the
shared `driver_log.<ts>.txt`.
