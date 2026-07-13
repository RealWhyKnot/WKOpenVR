# phantom module

Keeps full-body tracking usable when body trackers drop out or are missing: the driver piece bridges dropouts with dead reckoning, IK fallback, and a body-completion solver, and can publish virtual trackers for unassigned body roles. The overlay piece provides the master switch, per-device opt-in, role mapping (including one-shot snap calibration), and live per-tracker state badges.

## Pieces

- `src/driver` -- in-driver completion path: dropout state machine, dead reckoner, IK fallback, body-completion solver, blend controller, role arbiter, snap calibration, virtual tracker devices, replay recorder, and the bridge/supervisor for the optional sidecar.
- `src/overlay` -- overlay plugin (`openvr_pair_feature_phantom_overlay`): tab UI, config persistence, driver IPC client, state-badge rendering.
- `src/host` -- optional backend exe (`WKOpenVRPhantomSidecar`). Not launched by the deterministic completion path; kept buildable for a future out-of-process backend.
- `src/common` -- types, state and inference shared-memory segments, blend curves, role and tracker-model catalogs shared across the pieces.
- `src/analysis` -- replay player, trajectory, and metrics headers used by the test suite.
- `tests` -- unit and scenario tests plus replay fixtures (`phantom_tests`).
- `resources/input` -- SteamVR input profiles for the per-role virtual Vive trackers.
- `disabled-in-release.flag` -- excludes the module from release builds.

## Talks over

- `\\.\pipe\WKOpenVR-Phantom` (overlay -> driver): config push (master switch, timeout ladder) and per-device opt-in. Opened by the driver only when `enable_phantom.flag` is present.
- `WKOpenVRPhantomStateV2` shared memory (driver -> overlay): per-tracker state badges (REAL / RECKON / IK / ML / OUTOFRANGE / LOST), dropout counters, and body-completion diagnostics at ~10 Hz.
- `WKOpenVRPhantomInferenceInV1` / `WKOpenVRPhantomInferenceOutV1` shared memory (driver <-> sidecar): reserved for the optional backend; unused by the in-driver completion path.

## Build and test

Built as part of the umbrella build. Run the module suite with `./test.ps1 -Suite phantom`.

## Logs

Under `%LocalAppDataLow%\WKOpenVR\Logs`: driver-side entries tagged `[phantom]` in `driver_log.*.log`, overlay-side diagnostics in `diagnostics_log.*.log`, and replay recordings as `phantom_replay.*.csv`.
