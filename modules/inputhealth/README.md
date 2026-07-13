# inputhealth module

Watches controller input health. The driver piece hooks the IVRDriverInput
boolean/scalar update path, collecting per-component statistics and applying
learned compensation (button debounce, stick deadzone, trigger remap); the
overlay piece shows diagnostics, learns rules, keeps profiles, pushes config.

## Pieces

- src/driver -- static library openvr_pair_feature_inputhealth_driver: vtable
  hook installer, per-component observation and statistics, compensation
  rules, and a worker that publishes snapshots to shared memory at ~10 Hz.
- src/overlay -- static library openvr_pair_feature_inputhealth_overlay:
  overlay plugin with diagnostics/settings/advanced/logs tabs, learning
  engine, profile store, pipe client, and shared-memory snapshot reader.
- tests -- unit suite covering debounce, deadzone, trigger remap, learning
  rules, health summary, config round-trip, and UI logic.
- scripts -- capture-CSV analysis helper with per-input diagnostics.

## Talks over

- \\.\pipe\WKOpenVR-InputHealth (overlay -> driver): config and control
  requests via the shared protocol structs (versioned by protocol::Version in
  core/src/common/Protocol.h); opened only when enable_inputhealth.flag exists.
- WKOpenVRInputHealthMemoryV1 shared memory (driver -> overlay): per-component
  statistics snapshots published at ~10 Hz.

## Build and test

Built as part of the umbrella build. Module suite: ./test.ps1 -Suite inputhealth

## Logs

Under %LocalAppDataLow%\WKOpenVR\Logs, written only when debug logging is on:
inputhealth_log.<date>T<time>.txt (overlay); driver side shares driver_log.<date>T<time>.txt.
