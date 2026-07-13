# dev module

Dev-build-only tab inside the WKOpenVR overlay. It is an overlay piece with
no driver or sidecar part: the tab lists modules still gated out of release
builds and hosts the dev-tool panels other modules expose, so in-development
features stay reachable from one place without shipping in release binaries.

## Pieces

- `src/overlay` -- static library `openvr_pair_feature_dev_overlay` with the
  Dev tab. Compiled and linked into `WKOpenVR.exe` only when
  `WKOPENVR_RELEASE_BUILD` is off; release builds contain none of it.

## Talks over

- Nothing of its own: no flag file, no named pipe, no shared memory. Each
  dev-tool panel the tab hosts talks over the owning module's channel (see
  that module's README and `core/src/common/ProtocolNames.h`).

## Build and test

No dedicated test suite; the module is covered by the umbrella overlay build
in the dev configuration.

## Logs

No dedicated log file. The tab runs inside the overlay process, which writes
`diagnostics_log.<timestamp>.txt` under `%LocalAppDataLow%\WKOpenVR\Logs`
when debug logging is enabled.
