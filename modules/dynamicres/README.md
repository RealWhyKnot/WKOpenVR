# dynamicres module

Adjusts the SteamVR render resolution (supersample scale) automatically, lowering it when
GPU frame timings run over budget and raising it back when headroom returns. Everything
runs inside the overlay process as an ImGui tab plugin; there is no driver or sidecar piece.

## Pieces

- `src/overlay` -- overlay plugin (static lib `openvr_pair_feature_dynamicres_overlay`):
  GPU-pressure classification and scale controller, tab UI, and the settings profile
  persisted to `%LocalAppDataLow%\WKOpenVR\profiles\dynamicres.txt`.
- `tests` -- GoogleTest suite (`dynamicres_tests`) covering the controller logic and
  profile round-trip.
- `disabled-in-release.flag` -- marker that hides the module in release-channel builds.

## Talks over

- No named pipes or shared-memory segments. The overlay talks to SteamVR in-process
  through the OpenVR API: `IVRCompositor::GetFrameTimings` for GPU timing input, and
  `IVRSettings` reads/writes of the SteamVR supersample scale and manual-override keys.
- Enabled per install via the standard module flag file `enable_dynamicres.flag`
  (see `core/src/common/ModuleRegistry`).

## Build and test

Built as part of the umbrella overlay. Run the module suite with `./test.ps1 -Suite dynamicres`.

## Logs

Under `%LocalAppDataLow%\WKOpenVR\Logs`:

- `overlay_log.<timestamp>.txt` -- overlay process log, including this tab.
- `diagnostics_log.<timestamp>.txt` -- `[dynamicres]`-tagged decision lines (pressure
  classification, scale changes, baseline capture/restore); written only when debug
  logging is enabled.
