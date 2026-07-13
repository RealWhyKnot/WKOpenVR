# captions module

Real-time speech-to-text captions posted to the VRChat chatbox. A sidecar process
(WKOpenVR.CaptionsHost.exe) captures microphone audio, gates it with an energy
threshold plus Silero VAD, transcribes with Whisper (Vulkan GPU by default, CPU
fallback), and optionally translates; the driver supervises the sidecar and the
overlay carries the configuration tab.

## Pieces

- src/driver -- static library linked into driver_wkopenvr.dll: launches and
  supervises the host process and serves captions requests from the overlay.
- src/host -- the WKOpenVR.CaptionsHost.exe sidecar: WASAPI capture, speech
  gating, Whisper transcription, translation, chatbox rate-limit pacing,
  model/pack download, SteamVR push-to-talk bindings, status reporting.
- src/overlay -- static library linked into WKOpenVR.exe: Captions tab with
  input-device picker, level meter, caption preview, and the driver IPC client.
- tests -- unit test suite (captions_tests).

## Talks over

- \\.\pipe\WKOpenVR-Captions (OPENVR_PAIRDRIVER_CAPTIONS_PIPE_NAME in
  core/src/common/ProtocolNames.h), overlay -> driver: config push and host restart.
- \\.\pipe\WKOpenVR-Captions.host, driver -> host: control pipe served by the
  host; the driver-side supervisor writes config and control commands.
- \\.\pipe\WKOpenVR-OscRouterPub, host -> OSC router (driver): raw chatbox OSC
  frames the router forwards to VRChat. Requires the oscrouter module.
- %LocalAppDataLow%\WKOpenVR\captions\host_status.json, host -> overlay: status
  snapshot written at most once per second and polled by the overlay.

## Build and test

`./test.ps1 -Suite captions` builds and runs the module suite. Pass
`-CaptionsCpuOnly` to build the host without the Vulkan SDK.

## Logs

Under %LocalAppDataLow%\WKOpenVR\Logs, when debug logging is enabled:
captions_drv_log_<timestamp>.txt (driver) and captions_host_log_<timestamp>.txt
(host). The overlay piece logs through the umbrella overlay log.
