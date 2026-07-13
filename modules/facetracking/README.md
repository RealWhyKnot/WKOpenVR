# facetracking module

Streams eye and face expression data into SteamVR inputs and avatar OSC
parameters. A C# sidecar (FaceModuleHost) runs VRCFaceTracking-SDK tracking
modules out of process and publishes raw frames; the driver piece filters,
calibrates, and eyelid-syncs them; the overlay piece is the settings UI.

## Pieces

- src/driver -- static library in driver_wkopenvr.dll: frame reading, signal
  processing, calibration, eyelid sync, vergence lock, sidecar supervision.
- src/host -- C# solution: FaceModuleHost sidecar, sandboxed per-module
  worker process, registry client, and the VRCFaceTracking runtime fork.
- src/overlay -- static library in WKOpenVR.exe: settings, tuning, module
  sources, profiles, and log tabs.
- manifest -- enable_facetracking.flag; enables the feature when present in
  the driver resources directory.
- tests -- the facetracking_tests suite.

## Talks over

- \\.\pipe\WKOpenVR-FaceTracking (overlay -> driver): feature command pipe;
  protocol structs versioned by protocol::Version in core/src/common/Protocol.h.
- WKOpenVRFaceTrackingFrameRingV2 shmem (host -> driver): 32-slot ring of
  per-hardware-frame face/eye samples; single writer, single reader.
- \\.\pipe\WKOpenVR-FaceTracking.host (driver -> host): length-prefixed CBOR
  control messages.
- Avatar OSC output goes through the oscrouter module in process; the host
  also advertises OSCQuery for discovery.

## Build and test

./test.ps1 -Suite facetracking builds and runs the module suite; the C# host
builds from its own solution under src/host.

## Logs

Under %LocalAppDataLow%\WKOpenVR\Logs: facetracking_log.<timestamp>.txt
(overlay), facetracking_drv_log.* (driver), facetracking_host_log.* (host).
