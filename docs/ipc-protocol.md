# IPC protocol

How WKOpenVR.exe (overlay) and driver_wkopenvr.dll (driver, inside vrserver)
talk to each other. Definitions live in `core/src/common/Protocol.h`,
`core/src/common/ProtocolNames.h`, and `core/src/common/ProtocolPayloads.h`
(included through Protocol.h). Both binaries compile against the same headers.

## Transport

1. **Named pipes, request/response.** The driver opens one duplex message-mode
   pipe per enabled feature (gated on the `enable_<feature>.flag` files), with
   overlapped I/O and unlimited instances (`core/src/driver/IPCServer.cpp`).
   Every pipe carries the same wire format: one fixed-size `protocol::Request`
   struct per message in, one fixed-size `protocol::Response` out. The driver
   routes by request type and rejects requests that do not belong to the
   pipe's feature. Partial messages are dropped and the client disconnected.
   The overlay side (`IpcClientBase` in `core/src/overlay`) connects with
   `WaitNamedPipe` + `CreateFile`, switches the handle to message read mode,
   and sends `RequestHandshake` before anything else.
   Exception: the OscRouterPub pipe is one-way fire-and-forget for
   out-of-process sidecars. Its framing is a 32-byte source-id once per
   connection, then repeated frames of 4-byte little-endian length + raw OSC
   packet until disconnect.
2. **Shared-memory segments** for high-rate or polled data that would be
   wasteful over the pipe. Each segment has a header with a magic constant and
   a layout version, and uses atomics/seqlocks so readers detect torn writes.

Shared-memory channels defined in Protocol.h:

| Class | Segment | Writer | Reader | Purpose |
|---|---|---|---|---|
| `DriverPoseShmem` | WKOpenVRPoseMemoryV2 | driver | calibration overlay | Ring of driver-side pose snapshots + telemetry counters |
| `InputHealthSnapshotShmem` | WKOpenVRInputHealthMemoryV1 | driver (~10 Hz) | InputHealth overlay | Per-input-component statistics slot table |
| `FaceTrackingFrameShmem` | WKOpenVRFaceTrackingFrameRingV2 | FaceModuleHost.exe (~120 Hz) | driver | 32-slot ring of per-frame face/eye samples |
| `OscRouterStatsShmem` | WKOpenVROscRouterStatsV1 | driver (~10 Hz) | overlay | Per-route OSC counters for the Modules tab |
| `PerfStatsShmem` | WKOpenVRPerfStatsV1 | driver (1 Hz) | overlay | Process + per-module CPU/memory attribution; the only always-on segment, created in driver Init regardless of feature flags |

The Phantom module defines three more segments whose names live in
ProtocolNames.h and whose classes live under `modules/phantom/src/common`:
WKOpenVRPhantomStateV2 (driver -> overlay per-tracker state badges) and the
WKOpenVRPhantomInference{In,Out}V1 pair (reserved for an out-of-process
backend; unused by the in-driver completion path).

## Versioning

- `protocol::Version` in `core/src/common/Protocol.h` is the single wire
  version for all pipes. On connect, the overlay sends `RequestHandshake`; the
  driver replies `ResponseHandshake` carrying its version. Any mismatch makes
  the client close the pipe and surface an error immediately -- no degraded
  mode, no partial compatibility.
- Every shared-memory segment carries its own magic + version in its header,
  validated at `Open()`. Compile-time `static_assert`s pin the struct layouts.
- Bump discipline: any struct or layout change visible on the wire -- request
  or response payloads, union membership, shmem headers or slot bodies --
  bumps `protocol::Version` (and the affected `SHMEM_VERSION`). Request and
  response enum ordinals are append-only; retired entries keep a reserved slot
  so later ordinals stay stable.
- On-disk calibration profile compatibility is a separate contract. A protocol
  bump forces a paired overlay+driver reinstall, but existing profiles must
  keep loading unchanged; profile schema changes are versioned and migrated
  independently of the wire version.

## Message catalog

Requests are overlay -> driver; the driver answers each with a response
(`ResponseSuccess` unless noted). Payload columns name the union member in
ProtocolPayloads.h; see that header for field-level detail.

| Area | Name | Payload | Purpose |
|---|---|---|---|
| Core | RequestHandshake | none | Version check; answered by ResponseHandshake |
| Core | RequestSetFreezeAllTracking | FreezeConfig | Hold all device poses; resent ~1 Hz as a heartbeat, driver fails open to live tracking |
| Calibration | RequestSetDeviceTransform | SetDeviceTransform | Per-device calibration offset, scale, hide, blend flags |
| Calibration | RequestSetAlignmentSpeedParams | AlignmentSpeedParams | Blend thresholds and speeds for calibration alignment |
| Calibration | RequestDebugOffset | none | Apply a random test offset (debug aid) |
| Calibration | RequestSetTrackingSystemFallback | SetTrackingSystemFallback | Default transform for devices of a tracking system without a per-ID transform |
| Calibration | RequestSetHeadMountConfig | SetHeadMountConfig | Head-mounted tracker mode, rigid offset, synth timing |
| Smoothing | RequestSetFingerSmoothing | FingerSmoothingConfig | Per-finger skeletal smoothing strengths |
| Smoothing | RequestSetDevicePrediction | SetDevicePrediction | Per-device pose-prediction suppression strength |
| InputHealth | RequestSetInputHealthConfig | InputHealthConfig | Master and per-category toggles for the input detours |
| InputHealth | RequestResetInputHealthStats | InputHealthResetStats | Wipe accumulated stats for one device |
| InputHealth | RequestSetInputHealthCompensation | InputHealthCompensationEntry | Push a learned per-component compensation curve |
| FaceTracking | RequestSetFaceTrackingConfig | FaceTrackingConfig | Master face config: toggles, sync strengths, OSC endpoint |
| FaceTracking | RequestSetFaceCalibrationCommand | FaceCalibrationCommand | Save/reset driver-side learned calibration state |
| FaceTracking | RequestSetFaceActiveModule | FaceModuleSelection | Pick the hardware module the host sidecar loads |
| FaceTracking | RequestFaceHostRestart | none | Terminate and respawn the face host sidecar |
| FaceTracking | RequestSetFaceShapeTuning | FaceShapeTuning | Per-expression min/max output remap + calibration exclusion |
| OSC router | RequestOscRouteSubscribe | OscRouteSubscribe | Register interest in an OSC address pattern |
| OSC router | RequestOscRouteUnsubscribe | OscRouteUnsubscribe | Remove a route registration |
| OSC router | RequestOscPublish | OscPublish | Send one OSC datagram (e.g. test message) |
| OSC router | RequestOscGetStats | none | Counters snapshot; answered by ResponseOscRouterStats |
| OSC router | RequestSetOscRouterConfig | OscRouterConfig | Live edit of the outbound UDP send port |
| Captions | RequestSetCaptionsConfig | CaptionsConfig | Mic, language, and mode settings forwarded to the sidecar |
| Captions | RequestCaptionsRestartHost | none | Terminate and respawn the captions sidecar |
| Captions | RequestCaptionsGetSupervisorStatus | none | Answered by ResponseCaptionsSupervisorStatus |
| Phantom | RequestSetPhantomConfig | PhantomConfig | Master switch, timeout ladder, virtual render model |
| Phantom | RequestSetPhantomDeviceOptIn | PhantomDeviceOptIn | Per-device dropout bridging toggle |
| Phantom | RequestSetPhantomDeviceRole | PhantomDeviceRole | Map a tracker serial hash to a body role |
| Phantom | RequestSetPhantomTrackerOffset | PhantomTrackerOffset | Reserved legacy per-role rigid offset |
| Phantom | RequestSetPhantomVirtualEnabled | PhantomVirtualEnabled | Toggle a virtual tracker for a role without hardware |
| Phantom | RequestSetPhantomSolverConfig | PhantomSolverConfig | Body-completion solver confidence settings |
| Phantom | RequestSnapCalibrate | PhantomSnapCalibrate | One-shot role assignment from the current static pose; answered by ResponsePhantomSnap |
| Reserved | RequestReservedDashboardHandTrackingState | n/a | Retired slot kept to preserve request ordinals |

Responses (driver -> overlay):

| Name | Payload | Purpose |
|---|---|---|
| ResponseHandshake | Protocol | Driver's protocol version |
| ResponseSuccess | none | Generic acknowledgment |
| ResponseOscRouterStats | OscRouterStats | Route counters snapshot |
| ResponseCaptionsSupervisorStatus | CaptionsSupervisorStatus | Sidecar supervisor state, last exit code/description |
| ResponsePhantomSnap | PhantomSnapResult | Snap status + assigned tracker count |

## Pipe and segment names

Exact strings from `core/src/common/ProtocolNames.h`:

Pipes (request/response unless noted):

- `\\.\pipe\WKOpenVR-Calibration`
- `\\.\pipe\WKOpenVR-Smoothing`
- `\\.\pipe\WKOpenVR-InputHealth`
- `\\.\pipe\WKOpenVR-FaceTracking`
- `\\.\pipe\WKOpenVR-OscRouter`
- `\\.\pipe\WKOpenVR-OscRouterPub` (one-way publish framing, see Transport)
- `\\.\pipe\WKOpenVR-Captions`
- `\\.\pipe\WKOpenVR-Phantom`

Shared-memory segments:

- `WKOpenVRPoseMemoryV2`
- `WKOpenVRInputHealthMemoryV1`
- `WKOpenVRFaceTrackingFrameRingV2`
- `WKOpenVROscRouterStatsV1`
- `WKOpenVRPhantomStateV2`
- `WKOpenVRPhantomInferenceInV1`
- `WKOpenVRPhantomInferenceOutV1`
- `WKOpenVRPerfStatsV1`

## Changing the protocol

1. Bump `protocol::Version` in `core/src/common/Protocol.h` and add a dated
   comment describing what changed and why the bump is needed.
2. Update the pinned-version test in `modules/calibration/tests`
   (test_protocol.cpp asserts the current version and the request ordinals).
3. Append new request/response types at the end of their enums; never reorder
   or reuse ordinals. Keep the `sizeof(Request)` static_asserts in
   ProtocolPayloads.h in step with any payload growth.
4. If a shmem layout changed, bump that class's `SHMEM_VERSION` and its
   static_asserts; bump the segment name suffix when a half-updated install
   must fail loudly at attach.
5. Rebuild and deploy WKOpenVR.exe and driver_wkopenvr.dll together; the
   handshake rejects a skewed pair.
6. Add a row to the version history table below.

## Version history

| Version | Date | Change |
|---|---|---|

Entries start from the `protocol::Version` value current when this table was
added; earlier changes are documented in the version comment block at the
top of the `protocol` namespace in `core/src/common/Protocol.h`.
