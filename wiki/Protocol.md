# Wire protocol

## Version

`protocol::Version = 28`, defined in [core/src/common/Protocol.h](https://github.com/RealWhyKnot/WKOpenVR/blob/main/core/src/common/Protocol.h). The handshake fails closed on mismatch -- a paired install where overlay and driver are different versions is rejected at connect time rather than silently misrouting request bytes.

### Bump discipline

Bump `Version` whenever:

- a wire struct's field offsets change (adding, removing, reordering, or resizing fields),
- a new `RequestType` enum value is added,
- a shared-memory layout changes.

Each bump is documented in a comment above the `const uint32_t Version = N;` line stating the reason. The pin in `modules/calibration/tests/test_protocol.cpp` is bumped in lockstep so an out-of-band version-skew between the test and the wire is caught at CI time.

## Named pipes

| Pipe | Direction | Module |
|---|---|---|
| `\\.\pipe\OpenVR-Calibration` | overlay <-> driver | calibration |
| `\\.\pipe\WKOpenVR-Smoothing` | overlay <-> driver | smoothing |
| `\\.\pipe\WKOpenVR-InputHealth` | overlay <-> driver | inputhealth |
| `\\.\pipe\WKOpenVR-Phantom` | overlay <-> driver | phantom |

Each overlay sends only its own request types. The driver routes by request type and rejects messages on the wrong pipe. Pipe names are exposed in `Protocol.h` as `OPENVR_PAIRDRIVER_*_PIPE_NAME` macros so both sides resolve them through a single source of truth.

## Shared-memory segments

| Segment | Producer -> consumer | Notes |
|---|---|---|
| `WKOpenVRPoseMemoryV2` | driver -> overlay | calibration pose telemetry |
| `WKOpenVRInputHealthMemoryV1` | driver -> overlay | 10 Hz input-health snapshot ring |
| `WKOpenVRPhantomStateV2` | driver -> overlay | phantom dropout state and role-completion diagnostics |

The seqlock discipline used by the shmem classes:

1. Producer increments a per-slot generation counter to an odd value, writes the slot body, increments again to an even value.
2. Consumer reads the generation, then the body, then the generation again. If both reads return the same even value and that value matches the body's slot index, the body is consistent and accepted; otherwise the read is retried.
3. Each segment has a magic + version header validated at attach time.

## Request union

`RequestType` enum (in commit order):

| Request | Since | Payload |
|---|---|---|
| `RequestHandshake` | v1 | client identifies module + protocol version |
| `RequestSetDeviceTransform` | v1 | calibration: per-device offset + scale |
| `RequestSetAlignmentSpeedParams` | v1 | calibration: speed thresholds |
| `RequestDebugOffset` | v1 | calibration: nudge offset for diagnostics |
| `RequestSetTrackingSystemFallback` | v1 | calibration: fallback reference choice |
| `RequestSetFingerSmoothing` | v9 | smoothing: per-finger mask + strength |
| `RequestSetInputHealthConfig` | v10 | inputhealth: master + compensation family flags |
| `RequestResetInputHealthStats` | v10 | inputhealth: clear stats for a device |
| `RequestSetInputHealthCompensation` | v11 | inputhealth: push learned compensation curves |
| `RequestSetDevicePrediction` | v12 | smoothing: per-device pose-prediction strength |

Responses are limited to `ResponseHandshake` and `ResponseSuccess`; failure paths use named-pipe error codes so the driver doesn't have to invent its own.

## Compile-time invariants

Each POD payload is `static_assert`-checked against `sizeof(SetDeviceTransform)` so the request union's storage stays large enough. Adding a payload bigger than the current union size needs both an explicit bump of the underlying buffer and a corresponding update to the static_assert chain in `Protocol.h`.
