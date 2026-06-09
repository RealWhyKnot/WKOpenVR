#pragma once

// Per-feature pipe names. The driver opens up to three pipes depending on
// which enable_*.flag files are present in its resources directory; each
// consumer overlay connects only to its own pipe. Wire format on all pipes
// is the same protocol::Request/Response struct; the driver routes by
// request type and rejects out-of-feature requests.
#define OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-Calibration"
#define OPENVR_PAIRDRIVER_SMOOTHING_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-Smoothing"
#define OPENVR_PAIRDRIVER_DASHBOARDINPUT_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-DashboardInput"
#define OPENVR_PAIRDRIVER_INPUTHEALTH_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-InputHealth"
#define OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-FaceTracking"
// v16 (2026-05-13): OSC router module. Driver opens this pipe gated on
// enable_oscrouter.flag. Accepts route subscribe/unsubscribe, config push,
// and test-publish requests from the overlay.
#define OPENVR_PAIRDRIVER_OSCROUTER_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-OscRouter"
// Publish pipe: fire-and-forget from out-of-process sidecars. Wire format:
// 32-byte source-id, 4-byte LE frame length, then `length` bytes of raw OSC.
#define OPENVR_PAIRDRIVER_OSCROUTER_PUB_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-OscRouterPub"
// Captions module. Driver opens this pipe gated on enable_captions.flag.
// Accepts RequestSetCaptionsConfig and RequestCaptionsRestartHost.
#define OPENVR_PAIRDRIVER_CAPTIONS_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-Captions"
// Phantom Trackers module. Driver opens this pipe gated on
// enable_phantom.flag. Phase 1 accepts RequestSetPhantomConfig (global
// master switch + timeout ladder) and RequestSetPhantomDeviceOptIn (per-
// device dropout bridging toggle). Later phases extend with calibration,
// virtual-tracker, and ML messages without further pipe additions.
#define OPENVR_PAIRDRIVER_PHANTOM_PIPE_NAME "\\\\.\\pipe\\WKOpenVR-Phantom"

// Pose telemetry shmem segment. Created by the driver only when the calibration
// feature is enabled; the calibration overlay opens it to read driver-side
// pose snapshots and telemetry counters.
// V2 bumps the segment name to WKOpenVR* (V1 was OpenVRPairPoseMemoryV1) so a
// half-updated install -- old driver + new overlay or vice versa -- fails
// loudly on attach instead of silently mapping a foreign segment.
#define OPENVR_PAIRDRIVER_SHMEM_NAME "WKOpenVRPoseMemoryV2"

// Input-health snapshot shmem segment. Created by the driver only when the
// inputhealth feature is enabled; the InputHealth overlay opens it to read
// per-component statistics published at ~10 Hz from the driver-side worker.
#define OPENVR_PAIRDRIVER_INPUTHEALTH_SHMEM_NAME "WKOpenVRInputHealthMemoryV1"

// Face-tracking per-frame shmem ring. Created by the driver only when the
// facetracking feature is enabled; the C# FaceModuleHost.exe opens it for
// write and publishes per-hardware-frame face/eye samples (~120 Hz) into a
// 32-slot ring. The driver reads the latest frame on its pose-update path
// and applies calibration / eyelid-sync / vergence-lock before publishing to
// SteamVR inputs. Single writer (host) / single reader (driver).
// V2 bumps the segment name to WKOpenVR* (V1 was
// OpenVRPairFaceTrackingFrameRingV1) so a half-updated install fails loudly
// on attach instead of mapping a stale ring with the wrong shape.
#define OPENVR_PAIRDRIVER_FACETRACKING_SHMEM_NAME "WKOpenVRFaceTrackingFrameRingV2"
// OSC router stats shmem. Created by the driver when oscrouter is enabled;
// the overlay reads it at ~10 Hz to show per-route message rates.
#define OPENVR_PAIRDRIVER_OSCROUTER_SHMEM_NAME "WKOpenVROscRouterStatsV1"

// Phantom-tracker per-device state shmem. Created by the driver only when
// the phantom feature is enabled; the Phantom overlay opens it to render
// per-tracker state badges (REAL / RECKON / IK / ML / OUTOFRANGE / LOST)
// and dropout-count counters at ~10 Hz. V2 adds per-role body completion
// confidence/source diagnostics. Single writer (driver) / single reader
// (overlay).
#define OPENVR_PAIRDRIVER_PHANTOM_STATE_SHMEM_NAME "WKOpenVRPhantomStateV2"

// Phantom optional backend shmems. Not used by the deterministic in-driver
// completion path; kept stable for a future out-of-process backend.
#define OPENVR_PAIRDRIVER_PHANTOM_INFERENCE_IN_SHMEM_NAME "WKOpenVRPhantomInferenceInV1"
#define OPENVR_PAIRDRIVER_PHANTOM_INFERENCE_OUT_SHMEM_NAME "WKOpenVRPhantomInferenceOutV1"
