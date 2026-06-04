# WKOpenVR

Umbrella SteamVR overlay + driver. One binary (`WKOpenVR.exe`) and one driver DLL (`driver_wkopenvr.dll`) host the release modules under `modules/`.

## Built on top of

The calibration module builds on an existing open-source project; the other release modules were written in this repository.

**Calibration** descends from [pushrax/OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) by Justin Li (MIT). The fork chain that reached here:

- [pushrax/OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) -- original (Justin Li)
- [bdunderscore/OpenVR-SpaceCalibrator](https://github.com/bdunderscore/OpenVR-SpaceCalibrator) -- intermediate fork
- [ArcticFox8515/OpenVR-SpaceCalibrator](https://github.com/ArcticFox8515/OpenVR-SpaceCalibrator) -- intermediate fork
- [hyblocker/OpenVR-SpaceCalibrator](https://github.com/hyblocker/OpenVR-SpaceCalibrator) by Hyblocker -- the fork this repository was directly seeded from
- this repository's `modules/calibration/`

Calibration code that traces back to any of those forks remains available under MIT terms from each origin repo; the combined work in this repository is GPL-3.0.

Third-party libraries linked into the binary (OpenVR SDK, MinHook, ImGui, ImPlot, GLFW, Eigen, picojson, stb_image, gl3w, whisper.cpp, CTranslate2, ONNX Runtime, .NET runtime libraries) carry their own licenses; the full texts are reproduced in [NOTICE](NOTICE).

## Modules in release

- **calibration** -- continuous calibration of HMDs against lighthouse-tracked full-body trackers.
- **smoothing** -- One-Euro finger smoothing and per-device pose-prediction suppression for Valve Index Knuckles.
- **inputhealth** -- per-button / per-axis / per-finger drift and degradation detection with learned compensation.

Each is wired up at SteamVR startup by a marker file in the driver's `resources/` directory: `enable_calibration.flag`, `enable_smoothing.flag`, `enable_inputhealth.flag`. The umbrella overlay's Modules tab toggles these flags at runtime.

## Why one DLL instead of one per module

A SteamVR driver hooks into `vrserver.exe` via MinHook. MinHook is process-global: only one detour can exist per target function. Independently-installed driver DLLs trying to patch the same slot of `IVRDriverContext::GetGenericInterface` would collide; the second install silently fails and that driver's detours never fire. Sharing one DLL means one MinHook install per target function regardless of which features are enabled.

## Build

Requires CMake 3.15+, Visual Studio Build Tools 2022 (or the VS 2022 IDE), and submodules initialised.

```
git clone --recursive https://github.com/RealWhyKnot/WKOpenVR
cd WKOpenVR
./build.ps1
```

Output:

- `build/artifacts/Release/WKOpenVR.exe`
- `build/driver_wkopenvr/bin/win64/driver_wkopenvr.dll`

For local SteamVR iteration, run `./quick.ps1`. It builds, closes SteamVR and Steam for the deploy copy, installs the overlay files plus the full driver tree into the local SteamVR install, verifies deployed hashes, then launches SteamVR through Steam. Run `./test.ps1` when you only need a build plus the local test suite.

## Diagnostics

Debug logging is controlled from the Logs tab. When enabled, logs are written under `%LocalAppDataLow%\WKOpenVR\Logs\`; calibration also writes a replayable `spacecal_log.<ts>.txt` CSV that opens as soon as logging is enabled and flushes every row to disk. The Logs tab shows the active SpaceCal file path, size, row count, annotation count, open status, and a flush button.

## Pipes and shared memory

- `\\.\pipe\OpenVR-Calibration` -- calibration overlay <-> driver
- `\\.\pipe\WKOpenVR-Smoothing` -- smoothing overlay <-> driver
- `\\.\pipe\WKOpenVR-InputHealth` -- input-health overlay <-> driver

Plus the shmem segment `WKOpenVRInputHealthMemoryV1` for the input-health snapshot stream.

Wire format is defined in [core/src/common/Protocol.h](core/src/common/Protocol.h). Each overlay sends only its own request types; the driver routes by request type and rejects messages on the wrong pipe. The handshake fails fast on version skew so a mismatched pair is caught at startup rather than misrouting bytes.

## In development

These modules live under `modules/` and build in dev (`./build.ps1` with no `-Release` flag), but are excluded from published release artifacts via `modules/<slug>/disabled-in-release.flag`. They are exercised in the test suite but are not yet stable enough to ship. Removing the flag re-enables the matching release artifact on the next tag.

- **facetracking** -- face and eye tracking via a C# .NET 10 host sidecar that loads hardware-vendor VRCFT modules, normalises against Unified Expressions, and feeds the driver over a shared-memory ring. Adds the `\\.\pipe\WKOpenVR-FaceTracking` pipe and the `WKOpenVRFaceTrackingFrameRingV2` shmem ring. Toggled by `enable_facetracking.flag`.
- **oscrouter** -- OSC fan-out of pose / tracker / chatbox data to multiple downstream consumers with per-route filtering. Toggled by `enable_oscrouter.flag`.
- **captions** -- on-device speech recognition and translation via a whisper.cpp + CTranslate2 host sidecar with Silero VAD, OSC chatbox routing, and per-pack model download / install. Toggled by `enable_captions.flag`.
- **phantom** -- tracker dropout dead-reckoning, IK fallback, and virtual-tracker placeholders for missing hardware, with an out-of-process ML pose-completion sidecar. Toggled by `enable_phantom.flag`.

## License

GNU General Public License v3.0; see [LICENSE](LICENSE). Project copyright lines and the full third-party license texts are in [NOTICE](NOTICE).
