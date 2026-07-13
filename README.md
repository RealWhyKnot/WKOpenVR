# WKOpenVR

SteamVR overlay + driver source for WKOpenVR. One binary (`WKOpenVR.exe`) and one driver DLL (`driver_wkopenvr.dll`) host the modules under `modules/`.

End-user installers are published from the module repositories. Each installer drops the shared WKOpenVR driver and enables its module; other modules can still be enabled from the Modules tab inside WKOpenVR.

- Space Calibrator: https://github.com/RealWhyKnot/WKOpenVR-SpaceCalibrator
- Smoothing: https://github.com/RealWhyKnot/WKOpenVR-Smoothing
- Input Health: https://github.com/RealWhyKnot/WKOpenVR-InputHealth
- OSC Router: https://github.com/RealWhyKnot/WKOpenVR-OSCRouter
- Face Tracking: https://github.com/RealWhyKnot/WKOpenVR-FaceTracking
- Quest App: https://github.com/RealWhyKnot/WKOpenVR-QuestApp
- Captions: https://github.com/RealWhyKnot/WKOpenVR-Captions

## Built on top of

The calibration and face-tracking modules build on existing open-source work; the other release modules were written in this repository.

**Calibration** descends from SpaceCalibrator by Justin Li (MIT). The fork chain that reached here:

- https://github.com/pushrax/OpenVR-SpaceCalibrator -- original (Justin Li)
- https://github.com/bdunderscore/OpenVR-SpaceCalibrator -- intermediate fork
- https://github.com/ArcticFox8515/OpenVR-SpaceCalibrator -- intermediate fork
- https://github.com/hyblocker/OpenVR-SpaceCalibrator -- the fork this repository was directly seeded from
- this repository's `modules/calibration/`

Calibration code that traces back to any of those forks remains available under MIT terms from each origin repo; the combined work in this repository is GPL-3.0.

**Face Tracking** includes runtime components from VRCFaceTracking under Apache-2.0.

- https://github.com/benaclejames/VRCFaceTracking
- this repository's `modules/facetracking/`

Third-party libraries linked into the binary (OpenVR SDK, MinHook, ImGui, ImPlot, GLFW, Eigen, picojson, stb_image, gl3w, whisper.cpp, CTranslate2, ONNX Runtime, .NET runtime libraries) carry their own licenses; the full texts are reproduced in [NOTICE](NOTICE).

## Modules in release

- **calibration** -- continuous calibration of HMDs against lighthouse-tracked full-body trackers.
- **smoothing** -- One-Euro finger smoothing and per-device pose-prediction suppression for Valve Index Knuckles.
- **inputhealth** -- per-button / per-axis / per-finger drift and degradation detection with learned compensation.
- **oscrouter** -- OSC fan-out of pose, tracker, face, and chatbox data to multiple downstream consumers.
- **facetracking** -- face and eye tracking via a C# .NET 10 host sidecar that loads hardware-vendor VRCFT modules, normalizes against Unified Expressions, and feeds the driver over a shared-memory ring.
- **questapp** -- Quest companion setup, APK install, and Quest-side support tooling for WKOpenVR features.
- **captions** -- on-device speech recognition and translation via a whisper.cpp + CTranslate2 host sidecar with Silero VAD and OSC chatbox routing.

Each is wired up at SteamVR startup by a marker file in the driver's `resources/` directory: `enable_calibration.flag`, `enable_smoothing.flag`, `enable_inputhealth.flag`, `enable_oscrouter.flag`, `enable_facetracking.flag`, `enable_questapp.flag`, `enable_captions.flag`. The Modules tab toggles these flags at runtime.

## Why one DLL instead of one per module

A SteamVR driver hooks into `vrserver.exe` via MinHook. MinHook is process-global: only one detour can exist per target function. Independently-installed driver DLLs trying to patch the same slot of `IVRDriverContext::GetGenericInterface` would collide; the second install silently fails and that driver's detours never fire. Sharing one DLL means one MinHook install per target function regardless of which features are enabled.

## Modules

| Module | Purpose |
| --- | --- |
| [calibration](modules/calibration/README.md) | continuous playspace calibration between the HMD and lighthouse-tracked devices |
| [captions](modules/captions/README.md) | on-device speech recognition and translation with OSC chatbox routing |
| [dev](modules/dev/README.md) | dev-build overlay tab collecting development-module toggles and per-module dev tools |
| [dynamicres](modules/dynamicres/README.md) | automatic SteamVR render-resolution scaling under GPU load |
| [facetracking](modules/facetracking/README.md) | face and eye tracking through a .NET host that loads hardware-vendor VRCFT modules |
| [inputhealth](modules/inputhealth/README.md) | per-button, per-axis, and per-finger drift detection with learned compensation |
| [oscrouter](modules/oscrouter/README.md) | OSC fan-out of pose, tracker, face, and chatbox data |
| [phantom](modules/phantom/README.md) | tracker dropout dead-reckoning, IK fallback, and virtual placeholder trackers |
| [questapp](modules/questapp/README.md) | Quest companion setup, APK install, and Quest-side support tooling |
| [smoothing](modules/smoothing/README.md) | finger smoothing and pose-prediction suppression for Index controllers |

## Build

Requires CMake 3.15+, Visual Studio Build Tools 2022 (or the VS 2022 IDE), the .NET 10 SDK (without it the build silently skips the facetracking host), Python 3 (used by helper scripts in CI), and submodules initialised.

```
git clone --recursive https://github.com/RealWhyKnot/WKOpenVR
cd WKOpenVR
./build.ps1
```

Output:

- `build/artifacts/Release/WKOpenVR.exe`
- `build/driver_wkopenvr/bin/win64/driver_wkopenvr.dll`

For local SteamVR iteration, run `./quick.ps1`. It builds, closes SteamVR and Steam for the deploy copy, installs the overlay files plus the full driver tree into the local SteamVR install, verifies deployed hashes, then launches SteamVR through Steam. Run `./test.ps1` when you only need a build plus the local test suite.

### GPU-accelerated captions

The captions speech host is built with the Vulkan GPU backend (vendor-neutral -- NVIDIA, AMD, or Intel) by default. The standard `./build.ps1` and `./quick.ps1` build it and, when the [Vulkan SDK](https://vulkan.lunarg.com) (headers plus the `glslc` shader compiler) is missing, offer to download and install it; at runtime the host uses only the system `vulkan-1.dll`. Add `-InstallVulkanSdk` to accept the install without prompting, and `-VulkanSdkRoot <dir>` to install into a different directory. The LunarG installer requires administrator rights, so a UAC prompt appears during the install. With the backend compiled in, the host runs speech recognition on the GPU when a Vulkan device is present and falls back to the CPU automatically otherwise.

Building the Vulkan backend is recommended. To build a CPU-only captions host instead -- for example when you do not want to install the SDK -- pass `-CaptionsCpuOnly` to `build.ps1`, `quick.ps1`, or `test.ps1`:

```
./build.ps1 -CaptionsCpuOnly       # CPU-only build
./quick.ps1 -CaptionsCpuOnly       # CPU-only build + deploy for local SteamVR
```

## Diagnostics

Debug logging is controlled from the Logs tab. When enabled, logs are written under `%LocalAppDataLow%\WKOpenVR\Logs\`; calibration also writes a replayable `spacecal_log.<ts>.txt` CSV that opens as soon as logging is enabled and flushes every row to disk. The Logs tab shows the active SpaceCal file path, size, row count, annotation count, open status, and a flush button.

## Pipes and shared memory

- `\\.\pipe\WKOpenVR-Calibration` -- calibration overlay <-> driver
- `\\.\pipe\WKOpenVR-Smoothing` -- smoothing overlay <-> driver
- `\\.\pipe\WKOpenVR-InputHealth` -- input-health overlay <-> driver
- `\\.\pipe\WKOpenVR-FaceTracking` -- face-tracking overlay <-> driver
- `\\.\pipe\WKOpenVR-OscRouter` -- OSC router overlay <-> driver

Plus the shmem segments `WKOpenVRInputHealthMemoryV1` for the input-health snapshot stream and `WKOpenVRFaceTrackingFrameRingV2` for face-tracking frames.

Wire format is defined in [core/src/common/Protocol.h](core/src/common/Protocol.h). Each overlay sends only its own request types; the driver routes by request type and rejects messages on the wrong pipe. The handshake fails fast on version skew so a mismatched pair is caught at startup rather than misrouting bytes.

## In development

These modules live under `modules/` and build in dev (`./build.ps1` with no `-Release` flag), but are excluded from published release artifacts via `modules/<slug>/disabled-in-release.flag`. They are exercised in the test suite but are not yet stable enough to ship. Removing the flag re-enables the matching release artifact on the next tag.

- **phantom** -- tracker dropout dead-reckoning, IK fallback, and virtual-tracker placeholders for missing hardware, with an out-of-process ML pose-completion sidecar. Toggled by `enable_phantom.flag`.

## Documentation

- [docs/architecture.md](docs/architecture.md) -- process layout, module wiring, and host sidecars
- [docs/ipc-protocol.md](docs/ipc-protocol.md) -- pipe and shared-memory wire protocol

## License

GNU General Public License v3.0; see [LICENSE](LICENSE). Project copyright lines and the full third-party license texts are in [NOTICE](NOTICE).
