# WKOpenVR

Umbrella SteamVR overlay + driver. One binary (`WKOpenVR.exe`) and one driver DLL (`driver_wkopenvr.dll`) host six feature modules under `modules/`:

- **calibration** -- continuous calibration of HMDs against lighthouse-tracked full-body trackers. Forked from the original OpenVR-SpaceCalibrator. Mirrors releases to [WKOpenVR-SpaceCalibrator](https://github.com/RealWhyKnot/WKOpenVR-SpaceCalibrator).
- **smoothing** -- One-Euro finger smoothing and per-device pose-prediction suppression for Valve Index Knuckles. Mirrors releases to [WKOpenVR-Smoothing](https://github.com/RealWhyKnot/WKOpenVR-Smoothing).
- **inputhealth** -- per-button / per-axis / per-finger drift and degradation detection with learned compensation. Mirrors releases to [WKOpenVR-InputHealth](https://github.com/RealWhyKnot/WKOpenVR-InputHealth).
- **facetracking** -- face and eye tracking via a C# .NET 10 host sidecar that loads hardware-vendor modules, normalises against Unified Expressions, and feeds the driver over a shared-memory ring. Mirrors releases to [WKOpenVR-VRCFT](https://github.com/RealWhyKnot/WKOpenVR-VRCFT).
- **captions** -- on-device speech recognition and translation via a whisper.cpp + ctranslate2 host sidecar with Silero VAD, OSC chatbox routing, and per-pack model download / install. Mirrors releases to [WKOpenVR-Captions](https://github.com/RealWhyKnot/WKOpenVR-Captions).
- **phantom** -- tracker dropout dead-reckoning, IK fallback, and virtual-tracker placeholders for missing hardware, with an out-of-process ML pose-completion sidecar. Mirrors releases to [WKOpenVR-Phantom](https://github.com/RealWhyKnot/WKOpenVR-Phantom).

Each feature is wired up at SteamVR startup based on a marker file in the driver's `resources/` directory:

- `enable_calibration.flag` -- pose-update hook + calibration IPC pipe
- `enable_smoothing.flag` -- skeletal hook + smoothing IPC pipe
- `enable_inputhealth.flag` -- boolean / scalar input hooks + input-health IPC pipe
- `enable_facetracking.flag` -- face-tracking host sidecar + IPC pipe + shmem ring
- `enable_captions.flag` -- captions host sidecar + OSC chatbox publisher + IPC pipe
- `enable_phantom.flag` -- dropout detector + dead-reckoning + virtual-tracker device adds + sidecar bridge

The umbrella overlay's Modules tab toggles these flags at runtime.

## Why one DLL instead of six

A SteamVR driver hooks into `vrserver.exe` via MinHook. MinHook is process-global: only one detour can exist per target function. Six independently-installed driver DLLs trying to patch the same slot of `IVRDriverContext::GetGenericInterface` would collide; the second install silently fails and that driver's detours never fire. Sharing one DLL means one MinHook install per target function regardless of which features are enabled.

## Build

Requires CMake 3.15+, Visual Studio Build Tools 2022 (or the VS 2022 IDE), and submodules initialised. The .NET 10 SDK is needed for the face-tracking host; the build skips that target when the SDK is missing or when `-DOPENVR_PAIR_BUILD_FACE_HOST=OFF` is passed.

```
git clone --recursive https://github.com/RealWhyKnot/WKOpenVR
cd WKOpenVR
./build.ps1
```

Output:

- `build/artifacts/Release/WKOpenVR.exe`
- `build/driver_wkopenvr/bin/win64/driver_wkopenvr.dll`
- `build/driver_wkopenvr/resources/facetracking/host/WKOpenVR.FaceModuleHost.exe` (when the host build is enabled)

For local SteamVR iteration, run `./quick.ps1`. It builds, closes SteamVR and Steam for the deploy copy, installs the overlay files plus the full driver tree into the local SteamVR install, verifies deployed hashes, then launches SteamVR through Steam. Run `./test.ps1` when you only need a build plus the local test suite.

## Pipes and shared memory

- `\\.\pipe\OpenVR-Calibration` -- calibration overlay <-> driver
- `\\.\pipe\WKOpenVR-Smoothing` -- smoothing overlay <-> driver
- `\\.\pipe\WKOpenVR-InputHealth` -- input-health overlay <-> driver
- `\\.\pipe\WKOpenVR-FaceTracking` -- face-tracking overlay <-> driver

Plus the shmem ring `WKOpenVRFaceTrackingFrameRingV2` for high-rate samples from the C# host into the driver, and `WKOpenVRInputHealthMemoryV1` for the input-health snapshot stream.

Wire format is defined in [core/src/common/Protocol.h](core/src/common/Protocol.h) at protocol version 15. Each overlay sends only its own request types; the driver routes by request type and rejects messages on the wrong pipe. The handshake fails fast on version skew so a mismatched pair is caught at startup rather than misrouting bytes.

## Documentation

See the [wiki](https://github.com/RealWhyKnot/WKOpenVR/wiki) for architecture overview, per-module deep-dives, protocol reference, build environment notes, and the release pipeline.

## Upstreams and credits

Two of the four feature modules build on existing open-source projects. The other two were written from scratch inside this repository.

**Calibration** is descended from [pushrax/OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) by Justin Li (MIT). The fork chain that reached this repository was:

- [pushrax/OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) -- original (Justin Li)
- [bdunderscore/OpenVR-SpaceCalibrator](https://github.com/bdunderscore/OpenVR-SpaceCalibrator) -- intermediate fork
- [ArcticFox8515/OpenVR-SpaceCalibrator](https://github.com/ArcticFox8515/OpenVR-SpaceCalibrator) -- intermediate fork
- [hyblocker/OpenVR-SpaceCalibrator](https://github.com/hyblocker/OpenVR-SpaceCalibrator) by Hyblocker -- the fork this repository was directly seeded from
- this repository's `modules/calibration/`

Calibration code that traces back to any of those forks remains available under MIT terms from each origin repo; the combined work in this repository is GPL-3.0.

**Face tracking** is descended from [benaclejames/VRCFaceTracking](https://github.com/benaclejames/VRCFaceTracking) by Benjamin Clarke and contributors (Apache-2.0). The C# host sidecar, reflection bridge for upstream VRCFT modules, and the legacy module registry are derived from that project. The driver-side integration, shmem ring protocol, and overlay UI are new in this repository.

**Smoothing** and **input health** have no upstream -- both modules were authored in this repository.

The sibling mirror repos ([WKOpenVR-SpaceCalibrator](https://github.com/RealWhyKnot/WKOpenVR-SpaceCalibrator), [WKOpenVR-Smoothing](https://github.com/RealWhyKnot/WKOpenVR-Smoothing), [WKOpenVR-InputHealth](https://github.com/RealWhyKnot/WKOpenVR-InputHealth), [WKOpenVR-VRCFT](https://github.com/RealWhyKnot/WKOpenVR-VRCFT)) carry per-feature release zips and installers. Their `archive-source` branches hold either the original upstream source (for SpaceCalibrator and VRCFT) or the pre-monorepo state of the WK fork (for Smoothing and InputHealth). The release mirror workflow lives in [.github/workflows/release.yml](.github/workflows/release.yml).

Third-party libraries linked into the binary (OpenVR SDK, MinHook, ImGui, ImPlot, GLFW, Eigen, picojson, stb_image, gl3w) carry their own licenses; the full texts are reproduced in [NOTICE](NOTICE).

## License

GNU General Public License v3.0; see [LICENSE](LICENSE). Project copyright lines and the full third-party license texts are in [NOTICE](NOTICE).
