# WKOpenVR.FaceModuleHost

This directory contains the C# (.NET 10) host sidecar that loads hardware face and eye tracking
vendor modules, normalises their per-frame output, and publishes frames into the named shared-memory
ring `WKOpenVRFaceTrackingFrameRingV2`. The SteamVR driver reads that ring on its pose-update
path and applies continuous calibration, eyelid sync, and vergence lock before pushing data to
SteamVR inputs and the OSC sender.

Install upstream VRCFaceTracking modules under
`%LocalAppDataLow%\WKOpenVR\facetracking\modules\<uuid>\<version>\` with a `manifest.json`
matching the v1 schema served at `legacy-registry.whyknot.dev`. The host discovers modules on
startup and responds to `SelectModule` messages over the driver control pipe.

The runtime path loads upstream `ExtTrackingModule` assemblies directly in
`WKOpenVR.FaceModuleProcess.exe`. A legacy bridge-style manifest is still accepted: if
`entry_type` names the old compat adapter, the host reads `assemblies\bridge.json` and launches
the `upstream_assembly` DLL directly. The adapter DLL is not loaded.
