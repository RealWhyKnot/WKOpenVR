## PLEASE READ -- how the per-feature installers work

Each `WKOpenVR-<Feature>-v{version}-Setup.exe` turns on only that one feature on first launch. The installer drops the matching `enable_<feature>.flag` into the driver's resources directory so the feature is active the next time SteamVR starts.

You are not locked in by which installer you pick. Every feature is just a flag file under `<SteamVR runtime>\drivers\01wkopenvr\resources\`, and the WKOpenVR overlay's Modules tab toggles those flags from inside VR. If you install Calibration-only today and want Smoothing too tomorrow, open the Modules tab and turn it on. Same for turning anything off again. No reinstall needed.

Pick the installer that matches what you want enabled out of the gate; the Modules tab handles everything after that.
