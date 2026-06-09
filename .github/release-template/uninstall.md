## Uninstall

If you installed via the NSIS setup exe: close SteamVR, then run `Uninstall.exe` from `C:\Program Files\WKOpenVR\` (or use Add/Remove Programs). The uninstaller removes the install dir, the SteamVR driver tree, user data under `%LocalAppDataLow%\WKOpenVR\`, Start Menu shortcuts, and the relevant registry keys. Files that are still locked are queued for next-reboot deletion and you are prompted to reboot at the end.

To disable a single feature without uninstalling: from inside WKOpenVR, open the **Modules** tab and untick the feature (a UAC prompt removes its `enable_<feature>.flag` and Start Menu shortcut). Or delete the flag manually from `<SteamVR runtime>\drivers\01wkopenvr\resources\`:

- `enable_calibration.flag` -- disables calibration
- `enable_smoothing.flag` -- disables finger smoothing
- `enable_inputhealth.flag` -- disables input health monitoring
- `enable_oscrouter.flag` -- disables OSC routing
- `enable_facetracking.flag` -- disables face / eye tracking
- `enable_questapp.flag` -- disables Quest App tools
- `enable_captions.flag` -- disables captions

Restart SteamVR. With no flag files present the driver loads but stays inert (no hooks, no pipes).

To remove the driver entirely without using the uninstaller: delete `<SteamVR runtime>\drivers\01wkopenvr\` while SteamVR is closed.
