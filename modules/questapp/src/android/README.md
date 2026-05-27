# WKOpenVR Quest Companion

Quest-side companion app for the Quest App module.

The Windows module installs the APK with ADB and starts
`org.wkopenvr.quest/.QuestCompanionService` with `wkopenvr_pairing_key`.
The service stores the first valid key it receives and rejects later
configuration or launch requests unless the same key is supplied. If the
Windows-side key is deleted, the recovery path is uninstalling and reinstalling
the companion so both sides establish a fresh key.

Expected package name: `org.wkopenvr.quest`

Expected foreground service: `org.wkopenvr.quest/.QuestCompanionService`
