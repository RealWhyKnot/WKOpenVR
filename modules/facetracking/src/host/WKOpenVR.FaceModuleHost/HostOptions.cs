namespace WKOpenVR.FaceModuleHost;

/// <summary>
/// Runtime options for the host process. Populated from command-line args and
/// environment variable overrides. All paths default to the standard
/// %LocalAppDataLow%\WKOpenVR tree.
/// </summary>
public sealed class HostOptions
{
    /// <summary>Named pipe the driver uses to send control messages to the host.</summary>
    public string DriverHandshakePipe { get; set; } =
        @"\\.\pipe\WKOpenVR-FaceTracking.host";

    /// <summary>Name of the shmem ring the host writes into.</summary>
    public string ShmemName { get; set; } =
        "WKOpenVRFaceTrackingFrameRingV2";

    /// <summary>Directory where installed hardware modules live, one uuid/version sub-tree each.</summary>
    public string ModulesInstallDir { get; set; } = Path.Combine(
        AppPaths.FaceTrackingDir(), "modules");

    /// <summary>Where the host writes its periodic status JSON for the overlay to poll.</summary>
    public string StatusFilePath { get; set; } = Path.Combine(
        AppPaths.FaceTrackingDir(), "host_status.json");

    /// <summary>Optional explicit log file path passed by the driver.</summary>
    public string? LogFilePath { get; set; }

    public bool DebugLoggingEnabled { get; set; } = false;

    public bool E2eFakeFrames { get; set; } = false;
    public int E2eFakeFrameCount { get; set; } = 5;
    public int E2eFakeFrameIntervalMs { get; set; } = 5;
    public bool FaceReplayRecordEnabled { get; set; }
    public string FaceReplayDirectory { get; set; } = Path.Combine(
        AppPaths.FaceTrackingDir(), "replays");
    public double FaceReplayMaxHz { get; set; } = 30.0;
    public string? E2eSingletonSuffix { get; set; }
    public string? OwnerLivenessName { get; set; }
    public ulong OwnerLivenessNonce { get; set; }
    public bool OwnerLivenessConfigured => !string.IsNullOrWhiteSpace(OwnerLivenessName) && OwnerLivenessNonce != 0;

    public static HostOptions FromArgs(string[] args)
    {
        var opts = new HostOptions();
        bool faceReplayRecordConfigured = false;

        // Override from env vars first (lower priority than args).
        if (Environment.GetEnvironmentVariable("WKOPENVR_FACE_PIPE") is { } envPipe)
        {
            opts.DriverHandshakePipe = envPipe;
        }

        if (Environment.GetEnvironmentVariable("WKOPENVR_FACE_SHMEM") is { } envShmem)
        {
            opts.ShmemName = envShmem;
        }

        if (Environment.GetEnvironmentVariable("WKOPENVR_DEBUG_LOGGING") is { } envDebug)
        {
            opts.DebugLoggingEnabled = IsTruthy(envDebug);
        }

        if (Environment.GetEnvironmentVariable("WKOPENVR_FACE_REPLAY_RECORD") is { } envReplay)
        {
            opts.FaceReplayRecordEnabled = IsTruthy(envReplay);
            faceReplayRecordConfigured = true;
        }

        if (Environment.GetEnvironmentVariable("WKOPENVR_FACE_REPLAY_DIR") is { } envReplayDir &&
            !string.IsNullOrWhiteSpace(envReplayDir))
        {
            opts.FaceReplayDirectory = envReplayDir;
        }

        if (Environment.GetEnvironmentVariable("WKOPENVR_FACE_REPLAY_HZ") is { } envReplayHz &&
            double.TryParse(envReplayHz, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out double replayHz))
        {
            opts.FaceReplayMaxHz = Math.Max(0.0, replayHz);
        }

        // Command-line args override env vars.
        for (int i = 0; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--driver-handshake-pipe" when i + 1 < args.Length: opts.DriverHandshakePipe = args[++i]; break;
                case "--shmem-name" when i + 1 < args.Length: opts.ShmemName = args[++i]; break;
                case "--modules-dir" when i + 1 < args.Length: opts.ModulesInstallDir = args[++i]; break;
                case "--status-file" when i + 1 < args.Length: opts.StatusFilePath = args[++i]; break;
                case "--log-file" when i + 1 < args.Length: opts.LogFilePath = args[++i]; break;
                case "--debug-logging" when i + 1 < args.Length: opts.DebugLoggingEnabled = IsTruthy(args[++i]); break;
                case "--face-replay-record":
                    opts.FaceReplayRecordEnabled = true;
                    faceReplayRecordConfigured = true;
                    break;
                case "--no-face-replay-record":
                    opts.FaceReplayRecordEnabled = false;
                    faceReplayRecordConfigured = true;
                    break;
                case "--face-replay-dir" when i + 1 < args.Length: opts.FaceReplayDirectory = args[++i]; break;
                case "--face-replay-hz" when i + 1 < args.Length:
                    if (double.TryParse(args[++i], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out double argReplayHz))
                    {
                        opts.FaceReplayMaxHz = Math.Max(0.0, argReplayHz);
                    }

                    break;
                case "--e2e-singleton-suffix" when i + 1 < args.Length: opts.E2eSingletonSuffix = args[++i]; break;
                case "--owner-liveness" when i + 1 < args.Length: opts.OwnerLivenessName = args[++i]; break;
                case "--owner-liveness-nonce" when i + 1 < args.Length:
                    if (ulong.TryParse(args[++i], System.Globalization.NumberStyles.HexNumber, null, out ulong nonce))
                    {
                        opts.OwnerLivenessNonce = nonce;
                    }

                    break;
                case "--e2e-fake-face-output": opts.E2eFakeFrames = true; break;
                case "--e2e-fake-frame-count" when i + 1 < args.Length:
                    if (int.TryParse(args[++i], out int frameCount))
                    {
                        opts.E2eFakeFrameCount = Math.Max(1, frameCount);
                    }

                    break;
                case "--e2e-fake-frame-interval-ms" when i + 1 < args.Length:
                    if (int.TryParse(args[++i], out int intervalMs))
                    {
                        opts.E2eFakeFrameIntervalMs = Math.Max(0, intervalMs);
                    }

                    break;
            }
        }

        if (!faceReplayRecordConfigured && opts.DebugLoggingEnabled)
        {
            opts.FaceReplayRecordEnabled = true;
        }

        return opts;
    }

    private static bool IsTruthy(string value)
    {
        return value.Equals("1", StringComparison.OrdinalIgnoreCase) ||
        value.Equals("true", StringComparison.OrdinalIgnoreCase) ||
        value.Equals("yes", StringComparison.OrdinalIgnoreCase) ||
        value.Equals("on", StringComparison.OrdinalIgnoreCase);
    }
}
