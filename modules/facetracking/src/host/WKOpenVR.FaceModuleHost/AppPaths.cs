namespace WKOpenVR.FaceModuleHost;

internal static class AppPaths
{
    public static string LocalAppDataLow()
    {
        string local = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
        if (!string.IsNullOrWhiteSpace(local))
        {
            if (local.EndsWith("Local", StringComparison.OrdinalIgnoreCase))
            {
                return local[..^5] + "LocalLow";
            }

            string? parent = Directory.GetParent(local)?.FullName;
            if (!string.IsNullOrWhiteSpace(parent))
            {
                return Path.Combine(parent, "LocalLow");
            }
        }

        string? envLocal = Environment.GetEnvironmentVariable("LOCALAPPDATA");
        if (!string.IsNullOrWhiteSpace(envLocal))
        {
            if (envLocal.EndsWith("Local", StringComparison.OrdinalIgnoreCase))
            {
                return envLocal[..^5] + "LocalLow";
            }

            string? parent = Directory.GetParent(envLocal)?.FullName;
            if (!string.IsNullOrWhiteSpace(parent))
            {
                return Path.Combine(parent, "LocalLow");
            }
        }

        string? userProfile = Environment.GetEnvironmentVariable("USERPROFILE");
        if (!string.IsNullOrWhiteSpace(userProfile))
        {
            return Path.Combine(userProfile, "AppData", "LocalLow");
        }

        return Path.GetTempPath();
    }

    public static string RootDir()
    {
        string root = Path.Combine(LocalAppDataLow(), "WKOpenVR");
        try
        {
            Directory.CreateDirectory(root);
            return root;
        }
        catch
        {
            string fallback = Path.Combine(Path.GetTempPath(), "WKOpenVR");
            Directory.CreateDirectory(fallback);
            return fallback;
        }
    }

    public static string LogsDir()
    {
        string dir = Path.Combine(RootDir(), "Logs");
        Directory.CreateDirectory(dir);
        return dir;
    }

    public static string FaceTrackingDir()
    {
        string dir = Path.Combine(RootDir(), "facetracking");
        Directory.CreateDirectory(dir);
        return dir;
    }

    public static string TimestampedLogPath(string prefix)
    {
        string ts = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss");
        return Path.Combine(LogsDir(), $"{prefix}.{ts}.txt");
    }
}
