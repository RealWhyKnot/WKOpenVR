// Lightweight compatibility surface for upstream VRCFaceTracking.Core.VRChat.
using System.Diagnostics;
using System.Runtime.Versioning;
using Microsoft.Win32;

namespace VRCFaceTracking.Core;

public static class VRChat
{
    public static string VRCOSCDirectory { get; private set; } = "";

    public static void EnsureVRCOSCDirectory()
    {
        if (OperatingSystem.IsWindows())
        {
            VRCOSCDirectory = Path.Combine(
                $"{Environment.GetEnvironmentVariable("localappdata")}Low",
                "VRChat",
                "VRChat",
                "OSC");
        }
        else
        {
            VRCOSCDirectory = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                ".steam",
                "steam",
                "steamapps",
                "compatdata",
                "438100",
                "pfx",
                "drive_c",
                "users",
                "steamuser",
                "AppData",
                "LocalLow",
                "VRChat",
                "VRChat",
                "OSC");
        }
    }

    [SupportedOSPlatform("windows")]
    public static bool ForceEnableOsc()
    {
        using var regKey = Registry.CurrentUser.OpenSubKey("Software\\VRChat\\VRChat", true);
        if (regKey is null)
        {
            return true;
        }

        bool wasOscForced = false;
        var keys = regKey.GetValueNames()
            .Where(x => x.StartsWith("VRC_INPUT_OSC", StringComparison.Ordinal) ||
                        x.StartsWith("UI.Settings.Osc", StringComparison.Ordinal));
        foreach (var key in keys)
        {
            if (regKey.GetValue(key) is int value && value == 0)
            {
                regKey.SetValue(key, 1);
                wasOscForced = true;
            }
        }

        return wasOscForced;
    }

    public static bool IsVrChatRunning() =>
        Process.GetProcesses().Any(x => x.ProcessName is "VRChat.exe" or "VRChat");
}
