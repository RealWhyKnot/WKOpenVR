using System.Text;
using System.Text.Json;
using WKOpenVR.FaceModuleHost.Logging;

namespace WKOpenVR.FaceModuleHost;

internal static class AvatarParameterAllowList
{
    private static readonly Encoding Utf8NoBom = new UTF8Encoding(encoderShouldEmitUTF8Identifier: false);

    public static string PathOnDisk() =>
        Path.Combine(AppPaths.FaceTrackingDir(), "avatar_parameters.txt");

    public static void Clear(HostLogger logger)
    {
        try
        {
            AtomicWrite(PathOnDisk(), "");
            logger.Info("[ftq] avatar parameter allowlist cleared");
        }
        catch (Exception ex)
        {
            logger.Warn($"[ftq] avatar parameter allowlist clear failed: {ex.Message}");
        }
    }

    public static async Task RefreshForAvatarAsync(
        string avatarId,
        HostLogger logger,
        CancellationToken ct)
    {
        if (string.IsNullOrWhiteSpace(avatarId))
        {
            Clear(logger);
            return;
        }

        try
        {
            string? configPath = FindAvatarConfig(avatarId);
            if (configPath is null)
            {
                AtomicWrite(PathOnDisk(), "");
                logger.Warn($"[ftq] avatar config not found for {avatarId}; allowlist empty");
                return;
            }

            await using var fs = File.OpenRead(configPath);
            using var doc = await JsonDocument.ParseAsync(fs, cancellationToken: ct);
            var addresses = new List<string>();
            var seen = new HashSet<string>(StringComparer.Ordinal);

            if (doc.RootElement.TryGetProperty("parameters", out JsonElement parameters) &&
                parameters.ValueKind == JsonValueKind.Array)
            {
                foreach (JsonElement parameter in parameters.EnumerateArray())
                {
                    if (!parameter.TryGetProperty("input", out JsonElement input) ||
                        input.ValueKind != JsonValueKind.Object)
                    {
                        continue;
                    }

                    string? type = input.TryGetProperty("type", out JsonElement typeElement)
                        ? typeElement.GetString()
                        : null;
                    if (!string.Equals(type, "Float", StringComparison.OrdinalIgnoreCase))
                    {
                        continue;
                    }

                    string? address = input.TryGetProperty("address", out JsonElement addressElement)
                        ? addressElement.GetString()
                        : null;
                    if (address is not null &&
                        address.StartsWith("/avatar/parameters/", StringComparison.Ordinal) &&
                        seen.Add(address))
                    {
                        addresses.Add(address);
                    }
                }
            }

            string body = addresses.Count == 0
                ? ""
                : string.Join('\n', addresses) + "\n";
            AtomicWrite(PathOnDisk(), body);
            logger.Info($"[ftq] avatar parameter allowlist loaded: avatar={avatarId} floats={addresses.Count}");
        }
        catch (OperationCanceledException)
        {
        }
        catch (Exception ex)
        {
            AtomicWrite(PathOnDisk(), "");
            logger.Warn($"[ftq] avatar parameter allowlist load failed for {avatarId}: {ex.Message}");
        }
    }

    private static string? FindAvatarConfig(string avatarId)
    {
        string root = Path.Combine(AppPaths.LocalAppDataLow(), "VRChat", "VRChat", "OSC");
        if (!Directory.Exists(root)) return null;

        try
        {
            foreach (string userFolder in Directory.EnumerateDirectories(root))
            {
                string avatarsDir = Path.Combine(userFolder, "Avatars");
                if (!Directory.Exists(avatarsDir)) continue;

                foreach (string avatarFile in Directory.EnumerateFiles(avatarsDir, "*.json"))
                {
                    try
                    {
                        using FileStream fs = File.OpenRead(avatarFile);
                        using JsonDocument doc = JsonDocument.Parse(fs);
                        string? id = doc.RootElement.TryGetProperty("id", out JsonElement idElement)
                            ? idElement.GetString()
                            : null;
                        if (string.Equals(id, avatarId, StringComparison.Ordinal))
                        {
                            return avatarFile;
                        }
                    }
                    catch
                    {
                    }
                }
            }
        }
        catch
        {
        }

        return null;
    }

    private static void AtomicWrite(string path, string body)
    {
        Directory.CreateDirectory(System.IO.Path.GetDirectoryName(path)!);
        string tmp = path + ".tmp";
        File.WriteAllText(tmp, body, Utf8NoBom);
        File.Move(tmp, path, overwrite: true);
    }
}
