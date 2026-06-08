using System.Text;
using System.Text.Json;
using WKOpenVR.FaceModuleHost.Logging;

namespace WKOpenVR.FaceModuleHost;

internal static class AvatarParameterAllowList
{
    private static readonly Encoding Utf8NoBom = new UTF8Encoding(encoderShouldEmitUTF8Identifier: false);
    private const string AvatarAddressPrefix = "/avatar/parameters/";
    private const int MinObservedAddressesForInference = 8;
    private const int MinObservedMatchCount = 8;
    private const int MinObservedTieBreak = 3;

    public static string PathOnDisk()
    {
        return Path.Combine(AppPaths.FaceTrackingDir(), "avatar_parameters.txt");
    }

    public static void RestoreLastKnown(HostLogger logger)
    {
        RestoreLastKnown(logger, PathOnDisk(), CacheDirOnDisk());
    }

    internal static void RestoreLastKnown(HostLogger logger, string outputPath, string cacheDir)
    {
        try
        {
            int currentCount = CountAllowListFile(outputPath);
            if (currentCount > 0)
            {
                logger.Info($"[ftq] avatar parameter allowlist retained: floats={currentCount}");
                return;
            }

            AllowListState? state = ReadState(cacheDir);
            if (!string.IsNullOrWhiteSpace(state?.AvatarId) &&
                TryRestoreCachedAllowList(state.AvatarId, logger, outputPath, cacheDir))
            {
                return;
            }

            logger.Info("[ftq] avatar parameter allowlist empty; waiting for avatar change or parameter inference");
        }
        catch (Exception ex)
        {
            logger.Warn($"[ftq] avatar parameter allowlist restore failed: {ex.Message}");
        }
    }

    public static void Clear(HostLogger logger)
    {
        Clear(logger, PathOnDisk());
    }

    internal static void Clear(HostLogger logger, string outputPath)
    {
        try
        {
            AtomicWrite(outputPath, "");
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
        await RefreshForAvatarAsync(avatarId, logger, ct, OscRootOnDisk(), PathOnDisk(), CacheDirOnDisk());
    }

    internal static async Task RefreshForAvatarAsync(
        string avatarId,
        HostLogger logger,
        CancellationToken ct,
        string oscRoot,
        string outputPath,
        string cacheDir)
    {
        if (string.IsNullOrWhiteSpace(avatarId))
        {
            Clear(logger, outputPath);
            return;
        }

        try
        {
            string? configPath = FindAvatarConfig(avatarId, oscRoot);
            if (configPath is null)
            {
                if (TryRestoreCachedAllowList(avatarId, logger, outputPath, cacheDir))
                {
                    logger.Warn($"[ftq] avatar config not found for {avatarId}; restored cached allowlist");
                    return;
                }

                AtomicWrite(outputPath, "");
                logger.Warn($"[ftq] avatar config not found for {avatarId}; allowlist empty");
                return;
            }

            AvatarConfig config = await ReadAvatarConfigAsync(configPath, ct);
            WriteAllowList(config.Addresses, avatarId, configPath, outputPath, cacheDir);
            int count = config.Addresses.Count;
            logger.Info($"[ftq] avatar parameter allowlist loaded: avatar={avatarId} floats={count}");
        }
        catch (OperationCanceledException)
        {
        }
        catch (Exception ex)
        {
            if (TryRestoreCachedAllowList(avatarId, logger, outputPath, cacheDir))
            {
                logger.Warn($"[ftq] avatar parameter allowlist load failed for {avatarId}: {ex.Message}; restored cached allowlist");
                return;
            }

            AtomicWrite(outputPath, "");
            logger.Warn($"[ftq] avatar parameter allowlist load failed for {avatarId}: {ex.Message}");
        }
    }

    public static async Task<bool> RefreshFromObservedParametersAsync(
        IReadOnlyCollection<string> observedAddresses,
        HostLogger logger,
        CancellationToken ct)
    {
        return await RefreshFromObservedParametersAsync(
            observedAddresses,
            logger,
            ct,
            OscRootOnDisk(),
            PathOnDisk(),
            CacheDirOnDisk());
    }

    internal static async Task<bool> RefreshFromObservedParametersAsync(
        IReadOnlyCollection<string> observedAddresses,
        HostLogger logger,
        CancellationToken ct,
        string oscRoot,
        string outputPath,
        string cacheDir)
    {
        try
        {
            AvatarConfigMatch? match = FindBestAvatarConfigForObservedAddresses(oscRoot, observedAddresses);
            if (match is null)
            {
                return false;
            }

            AvatarConfig config = await ReadAvatarConfigAsync(match.ConfigPath, ct);
            WriteAllowList(config.Addresses, match.AvatarId, match.ConfigPath, outputPath, cacheDir);
            logger.Info(
                $"[ftq] avatar parameter allowlist inferred: avatar={match.AvatarId} floats={config.Addresses.Count} matches={match.MatchCount} observed={match.ObservedCount}");
            return true;
        }
        catch (OperationCanceledException)
        {
            return false;
        }
        catch (Exception ex)
        {
            logger.Warn($"[ftq] avatar parameter allowlist inference failed: {ex.Message}");
            return false;
        }
    }

    internal static string OscRootOnDisk()
    {
        return Path.Combine(AppPaths.LocalAppDataLow(), "VRChat", "VRChat", "OSC");
    }

    private static string CacheDirOnDisk()
    {
        return Path.Combine(AppPaths.FaceTrackingDir(), "avatar_parameter_cache");
    }

    internal static string? FindAvatarConfig(string avatarId, string oscRoot)
    {
        if (!Directory.Exists(oscRoot))
        {
            return null;
        }

        try
        {
            foreach (string userFolder in Directory.EnumerateDirectories(oscRoot))
            {
                string avatarsDir = Path.Combine(userFolder, "Avatars");
                if (!Directory.Exists(avatarsDir))
                {
                    continue;
                }

                foreach (string avatarFile in Directory.EnumerateFiles(avatarsDir, "*.json"))
                {
                    try
                    {
                        using FileStream fs = File.OpenRead(avatarFile);
                        using var doc = JsonDocument.Parse(fs);
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

    internal static AvatarConfigMatch? FindBestAvatarConfigForObservedAddresses(
        string oscRoot,
        IReadOnlyCollection<string> observedAddresses)
    {
        if (!Directory.Exists(oscRoot))
        {
            return null;
        }

        var observed = new HashSet<string>(
            observedAddresses.Where(IsAvatarParameterAddress),
            StringComparer.Ordinal);
        if (observed.Count < MinObservedAddressesForInference)
        {
            return null;
        }

        AvatarConfigMatch? best = null;
        int secondBestCount = 0;

        try
        {
            foreach (string userFolder in Directory.EnumerateDirectories(oscRoot))
            {
                string avatarsDir = Path.Combine(userFolder, "Avatars");
                if (!Directory.Exists(avatarsDir))
                {
                    continue;
                }

                foreach (string avatarFile in Directory.EnumerateFiles(avatarsDir, "*.json"))
                {
                    AvatarConfig? config = TryReadAvatarConfig(avatarFile);
                    if (config is null || string.IsNullOrWhiteSpace(config.AvatarId))
                    {
                        continue;
                    }

                    var addressSet = new HashSet<string>(config.Addresses, StringComparer.Ordinal);
                    int matchCount = observed.Count(addressSet.Contains);
                    if (matchCount <= 0)
                    {
                        continue;
                    }

                    if (best is null || matchCount > best.MatchCount)
                    {
                        secondBestCount = best?.MatchCount ?? 0;
                        best = new AvatarConfigMatch(
                            config.AvatarId,
                            avatarFile,
                            matchCount,
                            observed.Count,
                            config.Addresses.Count);
                    }
                    else if (matchCount > secondBestCount)
                    {
                        secondBestCount = matchCount;
                    }
                }
            }
        }
        catch
        {
            return null;
        }

        if (best is null || best.MatchCount < MinObservedMatchCount)
        {
            return null;
        }

        if (best.MatchCount < 24 && best.MatchCount - secondBestCount < MinObservedTieBreak)
        {
            return null;
        }

        return best;
    }

    private static async Task<AvatarConfig> ReadAvatarConfigAsync(string configPath, CancellationToken ct)
    {
        await using FileStream fs = File.OpenRead(configPath);
        using JsonDocument doc = await JsonDocument.ParseAsync(fs, cancellationToken: ct);
        return ReadAvatarConfig(doc.RootElement, configPath);
    }

    private static AvatarConfig? TryReadAvatarConfig(string configPath)
    {
        try
        {
            using FileStream fs = File.OpenRead(configPath);
            using var doc = JsonDocument.Parse(fs);
            return ReadAvatarConfig(doc.RootElement, configPath);
        }
        catch
        {
            return null;
        }
    }

    private static AvatarConfig ReadAvatarConfig(JsonElement root, string configPath)
    {
        string? id = root.TryGetProperty("id", out JsonElement idElement)
            ? idElement.GetString()
            : null;

        return new AvatarConfig(id ?? "", ExtractFloatAddresses(root), configPath);
    }

    internal static List<string> ExtractFloatAddresses(JsonElement root)
    {
        var addresses = new List<string>();
        var seen = new HashSet<string>(StringComparer.Ordinal);

        if (root.TryGetProperty("parameters", out JsonElement parameters) &&
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
                if (address is not null && IsAvatarParameterAddress(address) && seen.Add(address))
                {
                    addresses.Add(address);
                }
            }
        }

        return addresses;
    }

    private static void WriteAllowList(
        IReadOnlyList<string> addresses,
        string avatarId,
        string configPath,
        string outputPath,
        string cacheDir)
    {
        string body = BuildAllowListBody(addresses);
        AtomicWrite(outputPath, body);

        if (!string.IsNullOrWhiteSpace(avatarId) && addresses.Count > 0)
        {
            Directory.CreateDirectory(cacheDir);
            AtomicWrite(CachePath(cacheDir, avatarId), body);
            WriteState(cacheDir, avatarId, configPath);
        }
    }

    private static bool TryRestoreCachedAllowList(
        string avatarId,
        HostLogger logger,
        string outputPath,
        string cacheDir)
    {
        string cachePath = CachePath(cacheDir, avatarId);
        if (!File.Exists(cachePath))
        {
            return false;
        }

        string body = File.ReadAllText(cachePath, Encoding.UTF8);
        int count = CountAllowListBody(body);
        if (count == 0)
        {
            return false;
        }

        AtomicWrite(outputPath, body);
        logger.Info($"[ftq] avatar parameter allowlist restored: avatar={avatarId} floats={count}");
        return true;
    }

    private static AllowListState? ReadState(string cacheDir)
    {
        string path = StatePath(cacheDir);
        if (!File.Exists(path))
        {
            return null;
        }

        try
        {
            return JsonSerializer.Deserialize<AllowListState>(File.ReadAllText(path, Encoding.UTF8));
        }
        catch
        {
            return null;
        }
    }

    private static void WriteState(string cacheDir, string avatarId, string configPath)
    {
        var state = new AllowListState
        {
            AvatarId = avatarId,
            ConfigPath = configPath,
            UpdatedAtUtc = DateTime.UtcNow,
        };
        string body = JsonSerializer.Serialize(state, new JsonSerializerOptions { WriteIndented = true }) + "\n";
        AtomicWrite(StatePath(cacheDir), body);
    }

    private static string BuildAllowListBody(IReadOnlyList<string> addresses)
    {
        return addresses.Count == 0
            ? ""
            : string.Join('\n', addresses) + "\n";
    }

    private static int CountAllowListFile(string path)
    {
        return File.Exists(path)
            ? CountAllowListBody(File.ReadAllText(path, Encoding.UTF8))
            : 0;
    }

    private static int CountAllowListBody(string body)
    {
        return body.Split('\n').Count(line => IsAvatarParameterAddress(line.Trim()));
    }

    private static bool IsAvatarParameterAddress(string? address)
    {
        return address is not null &&
            address.StartsWith(AvatarAddressPrefix, StringComparison.Ordinal);
    }

    private static string CachePath(string cacheDir, string avatarId)
    {
        string safeId = string.Concat(avatarId.Select(ch =>
            char.IsLetterOrDigit(ch) || ch == '_' || ch == '-'
                ? ch
                : '_'));
        return Path.Combine(cacheDir, safeId + ".txt");
    }

    private static string StatePath(string cacheDir)
    {
        return Path.Combine(cacheDir, "state.json");
    }

    private static void AtomicWrite(string path, string body)
    {
        Directory.CreateDirectory(System.IO.Path.GetDirectoryName(path)!);
        string tmp = path + ".tmp";
        File.WriteAllText(tmp, body, Utf8NoBom);
        File.Move(tmp, path, overwrite: true);
    }

    internal sealed record AvatarConfig(string AvatarId, List<string> Addresses, string ConfigPath);

    internal sealed record AvatarConfigMatch(
        string AvatarId,
        string ConfigPath,
        int MatchCount,
        int ObservedCount,
        int AddressCount);

    private sealed class AllowListState
    {
        public string? AvatarId { get; set; }
        public string? ConfigPath { get; set; }
        public DateTime UpdatedAtUtc { get; set; }
    }
}
