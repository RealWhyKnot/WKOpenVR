using System.Text;
using System.Text.Json;
using WKOpenVR.FaceModuleHost;
using WKOpenVR.FaceModuleHost.Logging;

var failures = new List<string>();

await Run("extracts unique float avatar inputs", ExtractsUniqueFloatAvatarInputs);
await Run("infers current avatar from observed parameters", InfersCurrentAvatarFromObservedParameters);
await Run("restores empty output from cached avatar", RestoresEmptyOutputFromCachedAvatar);
await Run("does not infer ambiguous observed parameters", DoesNotInferAmbiguousObservedParameters);

if (failures.Count > 0)
{
    foreach (string failure in failures)
    {
        Console.Error.WriteLine(failure);
    }

    return 1;
}

Console.WriteLine("FaceModuleHost allowlist tests passed.");
return 0;

async Task Run(string name, Func<Task> test)
{
    try
    {
        await test();
    }
    catch (Exception ex)
    {
        failures.Add($"{name}: {ex.Message}");
    }
}

static Task ExtractsUniqueFloatAvatarInputs()
{
    using JsonDocument doc = JsonDocument.Parse("""
        {
          "id": "avtr_test",
          "parameters": [
            { "input": { "type": "Float", "address": "/avatar/parameters/FT/v2/JawOpen" } },
            { "input": { "type": "Float", "address": "/avatar/parameters/FT/v2/JawOpen" } },
            { "input": { "type": "Bool", "address": "/avatar/parameters/FT/v2/NotFloat" } },
            { "input": { "type": "Float", "address": "/not/avatar/parameters/Nope" } }
          ]
        }
        """);

    List<string> addresses = AvatarParameterAllowList.ExtractFloatAddresses(doc.RootElement);
    Require(addresses.Count == 1, $"expected 1 address, got {addresses.Count}");
    Require(addresses[0] == "/avatar/parameters/FT/v2/JawOpen", "wrong extracted address");
    return Task.CompletedTask;
}

static async Task InfersCurrentAvatarFromObservedParameters()
{
    using var fixture = new TempFixture();
    string[] active = MakeAddresses("Active", 12);
    string[] other = [.. active.Take(4), .. MakeAddresses("Other", 8)];

    fixture.WriteAvatarConfig("avtr_active", "Active", active);
    fixture.WriteAvatarConfig("avtr_other", "Other", other);

    bool restored = await AvatarParameterAllowList.RefreshFromObservedParametersAsync(
        [.. active.Take(10)],
        fixture.Logger,
        CancellationToken.None,
        fixture.OscRoot,
        fixture.OutputPath,
        fixture.CacheDir);

    Require(restored, "expected observed-parameter inference to restore an allowlist");
    string body = File.ReadAllText(fixture.OutputPath, Encoding.UTF8);
    Require(body.Contains("/avatar/parameters/Active9", StringComparison.Ordinal), "active address missing");
    Require(!body.Contains("/avatar/parameters/Other5", StringComparison.Ordinal), "other avatar leaked into allowlist");
}

static async Task RestoresEmptyOutputFromCachedAvatar()
{
    using var fixture = new TempFixture();
    string[] addresses = MakeAddresses("Cached", 9);
    fixture.WriteAvatarConfig("avtr_cached", "Cached", addresses);

    await AvatarParameterAllowList.RefreshForAvatarAsync(
        "avtr_cached",
        fixture.Logger,
        CancellationToken.None,
        fixture.OscRoot,
        fixture.OutputPath,
        fixture.CacheDir);

    File.WriteAllText(fixture.OutputPath, "", new UTF8Encoding(false));
    AvatarParameterAllowList.RestoreLastKnown(fixture.Logger, fixture.OutputPath, fixture.CacheDir);

    string body = File.ReadAllText(fixture.OutputPath, Encoding.UTF8);
    Require(body.Contains("/avatar/parameters/Cached8", StringComparison.Ordinal), "cached allowlist was not restored");
}

static async Task DoesNotInferAmbiguousObservedParameters()
{
    using var fixture = new TempFixture();
    string[] common = MakeAddresses("Common", 8);
    fixture.WriteAvatarConfig("avtr_a", "A", common);
    fixture.WriteAvatarConfig("avtr_b", "B", common);

    bool restored = await AvatarParameterAllowList.RefreshFromObservedParametersAsync(
        common,
        fixture.Logger,
        CancellationToken.None,
        fixture.OscRoot,
        fixture.OutputPath,
        fixture.CacheDir);

    Require(!restored, "ambiguous parameters should not pick an arbitrary avatar");
    Require(!File.Exists(fixture.OutputPath), "ambiguous inference should not write an allowlist");
}

static string[] MakeAddresses(string prefix, int count)
{
    return [.. Enumerable.Range(0, count).Select(i => $"/avatar/parameters/{prefix}{i}")];
}

static void Require(bool condition, string message)
{
    if (!condition)
    {
        throw new InvalidOperationException(message);
    }
}

sealed class TempFixture : IDisposable
{
    private readonly string _root;

    public TempFixture()
    {
        _root = Path.Combine(Path.GetTempPath(), "wkopenvr-ft-allowlist-tests-" + Guid.NewGuid().ToString("N"));
        OscRoot = Path.Combine(_root, "OSC");
        CacheDir = Path.Combine(_root, "cache");
        OutputPath = Path.Combine(_root, "avatar_parameters.txt");
        Logger = new HostLogger(Path.Combine(_root, "test.log"), forceEnabled: true);
    }

    public string OscRoot { get; }
    public string CacheDir { get; }
    public string OutputPath { get; }
    public HostLogger Logger { get; }

    public void WriteAvatarConfig(string avatarId, string name, IReadOnlyList<string> floatAddresses)
    {
        string avatarDir = Path.Combine(OscRoot, "usr_test", "Avatars");
        Directory.CreateDirectory(avatarDir);
        var parameters = floatAddresses
            .Select(address => new { input = new { type = "Float", address } })
            .Cast<object>()
            .ToArray();
        string json = JsonSerializer.Serialize(
            new { id = avatarId, name, parameters },
            new JsonSerializerOptions { WriteIndented = true });
        File.WriteAllText(Path.Combine(avatarDir, avatarId + ".json"), json, new UTF8Encoding(false));
    }

    public void Dispose()
    {
        Logger.Dispose();
        try { Directory.Delete(_root, recursive: true); } catch { }
    }
}
