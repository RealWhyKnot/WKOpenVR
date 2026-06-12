using System.Text;
using System.Text.Json;
using WKOpenVR.FaceModuleHost;
using WKOpenVR.FaceModuleHost.Logging;
using WKOpenVR.FaceModuleHost.Workers;

var failures = new List<string>();

await Run("extracts unique float avatar inputs", ExtractsUniqueFloatAvatarInputs);
await Run("infers current avatar from observed parameters", InfersCurrentAvatarFromObservedParameters);
await Run("restores empty output from cached avatar", RestoresEmptyOutputFromCachedAvatar);
await Run("does not infer ambiguous observed parameters", DoesNotInferAmbiguousObservedParameters);
await Run("parses face replay options", ParsesFaceReplayOptions);
await Run("debug logging enables face replay by default", DebugLoggingEnablesFaceReplayByDefault);
await Run("explicit face replay disable overrides debug logging", ExplicitFaceReplayDisableOverridesDebugLogging);
await Run("records face replay frame", RecordsFaceReplayFrame);

if (failures.Count > 0)
{
    foreach (string failure in failures)
    {
        Console.Error.WriteLine(failure);
    }

    return 1;
}

Console.WriteLine("FaceModuleHost tests passed.");
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

static Task ParsesFaceReplayOptions()
{
    using var env = ScopedFaceReplayEnvironment();
    string replayDir = Path.Combine(Path.GetTempPath(), "wkopenvr-ft-replay-options-" + Guid.NewGuid().ToString("N"));
    HostOptions opts = HostOptions.FromArgs(["--face-replay-record", "--face-replay-dir", replayDir, "--face-replay-hz", "12.5"]);

    Require(opts.FaceReplayRecordEnabled, "replay recording was not enabled");
    Require(opts.FaceReplayDirectory == replayDir, "replay directory was not parsed");
    Require(Math.Abs(opts.FaceReplayMaxHz - 12.5) < 0.001, "replay hz was not parsed");
    return Task.CompletedTask;
}

static Task DebugLoggingEnablesFaceReplayByDefault()
{
    using var env = ScopedFaceReplayEnvironment();
    HostOptions opts = HostOptions.FromArgs(["--debug-logging", "true"]);

    Require(opts.DebugLoggingEnabled, "debug logging was not enabled");
    Require(opts.FaceReplayRecordEnabled, "debug logging should enable face replay recording");
    return Task.CompletedTask;
}

static Task ExplicitFaceReplayDisableOverridesDebugLogging()
{
    using var env = ScopedFaceReplayEnvironment();
    HostOptions opts = HostOptions.FromArgs(["--debug-logging", "true", "--no-face-replay-record"]);

    Require(opts.DebugLoggingEnabled, "debug logging was not enabled");
    Require(!opts.FaceReplayRecordEnabled, "explicit replay disable should override debug logging");

    Environment.SetEnvironmentVariable("WKOPENVR_DEBUG_LOGGING", "1");
    Environment.SetEnvironmentVariable("WKOPENVR_FACE_REPLAY_RECORD", "0");
    opts = HostOptions.FromArgs([]);
    Require(opts.DebugLoggingEnabled, "env debug logging was not enabled");
    Require(!opts.FaceReplayRecordEnabled, "env replay disable should override env debug logging");
    return Task.CompletedTask;
}

static Task RecordsFaceReplayFrame()
{
    using var fixture = new TempFixture();
    string outputPath = Path.Combine(fixture.Root, "face_replay.test.jsonl");
    using (var recorder = new FaceFrameReplayRecorder(outputPath, maxHz: 0))
    {
        var eye = new EyeFrameSink
        {
            LeftOpenness = 0.6f,
            RightOpenness = 0.7f,
            PupilDilationLeft = 0.4f,
            PupilDilationRight = 0.5f,
        };
        var head = new HeadFrameSink { IsValid = true, Yaw = 1.0f };
        var expressions = new float[FaceFrameReplayRecorder.ShapeCount];
        expressions[22] = 0.42f;
        expressions[29] = 0.11f;

        recorder.RecordFrame(
            "module-uuid",
            "Test Module",
            0x1234u,
            frameNumber: 7,
            expressions,
            eye,
            head,
            eyeValid: true,
            expressionValid: true,
            validExpressionSignals: 88,
            validEyeSignals: 8);
    }

    string[] lines = File.ReadAllLines(outputPath, Encoding.UTF8);
    Require(lines.Length == 2, $"expected 2 replay lines, got {lines.Length}");

    using JsonDocument header = JsonDocument.Parse(lines[0]);
    Require(header.RootElement.GetProperty("type").GetString() == "header", "missing replay header");
    Require(header.RootElement.GetProperty("shapeCount").GetInt32() == FaceFrameReplayRecorder.ShapeCount, "wrong shape count");

    using JsonDocument frame = JsonDocument.Parse(lines[1]);
    JsonElement root = frame.RootElement;
    Require(root.GetProperty("type").GetString() == "frame", "missing frame record");
    Require(root.GetProperty("moduleUuid").GetString() == "module-uuid", "wrong module uuid");
    Require(root.GetProperty("flags").GetProperty("expression").GetBoolean(), "expression flag missing");
    float jaw = root.GetProperty("expressions").EnumerateArray().ElementAt(22).GetSingle();
    Require(Math.Abs(jaw - 0.42f) < 0.001f, "jaw value missing");
    Require(root.GetProperty("top").EnumerateArray().Any(e => e.GetProperty("name").GetString() == "JawOpen"), "top shapes missing JawOpen");
    return Task.CompletedTask;
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

static ScopedEnvironmentVariables ScopedFaceReplayEnvironment()
{
    return new ScopedEnvironmentVariables(
        "WKOPENVR_DEBUG_LOGGING",
        "WKOPENVR_FACE_REPLAY_RECORD",
        "WKOPENVR_FACE_REPLAY_DIR",
        "WKOPENVR_FACE_REPLAY_HZ");
}

sealed class ScopedEnvironmentVariables : IDisposable
{
    private readonly Dictionary<string, string?> previous_ = [];

    public ScopedEnvironmentVariables(params string[] names)
    {
        foreach (string name in names)
        {
            previous_[name] = Environment.GetEnvironmentVariable(name);
            Environment.SetEnvironmentVariable(name, null);
        }
    }

    public void Dispose()
    {
        foreach ((string name, string? value) in previous_)
        {
            Environment.SetEnvironmentVariable(name, value);
        }
    }
}

sealed class TempFixture : IDisposable
{
    public TempFixture()
    {
        Root = Path.Combine(Path.GetTempPath(), "wkopenvr-ft-allowlist-tests-" + Guid.NewGuid().ToString("N"));
        OscRoot = Path.Combine(Root, "OSC");
        CacheDir = Path.Combine(Root, "cache");
        OutputPath = Path.Combine(Root, "avatar_parameters.txt");
        Logger = new HostLogger(Path.Combine(Root, "test.log"), forceEnabled: true);
    }

    public string Root { get; }
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
        try { Directory.Delete(Root, recursive: true); } catch { }
    }
}
