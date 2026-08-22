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
await Run("prunes old face replay recordings", PrunesOldFaceReplayRecordings);
await Run("startup sweep respects retention limits", StartupSweepRespectsRetentionLimits);
await Run("parses face replay jsonl", ParsesFaceReplayJsonl);
await Run("compares face replay recordings", ComparesFaceReplayRecordings);
await Run("analyzes episode ramps", AnalyzesEpisodeRamps);
await Run("analyzes blinks and gaze", AnalyzesBlinksAndGaze);
await Run("analyzes idle activity and jaw ratios", AnalyzesIdleActivityAndJawRatios);
await Run("flags rounding against spreading", FlagsLipPostureConflicts);
await Run("analyzes shape slew", AnalyzesShapeSlew);
await Run("orders version directories numerically", OrdersVersionDirectoriesNumerically);

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

static Task PrunesOldFaceReplayRecordings()
{
    using var fixture = new TempFixture();
    string dir = Path.Combine(fixture.Root, "replays");
    Directory.CreateDirectory(dir);
    var baseTime = new DateTime(2026, 1, 1, 0, 0, 0, DateTimeKind.Utc);
    for (int i = 0; i < 5; i++)
    {
        string path = Path.Combine(dir, $"face_replay.old{i}.jsonl");
        File.WriteAllText(path, new string('x', 100));
        File.SetLastWriteTimeUtc(path, baseTime.AddMinutes(i));
    }

    string unrelated = Path.Combine(dir, "notes.txt");
    File.WriteAllText(unrelated, "keep");

    string[] deleted = FaceFrameReplayRecorder.PruneReplayDirectory(dir, maxFiles: 3, maxTotalBytes: long.MaxValue);
    Require(deleted.Length == 2, $"expected 2 count-cap deletions, got {deleted.Length}");
    Require(!File.Exists(Path.Combine(dir, "face_replay.old0.jsonl")), "oldest file should be deleted");
    Require(!File.Exists(Path.Combine(dir, "face_replay.old1.jsonl")), "second-oldest file should be deleted");
    Require(File.Exists(Path.Combine(dir, "face_replay.old4.jsonl")), "newest file should be kept");
    Require(File.Exists(unrelated), "non-replay files must not be touched");

    // Three 100-byte files remain; a 250-byte cap keeps only the newest two.
    deleted = FaceFrameReplayRecorder.PruneReplayDirectory(dir, maxFiles: int.MaxValue, maxTotalBytes: 250);
    Require(deleted.Length == 1, $"expected 1 byte-cap deletion, got {deleted.Length}");
    Require(!File.Exists(Path.Combine(dir, "face_replay.old2.jsonl")), "byte cap should delete the oldest file");
    Require(File.Exists(Path.Combine(dir, "face_replay.old3.jsonl")), "byte cap should keep files under the cap");
    return Task.CompletedTask;
}

static Task StartupSweepRespectsRetentionLimits()
{
    using var env = ScopedFaceReplayEnvironment();
    using var fixture = new TempFixture();
    string dir = Path.Combine(fixture.Root, "replays");
    Directory.CreateDirectory(dir);
    var baseTime = new DateTime(2026, 1, 1, 0, 0, 0, DateTimeKind.Utc);
    for (int i = 0; i < FaceFrameReplayRecorder.MaxRetainedReplayFiles + 2; i++)
    {
        string path = Path.Combine(dir, $"face_replay.old{i:D3}.jsonl");
        File.WriteAllText(path, "x");
        File.SetLastWriteTimeUtc(path, baseTime.AddMinutes(i));
    }

    HostOptions opts = HostOptions.FromArgs(["--face-replay-record", "--face-replay-dir", dir]);
    using FaceFrameReplayRecorder? recorder = FaceFrameReplayRecorder.CreateFromOptions(opts, fixture.Logger);
    Require(recorder is not null, "recorder should start");
    Require(!File.Exists(Path.Combine(dir, "face_replay.old000.jsonl")), "startup sweep should delete the oldest file");
    Require(!File.Exists(Path.Combine(dir, "face_replay.old001.jsonl")), "startup sweep should delete the second-oldest file");
    Require(File.Exists(Path.Combine(dir, "face_replay.old002.jsonl")), "files within retention must survive");
    int remaining = Directory.GetFiles(dir, "face_replay.*.jsonl").Length;
    Require(
        remaining == FaceFrameReplayRecorder.MaxRetainedReplayFiles + 1,
        $"expected {FaceFrameReplayRecorder.MaxRetainedReplayFiles + 1} files (retained + new recording), got {remaining}");
    return Task.CompletedTask;
}

static Task ParsesFaceReplayJsonl()
{
    string[] lines =
    [
        """{"type":"header","schema":1,"shapeOrder":"x","shapeCount":3,"shapeNames":["A","B","C"],"maxHz":30}""",
        "this line is not valid json",
        """{"type":"frame","timeMs":1.0,"frameNumber":1,"moduleUuid":"u","moduleName":"M","flags":{"eye":true,"expression":true,"head":false},"expressions":[0.1,0.9,0.0],"eye":{"leftOpenness":0.5},"head":{"yaw":2.0}}""",
    ];

    FaceFrameReplayPlayer.Recording rec = FaceFrameReplayPlayer.ParseLines(lines, "test");
    Require(rec.Ok, $"parse failed: {rec.Error}");
    Require(rec.ShapeCount == 3, $"wrong shape count {rec.ShapeCount}");
    Require(rec.ShapeNames.Length == 3 && rec.ShapeNames[1] == "B", "shape names not parsed");
    Require(rec.Frames.Count == 1, $"expected 1 frame (malformed line skipped), got {rec.Frames.Count}");
    Require(rec.Frames[0].ModuleName == "M", "module name not parsed");
    Require(Math.Abs(rec.Frames[0].Expressions[1] - 0.9f) < 0.001f, "expression value not parsed");
    Require(rec.Frames[0].ExpressionValid, "expression flag not parsed");
    Require(rec.PrimaryModuleName == "M", "primary module not derived");
    return Task.CompletedTask;
}

static Task ComparesFaceReplayRecordings()
{
    using var fixture = new TempFixture();
    string refPath = Path.Combine(fixture.Root, "ref.jsonl");
    string candPath = Path.Combine(fixture.Root, "cand.jsonl");

    // Reference: jaw opens and closes, mouth-closed stays quiet.
    WriteReplay(refPath, frame =>
    {
        var expr = new float[FaceFrameReplayRecorder.ShapeCount];
        expr[22] = (frame % 2 == 0) ? 0.8f : 0.1f; // JawOpen varies
        expr[29] = 0.0f;                            // MouthClosed quiet
        return expr;
    });

    // Candidate: mouth pinned closed, jaw never opens (the broken-lips symptom).
    WriteReplay(candPath, _ =>
    {
        var expr = new float[FaceFrameReplayRecorder.ShapeCount];
        expr[22] = 0.0f;
        expr[29] = 1.0f; // MouthClosed stuck at 1.0
        return expr;
    });

    FaceFrameReplayPlayer.Recording reference = FaceFrameReplayPlayer.Load(refPath);
    FaceFrameReplayPlayer.Recording candidate = FaceFrameReplayPlayer.Load(candPath);
    Require(reference.Ok, $"reference load failed: {reference.Error}");
    Require(candidate.Ok, $"candidate load failed: {candidate.Error}");
    Require(reference.Frames.Count == 10, $"expected 10 reference frames, got {reference.Frames.Count}");
    Require(reference.ShapeCount == FaceFrameReplayRecorder.ShapeCount, "wrong shape count from header");

    FaceFrameReplayComparer.Comparison cmp = FaceFrameReplayComparer.Compare(reference, candidate);
    Require(cmp.Ok, $"compare failed: {cmp.Error}");
    Require(cmp.StuckInCandidate.Any(d => d.Name == "MouthClosed"), "MouthClosed should be flagged stuck in candidate");
    Require(!cmp.StuckInCandidate.Any(d => d.Name == "JawOpen"), "JawOpen should not be flagged stuck");
    Require(cmp.TopDivergences.Any(d => d.Name == "MouthClosed"), "MouthClosed should be a top divergence");
    Require(cmp.TopDivergences.Any(d => d.Name == "JawOpen"), "JawOpen should be a top divergence");

    string report = FaceFrameReplayComparer.FormatReport(cmp);
    Require(report.Contains("MouthClosed", StringComparison.Ordinal), "report should mention MouthClosed");
    return Task.CompletedTask;
}

static Task AnalyzesEpisodeRamps()
{
    // One smile episode at 40 Hz: 1 s quiet, 300 ms linear rise to 0.8,
    // 1 s hold, 600 ms linear fall, 1 s quiet.
    const double dt = 25.0;
    var frames = new List<AnalyzerFrame>();
    double t = 0.0;
    void Add(float smile)
    {
        frames.Add(new AnalyzerFrame(t, [0.0f, smile], 0.9f, 0.0f, 0.0f));
        t += dt;
    }

    for (int i = 0; i < 40; i++) { Add(0.0f); }
    for (int i = 1; i <= 12; i++) { Add(0.8f * i / 12.0f); }
    for (int i = 0; i < 40; i++) { Add(0.8f); }
    for (int i = 1; i <= 24; i++) { Add(0.8f * (24 - i) / 24.0f); }
    for (int i = 0; i < 40; i++) { Add(0.0f); }

    FaceFrameReplayAnalyzer.Analysis a = AnalyzeFrames(["JawOpen", "MouthCornerPullLeft"], frames);
    FaceFrameReplayAnalyzer.EpisodeSummary smile =
        a.Episodes.Single(e => e.Name == "MouthCornerPullLeft");
    Require(smile.Count == 1, $"expected 1 smile episode, got {smile.Count}");
    Require(Math.Abs(smile.PeakP95 - 0.8) < 0.02, $"episode peak {smile.PeakP95:F3} != ~0.8");
    Require(smile.OnsetMsP50 is > 180 and < 330, $"onset {smile.OnsetMsP50:F0}ms outside 300ms ramp expectation");
    Require(smile.OffsetMsP50 is > 400 and < 650, $"offset {smile.OffsetMsP50:F0}ms outside 600ms ramp expectation");
    Require(smile.DurationMsP50 is > 1600 and < 2100, $"duration {smile.DurationMsP50:F0}ms unexpected");

    FaceFrameReplayAnalyzer.ShapeSummary stat = a.Shapes.Single(s => s.Name == "MouthCornerPullLeft");
    Require(Math.Abs(stat.Max - 0.8) < 0.001, "shape max wrong");
    Require(stat.StrongFraction > 0.2, "hold phase should register as strong");
    return Task.CompletedTask;
}

static Task AnalyzesBlinksAndGaze()
{
    // 40 Hz, 6 s: three blinks (second forms a double), one gaze step out and back.
    const double dt = 25.0;
    var frames = new List<AnalyzerFrame>();
    for (double t = 0.0; t < 6000.0; t += dt)
    {
        bool blink = (t >= 1000.0 && t < 1100.0) || (t >= 1400.0 && t < 1500.0) || (t >= 4500.0 && t < 4600.0);
        bool shifted = t >= 2000.0 && t < 4000.0;
        frames.Add(new AnalyzerFrame(
            t,
            [0.0f],
            blink ? 0.1f : 0.9f,
            shifted ? 0.3f : 0.0f,
            shifted ? -0.2f : 0.0f));
    }

    FaceFrameReplayAnalyzer.Analysis a = AnalyzeFrames(["JawOpen"], frames);
    Require(a.Blinks.Count == 3, $"expected 3 blinks, got {a.Blinks.Count}");
    Require(Math.Abs(a.Blinks.DoubleBlinkFraction - 1.0 / 3.0) < 0.01, "double-blink fraction wrong");
    Require(a.Blinks.ClosedMsP50 is > 50 and < 160, $"closed duration {a.Blinks.ClosedMsP50:F0}ms unexpected");
    Require(a.LidRest.Mean > 0.85, $"lid rest mean {a.LidRest.Mean:F3} should ignore blink frames");
    Require(a.Gaze.SaccadeAmplitudeP50 is > 0.30 and < 0.45,
        $"saccade amplitude {a.Gaze.SaccadeAmplitudeP50:F3} != ~0.36");
    Require(a.Gaze.DwellMsP50 is > 1700 and < 2300, $"dwell {a.Gaze.DwellMsP50:F0}ms != ~2000");
    return Task.CompletedTask;
}

// A lexicographic sort put "2026.8.4.0" above "2026.10.1.0" and would have kept loading the older
// build forever once a month rolled past 9.
static Task OrdersVersionDirectoriesNumerically()
{
    string[] dirs =
    [
        @"C:\m\2026.6.7.0-beta", @"C:\m\2026.8.21.0-4ED2", @"C:\m\2026.10.1.0", @"C:\m\2026.9.30.0",
    ];

    string newest = dirs
        .OrderByDescending(SubprocessManager.VersionSortKey, StringComparer.Ordinal)
        .ThenByDescending(Path.GetFileName, StringComparer.Ordinal)
        .First();
    Require(Path.GetFileName(newest) == "2026.10.1.0", $"picked {Path.GetFileName(newest)}");

    // Non-numeric and short version names must not throw or win outright.
    string[] mixed = [@"C:\m\1.4", @"C:\m\dev", @"C:\m\2026.8.21.0-4ED2"];
    string pick = mixed
        .OrderByDescending(SubprocessManager.VersionSortKey, StringComparer.Ordinal)
        .ThenByDescending(Path.GetFileName, StringComparer.Ordinal)
        .First();
    Require(Path.GetFileName(pick) == "2026.8.21.0-4ED2", $"mixed picked {Path.GetFileName(pick)}");

    // Same numeric core, differing build suffix: deterministic, never throws.
    string[] tied = [@"C:\m\2026.8.21.0-AAAA", @"C:\m\2026.8.21.0-ZZZZ"];
    string tie = tied
        .OrderByDescending(SubprocessManager.VersionSortKey, StringComparer.Ordinal)
        .ThenByDescending(Path.GetFileName, StringComparer.Ordinal)
        .First();
    Require(Path.GetFileName(tie) == "2026.8.21.0-ZZZZ", $"tie picked {Path.GetFileName(tie)}");
    return Task.CompletedTask;
}

static Task FlagsLipPostureConflicts()
{
    const double dt = 25.0;
    string[] names =
    [
        "LipFunnelUpperLeft", "LipPuckerUpperLeft", "MouthStretchLeft", "JawOpen",
    ];

    // First second rounds and spreads at once (impossible); the rest alternates cleanly.
    var frames = new List<AnalyzerFrame>();
    for (double t = 0.0; t < 4000.0; t += dt)
    {
        bool both = t < 1000.0;
        bool rounding = !both && t < 2500.0;
        frames.Add(new AnalyzerFrame(
            t,
            [
                both || rounding ? 0.4f : 0.0f,
                both || rounding ? 0.3f : 0.0f,
                both || !rounding ? 0.4f : 0.0f,
                0.5f,
            ],
            0.9f,
            0.0f,
            0.0f));
    }

    FaceFrameReplayAnalyzer.Analysis a = AnalyzeFrames(names, frames);
    Require(
        Math.Abs(a.LipPostures.ConflictFraction - 0.25) < 0.02,
        $"conflict fraction {a.LipPostures.ConflictFraction:F3} != ~0.25");
    Require(a.LipPostures.ConflictFrames == 40, $"expected 40 conflict frames, got {a.LipPostures.ConflictFrames}");
    return Task.CompletedTask;
}

static Task AnalyzesIdleActivityAndJawRatios()
{
    // 40 Hz: 3 s speaking (jaw 0.5, smile 0.4, lip shapes at fixed ratios),
    // then 3 s idle with two brow blips.
    const double dt = 25.0;
    var frames = new List<AnalyzerFrame>();
    for (double t = 0.0; t < 6000.0; t += dt)
    {
        bool speaking = t < 3000.0;
        bool browBlip = (t >= 4000.0 && t < 4100.0) || (t >= 5000.0 && t < 5100.0);
        frames.Add(new AnalyzerFrame(
            t,
            [
                speaking ? 0.5f : 0.0f,          // JawOpen
                speaking ? 0.4f : 0.0f,          // MouthCornerPullLeft
                browBlip ? 0.15f : 0.0f,         // BrowInnerUpLeft
                speaking ? 0.2f : 0.0f,          // MouthLowerDownLeft
                speaking ? 0.1f : 0.0f,          // MouthUpperUpLeft
            ],
            0.9f,
            0.0f,
            0.0f));
    }

    FaceFrameReplayAnalyzer.Analysis a = AnalyzeFrames(
        ["JawOpen", "MouthCornerPullLeft", "BrowInnerUpLeft", "MouthLowerDownLeft", "MouthUpperUpLeft"],
        frames);
    Require(Math.Abs(a.SpeechSmile.SmileMeanSpeaking - 0.4) < 0.01, "speaking smile mean wrong");
    Require(a.SpeechSmile.SmileMeanQuiet < 0.01, "quiet smile mean should be ~0");
    Require(Math.Abs(a.JawRatios.LowerDownOverJaw - 0.4) < 0.02, "lowerDown/jaw ratio wrong");
    Require(Math.Abs(a.JawRatios.UpperUpOverJaw - 0.2) < 0.02, "upperUp/jaw ratio wrong");
    Require(a.Idle.IdleFraction is > 0.4 and < 0.6, $"idle fraction {a.Idle.IdleFraction:F2} != ~0.5");
    Require(a.Idle.SegmentCount == 1, $"expected 1 idle segment, got {a.Idle.SegmentCount}");
    FaceFrameReplayAnalyzer.IdleShapeSummary brow = a.Idle.Shapes.Single(s => s.Name == "BrowInnerUpLeft");
    Require(brow.EventsPerMinute is > 30 and < 50, $"brow events/min {brow.EventsPerMinute:F1} != ~40");
    Require(brow.AmplitudeP90 < 0.16, "brow idle amplitude p90 too high");
    return Task.CompletedTask;
}

static Task AnalyzesShapeSlew()
{
    // 0.1 over 50 ms is 2.0/s twice; the 300 ms gap exceeds MaxFrameGapMs and must be ignored.
    List<AnalyzerFrame> frames =
    [
        new(0.0, [0.0f], 0.9f, 0.0f, 0.0f),
        new(50.0, [0.1f], 0.9f, 0.0f, 0.0f),
        new(100.0, [0.0f], 0.9f, 0.0f, 0.0f),
        new(400.0, [1.0f], 0.9f, 0.0f, 0.0f),
    ];
    FaceFrameReplayAnalyzer.ShapeSummary s = AnalyzeFrames(["JawOpen"], frames).Shapes.Single();
    Require(Math.Abs(s.SlewMaxPerSecond - 2.0) < 1e-3, $"slew max {s.SlewMaxPerSecond:F3} != 2.0");
    Require(Math.Abs(s.SlewP99PerSecond - 2.0) < 1e-3, $"slew p99 {s.SlewP99PerSecond:F3} != 2.0");
    return Task.CompletedTask;
}

static FaceFrameReplayAnalyzer.Analysis AnalyzeFrames(string[] shapeNames, List<AnalyzerFrame> frames)
{
    var lines = new List<string>
    {
        JsonSerializer.Serialize(new
        {
            type = "header",
            schema = 1,
            shapeOrder = "test",
            shapeCount = shapeNames.Length,
            shapeNames,
            maxHz = 40.0,
        }),
    };
    long frameNumber = 0;
    foreach (AnalyzerFrame f in frames)
    {
        lines.Add(JsonSerializer.Serialize(new
        {
            type = "frame",
            timeMs = f.TimeMs,
            frameNumber = frameNumber++,
            moduleUuid = "u",
            moduleName = "M",
            flags = new { eye = true, expression = true, head = false },
            expressions = f.Expressions,
            eye = new
            {
                leftOpenness = f.Openness,
                rightOpenness = f.Openness,
                leftPupilDilation = 0.5f,
                rightPupilDilation = 0.5f,
                leftGaze = new { x = f.GazeX, y = f.GazeY, z = -0.9f },
                rightGaze = new { x = f.GazeX, y = f.GazeY, z = -0.9f },
            },
            head = new { },
        }));
    }

    FaceFrameReplayPlayer.Recording rec = FaceFrameReplayPlayer.ParseLines(lines, "test");
    Require(rec.Ok, $"test recording failed to parse: {rec.Error}");
    FaceFrameReplayAnalyzer.Analysis analysis = FaceFrameReplayAnalyzer.Analyze(rec);
    Require(analysis.Ok, $"analysis failed: {analysis.Error}");
    return analysis;
}

static void WriteReplay(string path, Func<int, float[]> shapeFactory)
{
    using var recorder = new FaceFrameReplayRecorder(path, maxHz: 0);
    var eye = new EyeFrameSink { LeftOpenness = 0.5f, RightOpenness = 0.5f };
    var head = new HeadFrameSink { IsValid = true };
    for (int i = 0; i < 10; i++)
    {
        recorder.RecordFrame(
            "module-uuid",
            "Test Module",
            0x1234u,
            frameNumber: i,
            shapeFactory(i),
            eye,
            head,
            eyeValid: true,
            expressionValid: true,
            validExpressionSignals: 88,
            validEyeSignals: 8);
    }
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

sealed record AnalyzerFrame(double TimeMs, float[] Expressions, float Openness, float GazeX, float GazeY);

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
