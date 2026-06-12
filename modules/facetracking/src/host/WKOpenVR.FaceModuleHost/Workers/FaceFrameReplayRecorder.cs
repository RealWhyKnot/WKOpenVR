using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Text.Json;
using WKOpenVR.FaceModuleHost.Logging;

namespace WKOpenVR.FaceModuleHost.Workers;

public sealed class FaceFrameReplayRecorder : IDisposable
{
    public const int ShapeCount = FrameWriter.UpstreamShapeCount;

    private static readonly JsonSerializerOptions JsonOptions = new(JsonSerializerDefaults.Web);

    private static readonly string[] ShapeNames =
    [
        "EyeSquintRight",
        "EyeSquintLeft",
        "EyeWideRight",
        "EyeWideLeft",
        "BrowPinchRight",
        "BrowPinchLeft",
        "BrowLowererRight",
        "BrowLowererLeft",
        "BrowInnerUpRight",
        "BrowInnerUpLeft",
        "BrowOuterUpRight",
        "BrowOuterUpLeft",
        "NasalDilationRight",
        "NasalDilationLeft",
        "NasalConstrictRight",
        "NasalConstrictLeft",
        "CheekSquintRight",
        "CheekSquintLeft",
        "CheekPuffRight",
        "CheekPuffLeft",
        "CheekSuckRight",
        "CheekSuckLeft",
        "JawOpen",
        "JawRight",
        "JawLeft",
        "JawForward",
        "JawBackward",
        "JawClench",
        "JawMandibleRaise",
        "MouthClosed",
        "LipSuckUpperRight",
        "LipSuckUpperLeft",
        "LipSuckLowerRight",
        "LipSuckLowerLeft",
        "LipSuckCornerRight",
        "LipSuckCornerLeft",
        "LipFunnelUpperRight",
        "LipFunnelUpperLeft",
        "LipFunnelLowerRight",
        "LipFunnelLowerLeft",
        "LipPuckerUpperRight",
        "LipPuckerUpperLeft",
        "LipPuckerLowerRight",
        "LipPuckerLowerLeft",
        "MouthUpperUpRight",
        "MouthUpperUpLeft",
        "MouthUpperDeepenRight",
        "MouthUpperDeepenLeft",
        "NoseSneerRight",
        "NoseSneerLeft",
        "MouthLowerDownRight",
        "MouthLowerDownLeft",
        "MouthUpperRight",
        "MouthUpperLeft",
        "MouthLowerRight",
        "MouthLowerLeft",
        "MouthCornerPullRight",
        "MouthCornerPullLeft",
        "MouthCornerSlantRight",
        "MouthCornerSlantLeft",
        "MouthFrownRight",
        "MouthFrownLeft",
        "MouthStretchRight",
        "MouthStretchLeft",
        "MouthDimpleRight",
        "MouthDimpleLeft",
        "MouthRaiserUpper",
        "MouthRaiserLower",
        "MouthPressRight",
        "MouthPressLeft",
        "MouthTightenerRight",
        "MouthTightenerLeft",
        "TongueOut",
        "TongueUp",
        "TongueDown",
        "TongueRight",
        "TongueLeft",
        "TongueRoll",
        "TongueBendDown",
        "TongueCurlUp",
        "TongueSquish",
        "TongueFlat",
        "TongueTwistRight",
        "TongueTwistLeft",
        "SoftPalateClose",
        "ThroatSwallow",
        "NeckFlexRight",
        "NeckFlexLeft",
    ];

    private readonly StreamWriter _writer;
    private readonly Stopwatch _elapsed = Stopwatch.StartNew();
    private readonly long _minTicksBetweenWrites;
    private long _lastWriteTick = long.MinValue;
    private bool _disposed;

    public FaceFrameReplayRecorder(string outputPath, double maxHz)
    {
        if (string.IsNullOrWhiteSpace(outputPath))
        {
            throw new ArgumentException("Replay output path is required.", nameof(outputPath));
        }

        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(outputPath))!);
        var stream = new FileStream(
            outputPath,
            FileMode.CreateNew,
            FileAccess.Write,
            FileShare.ReadWrite | FileShare.Delete);
        _writer = new StreamWriter(stream, new UTF8Encoding(false)) { AutoFlush = true };
        _minTicksBetweenWrites = maxHz <= 0.0 ? 0 : (long)Math.Max(1.0, Stopwatch.Frequency / maxHz);
        OutputPath = outputPath;

        WriteJsonLine(new ReplayHeader(
            "header",
            1,
            DateTimeOffset.UtcNow.ToString("O", CultureInfo.InvariantCulture),
            "VRCFaceTracking.UnifiedExpressions.v5.1.1",
            ShapeCount,
            ShapeNames,
            maxHz));
    }

    public string OutputPath { get; }

    public static FaceFrameReplayRecorder? CreateFromOptions(HostOptions opts, HostLogger logger)
    {
        if (!opts.FaceReplayRecordEnabled)
        {
            return null;
        }

        try
        {
            string ts = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss_fff", CultureInfo.InvariantCulture);
            string outputPath = Path.Combine(opts.FaceReplayDirectory, $"face_replay.{ts}.jsonl");
            var recorder = new FaceFrameReplayRecorder(outputPath, opts.FaceReplayMaxHz);
            logger.Info($"[face-replay] recording enabled path={outputPath} max_hz={opts.FaceReplayMaxHz:F1}");
            return recorder;
        }
        catch (Exception ex)
        {
            logger.Warn($"[face-replay] recording disabled: {ex.GetType().Name} {ex.Message}");
            return null;
        }
    }

    public void RecordFrame(
        string moduleUuid,
        string moduleName,
        ulong moduleUuidHash,
        long frameNumber,
        ReadOnlySpan<float> upstreamExpressions,
        EyeFrameSink eye,
        HeadFrameSink head,
        bool eyeValid,
        bool expressionValid,
        int validExpressionSignals,
        int validEyeSignals)
    {
        if (_disposed || !ShouldWriteNow())
        {
            return;
        }

        float[] expressions = new float[ShapeCount];
        int n = Math.Min(upstreamExpressions.Length, ShapeCount);
        for (int i = 0; i < n; i++)
        {
            expressions[i] = upstreamExpressions[i];
        }

        WriteJsonLine(new FrameRecord(
            "frame",
            Math.Round(_elapsed.Elapsed.TotalMilliseconds, 3),
            frameNumber,
            moduleUuid,
            moduleName,
            "0x" + moduleUuidHash.ToString("X16", CultureInfo.InvariantCulture),
            new FrameFlags(eyeValid, expressionValid, head.IsValid),
            validExpressionSignals,
            validEyeSignals,
            CountNonZero(expressions),
            TopExpressions(expressions, 8),
            expressions,
            new EyeRecord(
                eye.LeftOpenness,
                eye.RightOpenness,
                eye.PupilDilationLeft,
                eye.PupilDilationRight,
                new Vector3Record(eye.Left.DirHmd.X, eye.Left.DirHmd.Y, eye.Left.DirHmd.Z),
                new Vector3Record(eye.Right.DirHmd.X, eye.Right.DirHmd.Y, eye.Right.DirHmd.Z)),
            new HeadRecord(head.Yaw, head.Pitch, head.Roll, head.PosX, head.PosY, head.PosZ)));
    }

    public void Dispose()
    {
        _disposed = true;
        _writer.Dispose();
    }

    private bool ShouldWriteNow()
    {
        long now = Stopwatch.GetTimestamp();
        if (_lastWriteTick != long.MinValue && _minTicksBetweenWrites > 0 && now - _lastWriteTick < _minTicksBetweenWrites)
        {
            return false;
        }

        _lastWriteTick = now;
        return true;
    }

    private void WriteJsonLine<T>(T value)
    {
        _writer.WriteLine(JsonSerializer.Serialize(value, JsonOptions));
    }

    private static int CountNonZero(float[] expressions)
    {
        int count = 0;
        foreach (float value in expressions)
        {
            if (Math.Abs(value) > 0.001f)
            {
                count++;
            }
        }

        return count;
    }

    private static TopExpression[] TopExpressions(float[] expressions, int maxCount)
    {
        var top = new List<TopExpression>(maxCount);
        for (int i = 0; i < expressions.Length; i++)
        {
            float value = expressions[i];
            if (value <= 0.001f)
            {
                continue;
            }

            int insert = top.FindIndex(entry => value > entry.Value);
            if (insert < 0)
            {
                insert = top.Count;
            }

            if (insert >= maxCount)
            {
                continue;
            }

            top.Insert(insert, new TopExpression(i, ShapeNames[i], value));
            if (top.Count > maxCount)
            {
                top.RemoveAt(top.Count - 1);
            }
        }

        return [.. top];
    }

    private sealed record ReplayHeader(
        string Type,
        int Schema,
        string RecordedAtUtc,
        string ShapeOrder,
        int ShapeCount,
        string[] ShapeNames,
        double MaxHz);

    private sealed record FrameRecord(
        string Type,
        double TimeMs,
        long FrameNumber,
        string ModuleUuid,
        string ModuleName,
        string ModuleUuidHash,
        FrameFlags Flags,
        int ValidExpressionSignals,
        int ValidEyeSignals,
        int NonZeroExpressions,
        TopExpression[] Top,
        float[] Expressions,
        EyeRecord Eye,
        HeadRecord Head);

    public sealed record TopExpression(int Index, string Name, float Value);

    private sealed record FrameFlags(bool Eye, bool Expression, bool Head);

    private sealed record EyeRecord(
        float LeftOpenness,
        float RightOpenness,
        float LeftPupilDilation,
        float RightPupilDilation,
        Vector3Record LeftGaze,
        Vector3Record RightGaze);

    private sealed record HeadRecord(
        float Yaw,
        float Pitch,
        float Roll,
        float PosX,
        float PosY,
        float PosZ);

    private sealed record Vector3Record(float X, float Y, float Z);
}
