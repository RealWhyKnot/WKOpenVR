using System.Globalization;
using System.Text;
using System.Text.Json;

namespace WKOpenVR.FaceModuleHost.Workers;

/// <summary>
/// Loads a face-frame replay JSONL file (produced by <see cref="FaceFrameReplayRecorder"/>)
/// back into a typed, in-memory model so recordings can be played back and compared.
/// Parsing is defensive: a frame missing a field falls back to a neutral default rather
/// than aborting the load, so partial or older recordings still open.
/// </summary>
public static class FaceFrameReplayPlayer
{
    public sealed class Eye
    {
        public float LeftOpenness { get; init; }
        public float RightOpenness { get; init; }
        public float LeftPupilDilation { get; init; }
        public float RightPupilDilation { get; init; }
        public float LeftGazeX { get; init; }
        public float LeftGazeY { get; init; }
        public float LeftGazeZ { get; init; }
        public float RightGazeX { get; init; }
        public float RightGazeY { get; init; }
        public float RightGazeZ { get; init; }
    }

    public sealed class Head
    {
        public float Yaw { get; init; }
        public float Pitch { get; init; }
        public float Roll { get; init; }
        public float PosX { get; init; }
        public float PosY { get; init; }
        public float PosZ { get; init; }
    }

    public sealed class Frame
    {
        public double TimeMs { get; init; }
        public long FrameNumber { get; init; }
        public string ModuleUuid { get; init; } = string.Empty;
        public string ModuleName { get; init; } = string.Empty;
        public string ModuleUuidHash { get; init; } = string.Empty;
        public bool EyeValid { get; init; }
        public bool ExpressionValid { get; init; }
        public bool HeadValid { get; init; }
        public int ValidExpressionSignals { get; init; }
        public int ValidEyeSignals { get; init; }
        public int NonZeroExpressions { get; init; }
        public float[] Expressions { get; init; } = [];
        public Eye EyeData { get; init; } = new();
        public Head HeadData { get; init; } = new();
    }

    public sealed class Recording
    {
        public string SourcePath { get; init; } = string.Empty;
        public int Schema { get; init; }
        public string RecordedAtUtc { get; init; } = string.Empty;
        public string ShapeOrder { get; init; } = string.Empty;
        public int ShapeCount { get; init; }
        public string[] ShapeNames { get; init; } = [];
        public double MaxHz { get; init; }
        public List<Frame> Frames { get; init; } = [];
        public string? Error { get; init; }

        /// <summary>The most frequently seen module name across frames (the recording's source tracker).</summary>
        public string PrimaryModuleName { get; init; } = string.Empty;

        public bool Ok => Error is null;
        public double DurationMs => Frames.Count >= 2 ? Frames[^1].TimeMs - Frames[0].TimeMs : 0.0;
        public double EffectiveHz => DurationMs > 0.0 ? (Frames.Count - 1) * 1000.0 / DurationMs : 0.0;
    }

    public static Recording Load(string path)
    {
        try
        {
            // FileShare so a recording that is still being written can also be read.
            using var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite | FileShare.Delete);
            using var reader = new StreamReader(stream, Encoding.UTF8);
            IEnumerable<string> lines = ReadLines(reader);
            return ParseLines(lines, path);
        }
        catch (Exception ex)
        {
            return new Recording { SourcePath = path, Error = $"{ex.GetType().Name}: {ex.Message}" };
        }
    }

    private static IEnumerable<string> ReadLines(StreamReader reader)
    {
        string? line;
        while ((line = reader.ReadLine()) is not null)
        {
            yield return line;
        }
    }

    /// <summary>Pure parser over JSONL text lines; used by <see cref="Load"/> and directly by tests.</summary>
    public static Recording ParseLines(IEnumerable<string> lines, string sourcePath)
    {
        int schema = 0;
        string recordedAt = string.Empty;
        string shapeOrder = string.Empty;
        int shapeCount = 0;
        string[] shapeNames = [];
        double maxHz = 0.0;
        var frames = new List<Frame>();
        var moduleCounts = new Dictionary<string, int>(StringComparer.Ordinal);
        string? error = null;

        foreach (string raw in lines)
        {
            if (string.IsNullOrWhiteSpace(raw))
            {
                continue;
            }

            JsonElement root;
            try
            {
                using JsonDocument doc = JsonDocument.Parse(raw);
                root = doc.RootElement.Clone();
            }
            catch (JsonException)
            {
                // Skip a single malformed line rather than failing the whole recording.
                continue;
            }

            string type = GetString(root, "type");
            if (type == "header")
            {
                schema = GetInt(root, "schema");
                recordedAt = GetString(root, "recordedAtUtc");
                shapeOrder = GetString(root, "shapeOrder");
                shapeCount = GetInt(root, "shapeCount");
                maxHz = GetDouble(root, "maxHz");
                shapeNames = GetStringArray(root, "shapeNames");
                continue;
            }

            if (type != "frame")
            {
                continue;
            }

            float[] expressions = GetFloatArray(root, "expressions");
            if (shapeCount == 0)
            {
                shapeCount = expressions.Length;
            }

            JsonElement flags = GetObject(root, "flags");
            JsonElement eye = GetObject(root, "eye");
            JsonElement head = GetObject(root, "head");
            string moduleName = GetString(root, "moduleName");
            if (!string.IsNullOrEmpty(moduleName))
            {
                moduleCounts[moduleName] = moduleCounts.GetValueOrDefault(moduleName) + 1;
            }

            frames.Add(new Frame
            {
                TimeMs = GetDouble(root, "timeMs"),
                FrameNumber = GetLong(root, "frameNumber"),
                ModuleUuid = GetString(root, "moduleUuid"),
                ModuleName = moduleName,
                ModuleUuidHash = GetString(root, "moduleUuidHash"),
                EyeValid = GetBool(flags, "eye"),
                ExpressionValid = GetBool(flags, "expression"),
                HeadValid = GetBool(flags, "head"),
                ValidExpressionSignals = GetInt(root, "validExpressionSignals"),
                ValidEyeSignals = GetInt(root, "validEyeSignals"),
                NonZeroExpressions = GetInt(root, "nonZeroExpressions"),
                Expressions = expressions,
                EyeData = new Eye
                {
                    LeftOpenness = GetFloat(eye, "leftOpenness"),
                    RightOpenness = GetFloat(eye, "rightOpenness"),
                    LeftPupilDilation = GetFloat(eye, "leftPupilDilation"),
                    RightPupilDilation = GetFloat(eye, "rightPupilDilation"),
                    LeftGazeX = GetVecComponent(eye, "leftGaze", "x"),
                    LeftGazeY = GetVecComponent(eye, "leftGaze", "y"),
                    LeftGazeZ = GetVecComponent(eye, "leftGaze", "z"),
                    RightGazeX = GetVecComponent(eye, "rightGaze", "x"),
                    RightGazeY = GetVecComponent(eye, "rightGaze", "y"),
                    RightGazeZ = GetVecComponent(eye, "rightGaze", "z"),
                },
                HeadData = new Head
                {
                    Yaw = GetFloat(head, "yaw"),
                    Pitch = GetFloat(head, "pitch"),
                    Roll = GetFloat(head, "roll"),
                    PosX = GetFloat(head, "posX"),
                    PosY = GetFloat(head, "posY"),
                    PosZ = GetFloat(head, "posZ"),
                },
            });
        }

        if (frames.Count == 0)
        {
            error = "recording contained no frame records";
        }

        string primaryModule = string.Empty;
        int best = -1;
        foreach ((string name, int count) in moduleCounts)
        {
            if (count > best)
            {
                best = count;
                primaryModule = name;
            }
        }

        return new Recording
        {
            SourcePath = sourcePath,
            Schema = schema,
            RecordedAtUtc = recordedAt,
            ShapeOrder = shapeOrder,
            ShapeCount = shapeCount,
            ShapeNames = shapeNames,
            MaxHz = maxHz,
            Frames = frames,
            PrimaryModuleName = primaryModule,
            Error = error,
        };
    }

    private static JsonElement GetObject(JsonElement parent, string name)
    {
        return parent.ValueKind == JsonValueKind.Object &&
            parent.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.Object
            ? value
            : default;
    }

    private static string GetString(JsonElement parent, string name)
    {
        return parent.ValueKind == JsonValueKind.Object &&
            parent.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.String
            ? value.GetString() ?? string.Empty
            : string.Empty;
    }

    private static bool GetBool(JsonElement parent, string name)
    {
        return parent.ValueKind == JsonValueKind.Object &&
            parent.TryGetProperty(name, out JsonElement value) &&
            (value.ValueKind == JsonValueKind.True || value.ValueKind == JsonValueKind.False) &&
            value.GetBoolean();
    }

    private static int GetInt(JsonElement parent, string name)
    {
        return parent.ValueKind == JsonValueKind.Object &&
            parent.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.Number &&
            value.TryGetInt32(out int parsed)
            ? parsed
            : 0;
    }

    private static long GetLong(JsonElement parent, string name)
    {
        return parent.ValueKind == JsonValueKind.Object &&
            parent.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.Number &&
            value.TryGetInt64(out long parsed)
            ? parsed
            : 0L;
    }

    private static double GetDouble(JsonElement parent, string name)
    {
        return parent.ValueKind == JsonValueKind.Object &&
            parent.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.Number &&
            value.TryGetDouble(out double parsed)
            ? parsed
            : 0.0;
    }

    private static float GetFloat(JsonElement parent, string name)
    {
        return parent.ValueKind == JsonValueKind.Object &&
            parent.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.Number &&
            value.TryGetDouble(out double parsed)
            ? (float)parsed
            : 0.0f;
    }

    private static float GetVecComponent(JsonElement parent, string vecName, string component)
    {
        JsonElement vec = GetObject(parent, vecName);
        return GetFloat(vec, component);
    }

    private static string[] GetStringArray(JsonElement parent, string name)
    {
        if (parent.ValueKind != JsonValueKind.Object ||
            !parent.TryGetProperty(name, out JsonElement value) || value.ValueKind != JsonValueKind.Array)
        {
            return [];
        }

        var list = new List<string>(value.GetArrayLength());
        foreach (JsonElement item in value.EnumerateArray())
        {
            list.Add(item.ValueKind == JsonValueKind.String ? item.GetString() ?? string.Empty : string.Empty);
        }

        return [.. list];
    }

    private static float[] GetFloatArray(JsonElement parent, string name)
    {
        if (parent.ValueKind != JsonValueKind.Object ||
            !parent.TryGetProperty(name, out JsonElement value) || value.ValueKind != JsonValueKind.Array)
        {
            return [];
        }

        var list = new List<float>(value.GetArrayLength());
        foreach (JsonElement item in value.EnumerateArray())
        {
            list.Add(item.ValueKind == JsonValueKind.Number && item.TryGetDouble(out double parsed) ? (float)parsed : 0.0f);
        }

        return [.. list];
    }

    internal static string FormatInvariant(double value, int decimals)
    {
        return value.ToString("F" + decimals.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);
    }
}
