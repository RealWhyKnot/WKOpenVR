using System.Text;
using System.Text.Json;

namespace WKOpenVR.FaceModuleHost.Workers;

/// <summary>
/// Extracts per-channel dynamics from face-frame replay recordings: amplitude
/// percentiles, episode ramp times, blink and eyelid statistics, gaze saccade and
/// dwell distributions, speech co-occurrence, and idle-period micro-activity.
/// Where the comparer answers "how do two recordings differ", the analyzer answers
/// "what does this tracker's behavior look like in numbers" so those numbers can be
/// used as tuning targets. Multiple recordings pool by per-file medians, so one
/// unusual session cannot skew a pooled value.
/// </summary>
public static class FaceFrameReplayAnalyzer
{
    public sealed class Options
    {
        /// <summary>A shape counts as "active" in a frame when its value exceeds this.</summary>
        public float ActiveThreshold { get; init; } = 0.05f;

        /// <summary>A shape counts as "strong" in a frame when its value exceeds this.</summary>
        public float StrongThreshold { get; init; } = 0.30f;

        /// <summary>Episodes whose peak stays below this are ignored as noise.</summary>
        public float EpisodeMinPeak { get; init; } = 0.30f;

        /// <summary>JawOpen at or below this counts toward an idle (non-speech) segment.</summary>
        public float IdleJawMax { get; init; } = 0.05f;

        /// <summary>JawOpen above this counts as actively speaking.</summary>
        public float SpeakingJawMin { get; init; } = 0.20f;

        /// <summary>Minimum sustained quiet time before frames count as idle.</summary>
        public double IdleMinSeconds { get; init; } = 1.0;

        /// <summary>Eye openness below this counts as a blink closure.</summary>
        public float BlinkCloseThreshold { get; init; } = 0.35f;

        /// <summary>Eye openness at or above this counts as fully reopened / at rest.</summary>
        public float BlinkReopenThreshold { get; init; } = 0.60f;

        /// <summary>Two blinks starting within this window count as a double blink.</summary>
        public double DoubleBlinkGapMs { get; init; } = 500.0;

        /// <summary>Frame-to-frame gaze displacement above this counts as a saccade frame.</summary>
        public float SaccadeMinDelta { get; init; } = 0.06f;

        /// <summary>Frame pairs further apart than this are recording gaps, not motion.</summary>
        public double MaxFrameGapMs { get; init; } = 200.0;
    }

    public sealed class ShapeSummary
    {
        public int Index { get; init; }
        public string Name { get; init; } = string.Empty;
        public double Mean { get; init; }
        public double StdDev { get; init; }
        public double P50 { get; init; }
        public double P90 { get; init; }
        public double P95 { get; init; }
        public double Max { get; init; }
        public double ActiveFraction { get; init; }
        public double StrongFraction { get; init; }
        public double SlewP99PerSecond { get; init; }
        public double SlewMaxPerSecond { get; init; }
    }

    public sealed class EpisodeSummary
    {
        public string Name { get; init; } = string.Empty;
        public int Count { get; init; }
        public double PerMinute { get; init; }
        public double PeakP50 { get; init; }
        public double PeakP95 { get; init; }
        public double DurationMsP50 { get; init; }
        /// <summary>Time from episode start to first frame at 90% of episode peak.</summary>
        public double OnsetMsP50 { get; init; }
        /// <summary>Time from last frame at 90% of episode peak to episode end.</summary>
        public double OffsetMsP50 { get; init; }
        public double GapMsP50 { get; init; }
    }

    public sealed class BlinkSummary
    {
        public int Count { get; init; }
        public double PerMinute { get; init; }
        public double ClosedMsP50 { get; init; }
        public double DoubleBlinkFraction { get; init; }
    }

    public sealed class LidRestSummary
    {
        public double Mean { get; init; }
        public double P10 { get; init; }
        public double P50 { get; init; }
        public double P90 { get; init; }
    }

    public sealed class GazeSummary
    {
        public double XP05 { get; init; }
        public double XP25 { get; init; }
        public double XP50 { get; init; }
        public double XP75 { get; init; }
        public double XP95 { get; init; }
        public double YP05 { get; init; }
        public double YP25 { get; init; }
        public double YP50 { get; init; }
        public double YP75 { get; init; }
        public double YP95 { get; init; }
        public double SaccadesPerMinute { get; init; }
        public double SaccadeAmplitudeP50 { get; init; }
        public double SaccadeAmplitudeP95 { get; init; }
        public double DwellMsP25 { get; init; }
        public double DwellMsP50 { get; init; }
        public double DwellMsP75 { get; init; }
        public double DwellMsP90 { get; init; }
    }

    public sealed class PupilSummary
    {
        public double Min { get; init; }
        public double Mean { get; init; }
        public double Max { get; init; }
        public double StdDev { get; init; }
    }

    public sealed class SpeechSmileSummary
    {
        public double SmileMeanSpeaking { get; init; }
        public double SmileMeanQuiet { get; init; }
        public double SmileActiveSpeaking { get; init; }
        public double SmileActiveQuiet { get; init; }
    }

    public sealed class JawRatioSummary
    {
        /// <summary>mean(MouthLowerDownLeft) / mean(JawOpen) over speaking frames.</summary>
        public double LowerDownOverJaw { get; init; }
        /// <summary>mean(MouthUpperUpLeft) / mean(JawOpen) over speaking frames.</summary>
        public double UpperUpOverJaw { get; init; }
        public int SpeakingFrames { get; init; }
    }

    public sealed class LipPostureSummary
    {
        /// <summary>Fraction of frames holding two or more of funnel/pucker/stretch above the active threshold.</summary>
        public double ConflictFraction { get; init; }
        public double FunnelActive { get; init; }
        public double PuckerActive { get; init; }
        public double StretchActive { get; init; }
        public int ConflictFrames { get; init; }
    }

    public sealed class IdleShapeSummary
    {
        public string Name { get; init; } = string.Empty;
        public double EventsPerMinute { get; init; }
        public double AmplitudeP90 { get; init; }
    }

    public sealed class IdleSummary
    {
        public double IdleFraction { get; init; }
        public int SegmentCount { get; init; }
        public IdleShapeSummary[] Shapes { get; init; } = [];
    }

    public sealed class Analysis
    {
        public string SourcePath { get; init; } = string.Empty;
        public string PrimaryModuleName { get; init; } = string.Empty;
        public int FrameCount { get; init; }
        public double DurationMs { get; init; }
        public double EffectiveHz { get; init; }
        public string? Error { get; init; }
        public bool Ok => Error is null;
        public ShapeSummary[] Shapes { get; init; } = [];
        public EpisodeSummary[] Episodes { get; init; } = [];
        public BlinkSummary Blinks { get; init; } = new();
        public LidRestSummary LidRest { get; init; } = new();
        public GazeSummary Gaze { get; init; } = new();
        public PupilSummary Pupil { get; init; } = new();
        public SpeechSmileSummary SpeechSmile { get; init; } = new();
        public JawRatioSummary JawRatios { get; init; } = new();
        public LipPostureSummary LipPostures { get; init; } = new();
        public IdleSummary Idle { get; init; } = new();
    }

    /// <summary>Shapes whose episode ramps feed attack/decay tuning. Left side stands in for both.</summary>
    private static readonly string[] EpisodeShapeNames =
    [
        "MouthCornerPullLeft",
        "MouthFrownLeft",
        "BrowInnerUpLeft",
        "BrowOuterUpLeft",
        "MouthPressLeft",
        "MouthStretchLeft",
        "EyeWideLeft",
        "EyeSquintLeft",
    ];

    /// <summary>Shapes whose activity during quiet (jaw-idle) periods feeds idle-motion tuning.</summary>
    private static readonly string[] IdleShapeNames =
    [
        "BrowInnerUpLeft",
        "BrowOuterUpLeft",
        "MouthPressLeft",
        "MouthCornerPullLeft",
        "EyeSquintLeft",
    ];

    public static Analysis Analyze(FaceFrameReplayPlayer.Recording rec, Options? options = null)
    {
        options ??= new Options();

        if (!rec.Ok)
        {
            return new Analysis { SourcePath = rec.SourcePath, Error = rec.Error };
        }

        List<FaceFrameReplayPlayer.Frame> frames = rec.Frames;
        double durationMs = rec.DurationMs;
        if (frames.Count < 2 || durationMs <= 0.0)
        {
            return new Analysis { SourcePath = rec.SourcePath, Error = "recording too short to analyze" };
        }

        double minutes = durationMs / 60000.0;
        int shapeCount = rec.ShapeCount > 0 ? rec.ShapeCount : frames[0].Expressions.Length;
        int jaw = FindShape(rec, "JawOpen");

        return new Analysis
        {
            SourcePath = rec.SourcePath,
            PrimaryModuleName = rec.PrimaryModuleName,
            FrameCount = frames.Count,
            DurationMs = durationMs,
            EffectiveHz = rec.EffectiveHz,
            Shapes = SummarizeShapes(rec, shapeCount, options),
            Episodes = [.. EpisodeShapeNames
                .Select(name => SummarizeEpisodes(rec, name, minutes, options))
                .OfType<EpisodeSummary>()],
            Blinks = SummarizeBlinks(frames, minutes, options),
            LidRest = SummarizeLidRest(frames, options),
            Gaze = SummarizeGaze(frames, minutes, options),
            Pupil = SummarizePupil(frames),
            SpeechSmile = SummarizeSpeechSmile(rec, jaw, options),
            JawRatios = SummarizeJawRatios(rec, jaw, options),
            LipPostures = SummarizeLipPostures(rec, options),
            Idle = SummarizeIdle(rec, jaw, minutes, options),
        };
    }

    private static int FindShape(FaceFrameReplayPlayer.Recording rec, string name)
    {
        return Array.IndexOf(rec.ShapeNames, name);
    }

    private static ShapeSummary[] SummarizeShapes(
        FaceFrameReplayPlayer.Recording rec, int shapeCount, Options options)
    {
        var shapes = new ShapeSummary[shapeCount];
        var values = new List<double>(rec.Frames.Count);
        var slews = new List<double>(rec.Frames.Count);
        for (int i = 0; i < shapeCount; i++)
        {
            values.Clear();
            slews.Clear();
            double sum = 0.0;
            double sumSq = 0.0;
            double max = 0.0;
            int active = 0;
            int strong = 0;
            for (int f = 0; f < rec.Frames.Count; f++)
            {
                FaceFrameReplayPlayer.Frame frame = rec.Frames[f];
                if (i >= frame.Expressions.Length)
                {
                    continue;
                }

                double v = frame.Expressions[i];
                values.Add(v);
                if (f > 0 && i < rec.Frames[f - 1].Expressions.Length)
                {
                    // Per second, so a 21 Hz tracker and a 120 Hz module compare directly.
                    double dtMs = frame.TimeMs - rec.Frames[f - 1].TimeMs;
                    if (dtMs > 0.0 && dtMs <= options.MaxFrameGapMs)
                    {
                        slews.Add(Math.Abs(v - rec.Frames[f - 1].Expressions[i]) * 1000.0 / dtMs);
                    }
                }

                sum += v;
                sumSq += v * v;
                if (v > max)
                {
                    max = v;
                }

                if (v > options.ActiveThreshold)
                {
                    active++;
                }

                if (v > options.StrongThreshold)
                {
                    strong++;
                }
            }

            int n = values.Count;
            double inv = n > 0 ? 1.0 / n : 0.0;
            double mean = sum * inv;
            double[] sorted = [.. values];
            Array.Sort(sorted);
            double[] sortedSlew = [.. slews];
            Array.Sort(sortedSlew);
            shapes[i] = new ShapeSummary
            {
                Index = i,
                Name = i < rec.ShapeNames.Length && !string.IsNullOrEmpty(rec.ShapeNames[i])
                    ? rec.ShapeNames[i]
                    : $"shape[{i}]",
                Mean = mean,
                StdDev = Math.Sqrt(Math.Max(0.0, (sumSq * inv) - (mean * mean))),
                P50 = Percentile(sorted, 0.50),
                P90 = Percentile(sorted, 0.90),
                P95 = Percentile(sorted, 0.95),
                Max = max,
                ActiveFraction = active * inv,
                StrongFraction = strong * inv,
                SlewP99PerSecond = Percentile(sortedSlew, 0.99),
                SlewMaxPerSecond = Percentile(sortedSlew, 1.0),
            };
        }

        return shapes;
    }

    private static EpisodeSummary? SummarizeEpisodes(
        FaceFrameReplayPlayer.Recording rec, string name, double minutes, Options options)
    {
        int idx = FindShape(rec, name);
        if (idx < 0)
        {
            return null;
        }

        List<FaceFrameReplayPlayer.Frame> frames = rec.Frames;
        var peaks = new List<double>();
        var durations = new List<double>();
        var onsets = new List<double>();
        var offsets = new List<double>();
        var gaps = new List<double>();

        int start = -1;
        double lastEndMs = double.NaN;
        for (int i = 0; i <= frames.Count; i++)
        {
            bool activeFrame = i < frames.Count &&
                idx < frames[i].Expressions.Length &&
                frames[i].Expressions[idx] > options.ActiveThreshold;
            if (activeFrame)
            {
                if (start < 0)
                {
                    start = i;
                }

                continue;
            }

            if (start < 0)
            {
                continue;
            }

            int end = i - 1;
            double peak = 0.0;
            for (int f = start; f <= end; f++)
            {
                peak = Math.Max(peak, frames[f].Expressions[idx]);
            }

            if (peak >= options.EpisodeMinPeak)
            {
                double startMs = frames[start].TimeMs;
                double endMs = frames[end].TimeMs;
                double high = 0.9 * peak;
                int firstHigh = start;
                int lastHigh = end;
                for (int f = start; f <= end; f++)
                {
                    if (frames[f].Expressions[idx] >= high)
                    {
                        firstHigh = f;
                        break;
                    }
                }

                for (int f = end; f >= start; f--)
                {
                    if (frames[f].Expressions[idx] >= high)
                    {
                        lastHigh = f;
                        break;
                    }
                }

                peaks.Add(peak);
                durations.Add(endMs - startMs);
                onsets.Add(frames[firstHigh].TimeMs - startMs);
                offsets.Add(endMs - frames[lastHigh].TimeMs);
                if (!double.IsNaN(lastEndMs))
                {
                    gaps.Add(startMs - lastEndMs);
                }

                lastEndMs = endMs;
            }

            start = -1;
        }

        peaks.Sort();
        durations.Sort();
        onsets.Sort();
        offsets.Sort();
        gaps.Sort();
        return new EpisodeSummary
        {
            Name = name,
            Count = peaks.Count,
            PerMinute = minutes > 0.0 ? peaks.Count / minutes : 0.0,
            PeakP50 = Percentile([.. peaks], 0.50),
            PeakP95 = Percentile([.. peaks], 0.95),
            DurationMsP50 = Percentile([.. durations], 0.50),
            OnsetMsP50 = Percentile([.. onsets], 0.50),
            OffsetMsP50 = Percentile([.. offsets], 0.50),
            GapMsP50 = Percentile([.. gaps], 0.50),
        };
    }

    private static BlinkSummary SummarizeBlinks(
        List<FaceFrameReplayPlayer.Frame> frames, double minutes, Options options)
    {
        var startTimes = new List<double>();
        var closedDurations = new List<double>();
        bool closed = false;
        double closeStartMs = 0.0;
        foreach (FaceFrameReplayPlayer.Frame frame in frames)
        {
            float openness = frame.EyeData.LeftOpenness;
            if (!closed && openness < options.BlinkCloseThreshold)
            {
                closed = true;
                closeStartMs = frame.TimeMs;
                startTimes.Add(closeStartMs);
            }
            else if (closed && openness >= options.BlinkCloseThreshold)
            {
                closed = false;
                closedDurations.Add(frame.TimeMs - closeStartMs);
            }
        }

        int doubles = 0;
        for (int i = 1; i < startTimes.Count; i++)
        {
            if (startTimes[i] - startTimes[i - 1] <= options.DoubleBlinkGapMs)
            {
                doubles++;
            }
        }

        closedDurations.Sort();
        return new BlinkSummary
        {
            Count = startTimes.Count,
            PerMinute = minutes > 0.0 ? startTimes.Count / minutes : 0.0,
            ClosedMsP50 = Percentile([.. closedDurations], 0.50),
            DoubleBlinkFraction = startTimes.Count > 0 ? (double)doubles / startTimes.Count : 0.0,
        };
    }

    private static LidRestSummary SummarizeLidRest(
        List<FaceFrameReplayPlayer.Frame> frames, Options options)
    {
        // Rest = frames at/above the reopen level whose immediate neighbors are too,
        // so partial frames on a blink's edges do not drag the rest distribution down.
        var rest = new List<double>(frames.Count);
        for (int i = 0; i < frames.Count; i++)
        {
            bool ok = frames[i].EyeData.LeftOpenness >= options.BlinkReopenThreshold &&
                (i == 0 || frames[i - 1].EyeData.LeftOpenness >= options.BlinkReopenThreshold) &&
                (i == frames.Count - 1 || frames[i + 1].EyeData.LeftOpenness >= options.BlinkReopenThreshold);
            if (ok)
            {
                rest.Add(frames[i].EyeData.LeftOpenness);
            }
        }

        double[] sorted = [.. rest];
        Array.Sort(sorted);
        return new LidRestSummary
        {
            Mean = rest.Count > 0 ? rest.Average() : 0.0,
            P10 = Percentile(sorted, 0.10),
            P50 = Percentile(sorted, 0.50),
            P90 = Percentile(sorted, 0.90),
        };
    }

    private static GazeSummary SummarizeGaze(
        List<FaceFrameReplayPlayer.Frame> frames, double minutes, Options options)
    {
        var xs = new List<double>(frames.Count);
        var ys = new List<double>(frames.Count);
        var amplitudes = new List<double>();
        var dwells = new List<double>();

        bool inSaccade = false;
        double saccadeStartX = 0.0;
        double saccadeStartY = 0.0;
        double lastSaccadeEndMs = double.NaN;
        for (int i = 0; i < frames.Count; i++)
        {
            FaceFrameReplayPlayer.Frame frame = frames[i];
            if (!frame.EyeValid)
            {
                continue;
            }

            double x = frame.EyeData.LeftGazeX;
            double y = frame.EyeData.LeftGazeY;
            xs.Add(x);
            ys.Add(y);
            if (i == 0)
            {
                continue;
            }

            FaceFrameReplayPlayer.Frame prev = frames[i - 1];
            double dtMs = frame.TimeMs - prev.TimeMs;
            if (dtMs <= 0.0 || dtMs > options.MaxFrameGapMs)
            {
                inSaccade = false;
                continue;
            }

            double dx = x - prev.EyeData.LeftGazeX;
            double dy = y - prev.EyeData.LeftGazeY;
            bool moving = Math.Sqrt((dx * dx) + (dy * dy)) > options.SaccadeMinDelta;
            if (moving && !inSaccade)
            {
                inSaccade = true;
                saccadeStartX = prev.EyeData.LeftGazeX;
                saccadeStartY = prev.EyeData.LeftGazeY;
                if (!double.IsNaN(lastSaccadeEndMs))
                {
                    dwells.Add(prev.TimeMs - lastSaccadeEndMs);
                }
            }
            else if (!moving && inSaccade)
            {
                inSaccade = false;
                double ax = x - saccadeStartX;
                double ay = y - saccadeStartY;
                amplitudes.Add(Math.Sqrt((ax * ax) + (ay * ay)));
                lastSaccadeEndMs = frame.TimeMs;
            }
        }

        double[] xSorted = [.. xs];
        double[] ySorted = [.. ys];
        Array.Sort(xSorted);
        Array.Sort(ySorted);
        amplitudes.Sort();
        dwells.Sort();
        return new GazeSummary
        {
            XP05 = Percentile(xSorted, 0.05),
            XP25 = Percentile(xSorted, 0.25),
            XP50 = Percentile(xSorted, 0.50),
            XP75 = Percentile(xSorted, 0.75),
            XP95 = Percentile(xSorted, 0.95),
            YP05 = Percentile(ySorted, 0.05),
            YP25 = Percentile(ySorted, 0.25),
            YP50 = Percentile(ySorted, 0.50),
            YP75 = Percentile(ySorted, 0.75),
            YP95 = Percentile(ySorted, 0.95),
            SaccadesPerMinute = minutes > 0.0 ? amplitudes.Count / minutes : 0.0,
            SaccadeAmplitudeP50 = Percentile([.. amplitudes], 0.50),
            SaccadeAmplitudeP95 = Percentile([.. amplitudes], 0.95),
            DwellMsP25 = Percentile([.. dwells], 0.25),
            DwellMsP50 = Percentile([.. dwells], 0.50),
            DwellMsP75 = Percentile([.. dwells], 0.75),
            DwellMsP90 = Percentile([.. dwells], 0.90),
        };
    }

    private static PupilSummary SummarizePupil(List<FaceFrameReplayPlayer.Frame> frames)
    {
        double min = double.MaxValue;
        double max = double.MinValue;
        double sum = 0.0;
        double sumSq = 0.0;
        int n = 0;
        foreach (FaceFrameReplayPlayer.Frame frame in frames)
        {
            if (!frame.EyeValid)
            {
                continue;
            }

            double v = frame.EyeData.LeftPupilDilation;
            min = Math.Min(min, v);
            max = Math.Max(max, v);
            sum += v;
            sumSq += v * v;
            n++;
        }

        if (n == 0)
        {
            return new PupilSummary();
        }

        double mean = sum / n;
        return new PupilSummary
        {
            Min = min,
            Mean = mean,
            Max = max,
            StdDev = Math.Sqrt(Math.Max(0.0, (sumSq / n) - (mean * mean))),
        };
    }

    private static SpeechSmileSummary SummarizeSpeechSmile(
        FaceFrameReplayPlayer.Recording rec, int jaw, Options options)
    {
        int smile = FindShape(rec, "MouthCornerPullLeft");
        if (jaw < 0 || smile < 0)
        {
            return new SpeechSmileSummary();
        }

        double speakSum = 0.0;
        int speakN = 0;
        int speakActive = 0;
        double quietSum = 0.0;
        int quietN = 0;
        int quietActive = 0;
        foreach (FaceFrameReplayPlayer.Frame frame in rec.Frames)
        {
            if (jaw >= frame.Expressions.Length || smile >= frame.Expressions.Length)
            {
                continue;
            }

            double jawV = frame.Expressions[jaw];
            double smileV = frame.Expressions[smile];
            if (jawV > options.SpeakingJawMin)
            {
                speakSum += smileV;
                speakN++;
                if (smileV > options.StrongThreshold)
                {
                    speakActive++;
                }
            }
            else if (jawV <= options.IdleJawMax)
            {
                quietSum += smileV;
                quietN++;
                if (smileV > options.StrongThreshold)
                {
                    quietActive++;
                }
            }
        }

        return new SpeechSmileSummary
        {
            SmileMeanSpeaking = speakN > 0 ? speakSum / speakN : 0.0,
            SmileMeanQuiet = quietN > 0 ? quietSum / quietN : 0.0,
            SmileActiveSpeaking = speakN > 0 ? (double)speakActive / speakN : 0.0,
            SmileActiveQuiet = quietN > 0 ? (double)quietActive / quietN : 0.0,
        };
    }

    // Rounding (funnel/pucker) and spreading (stretch) never co-occur in hardware recordings, so a
    // non-trivial conflict fraction means the source is blending postures a real face cannot hold.
    private static LipPostureSummary SummarizeLipPostures(FaceFrameReplayPlayer.Recording rec, Options options)
    {
        int[] funnel =
        [
            FindShape(rec, "LipFunnelUpperLeft"), FindShape(rec, "LipFunnelUpperRight"),
            FindShape(rec, "LipFunnelLowerLeft"), FindShape(rec, "LipFunnelLowerRight"),
        ];
        int[] pucker =
        [
            FindShape(rec, "LipPuckerUpperLeft"), FindShape(rec, "LipPuckerUpperRight"),
            FindShape(rec, "LipPuckerLowerLeft"), FindShape(rec, "LipPuckerLowerRight"),
        ];
        int[] stretch = [FindShape(rec, "MouthStretchLeft"), FindShape(rec, "MouthStretchRight")];

        int conflicts = 0;
        int funnelActive = 0;
        int puckerActive = 0;
        int stretchActive = 0;
        int n = 0;
        foreach (FaceFrameReplayPlayer.Frame frame in rec.Frames)
        {
            n++;
            bool f = Mean(frame.Expressions, funnel) > options.ActiveThreshold;
            bool p = Mean(frame.Expressions, pucker) > options.ActiveThreshold;
            bool s = Mean(frame.Expressions, stretch) > options.ActiveThreshold;
            if (f)
            {
                funnelActive++;
            }

            if (p)
            {
                puckerActive++;
            }

            if (s)
            {
                stretchActive++;
            }

            // Funnel and pucker are a rounding pair and co-occur legitimately; only rounding
            // against spreading is impossible.
            if ((f || p) && s)
            {
                conflicts++;
            }
        }

        if (n == 0)
        {
            return new LipPostureSummary();
        }

        return new LipPostureSummary
        {
            ConflictFraction = (double)conflicts / n,
            ConflictFrames = conflicts,
            FunnelActive = (double)funnelActive / n,
            PuckerActive = (double)puckerActive / n,
            StretchActive = (double)stretchActive / n,
        };
    }

    private static double Mean(float[] expressions, int[] indices)
    {
        double sum = 0.0;
        int n = 0;
        foreach (int i in indices)
        {
            if (i >= 0 && i < expressions.Length)
            {
                sum += expressions[i];
                n++;
            }
        }

        return n == 0 ? 0.0 : sum / n;
    }

    private static JawRatioSummary SummarizeJawRatios(
        FaceFrameReplayPlayer.Recording rec, int jaw, Options options)
    {
        int lower = FindShape(rec, "MouthLowerDownLeft");
        int upper = FindShape(rec, "MouthUpperUpLeft");
        if (jaw < 0)
        {
            return new JawRatioSummary();
        }

        double jawSum = 0.0;
        double lowerSum = 0.0;
        double upperSum = 0.0;
        int n = 0;
        foreach (FaceFrameReplayPlayer.Frame frame in rec.Frames)
        {
            if (jaw >= frame.Expressions.Length || frame.Expressions[jaw] <= options.SpeakingJawMin)
            {
                continue;
            }

            jawSum += frame.Expressions[jaw];
            lowerSum += lower >= 0 && lower < frame.Expressions.Length ? frame.Expressions[lower] : 0.0;
            upperSum += upper >= 0 && upper < frame.Expressions.Length ? frame.Expressions[upper] : 0.0;
            n++;
        }

        return new JawRatioSummary
        {
            LowerDownOverJaw = jawSum > 0.0 ? lowerSum / jawSum : 0.0,
            UpperUpOverJaw = jawSum > 0.0 ? upperSum / jawSum : 0.0,
            SpeakingFrames = n,
        };
    }

    private static IdleSummary SummarizeIdle(
        FaceFrameReplayPlayer.Recording rec, int jaw, double minutes, Options options)
    {
        if (jaw < 0)
        {
            return new IdleSummary();
        }

        // Mark idle segments: contiguous jaw-quiet runs lasting at least IdleMinSeconds.
        List<FaceFrameReplayPlayer.Frame> frames = rec.Frames;
        var idle = new bool[frames.Count];
        int segmentCount = 0;
        double idleMs = 0.0;
        int start = -1;
        for (int i = 0; i <= frames.Count; i++)
        {
            bool quiet = i < frames.Count &&
                jaw < frames[i].Expressions.Length &&
                frames[i].Expressions[jaw] <= options.IdleJawMax;
            if (quiet)
            {
                if (start < 0)
                {
                    start = i;
                }

                continue;
            }

            if (start >= 0)
            {
                int end = i - 1;
                double lengthMs = frames[end].TimeMs - frames[start].TimeMs;
                if (lengthMs >= options.IdleMinSeconds * 1000.0)
                {
                    segmentCount++;
                    idleMs += lengthMs;
                    for (int f = start; f <= end; f++)
                    {
                        idle[f] = true;
                    }
                }

                start = -1;
            }
        }

        double idleMinutes = idleMs / 60000.0;
        var shapeSummaries = new List<IdleShapeSummary>(IdleShapeNames.Length);
        foreach (string name in IdleShapeNames)
        {
            int idx = FindShape(rec, name);
            if (idx < 0)
            {
                continue;
            }

            var amplitudes = new List<double>();
            int events = 0;
            bool above = false;
            for (int i = 0; i < frames.Count; i++)
            {
                if (!idle[i] || idx >= frames[i].Expressions.Length)
                {
                    above = false;
                    continue;
                }

                double v = frames[i].Expressions[idx];
                amplitudes.Add(v);
                if (v > options.ActiveThreshold)
                {
                    if (!above)
                    {
                        events++;
                    }

                    above = true;
                }
                else
                {
                    above = false;
                }
            }

            double[] sorted = [.. amplitudes];
            Array.Sort(sorted);
            shapeSummaries.Add(new IdleShapeSummary
            {
                Name = name,
                EventsPerMinute = idleMinutes > 0.0 ? events / idleMinutes : 0.0,
                AmplitudeP90 = Percentile(sorted, 0.90),
            });
        }

        double totalMs = rec.DurationMs;
        return new IdleSummary
        {
            IdleFraction = totalMs > 0.0 ? idleMs / totalMs : 0.0,
            SegmentCount = segmentCount,
            Shapes = [.. shapeSummaries],
        };
    }

    /// <summary>Nearest-rank percentile over an already-sorted array; 0 when empty.</summary>
    private static double Percentile(double[] sorted, double q)
    {
        if (sorted.Length == 0)
        {
            return 0.0;
        }

        int index = (int)Math.Round(q * (sorted.Length - 1));
        return sorted[Math.Clamp(index, 0, sorted.Length - 1)];
    }

    /// <summary>Median across per-file values; the pooling primitive for multi-recording runs.</summary>
    private static double MedianOf(IReadOnlyList<Analysis> analyses, Func<Analysis, double> select)
    {
        double[] values = [.. analyses.Select(select)];
        Array.Sort(values);
        return Percentile(values, 0.50);
    }

    /// <summary>Renders one or more analyses as a plain-text report, with a pooled-median section when several are given.</summary>
    public static string FormatReport(IReadOnlyList<Analysis> analyses)
    {
        var sb = new StringBuilder();
        foreach (Analysis a in analyses)
        {
            AppendAnalysis(sb, a);
        }

        List<Analysis> ok = [.. analyses.Where(a => a.Ok)];
        if (ok.Count > 1)
        {
            AppendPooled(sb, ok);
        }

        return sb.ToString();
    }

    private static void AppendAnalysis(StringBuilder sb, Analysis a)
    {
        if (!a.Ok)
        {
            sb.AppendLine($"{a.SourcePath}: analysis failed: {a.Error}");
            return;
        }

        sb.AppendLine($"Recording {a.SourcePath}");
        sb.AppendLine(
            $"  module='{a.PrimaryModuleName}' frames={a.FrameCount} " +
            $"duration={FaceFrameReplayPlayer.FormatInvariant(a.DurationMs / 1000.0, 1)}s " +
            $"rate={FaceFrameReplayPlayer.FormatInvariant(a.EffectiveHz, 1)}Hz");

        sb.AppendLine($"  {"shape",-24} {"mean",7} {"p50",6} {"p95",6} {"max",6} {"act%",6} {"str%",6} {"slew99",7} {"slewMx",7}");
        foreach (ShapeSummary s in a.Shapes.Where(s => s.ActiveFraction > 0.005 || s.Max > 0.05)
            .OrderByDescending(s => s.ActiveFraction))
        {
            sb.AppendLine(
                $"  {s.Name,-24} {s.Mean,7:F3} {s.P50,6:F3} {s.P95,6:F3} {s.Max,6:F3} " +
                $"{s.ActiveFraction * 100.0,6:F1} {s.StrongFraction * 100.0,6:F1} " +
                $"{s.SlewP99PerSecond,7:F2} {s.SlewMaxPerSecond,7:F2}");
        }

        sb.AppendLine($"  {"episodes",-24} {"n",5} {"/min",6} {"peakP95",8} {"durP50",7} {"onP50",6} {"offP50",7} {"gapP50",7}");
        foreach (EpisodeSummary e in a.Episodes)
        {
            sb.AppendLine(
                $"  {e.Name,-24} {e.Count,5} {e.PerMinute,6:F1} {e.PeakP95,8:F2} {e.DurationMsP50,7:F0} " +
                $"{e.OnsetMsP50,6:F0} {e.OffsetMsP50,7:F0} {e.GapMsP50,7:F0}");
        }

        sb.AppendLine(
            $"  blinks: n={a.Blinks.Count} rate={a.Blinks.PerMinute:F1}/min closedP50={a.Blinks.ClosedMsP50:F0}ms " +
            $"double={a.Blinks.DoubleBlinkFraction * 100.0:F0}%");
        sb.AppendLine(
            $"  lid rest: mean={a.LidRest.Mean:F3} p10={a.LidRest.P10:F3} p50={a.LidRest.P50:F3} p90={a.LidRest.P90:F3}");
        sb.AppendLine(
            $"  gaze X p05..p95: {a.Gaze.XP05:F3}..{a.Gaze.XP95:F3} (p50 {a.Gaze.XP50:F3})  " +
            $"Y p05..p95: {a.Gaze.YP05:F3}..{a.Gaze.YP95:F3} (p50 {a.Gaze.YP50:F3})");
        sb.AppendLine(
            $"  saccades: {a.Gaze.SaccadesPerMinute:F1}/min ampP50={a.Gaze.SaccadeAmplitudeP50:F3} " +
            $"ampP95={a.Gaze.SaccadeAmplitudeP95:F3} dwellP25/50/75/90={a.Gaze.DwellMsP25:F0}/" +
            $"{a.Gaze.DwellMsP50:F0}/{a.Gaze.DwellMsP75:F0}/{a.Gaze.DwellMsP90:F0}ms");
        sb.AppendLine(
            $"  pupil: min={a.Pupil.Min:F3} mean={a.Pupil.Mean:F3} max={a.Pupil.Max:F3} sd={a.Pupil.StdDev:F3}");
        sb.AppendLine(
            $"  smile speaking/quiet: mean={a.SpeechSmile.SmileMeanSpeaking:F3}/{a.SpeechSmile.SmileMeanQuiet:F3} " +
            $"strong={a.SpeechSmile.SmileActiveSpeaking * 100.0:F1}%/{a.SpeechSmile.SmileActiveQuiet * 100.0:F1}%");
        sb.AppendLine(
            $"  jaw-linked ratios (n={a.JawRatios.SpeakingFrames}): lowerDown/jaw={a.JawRatios.LowerDownOverJaw:F2} " +
            $"upperUp/jaw={a.JawRatios.UpperUpOverJaw:F2}");
        sb.AppendLine(
            $"  lip postures: rounding-vs-stretch conflict={a.LipPostures.ConflictFraction * 100.0:F2}% " +
            $"({a.LipPostures.ConflictFrames} frames) active funnel/pucker/stretch=" +
            $"{a.LipPostures.FunnelActive * 100.0:F2}%/{a.LipPostures.PuckerActive * 100.0:F2}%/" +
            $"{a.LipPostures.StretchActive * 100.0:F2}%");
        sb.AppendLine(
            $"  idle: fraction={a.Idle.IdleFraction * 100.0:F1}% segments={a.Idle.SegmentCount}");
        foreach (IdleShapeSummary s in a.Idle.Shapes)
        {
            sb.AppendLine($"    {s.Name,-24} {s.EventsPerMinute,6:F1}/min ampP90={s.AmplitudeP90:F3}");
        }

        sb.AppendLine();
    }

    private static void AppendPooled(StringBuilder sb, List<Analysis> ok)
    {
        sb.AppendLine($"Pooled medians across {ok.Count} recordings");
        sb.AppendLine(
            $"  blinks/min={MedianOf(ok, a => a.Blinks.PerMinute):F1} " +
            $"closedP50={MedianOf(ok, a => a.Blinks.ClosedMsP50):F0}ms " +
            $"double={MedianOf(ok, a => a.Blinks.DoubleBlinkFraction) * 100.0:F0}%");
        sb.AppendLine(
            $"  lid rest mean={MedianOf(ok, a => a.LidRest.Mean):F3} p50={MedianOf(ok, a => a.LidRest.P50):F3}");
        sb.AppendLine(
            $"  gaze X p05..p95: {MedianOf(ok, a => a.Gaze.XP05):F3}..{MedianOf(ok, a => a.Gaze.XP95):F3}  " +
            $"Y p05..p95: {MedianOf(ok, a => a.Gaze.YP05):F3}..{MedianOf(ok, a => a.Gaze.YP95):F3} " +
            $"(Y p50 {MedianOf(ok, a => a.Gaze.YP50):F3})");
        sb.AppendLine(
            $"  saccades/min={MedianOf(ok, a => a.Gaze.SaccadesPerMinute):F1} " +
            $"ampP50={MedianOf(ok, a => a.Gaze.SaccadeAmplitudeP50):F3} " +
            $"dwellP50={MedianOf(ok, a => a.Gaze.DwellMsP50):F0}ms");
        sb.AppendLine(
            $"  smile speaking/quiet mean={MedianOf(ok, a => a.SpeechSmile.SmileMeanSpeaking):F3}/" +
            $"{MedianOf(ok, a => a.SpeechSmile.SmileMeanQuiet):F3}");
        sb.AppendLine(
            $"  lowerDown/jaw={MedianOf(ok, a => a.JawRatios.LowerDownOverJaw):F2} " +
            $"upperUp/jaw={MedianOf(ok, a => a.JawRatios.UpperUpOverJaw):F2}");

        sb.AppendLine($"  {"episodes",-24} {"/min",6} {"peakP95",8} {"durP50",7} {"onP50",6} {"offP50",7}");
        foreach (string name in EpisodeShapeNames)
        {
            List<Analysis> with = [.. ok.Where(a => a.Episodes.Any(e => e.Name == name))];
            if (with.Count == 0)
            {
                continue;
            }

            double Field(Func<EpisodeSummary, double> f)
            {
                return MedianOf(with, a => f(a.Episodes.First(e => e.Name == name)));
            }

            sb.AppendLine(
                $"  {name,-24} {Field(e => e.PerMinute),6:F1} {Field(e => e.PeakP95),8:F2} " +
                $"{Field(e => e.DurationMsP50),7:F0} {Field(e => e.OnsetMsP50),6:F0} {Field(e => e.OffsetMsP50),7:F0}");
        }

        sb.AppendLine($"  {"idle shape",-24} {"/min",6} {"ampP90",7}");
        foreach (string name in IdleShapeNames)
        {
            List<Analysis> with = [.. ok.Where(a => a.Idle.Shapes.Any(s => s.Name == name))];
            if (with.Count == 0)
            {
                continue;
            }

            sb.AppendLine(
                $"  {name,-24} {MedianOf(with, a => a.Idle.Shapes.First(s => s.Name == name).EventsPerMinute),6:F1} " +
                $"{MedianOf(with, a => a.Idle.Shapes.First(s => s.Name == name).AmplitudeP90),7:F3}");
        }

        sb.AppendLine();
    }

    /// <summary>Serializes the analyses (plus pooled medians when several) for downstream tooling.</summary>
    public static string ToJson(IReadOnlyList<Analysis> analyses)
    {
        List<Analysis> ok = [.. analyses.Where(a => a.Ok)];
        object payload = new
        {
            schema = 1,
            files = analyses,
            pooled = ok.Count > 1
                ? new
                {
                    blinksPerMinute = MedianOf(ok, a => a.Blinks.PerMinute),
                    blinkClosedMsP50 = MedianOf(ok, a => a.Blinks.ClosedMsP50),
                    doubleBlinkFraction = MedianOf(ok, a => a.Blinks.DoubleBlinkFraction),
                    lidRestMean = MedianOf(ok, a => a.LidRest.Mean),
                    lidRestP50 = MedianOf(ok, a => a.LidRest.P50),
                    gazeXP05 = MedianOf(ok, a => a.Gaze.XP05),
                    gazeXP95 = MedianOf(ok, a => a.Gaze.XP95),
                    gazeYP05 = MedianOf(ok, a => a.Gaze.YP05),
                    gazeYP50 = MedianOf(ok, a => a.Gaze.YP50),
                    gazeYP95 = MedianOf(ok, a => a.Gaze.YP95),
                    saccadesPerMinute = MedianOf(ok, a => a.Gaze.SaccadesPerMinute),
                    saccadeAmplitudeP50 = MedianOf(ok, a => a.Gaze.SaccadeAmplitudeP50),
                    dwellMsP50 = MedianOf(ok, a => a.Gaze.DwellMsP50),
                    smileMeanSpeaking = MedianOf(ok, a => a.SpeechSmile.SmileMeanSpeaking),
                    smileMeanQuiet = MedianOf(ok, a => a.SpeechSmile.SmileMeanQuiet),
                    lowerDownOverJaw = MedianOf(ok, a => a.JawRatios.LowerDownOverJaw),
                    upperUpOverJaw = MedianOf(ok, a => a.JawRatios.UpperUpOverJaw),
                }
                : null,
        };
        return JsonSerializer.Serialize(payload, new JsonSerializerOptions { WriteIndented = true });
    }
}
