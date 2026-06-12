using System.Globalization;
using System.Text;

namespace WKOpenVR.FaceModuleHost.Workers;

/// <summary>
/// Compares two face-frame replay recordings - typically a reference capture from a real
/// headset face tracker (e.g. Virtual Desktop) against a candidate from the synthetic module -
/// and reports where the candidate's per-blendshape behavior diverges.
///
/// The two recordings are independent performances, so this is a distribution comparison, not a
/// frame-synced diff: per shape it reports how often and how hard each side drives it, then ranks
/// the largest divergences. That surfaces the concrete failure modes seen on the synthetic avatar -
/// a shape pinned high (the classic MouthClosed=1.0 stuck case), a shape the candidate drives that
/// the reference never touches, or a shape the candidate barely moves that the reference relies on.
/// </summary>
public static class FaceFrameReplayComparer
{
    public sealed class Options
    {
        /// <summary>A shape counts as "active" in a frame when its value exceeds this.</summary>
        public float ActiveThreshold { get; init; } = 0.05f;

        /// <summary>Mean level above which a low-variance shape is flagged as stuck/pinned.</summary>
        public float StuckMinMean { get; init; } = 0.30f;

        /// <summary>Std-dev below which an always-on shape is considered constant (pinned).</summary>
        public float StuckMaxStdDev { get; init; } = 0.03f;

        /// <summary>How many ranked divergences to keep in the report.</summary>
        public int TopCount { get; init; } = 16;
    }

    public sealed class ShapeStat
    {
        public int Index { get; init; }
        public string Name { get; init; } = string.Empty;
        public double Mean { get; init; }
        public double Max { get; init; }
        public double StdDev { get; init; }
        public double ActiveFraction { get; init; }
        public bool Stuck { get; init; }
    }

    public sealed class Summary
    {
        public string Label { get; init; } = string.Empty;
        public string SourcePath { get; init; } = string.Empty;
        public string PrimaryModuleName { get; init; } = string.Empty;
        public int FrameCount { get; init; }
        public double DurationMs { get; init; }
        public double EffectiveHz { get; init; }
        public double ExpressionValidFraction { get; init; }
        public double EyeValidFraction { get; init; }
        public double HeadValidFraction { get; init; }
        public double AvgNonZeroExpressions { get; init; }
        public double LeftOpennessMean { get; init; }
        public double RightOpennessMean { get; init; }
        public ShapeStat[] Shapes { get; init; } = [];
    }

    public sealed class ShapeDivergence
    {
        public int Index { get; init; }
        public string Name { get; init; } = string.Empty;
        public double ReferenceMean { get; init; }
        public double CandidateMean { get; init; }
        public double MeanDelta { get; init; }
        public double ReferenceActiveFraction { get; init; }
        public double CandidateActiveFraction { get; init; }
        public double ActiveFractionDelta { get; init; }
        public double Score { get; init; }
        public string Note { get; init; } = string.Empty;
    }

    public sealed class Comparison
    {
        public Summary Reference { get; init; } = new();
        public Summary Candidate { get; init; } = new();
        public ShapeDivergence[] TopDivergences { get; init; } = [];
        public ShapeDivergence[] StuckInCandidate { get; init; } = [];
        public string? Error { get; init; }
        public bool Ok => Error is null;
    }

    public static Comparison Compare(
        FaceFrameReplayPlayer.Recording reference,
        FaceFrameReplayPlayer.Recording candidate,
        Options? options = null)
    {
        options ??= new Options();

        if (!reference.Ok)
        {
            return new Comparison { Error = $"reference recording invalid: {reference.Error}" };
        }

        if (!candidate.Ok)
        {
            return new Comparison { Error = $"candidate recording invalid: {candidate.Error}" };
        }

        int shapeCount = Math.Min(
            reference.ShapeCount > 0 ? reference.ShapeCount : MaxExpressionLength(reference),
            candidate.ShapeCount > 0 ? candidate.ShapeCount : MaxExpressionLength(candidate));
        if (shapeCount <= 0)
        {
            return new Comparison { Error = "neither recording contains expression data" };
        }

        string[] names = reference.ShapeNames.Length >= shapeCount
            ? reference.ShapeNames
            : candidate.ShapeNames;

        Summary refSummary = Summarize(reference, "reference", shapeCount, names, options);
        Summary candSummary = Summarize(candidate, "candidate", shapeCount, names, options);

        var divergences = new List<ShapeDivergence>(shapeCount);
        for (int i = 0; i < shapeCount; i++)
        {
            ShapeStat r = refSummary.Shapes[i];
            ShapeStat c = candSummary.Shapes[i];
            double meanDelta = c.Mean - r.Mean;
            double activeDelta = c.ActiveFraction - r.ActiveFraction;
            double score = Math.Abs(meanDelta) + Math.Abs(activeDelta);
            divergences.Add(new ShapeDivergence
            {
                Index = i,
                Name = r.Name,
                ReferenceMean = r.Mean,
                CandidateMean = c.Mean,
                MeanDelta = meanDelta,
                ReferenceActiveFraction = r.ActiveFraction,
                CandidateActiveFraction = c.ActiveFraction,
                ActiveFractionDelta = activeDelta,
                Score = score,
                Note = DescribeDivergence(r, c),
            });
        }

        ShapeDivergence[] top = [.. divergences
            .Where(d => d.Score > 0.02)
            .OrderByDescending(d => d.Score)
            .Take(options.TopCount)];

        ShapeDivergence[] stuck = [.. divergences
            .Where(d => candSummary.Shapes[d.Index].Stuck && !refSummary.Shapes[d.Index].Stuck)
            .OrderByDescending(d => candSummary.Shapes[d.Index].Mean)];

        return new Comparison
        {
            Reference = refSummary,
            Candidate = candSummary,
            TopDivergences = top,
            StuckInCandidate = stuck,
        };
    }

    private static int MaxExpressionLength(FaceFrameReplayPlayer.Recording rec)
    {
        int max = 0;
        foreach (FaceFrameReplayPlayer.Frame frame in rec.Frames)
        {
            if (frame.Expressions.Length > max)
            {
                max = frame.Expressions.Length;
            }
        }

        return max;
    }

    private static Summary Summarize(
        FaceFrameReplayPlayer.Recording rec,
        string label,
        int shapeCount,
        string[] names,
        Options options)
    {
        int frames = rec.Frames.Count;
        var sum = new double[shapeCount];
        var sumSq = new double[shapeCount];
        var max = new double[shapeCount];
        var activeCount = new int[shapeCount];
        int exprValid = 0;
        int eyeValid = 0;
        int headValid = 0;
        long nonZeroTotal = 0;
        double leftOpenSum = 0.0;
        double rightOpenSum = 0.0;

        foreach (FaceFrameReplayPlayer.Frame frame in rec.Frames)
        {
            if (frame.ExpressionValid)
            {
                exprValid++;
            }

            if (frame.EyeValid)
            {
                eyeValid++;
            }

            if (frame.HeadValid)
            {
                headValid++;
            }

            nonZeroTotal += frame.NonZeroExpressions;
            leftOpenSum += frame.EyeData.LeftOpenness;
            rightOpenSum += frame.EyeData.RightOpenness;

            int n = Math.Min(shapeCount, frame.Expressions.Length);
            for (int i = 0; i < n; i++)
            {
                double v = frame.Expressions[i];
                sum[i] += v;
                sumSq[i] += v * v;
                if (v > max[i])
                {
                    max[i] = v;
                }

                if (v > options.ActiveThreshold)
                {
                    activeCount[i]++;
                }
            }
        }

        var shapes = new ShapeStat[shapeCount];
        double invFrames = frames > 0 ? 1.0 / frames : 0.0;
        for (int i = 0; i < shapeCount; i++)
        {
            double mean = sum[i] * invFrames;
            double variance = Math.Max(0.0, (sumSq[i] * invFrames) - (mean * mean));
            double stdDev = Math.Sqrt(variance);
            double activeFraction = activeCount[i] * invFrames;
            bool stuck = frames > 0 && mean >= options.StuckMinMean && stdDev <= options.StuckMaxStdDev;
            shapes[i] = new ShapeStat
            {
                Index = i,
                Name = i < names.Length && !string.IsNullOrEmpty(names[i]) ? names[i] : $"shape[{i}]",
                Mean = mean,
                Max = max[i],
                StdDev = stdDev,
                ActiveFraction = activeFraction,
                Stuck = stuck,
            };
        }

        return new Summary
        {
            Label = label,
            SourcePath = rec.SourcePath,
            PrimaryModuleName = rec.PrimaryModuleName,
            FrameCount = frames,
            DurationMs = rec.DurationMs,
            EffectiveHz = rec.EffectiveHz,
            ExpressionValidFraction = frames > 0 ? exprValid * invFrames : 0.0,
            EyeValidFraction = frames > 0 ? eyeValid * invFrames : 0.0,
            HeadValidFraction = frames > 0 ? headValid * invFrames : 0.0,
            AvgNonZeroExpressions = frames > 0 ? nonZeroTotal * invFrames : 0.0,
            LeftOpennessMean = frames > 0 ? leftOpenSum * invFrames : 0.0,
            RightOpennessMean = frames > 0 ? rightOpenSum * invFrames : 0.0,
            Shapes = shapes,
        };
    }

    private static string DescribeDivergence(ShapeStat reference, ShapeStat candidate)
    {
        if (candidate.Stuck && !reference.Stuck)
        {
            return $"pinned ~{candidate.Mean:F2} in candidate (likely stuck); reference varies (mean {reference.Mean:F2})";
        }

        if (candidate.ActiveFraction >= 0.5 && reference.ActiveFraction <= 0.1)
        {
            return "candidate drives this shape that reference rarely uses";
        }

        if (reference.ActiveFraction >= 0.5 && candidate.ActiveFraction <= 0.1)
        {
            return "candidate under-drives a shape the reference relies on";
        }

        if (candidate.Mean > reference.Mean + 0.1)
        {
            return "candidate over-drives vs reference";
        }

        if (reference.Mean > candidate.Mean + 0.1)
        {
            return "candidate under-drives vs reference";
        }

        return "differs";
    }

    /// <summary>Renders a comparison as a plain-text report for the CLI.</summary>
    public static string FormatReport(Comparison cmp)
    {
        if (!cmp.Ok)
        {
            return $"comparison failed: {cmp.Error}";
        }

        var sb = new StringBuilder();
        sb.AppendLine("Face replay comparison (reference vs candidate)");
        sb.AppendLine(new string('-', 64));
        AppendSummaryLine(sb, cmp.Reference);
        AppendSummaryLine(sb, cmp.Candidate);
        sb.AppendLine();

        if (cmp.StuckInCandidate.Length > 0)
        {
            sb.AppendLine($"Stuck/pinned shapes in candidate ({cmp.StuckInCandidate.Length}):");
            foreach (ShapeDivergence d in cmp.StuckInCandidate)
            {
                sb.AppendLine($"  {d.Name,-24} candidate~{d.CandidateMean:F2}  reference~{d.ReferenceMean:F2}");
            }

            sb.AppendLine();
        }

        sb.AppendLine($"Top divergent shapes (candidate - reference), {cmp.TopDivergences.Length} shown:");
        sb.AppendLine($"  {"shape",-24} {"ref.mean",9} {"cand.mean",9} {"d.mean",8} {"d.active",9}  note");
        foreach (ShapeDivergence d in cmp.TopDivergences)
        {
            sb.AppendLine(
                $"  {d.Name,-24} {d.ReferenceMean,9:F3} {d.CandidateMean,9:F3} {d.MeanDelta,8:+0.000;-0.000} " +
                $"{d.ActiveFractionDelta,9:+0.00;-0.00}  {d.Note}");
        }

        if (cmp.TopDivergences.Length == 0)
        {
            sb.AppendLine("  (no shape diverged beyond the reporting threshold)");
        }

        return sb.ToString();
    }

    private static void AppendSummaryLine(StringBuilder sb, Summary s)
    {
        sb.AppendLine(
            $"  {s.Label,-10} module='{s.PrimaryModuleName}' frames={s.FrameCount} " +
            $"{FaceFrameReplayPlayer.FormatInvariant(s.EffectiveHz, 1)}Hz " +
            $"exprValid={Pct(s.ExpressionValidFraction)} eyeValid={Pct(s.EyeValidFraction)} " +
            $"avgActiveShapes={FaceFrameReplayPlayer.FormatInvariant(s.AvgNonZeroExpressions, 1)}");
    }

    private static string Pct(double fraction)
    {
        return ((int)Math.Round(fraction * 100.0)).ToString(CultureInfo.InvariantCulture) + "%";
    }
}
