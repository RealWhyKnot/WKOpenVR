namespace WKOpenVR.FaceTracking.Sdk;

public interface IFaceTrackingModule
{
    FaceModuleInfo ModuleInfo { get; }

    FaceModuleCapabilities Capabilities { get; }

    ValueTask<FaceModuleInitResult> InitializeAsync(
        FaceModuleContext context,
        FaceModuleInitRequest request,
        CancellationToken cancellationToken);

    ValueTask UpdateAsync(FaceFrame frame, CancellationToken cancellationToken);

    ValueTask TeardownAsync(CancellationToken cancellationToken);
}

public sealed record FaceModuleInfo(
    string Uuid,
    string Name,
    string Vendor,
    Version Version);

[Flags]
public enum FaceModuleCapabilities
{
    None = 0,
    Eye = 1,
    Expression = 2,
    Head = 4
}

[Flags]
public enum FaceFrameFlags
{
    None = 0,
    ExpressionsValid = 1,
    EyeValid = 2,
    HeadValid = 4
}

public sealed record FaceModuleInitRequest(
    bool EyeAvailable,
    bool ExpressionAvailable,
    bool HeadAvailable);

public sealed record FaceModuleInitResult(
    bool EyeActive,
    bool ExpressionActive,
    bool HeadActive);

public sealed class FaceModuleContext
{
    public FaceModuleContext(string configDirectory, Action<string>? log = null)
    {
        ConfigDirectory = configDirectory;
        Log = log;
    }

    public string ConfigDirectory { get; }

    public Action<string>? Log { get; }
}

public sealed class FaceFrame
{
    public FaceFrame()
    {
        Expressions = new float[88];
    }

    public FaceFrameFlags Flags { get; set; }

    public float[] Expressions { get; }

    public EyeFrame Eye { get; } = new();

    public HeadFrame Head { get; } = new();

    public void Clear()
    {
        Array.Clear(Expressions);
        Eye.Clear();
        Head.Clear();
        Flags = FaceFrameFlags.None;
    }
}

public sealed class EyeFrame
{
    public EyeState Left { get; } = new();

    public EyeState Right { get; } = new();

    public float MinDilation { get; set; }

    public float MaxDilation { get; set; }

    public void Clear()
    {
        Left.Clear();
        Right.Clear();
        MinDilation = 0.0f;
        MaxDilation = 0.0f;
    }
}

public sealed class EyeState
{
    public float GazeX { get; set; }

    public float GazeY { get; set; }

    public float PupilDiameterMm { get; set; }

    public float Openness { get; set; }

    public void Clear()
    {
        GazeX = 0.0f;
        GazeY = 0.0f;
        PupilDiameterMm = 0.0f;
        Openness = 0.0f;
    }
}

public sealed class HeadFrame
{
    public float Yaw { get; set; }

    public float Pitch { get; set; }

    public float Roll { get; set; }

    public float PosX { get; set; }

    public float PosY { get; set; }

    public float PosZ { get; set; }

    public void Clear()
    {
        Yaw = 0.0f;
        Pitch = 0.0f;
        Roll = 0.0f;
        PosX = 0.0f;
        PosY = 0.0f;
        PosZ = 0.0f;
    }
}
