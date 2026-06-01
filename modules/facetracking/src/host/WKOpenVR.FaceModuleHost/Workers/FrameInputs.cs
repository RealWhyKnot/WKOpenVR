using System.Numerics;

namespace WKOpenVR.FaceModuleHost.Workers;

public sealed class EyeFrameSink
{
    public EyeGaze Left { get; } = new();
    public EyeGaze Right { get; } = new();

    public float LeftOpenness { get; set; }
    public float RightOpenness { get; set; }
    public float PupilDilationLeft { get; set; }
    public float PupilDilationRight { get; set; }
}

public sealed class EyeGaze
{
    public Vector3 OriginHmd { get; set; }
    public Vector3 DirHmd { get; set; }
    public float Confidence { get; set; }
}
