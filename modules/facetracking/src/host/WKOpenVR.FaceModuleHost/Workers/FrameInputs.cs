using System.Numerics;

namespace WKOpenVR.FaceModuleHost.Workers;

public sealed class EyeFrameSink
{
    public EyeGaze Left { get; } = new();
    public EyeGaze Right { get; } = new();

    public EyeFrameSink()
    {
        Left.OriginHmd = new Vector3(-0.032f, 0.0f, 0.0f);
        Right.OriginHmd = new Vector3(0.032f, 0.0f, 0.0f);
        Left.DirHmd = new Vector3(0.0f, 0.0f, -1.0f);
        Right.DirHmd = new Vector3(0.0f, 0.0f, -1.0f);
        LeftOpenness = 1.0f;
        RightOpenness = 1.0f;
        PupilDilationLeft = 0.5f;
        PupilDilationRight = 0.5f;
    }

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
