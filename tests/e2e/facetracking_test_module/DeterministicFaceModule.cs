using VRCFaceTracking;
using VRCFaceTracking.Core;
using VRCFaceTracking.Core.Params.Expressions;
using VRCFaceTracking.Core.Types;

namespace WKOpenVR.FaceTracking.TestModule;

public sealed class DeterministicFaceModule : ExtTrackingModule
{
    public override (bool SupportsEye, bool SupportsExpression) Supported => (true, true);

    public DeterministicFaceModule()
    {
        ModuleInformation = new ModuleMetadata
        {
            Name = "WKOpenVR E2E Face Module",
            StaticImages = [],
        };
    }

    public override (bool eyeSuccess, bool expressionSuccess) Initialize(
        bool eyeAvailable,
        bool expressionAvailable)
    {
        _ = Utils.CustomLibsDirectory;
        VRChat.EnsureVRCOSCDirectory();
        UnifiedTracking.OnUnifiedDataUpdated += _ => { };
        UnifiedTracking.EyeImageData = new Image
        {
            ImageSize = (1, 1),
            ImageData = [0, 0, 0, 255],
            SupportsImage = true,
        };
        UnifiedTracking.LipImageData = new Image
        {
            ImageSize = (1, 1),
            ImageData = [255, 255, 255, 255],
            SupportsImage = true,
        };

        ModuleInformation.Active = true;
        ModuleInformation.UsingEye = eyeAvailable;
        ModuleInformation.UsingExpression = expressionAvailable;
        WriteFrame();
        return (eyeAvailable, expressionAvailable);
    }

    public override void Update()
    {
        WriteFrame();
        Thread.Sleep(1);
    }

    public override void Teardown()
    {
        ModuleInformation.Active = false;
    }

    private static void WriteFrame()
    {
        for (int i = 0; i < UnifiedTracking.Data.Shapes.Length; ++i)
        {
            UnifiedTracking.Data.Shapes[i].Weight = 0.0f;
        }

        UnifiedTracking.Data.Eye.Left.Gaze = new Vector2(0.11f, -0.22f);
        UnifiedTracking.Data.Eye.Left.Openness = 0.62f;
        UnifiedTracking.Data.Eye.Left.PupilDiameter_MM = 4.0f;

        UnifiedTracking.Data.Eye.Right.Gaze = new Vector2(-0.13f, -0.18f);
        UnifiedTracking.Data.Eye.Right.Openness = 0.58f;
        UnifiedTracking.Data.Eye.Right.PupilDiameter_MM = 4.2f;
        UnifiedTracking.Data.Eye._minDilation = 3.0f;
        UnifiedTracking.Data.Eye._maxDilation = 5.0f;

        UnifiedTracking.Data.Shapes[(int)UnifiedExpressions.JawOpen].Weight = 0.75f;
        UnifiedTracking.Data.Shapes[(int)UnifiedExpressions.EyeWideLeft].Weight = 0xFFFFFFFF;
        UnifiedTracking.Data.Shapes[(int)UnifiedExpressions.MouthCornerPullLeft].Weight = 0.25f;
        _ = UnifiedSimplifier.ExpressionMap[UnifiedSimpleExpressions.MouthSmileLeft](UnifiedTracking.Data);
    }
}
