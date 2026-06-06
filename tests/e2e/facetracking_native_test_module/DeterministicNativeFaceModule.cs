using WKOpenVR.FaceTracking.Sdk;

namespace WKOpenVR.FaceTracking.NativeTestModule;

public sealed class DeterministicNativeFaceModule : IFaceTrackingModule
{
    private bool expressionActive;
    private bool eyeActive;

    public FaceModuleInfo ModuleInfo { get; } = new(
        "44444444-5555-6666-7777-888888888888",
        "WKOpenVR E2E Native Face Module",
        "WKOpenVR Tests",
        new Version(1, 0, 0));

    public FaceModuleCapabilities Capabilities =>
        FaceModuleCapabilities.Eye | FaceModuleCapabilities.Expression | FaceModuleCapabilities.Head;

    public ValueTask<FaceModuleInitResult> InitializeAsync(
        FaceModuleContext context,
        FaceModuleInitRequest request,
        CancellationToken cancellationToken)
    {
        eyeActive = request.EyeAvailable;
        expressionActive = request.ExpressionAvailable;
        return ValueTask.FromResult(new FaceModuleInitResult(
            EyeActive: eyeActive,
            ExpressionActive: expressionActive,
            HeadActive: request.HeadAvailable));
    }

    public ValueTask UpdateAsync(FaceFrame frame, CancellationToken cancellationToken)
    {
        frame.Clear();

        if (eyeActive)
        {
            frame.Eye.Left.GazeX = 0.11f;
            frame.Eye.Left.GazeY = -0.22f;
            frame.Eye.Left.Openness = 0.62f;
            frame.Eye.Left.PupilDiameterMm = 4.0f;
            frame.Eye.Right.GazeX = -0.13f;
            frame.Eye.Right.GazeY = -0.18f;
            frame.Eye.Right.Openness = 0.58f;
            frame.Eye.Right.PupilDiameterMm = 4.2f;
            frame.Eye.MinDilation = 3.0f;
            frame.Eye.MaxDilation = 5.0f;
            frame.Flags |= FaceFrameFlags.EyeValid;
        }

        if (expressionActive)
        {
            frame.Expressions[22] = 0.65f; // JawOpen in upstream UnifiedExpressions order.
            frame.Expressions[57] = 0.35f; // MouthCornerPullLeft in upstream UnifiedExpressions order.
            frame.Flags |= FaceFrameFlags.ExpressionsValid;
        }

        frame.Head.Yaw = 0.15f;
        frame.Head.Pitch = -0.05f;
        frame.Head.Roll = 0.02f;
        frame.Head.PosX = 0.01f;
        frame.Head.PosY = 0.02f;
        frame.Head.PosZ = -0.03f;
        frame.Flags |= FaceFrameFlags.HeadValid;

        Thread.Sleep(1);
        return ValueTask.CompletedTask;
    }

    public ValueTask TeardownAsync(CancellationToken cancellationToken)
    {
        eyeActive = false;
        expressionActive = false;
        return ValueTask.CompletedTask;
    }
}
