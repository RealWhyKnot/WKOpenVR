// Vendored from VRCFaceTracking (Apache-2.0) -- namespace VRCFaceTracking.Core.Params.Expressions preserved verbatim.
using VRCFaceTracking.Core.Params.Data;

namespace VRCFaceTracking.Core.Params.Expressions;

public enum UnifiedSimpleExpressions
{
    BrowUpRight,
    BrowUpLeft,
    BrowDownRight,
    BrowDownLeft,
    MouthSmileRight,
    MouthSmileLeft,
    MouthSadRight,
    MouthSadLeft,
}

public static class UnifiedSimplifier
{
    public static Dictionary<UnifiedSimpleExpressions, Func<UnifiedTrackingData, float>> ExpressionMap = new()
    {
        { UnifiedSimpleExpressions.BrowUpRight, exp =>
            exp.Shapes[(int)UnifiedExpressions.BrowOuterUpRight].Weight * .60f + exp.Shapes[(int)UnifiedExpressions.BrowInnerUpRight].Weight * .40f },
        { UnifiedSimpleExpressions.BrowUpLeft, exp =>
            exp.Shapes[(int)UnifiedExpressions.BrowOuterUpLeft].Weight * .60f + exp.Shapes[(int)UnifiedExpressions.BrowInnerUpLeft].Weight * .40f },
        { UnifiedSimpleExpressions.BrowDownRight, exp =>
            exp.Shapes[(int)UnifiedExpressions.BrowLowererRight].Weight * .75f + exp.Shapes[(int)UnifiedExpressions.BrowPinchRight].Weight * .25f },
        { UnifiedSimpleExpressions.BrowDownLeft, exp =>
            exp.Shapes[(int)UnifiedExpressions.BrowLowererLeft].Weight * .75f + exp.Shapes[(int)UnifiedExpressions.BrowPinchLeft].Weight * .25f },
        { UnifiedSimpleExpressions.MouthSmileRight, exp =>
            exp.Shapes[(int)UnifiedExpressions.MouthCornerPullRight].Weight * .8f + exp.Shapes[(int)UnifiedExpressions.MouthCornerSlantRight].Weight * .2f },
        { UnifiedSimpleExpressions.MouthSmileLeft, exp =>
            exp.Shapes[(int)UnifiedExpressions.MouthCornerPullLeft].Weight * .8f + exp.Shapes[(int)UnifiedExpressions.MouthCornerSlantLeft].Weight * .2f },
        { UnifiedSimpleExpressions.MouthSadRight, exp =>
            exp.Shapes[(int)UnifiedExpressions.MouthFrownRight].Weight > exp.Shapes[(int)UnifiedExpressions.MouthStretchRight].Weight
                ? exp.Shapes[(int)UnifiedExpressions.MouthFrownRight].Weight
                : exp.Shapes[(int)UnifiedExpressions.MouthStretchRight].Weight },
        { UnifiedSimpleExpressions.MouthSadLeft, exp =>
            exp.Shapes[(int)UnifiedExpressions.MouthFrownLeft].Weight > exp.Shapes[(int)UnifiedExpressions.MouthStretchLeft].Weight
                ? exp.Shapes[(int)UnifiedExpressions.MouthFrownLeft].Weight
                : exp.Shapes[(int)UnifiedExpressions.MouthStretchLeft].Weight },
    };
}
