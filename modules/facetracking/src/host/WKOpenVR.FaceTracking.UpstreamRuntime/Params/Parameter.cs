// Lightweight compatibility surface for upstream VRCFaceTracking.Core.Params.Parameter.
namespace VRCFaceTracking.Core.Params;

public class Parameter
{
    public string Name { get; protected set; } = "";
    public string Address { get; protected set; } = "";

    public virtual (string paramName, Parameter paramLiteral)[] GetParamNames() =>
        string.IsNullOrEmpty(Name) ? [] : [(Name, this)];
}
