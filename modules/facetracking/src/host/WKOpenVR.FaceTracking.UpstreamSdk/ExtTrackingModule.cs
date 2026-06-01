// Vendored from VRCFaceTracking (Apache-2.0) -- namespace VRCFaceTracking preserved verbatim.
using Microsoft.Extensions.Logging;
using VRCFaceTracking.Core.Library;

namespace VRCFaceTracking;

public abstract class ExtTrackingModule
{
    public virtual (bool SupportsEye, bool SupportsExpression) Supported => (false, false);

    public ModuleState Status = ModuleState.Uninitialized;

    public ILogger Logger;

    public ModuleMetadata ModuleInformation;

    public abstract (bool eyeSuccess, bool expressionSuccess) Initialize(bool eyeAvailable, bool expressionAvailable);

    public abstract void Update();

    public abstract void Teardown();
}
