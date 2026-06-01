// Lightweight compatibility surface for upstream VRCFaceTracking.Core.Params.Data.UnifiedTrackingMutator.
namespace VRCFaceTracking.Core.Params.Data;

public class UnifiedTrackingMutator
{
    public virtual UnifiedTrackingData MutateData(UnifiedTrackingData input) => input;

    public virtual void Load()
    {
    }

    public virtual Task Save() => Task.CompletedTask;
}
