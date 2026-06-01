// ----------------------------------------------------------------------------
// Vendored from VRCFaceTracking (Apache-2.0), HEAVILY MODIFIED.
// Original: https://github.com/benaclejames/VRCFaceTracking/blob/35857c01315c32e0e45dcde2f6f8fe495216fa0c/
//   VRCFaceTracking.Core/UnifiedTracking.cs
// Copyright (c) benaclejames and contributors. Licensed under Apache 2.0.
//
// Modifications: output-parameter arrays are present for binary/API
// compatibility, but left empty because WKOpenVR publishes frames itself.
// ----------------------------------------------------------------------------
using VRCFaceTracking.Core.Params;
using VRCFaceTracking.Core.Params.Data;
using VRCFaceTracking.Core.Types;

namespace VRCFaceTracking;

public class UnifiedTracking
{
    /// <summary>
    /// Eye image data sent from the loaded eye module.
    /// </summary>
    public static Image EyeImageData = new();

    /// <summary>
    /// Lip / Expression image data sent from the loaded expressions module.
    /// </summary>
    public static Image LipImageData = new();

    /// <summary>
    /// Latest Expression Data accessible and sent by all VRCFaceTracking modules.
    /// </summary>
    public static UnifiedTrackingData Data = new();

    /// <summary>
    /// Container of features that mutate incoming expression data.
    /// </summary>
    public static UnifiedTrackingMutator? Mutator;

    public static readonly Parameter[] AllParameters_v1 = [];
    public static readonly Parameter[] AllParameters_v2 = [];
    public static readonly Parameter[] HeadParameters = [];
    public static readonly Parameter[] AllParameters = [];

    /// <summary>
    /// Central update action for expression data subscribers.
    /// </summary>
    public static Action<UnifiedTrackingData> OnUnifiedDataUpdated = _ => { };

    /// <summary>
    /// Updates subscribers with the current data snapshot.
    /// </summary>
    public static Task UpdateData(CancellationToken ct)
    {
        ct.ThrowIfCancellationRequested();
        UnifiedTrackingData data = Mutator?.MutateData(Data) ?? Data;
        OnUnifiedDataUpdated.Invoke(data);
        return Task.CompletedTask;
    }
}
