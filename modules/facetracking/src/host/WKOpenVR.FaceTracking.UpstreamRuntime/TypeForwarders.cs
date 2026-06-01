// Mirrors upstream VRCFaceTracking.Core: SDK-owned module contract types are
// forwarded from Core so modules compiled against either assembly identity bind
// to the same runtime types.
using System.Runtime.CompilerServices;
using VRCFaceTracking;
using VRCFaceTracking.Core.Library;

[assembly: TypeForwardedTo(typeof(ExtTrackingModule))]
[assembly: TypeForwardedTo(typeof(ModuleMetadata))]
[assembly: TypeForwardedTo(typeof(ModuleState))]
