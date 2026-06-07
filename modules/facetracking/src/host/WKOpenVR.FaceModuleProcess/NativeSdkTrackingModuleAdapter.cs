using System.Reflection;
using Microsoft.Extensions.Logging;
using VRCFaceTracking;
using VRCFaceTracking.Core.Params.Data;
using VRCFaceTracking.Core.Params.Expressions;

namespace WKOpenVR.FaceModuleProcess;

internal sealed class NativeSdkTrackingModuleAdapter : ExtTrackingModule
{
    private const string NativeInterfaceName = "WKOpenVR.FaceTracking.Sdk.IFaceTrackingModule";
    private const int CapEye = 1;
    private const int CapExpression = 2;
    private const long FlagExpressionsValid = 1;
    private const long FlagEyeValid = 2;
    private const long FlagHeadValid = 4;
    private const float InvalidFloat = 0xFFFFFFFF;

    private readonly object module;
    private readonly Type moduleType;
    private readonly Assembly sdkAssembly;
    private readonly string moduleDirectory;
    private readonly MethodInfo initializeAsync;
    private readonly MethodInfo updateAsync;
    private readonly MethodInfo teardownAsync;
    private readonly object frame;
    private readonly MethodInfo? clearFrame;
    private readonly MethodInfo? sanitizeFrame;
    private readonly PropertyInfo flagsProperty;
    private readonly PropertyInfo expressionsProperty;
    private readonly PropertyInfo eyeProperty;
    private readonly PropertyInfo headProperty;
    private readonly bool supportsEye;
    private readonly bool supportsExpression;

    private NativeSdkTrackingModuleAdapter(Type moduleType, string modulePath)
    {
        this.moduleType = moduleType;
        module = Activator.CreateInstance(moduleType)
            ?? throw new InvalidOperationException($"Could not instantiate native module {moduleType.FullName}.");

        var nativeInterface = moduleType.GetInterfaces().FirstOrDefault(i => i.FullName == NativeInterfaceName)
            ?? throw new InvalidOperationException($"{moduleType.FullName} does not implement {NativeInterfaceName}.");
        sdkAssembly = nativeInterface.Assembly;
        moduleDirectory = Path.GetDirectoryName(modulePath) ?? Environment.CurrentDirectory;

        initializeAsync = moduleType.GetMethod("InitializeAsync")
            ?? throw new MissingMethodException(moduleType.FullName, "InitializeAsync");
        updateAsync = moduleType.GetMethod("UpdateAsync")
            ?? throw new MissingMethodException(moduleType.FullName, "UpdateAsync");
        teardownAsync = moduleType.GetMethod("TeardownAsync")
            ?? throw new MissingMethodException(moduleType.FullName, "TeardownAsync");

        var faceFrameType = SdkType("WKOpenVR.FaceTracking.Sdk.FaceFrame");
        frame = Activator.CreateInstance(faceFrameType)
            ?? throw new InvalidOperationException("Could not create native FaceFrame.");
        clearFrame = faceFrameType.GetMethod("Clear", Type.EmptyTypes);
        flagsProperty = RequiredProperty(faceFrameType, "Flags");
        expressionsProperty = RequiredProperty(faceFrameType, "Expressions");
        eyeProperty = RequiredProperty(faceFrameType, "Eye");
        headProperty = RequiredProperty(faceFrameType, "Head");

        var validatorType = sdkAssembly.GetType("WKOpenVR.FaceTracking.Sdk.FaceFrameValidator");
        sanitizeFrame = validatorType?.GetMethod("Sanitize", BindingFlags.Public | BindingFlags.Static);

        long caps = GetCapabilities();
        supportsEye = (caps & CapEye) != 0;
        supportsExpression = (caps & CapExpression) != 0;

        ModuleInformation = new ModuleMetadata
        {
            Name = GetModuleName(),
            StaticImages = []
        };
    }

    public override (bool SupportsEye, bool SupportsExpression) Supported => (supportsEye, supportsExpression);

    public static bool IsNativeSdkModule(Type type)
    {
        return !type.IsAbstract &&
               !type.IsInterface &&
               type.GetInterfaces().Any(i => i.FullName == NativeInterfaceName);
    }

    public static NativeSdkTrackingModuleAdapter Create(Type moduleType, string modulePath)
    {
        return new NativeSdkTrackingModuleAdapter(moduleType, modulePath);
    }

    public override (bool eyeSuccess, bool expressionSuccess) Initialize(bool eyeAvailable, bool expressionAvailable)
    {
        var contextType = SdkType("WKOpenVR.FaceTracking.Sdk.FaceModuleContext");
        var requestType = SdkType("WKOpenVR.FaceTracking.Sdk.FaceModuleInitRequest");
        object context = CreateModuleContext(contextType);
        var request = Activator.CreateInstance(requestType, eyeAvailable, expressionAvailable, true)
            ?? throw new InvalidOperationException("Could not create native FaceModuleInitRequest.");

        var result = AwaitAsyncResult(initializeAsync.Invoke(module, [context, request, CancellationToken.None]));
        bool eyeActive = GetBool(result, "EyeActive");
        bool expressionActive = GetBool(result, "ExpressionActive");

        ModuleInformation.UsingEye = eyeActive;
        ModuleInformation.UsingExpression = expressionActive;
        ModuleInformation.Active = eyeActive || expressionActive;
        return (eyeActive, expressionActive);
    }

    public override void Update()
    {
        clearFrame?.Invoke(frame, []);
        AwaitAsyncResult(updateAsync.Invoke(module, [frame, CancellationToken.None]));
        sanitizeFrame?.Invoke(null, [frame]);
        CopyNativeFrameToUnifiedTracking();
    }

    public override void Teardown()
    {
        AwaitAsyncResult(teardownAsync.Invoke(module, [CancellationToken.None]));
    }

    private object CreateModuleContext(Type contextType)
    {
        // Prefer the leveled-logger constructor (string, Action<int,string>, int) so the module's
        // Trace/Debug output flows when the host is in a verbose logging mode; fall back to the legacy
        // (string, Action<string>) constructor for modules built against an older SDK.
        var leveledCtor = contextType.GetConstructor(new[] { typeof(string), typeof(Action<int, string>), typeof(int) });
        if (leveledCtor != null)
        {
            var sink = new Action<int, string>((level, message) => Logger?.Log(MapModuleLogLevel(level), "{message}", message));
            return Activator.CreateInstance(contextType, moduleDirectory, sink, CurrentMinModuleLogLevel())
                ?? throw new InvalidOperationException("Could not create native FaceModuleContext.");
        }

        var logAction = new Action<string>(message => Logger?.LogInformation("{message}", message));
        return Activator.CreateInstance(contextType, moduleDirectory, logAction)
            ?? throw new InvalidOperationException("Could not create native FaceModuleContext.");
    }

    private int CurrentMinModuleLogLevel()
    {
        if (Logger == null)
        {
            return 2;
        }

        if (Logger.IsEnabled(LogLevel.Trace))
        {
            return 0;
        }

        if (Logger.IsEnabled(LogLevel.Debug))
        {
            return 1;
        }

        if (Logger.IsEnabled(LogLevel.Information))
        {
            return 2;
        }

        return Logger.IsEnabled(LogLevel.Warning) ? 3 : 4;
    }

    private static LogLevel MapModuleLogLevel(int level) => level switch
    {
        0 => LogLevel.Trace,
        1 => LogLevel.Debug,
        2 => LogLevel.Information,
        3 => LogLevel.Warning,
        _ => LogLevel.Error,
    };

    private void CopyNativeFrameToUnifiedTracking()
    {
        long flags = Convert.ToInt64(flagsProperty.GetValue(frame) ?? 0);

        if ((flags & FlagExpressionsValid) != 0)
        {
            var expressions = (float[]?)expressionsProperty.GetValue(frame);
            int count = Math.Min(expressions?.Length ?? 0, (int)UnifiedExpressions.Max);
            for (int i = 0; i < count; i++)
            {
                UnifiedTracking.Data.Shapes[i].Weight = expressions![i];
            }
        }
        else
        {
            InvalidateExpressions();
        }

        if ((flags & FlagEyeValid) != 0)
        {
            object eye = eyeProperty.GetValue(frame)
                ?? throw new InvalidOperationException("Native FaceFrame.Eye was null.");
            object left = RequiredProperty(eye.GetType(), "Left").GetValue(eye)
                ?? throw new InvalidOperationException("Native FaceFrame.Eye.Left was null.");
            object right = RequiredProperty(eye.GetType(), "Right").GetValue(eye)
                ?? throw new InvalidOperationException("Native FaceFrame.Eye.Right was null.");

            UnifiedTracking.Data.Eye.Left.Gaze.x = GetFloat(left, "GazeX");
            UnifiedTracking.Data.Eye.Left.Gaze.y = GetFloat(left, "GazeY");
            UnifiedTracking.Data.Eye.Left.PupilDiameter_MM = GetFloat(left, "PupilDiameterMm");
            UnifiedTracking.Data.Eye.Left.Openness = GetFloat(left, "Openness");
            UnifiedTracking.Data.Eye.Right.Gaze.x = GetFloat(right, "GazeX");
            UnifiedTracking.Data.Eye.Right.Gaze.y = GetFloat(right, "GazeY");
            UnifiedTracking.Data.Eye.Right.PupilDiameter_MM = GetFloat(right, "PupilDiameterMm");
            UnifiedTracking.Data.Eye.Right.Openness = GetFloat(right, "Openness");
            UnifiedTracking.Data.Eye._minDilation = GetFloat(eye, "MinDilation");
            UnifiedTracking.Data.Eye._maxDilation = GetFloat(eye, "MaxDilation");
        }
        else
        {
            InvalidateEye();
        }

        if ((flags & FlagHeadValid) != 0)
        {
            object head = headProperty.GetValue(frame)
                ?? throw new InvalidOperationException("Native FaceFrame.Head was null.");
            UnifiedTracking.Data.Head.HeadYaw = GetFloat(head, "Yaw");
            UnifiedTracking.Data.Head.HeadPitch = GetFloat(head, "Pitch");
            UnifiedTracking.Data.Head.HeadRoll = GetFloat(head, "Roll");
            UnifiedTracking.Data.Head.HeadPosX = GetFloat(head, "PosX");
            UnifiedTracking.Data.Head.HeadPosY = GetFloat(head, "PosY");
            UnifiedTracking.Data.Head.HeadPosZ = GetFloat(head, "PosZ");
        }
        else
        {
            InvalidateHead();
        }
    }

    private static void InvalidateExpressions()
    {
        for (int i = 0; i < (int)UnifiedExpressions.Max; i++)
        {
            UnifiedTracking.Data.Shapes[i].Weight = InvalidFloat;
        }
    }

    private static void InvalidateEye()
    {
        UnifiedTracking.Data.Eye.Left.Gaze.x = InvalidFloat;
        UnifiedTracking.Data.Eye.Left.Gaze.y = InvalidFloat;
        UnifiedTracking.Data.Eye.Left.PupilDiameter_MM = InvalidFloat;
        UnifiedTracking.Data.Eye.Left.Openness = InvalidFloat;
        UnifiedTracking.Data.Eye.Right.Gaze.x = InvalidFloat;
        UnifiedTracking.Data.Eye.Right.Gaze.y = InvalidFloat;
        UnifiedTracking.Data.Eye.Right.PupilDiameter_MM = InvalidFloat;
        UnifiedTracking.Data.Eye.Right.Openness = InvalidFloat;
        UnifiedTracking.Data.Eye._minDilation = InvalidFloat;
        UnifiedTracking.Data.Eye._maxDilation = InvalidFloat;
    }

    private static void InvalidateHead()
    {
        UnifiedTracking.Data.Head.HeadYaw = InvalidFloat;
        UnifiedTracking.Data.Head.HeadPitch = InvalidFloat;
        UnifiedTracking.Data.Head.HeadRoll = InvalidFloat;
        UnifiedTracking.Data.Head.HeadPosX = InvalidFloat;
        UnifiedTracking.Data.Head.HeadPosY = InvalidFloat;
        UnifiedTracking.Data.Head.HeadPosZ = InvalidFloat;
    }

    private long GetCapabilities()
    {
        object? caps = moduleType.GetProperty("Capabilities")?.GetValue(module);
        return caps == null ? 0 : Convert.ToInt64(caps);
    }

    private string GetModuleName()
    {
        object? info = moduleType.GetProperty("ModuleInfo")?.GetValue(module);
        object? name = info?.GetType().GetProperty("Name")?.GetValue(info);
        return name?.ToString() ?? moduleType.Name;
    }

    private Type SdkType(string fullName)
    {
        return sdkAssembly.GetType(fullName)
            ?? throw new TypeLoadException($"SDK type not found: {fullName}");
    }

    private static PropertyInfo RequiredProperty(Type type, string name)
    {
        return type.GetProperty(name)
            ?? throw new MissingMemberException(type.FullName, name);
    }

    private static object? AwaitAsyncResult(object? result)
    {
        if (result == null)
        {
            return null;
        }

        if (result is Task task)
        {
            task.GetAwaiter().GetResult();
            return task.GetType().IsGenericType ? task.GetType().GetProperty("Result")?.GetValue(task) : null;
        }

        MethodInfo? asTask = result.GetType().GetMethod("AsTask", Type.EmptyTypes);
        if (asTask != null)
        {
            var valueTaskAsTask = (Task?)asTask.Invoke(result, []);
            if (valueTaskAsTask == null)
            {
                return null;
            }

            valueTaskAsTask.GetAwaiter().GetResult();
            return valueTaskAsTask.GetType().IsGenericType
                ? valueTaskAsTask.GetType().GetProperty("Result")?.GetValue(valueTaskAsTask)
                : null;
        }

        return result;
    }

    private static bool GetBool(object? source, string property)
    {
        object? value = source?.GetType().GetProperty(property)?.GetValue(source);
        return value is bool b && b;
    }

    private static float GetFloat(object source, string property)
    {
        object? value = RequiredProperty(source.GetType(), property).GetValue(source);
        return value == null ? 0.0f : Convert.ToSingle(value);
    }
}
