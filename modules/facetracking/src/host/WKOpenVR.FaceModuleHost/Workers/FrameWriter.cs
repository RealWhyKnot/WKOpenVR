using System.IO.MemoryMappedFiles;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using WKOpenVR.FaceModuleHost.Logging;
using UnsafeHelper = System.Runtime.CompilerServices.Unsafe;

namespace WKOpenVR.FaceModuleHost.Workers;

// ---------------------------------------------------------------------------
// Shmem layout mirrors Protocol.h FaceTrackingFrameShmem::ShmemData exactly.
// Pack=8 matches MSVC default for x64. Any structural change here must be
// reflected in Protocol.h (and vice versa) or the driver will read garbage.
// ---------------------------------------------------------------------------

[InlineArray(3)]
internal struct Float3 { private float _e0; }

// Wire-side expression array carries upstream VRCFaceTracking.UnifiedExpressions
// order (88 entries excluding Max). The driver remaps to our 63-slot ordering
// inside FaceFrameReader before any consumer touches the frame.
[InlineArray(88)]
internal struct Float88 { private float _e0; }

[StructLayout(LayoutKind.Sequential, Pack = 8)]
internal struct FaceTrackingFrameBodyNative
{
    public ulong qpc_sample_time;
    public ulong source_module_uuid_hash;
    public Float3 eye_origin_l;
    public Float3 eye_origin_r;
    public Float3 eye_gaze_l;
    public Float3 eye_gaze_r;
    public float eye_openness_l;
    public float eye_openness_r;
    public float pupil_dilation_l;
    public float pupil_dilation_r;
    public float eye_confidence_l;
    public float eye_confidence_r;
    public Float88 expressions;
    public uint flags;

    // v2 head pose fields. head_flags bit 0 = head valid this frame.
    // SubprocessManager will populate these when head data flows from the
    // upstream ReplyUpdate packet; default zero means "no head data".
    public float head_yaw;
    public float head_pitch;
    public float head_roll;
    public float head_pos_x;
    public float head_pos_y;
    public float head_pos_z;
    public uint head_flags;
}

// Per-slot seqlock layout. generation precedes body; offset validated by static_assert below.
[StructLayout(LayoutKind.Sequential, Pack = 8)]
internal struct FaceTrackingFrameSlotNative
{
    public ulong generation;    // seqlock: odd = mid-write, even = stable
    public FaceTrackingFrameBodyNative body;
}

/// <summary>
/// Publishes face/eye frames into the named shmem ring created by the driver.
/// The driver creates the segment; the host opens it for write. The seqlock
/// discipline matches the C++ Publish() helper in Protocol.h.
/// </summary>
public sealed class FrameWriter(string shmemName, HostLogger logger) : IDisposable
{
    private const uint Magic = 0x46544652u; // 'FTFR'
    private const uint ShmemVersion = 3; // v3: expressions grown to upstream-format 88 slots
    private const int RingSize = 32;

    // Public constant so callers can size their upstream-shape buffers
    // consistently. Mirrors protocol::FACETRACKING_UPSTREAM_EXPRESSION_COUNT.
    public const int UpstreamShapeCount = 88;

    // Byte offsets inside ShmemData. Layout must match Protocol.h exactly:
    //   uint32_t magic                        @  0
    //   uint32_t shmem_version                @  4
    //   uint32_t ring_size                    @  8
    //   uint32_t host_state                   @ 12 -- HostState enum (this side writes)
    //   atomic<uint64_t> host_heartbeat_qpc   @ 16 -- QPC at last tick (this side writes)
    //   uint32_t _reserved_header[2]          @ 24..31
    //   atomic<uint64_t> publish_index        @ 32
    //   FaceTrackingFrameSlot slots[RING_SIZE] @ 40
    private const int HostStateOffset = 12;
    private const int HostHeartbeatOffset = 16;
    private const int PublishIndexOffset = 32;
    private const int SlotsOffset = PublishIndexOffset + 8; // 40

    // host_state values mirror Protocol.h HostState enum.
    public const uint HostStateLegacy = 0;
    public const uint HostStatePublishing = 1;
    public const uint HostStateIdle = 2;
    public const uint HostStateDraining = 3;

    private static readonly int SlotSize = UnsafeHelper.SizeOf<FaceTrackingFrameSlotNative>();
    private static readonly int BodyOffsetInSlot =
        (int)Marshal.OffsetOf<FaceTrackingFrameSlotNative>(nameof(FaceTrackingFrameSlotNative.body));

    private MemoryMappedFile? _mmf;
    private MemoryMappedViewAccessor? _view;
    private long _localPublishIndex;
    private bool _sanitizationLogged;

    // Pinned base pointer acquired once at open, valid until Dispose.
    // All seqlock generation and publish_index reads/writes go through this
    // pointer so Volatile.Read/Write can give us acquire/release semantics on
    // x64 without the overhead of _view.ReadUInt64 / _view.Write.
    private unsafe byte* _basePtr;
    private bool _ptrAcquired;

    // Flipped to true by Dispose; checked at the top of WriteUnderSeqlock so
    // a publish racing with shutdown drops the frame instead of dereferencing
    // a torn-down mapping (AV would tear down the whole host).
    private volatile bool _disposed;
    private int _droppedFrameLogCounter;

    private const float InvalidSignalMin = 1_000_000.0f;

    private static bool IsInvalidSignal(float value)
    {
        return !float.IsFinite(value) || value >= InvalidSignalMin;
    }

    private static float SanitizeUnit(float value, float fallback, ref int replaced)
    {
        if (IsInvalidSignal(value)) { replaced++; return fallback; }
        float clamped = Math.Clamp(value, 0.0f, 1.0f);
        if (clamped != value)
        {
            replaced++;
        }

        return clamped;
    }

    private static float SanitizeFinite(float value, float fallback, ref int replaced)
    {
        if (IsInvalidSignal(value)) { replaced++; return fallback; }
        return value;
    }

    public async Task OpenAsync(CancellationToken ct)
    {
        await Task.Run(() =>
        {
            int totalSize = SlotsOffset + RingSize * SlotSize; // 40 + 32 * sizeof(slot)
            _mmf = MemoryMappedFile.OpenExisting(
                shmemName,
                MemoryMappedFileRights.ReadWrite);
            _view = _mmf.CreateViewAccessor(0, totalSize, MemoryMappedFileAccess.ReadWrite);

            // Validate header using the managed accessor (fine for one-time reads).
            uint magic = _view.ReadUInt32(0);
            uint version = _view.ReadUInt32(4);
            if (magic != Magic)
            {
                throw new InvalidOperationException(
                    $"Shmem magic mismatch: got 0x{magic:X8}, expected 0x{Magic:X8}");
            }

            if (version != ShmemVersion)
            {
                throw new InvalidOperationException(
                    $"Shmem version mismatch: got {version}, expected {ShmemVersion}");
            }

            // Acquire the raw base pointer for seqlock generation / publish_index fields.
            // AcquirePointer increments the SafeHandle refcount; released in Dispose.
            unsafe
            {
                byte* p = null;
                _view.SafeMemoryMappedViewHandle.AcquirePointer(ref p);
                _basePtr = p;
                _ptrAcquired = true;
            }

            logger.Info($"FrameWriter: opened '{shmemName}' ({totalSize} bytes). Magic OK.");
        }, ct);
    }

    /// <summary>
    /// Publish a frame to the shmem ring. The expression array is passed in
    /// upstream VRCFaceTracking.UnifiedExpressions order (88 entries); the
    /// driver remaps to our 63-slot ordering on the read side. The
    /// <paramref name="moduleUuidHash"/> must be pre-computed (FNV-1a-64 of
    /// the UUID) by the caller -- do not recompute per frame. Must be called
    /// from a single producer thread.
    /// </summary>
    public ValueTask PublishAsync(
        EyeFrameSink eye,
        HeadFrameSink head,
        ReadOnlySpan<float> upstreamShapes,
        bool eyeValid,
        bool exprValid,
        ulong moduleUuidHash,
        CancellationToken ct)
    {
        if (_view is null)
        {
            return ValueTask.CompletedTask;
        }

        ulong qpc = (ulong)System.Diagnostics.Stopwatch.GetTimestamp();

        var body = new FaceTrackingFrameBodyNative
        {
            qpc_sample_time = qpc,
            source_module_uuid_hash = moduleUuidHash,
            eye_openness_l = eye.LeftOpenness,
            eye_openness_r = eye.RightOpenness,
            pupil_dilation_l = eye.PupilDilationLeft,
            pupil_dilation_r = eye.PupilDilationRight,
            eye_confidence_l = eye.Left.Confidence,
            eye_confidence_r = eye.Right.Confidence,
            flags = (eyeValid ? 1u : 0u) | (exprValid ? 2u : 0u),
            head_yaw = head.Yaw,
            head_pitch = head.Pitch,
            head_roll = head.Roll,
            head_pos_x = head.PosX,
            head_pos_y = head.PosY,
            head_pos_z = head.PosZ,
            head_flags = head.IsValid ? 1u : 0u,
        };

        body.eye_origin_l[0] = eye.Left.OriginHmd.X;
        body.eye_origin_l[1] = eye.Left.OriginHmd.Y;
        body.eye_origin_l[2] = eye.Left.OriginHmd.Z;
        body.eye_origin_r[0] = eye.Right.OriginHmd.X;
        body.eye_origin_r[1] = eye.Right.OriginHmd.Y;
        body.eye_origin_r[2] = eye.Right.OriginHmd.Z;
        body.eye_gaze_l[0] = eye.Left.DirHmd.X;
        body.eye_gaze_l[1] = eye.Left.DirHmd.Y;
        body.eye_gaze_l[2] = eye.Left.DirHmd.Z;
        body.eye_gaze_r[0] = eye.Right.DirHmd.X;
        body.eye_gaze_r[1] = eye.Right.DirHmd.Y;
        body.eye_gaze_r[2] = eye.Right.DirHmd.Z;

        int count = Math.Min(upstreamShapes.Length, UpstreamShapeCount);
        for (int i = 0; i < count; i++)
        {
            body.expressions[i] = upstreamShapes[i];
        }

        SanitizeSignals(ref body);
        WriteUnderSeqlock(ref body);
        return ValueTask.CompletedTask;
    }

    private void SanitizeSignals(ref FaceTrackingFrameBodyNative body)
    {
        int replaced = 0;

        // Invalid openness falls back to neutral (0.5), not fully-open (1.0). A
        // 1.0 fallback forces an eye wide open on bad data, which -- blended into
        // the EyeLid OSC params -- reads as a stuck-open eye. 0.5 matches the pupil
        // fallback, and eye_confidence below drops to 0 so consumers can gate.
        body.eye_openness_l = SanitizeUnit(body.eye_openness_l, 0.5f, ref replaced);
        body.eye_openness_r = SanitizeUnit(body.eye_openness_r, 0.5f, ref replaced);
        body.pupil_dilation_l = SanitizeUnit(body.pupil_dilation_l, 0.5f, ref replaced);
        body.pupil_dilation_r = SanitizeUnit(body.pupil_dilation_r, 0.5f, ref replaced);
        body.eye_confidence_l = SanitizeUnit(body.eye_confidence_l, 0.0f, ref replaced);
        body.eye_confidence_r = SanitizeUnit(body.eye_confidence_r, 0.0f, ref replaced);

        for (int i = 0; i < 3; i++)
        {
            body.eye_origin_l[i] = SanitizeFinite(body.eye_origin_l[i], 0.0f, ref replaced);
            body.eye_origin_r[i] = SanitizeFinite(body.eye_origin_r[i], 0.0f, ref replaced);
            body.eye_gaze_l[i] = SanitizeFinite(body.eye_gaze_l[i], i == 2 ? -1.0f : 0.0f, ref replaced);
            body.eye_gaze_r[i] = SanitizeFinite(body.eye_gaze_r[i], i == 2 ? -1.0f : 0.0f, ref replaced);
        }

        for (int i = 0; i < UpstreamShapeCount; i++)
        {
            body.expressions[i] = SanitizeUnit(body.expressions[i], 0.0f, ref replaced);
        }

        body.head_yaw = SanitizeFinite(body.head_yaw, 0.0f, ref replaced);
        body.head_pitch = SanitizeFinite(body.head_pitch, 0.0f, ref replaced);
        body.head_roll = SanitizeFinite(body.head_roll, 0.0f, ref replaced);
        body.head_pos_x = SanitizeFinite(body.head_pos_x, 0.0f, ref replaced);
        body.head_pos_y = SanitizeFinite(body.head_pos_y, 0.0f, ref replaced);
        body.head_pos_z = SanitizeFinite(body.head_pos_z, 0.0f, ref replaced);

        if (replaced > 0 && !_sanitizationLogged)
        {
            _sanitizationLogged = true;
            logger.Warn($"FrameWriter: sanitized {replaced} invalid float signal(s) in module frame; further occurrences suppressed.");
        }
    }

    private unsafe void WriteUnderSeqlock(ref FaceTrackingFrameBodyNative body)
    {
        // Guard against the publish path racing the shutdown path: if Dispose
        // has released the SafeMemoryMappedViewHandle pointer, _basePtr is
        // null and the next dereference would AV the host. Drop the frame.
        if (_disposed || !_ptrAcquired || _basePtr == null || _view is null)
        {
            int n = Interlocked.Increment(ref _droppedFrameLogCounter);
            if ((n % 60) == 1)
            {
                logger.Warn("FrameWriter: dropping publish; mapping not available (shutdown race or Dispose called).");
            }

            return;
        }

        long next = Interlocked.Increment(ref _localPublishIndex);
        int slotIdx = (int)((next - 1) % RingSize);

        int slotOffset = SlotsOffset + slotIdx * SlotSize;
        long genOffset = slotOffset;
        int bodyOffset = slotOffset + BodyOffsetInSlot;

        long* genPtr = (long*)(_basePtr + genOffset);

        // Read generation, mark odd (mid-write), copy body, mark even (complete).
        // Volatile.Read/Write give acquire/release semantics matching C++ atomic<uint64_t>.
        long prevGen = Volatile.Read(ref *genPtr);

        Volatile.Write(ref *genPtr, prevGen + 1L);

        _view!.Write(bodyOffset, ref body);
        Thread.MemoryBarrier();

        Volatile.Write(ref *genPtr, prevGen + 2L);

        // Publish the new slot index so the driver's reader picks it up.
        long* publishPtr = (long*)(_basePtr + PublishIndexOffset);
        Volatile.Write(ref *publishPtr, Interlocked.Read(ref _localPublishIndex));

        // Heartbeat: bump the host's QPC stamp every published frame so the
        // driver can distinguish "host alive at 120 Hz" from "host wedged".
        WriteHeartbeatUnlocked();
    }

    /// <summary>
    /// Publish the host's current activity state. Atomic 32-bit write at
    /// the documented offset; the driver reads this with the heartbeat to
    /// decide whether stalled publish_index counts as a wedge.
    /// </summary>
    public unsafe void WriteHostState(uint state)
    {
        if (_disposed || !_ptrAcquired || _basePtr == null)
        {
            return;
        }

        int* statePtr = (int*)(_basePtr + HostStateOffset);
        Volatile.Write(ref *statePtr, (int)state);
    }

    /// <summary>
    /// Bump the heartbeat field without publishing a frame. Used by the
    /// idle-mode tick so a host with no module selected still proves it's
    /// alive within the driver's heartbeat threshold.
    /// </summary>
    public unsafe void WriteHeartbeatTick()
    {
        if (_disposed || !_ptrAcquired || _basePtr == null)
        {
            return;
        }

        WriteHeartbeatUnlocked();
    }

    private unsafe void WriteHeartbeatUnlocked()
    {
        long qpc = System.Diagnostics.Stopwatch.GetTimestamp();
        long* hbPtr = (long*)(_basePtr + HostHeartbeatOffset);
        Volatile.Write(ref *hbPtr, qpc);
    }

    public unsafe void Dispose()
    {
        // Set the disposed flag BEFORE tearing down the mapping so a publish
        // racing in flight observes _disposed=true and short-circuits before
        // touching _basePtr. The volatile write provides the release fence.
        _disposed = true;

        if (_ptrAcquired)
        {
            _view?.SafeMemoryMappedViewHandle.ReleasePointer();
            _ptrAcquired = false;
            _basePtr = null;
        }
        _view?.Dispose();
        _mmf?.Dispose();
        _view = null;
        _mmf = null;
    }
}
