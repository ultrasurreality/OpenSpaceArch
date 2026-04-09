// StageResult.cs — per-stage progress snapshot for the cinematic viewer.
// FluidFirst.Build emits one StageResult after every major pipeline step,
// so the Viewer can reveal the engine materializing piece by piece.

using PicoGK;

namespace OpenSpaceArch.Engine;

public enum StageId
{
    AnalyticalSdfs,      // shroud + spike revolution SDFs voxelized
    ChannelSdfs,         // routed channel SDFs voxelized alone
    CoreBodySlab,        // one Z-slice slab of EngineBodyImplicit (multiple per build)
    CoreBody,            // EngineBodyImplicit → one voxel field (atomic mode)
    AxialManifold,       // central LOX manifold
    ShroudCollector,     // toroidal manifold at top
    ShroudInlet,         // toroidal manifold at bottom
    FeedPorts,           // fuel/lox/igniter ports
    SpikeVanes,          // radial structural bridges
    TopFlange,           // mounting plate
    Smoothen,            // light smoothing pass
    PostBores,           // injector + film cooling + bolt holes
    Final                // final combined voxels
}

public enum BuildMode
{
    Atomic,         // each stage = single voxelization (fast, less wow)
    ZSliceSlabs,    // CoreBody split into N horizontal slabs revealed slab-by-slab
}

public readonly record struct StageResult(
    StageId Stage,
    Mesh Mesh,
    string Description,
    float ElapsedSec);
