// StageResult.cs — per-stage progress snapshot for the cinematic viewer.
// FluidFirst.Build emits one StageResult after every major pipeline step,
// so the Viewer can reveal the engine materializing piece by piece.

using PicoGK;

namespace OpenSpaceArch.Engine;

public enum StageId
{
    Channels,            // cooling channel voids (Lattice tubes)
    Nozzle,              // convergent section + throat (gas path)
    Chamber,             // cylindrical combustion section (gas path)
    Dome,                // injector dome closure (gas path)
    Spike,               // aerospike nozzle cone (solid body)
    AllVoids,            // union of all fluid voids
    Shell,               // solid walls (offset - voids)
    Collector,           // toroidal manifold at top (hot CH4 out)
    Inlet,               // toroidal manifold at bottom (cold CH4 in)
    FeedPorts,           // fuel/lox/igniter ports
    SpikeVanes,          // radial structural bridges
    TopFlange,           // mounting plate
    Final                // final combined engine
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
