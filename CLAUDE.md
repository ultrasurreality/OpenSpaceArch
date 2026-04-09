# OpenSpaceArch v4 — Field-Driven Aerospike Engine

## STATUS: CLEAN SLATE

v3 (geometric approach) archived at `../OpenSpaceArch_v3_geometric_archive/`.
v4 starts from scratch with field-driven paradigm.

## THE ONE RULE

**Geometry EMERGES from physics fields. Nobody "places" geometry.**

If you find yourself writing code that says "put channel at position X with radius Y" — STOP. That's the old paradigm. Instead: fill a ScalarField with physics, let an IImplicit generate geometry from the field.

## What to keep from v3

- `Engine/AeroSpec.cs` — parameter structure (good)
- `Engine/Thermochemistry.cs` — CEA lookup (correct physics)
- `Engine/ChamberSizing.cs` — nozzle sizing (correct physics, but outputs should fill fields)
- `Engine/HeatTransfer.cs` — Bartz equation (correct physics, but q should → ScalarField)
- `Engine/RevolutionSDF.cs` — perfect SDF revolution bodies (for gas-side walls)
- `OrganicDemo.cs` — proven organic pipeline (BoolAdd + OverOffset pattern)
- `analyze_stl.py` — STL analysis tool (run after every build)
- `Program.cs` — entry point with modes

## What to DELETE/rewrite

- `Engine/EngineAssembly.cs` — rewrite for field-driven assembly
- `Engine/FluidVolumes.cs` — rewrite: no explicit channel placement
- `Engine/BranchingChannels.cs` — DELETE: explicit branching is old paradigm
- `Engine/StructuralFeatures.cs` — rewrite: features should emerge from stress field

## Architecture: FluidFirst pipeline

```
Physics:     Thermochemistry → ChamberSizing → HeatTransfer
                                                    ↓
             mDot_fuel, mDot_ox, q(z), wallThickness(z)
                                                    ↓
Channels:    N = 2πr/(2·r_ch+wall)    r_ch = sqrt(ṁ/(N·ρ·v·π))
                         ↓
FluidFirst:  1. Build fluid voids (chamber + helical channels + manifold)
             2. Mutual exclusion (min wall between flows)
             3. voxOffset → walls GROW from voids
             4. Subtract voids → solid body
             5. OverOffset + Smoothen → organic
```

CHANNELS, not TPMS/gyroid. Channels have inlet, outlet, flow direction, mass-flow-derived cross-section.

## Key PicoGK APIs

```csharp
Lattice lat = new(); lat.AddBeam(p0, p1, r0, r1);  // channel segment
Voxels v = new(lat);                                 // lattice → voxels
v.voxOffset(thickness);                              // grow walls
v.OverOffset(2.0f, 0f);                             // organic fusion
v.Smoothen(0.5f);                                   // biological surface
```

## References

- ShapeKernel (pipes): `PicoGK/LEAP71_ShapeKernel/ShapeKernel/`
- LatticeLibrary: `PicoGK/LEAP71_LatticeLibrary/`
- Full context: `.claude/projects/C--Users-ultra/memory/project_openspacearch_v4.md`

## Build & verify

```bash
dotnet build && echo "1" | dotnet run    # generate + view
python analyze_stl.py ~/Desktop/AerospikeV3.stl --json ~/Desktop/AerospikeV3_spec.json
```
