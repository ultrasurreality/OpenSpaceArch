# OpenSpaceArch — Architecture

> A 5 kN LOX/CH4 aerospike generated spec→STL on the PicoGK voxel kernel, wrapped in a
> bounded design-search layer and a live physics-driven viewer. This document describes
> how the generator is put together and which invariants must not be broken.

## THE ONE RULE

**Geometry EMERGES from physics fields. Nobody "places" geometry.**

If you write code that says "put channel at position X with radius Y" — STOP. Instead:
compute physics, model the fluid VOIDS, grow walls outward from the voids as signed-distance
fields, subtract the voids. The metal that remains IS the engine; the outer surface is a
consequence, never an input.

## Pipeline (read top→bottom; a bounded search loop wraps it)

```
INPUTS (AeroSpec, ~6):  F_thrust, Pc, O/F, voxelSize, channelMode, printer limits
        │   everything below is COMPUTED and written back into the one AeroSpec object
        ▼
PHYSICS (forward algebra):
  Thermochemistry  → Tc, γ, c*, Cf, Isp, ṁ        (12-row CEA lookup; Cp_transport=2200 NOT 6795)
  ChamberSizing    → throat area, spike/shroud radii, all z-stations
  HeatTransfer     → Bartz throat flux, Barlow wall t(z), iterative channel count/size (fit loop)
  DerivePortPositions → ports as OUTPUT (v7)
        ▼
FIELDS:  PhysicsFields projects 1-D physics onto the 3-D voxel grid as
         ScalarFields (Temperature/Stress/HeatFlux) + CoolantFlow VectorField
        ▼
ROUTING: ChannelRouter → analytical helical channel spines (golden-phyllotaxis,
         deterministic seeded symmetry-breaking). NOT SurfaceTurtle (retained for
         reference only, see below).
        ▼
GEOMETRY:  EngineAssembly.Build → FluidFirst.Build  (the production path)
  1. Channel voids   = RoutedChannelFieldImplicit  (TRUE superellipse cross-section,
                        print-angle exponent, port widening, turbulator ribs) → Voxels per group
  2. Mutual exclusion = each flow BoolSubtract(other.voxOffset(minWall))   (min wall guaranteed)
  3. Outer wall      = GrowVariableWall → VariableWallImplicit  (per-z Barlow wall, NOT a scalar)
  4. Subtract voids, add spike / LOX bore / manifolds / ports / flange (Lattice helpers)
  5. Smoothen(0.3f)   (NO OverOffset), then post-smoothing bores
        ▲
VALIDATION:  EngineValidator → 16 slack-bearing predicates (combustion, throat, wall burst,
             cooling, voxel resolution + thermal stress, coolant capacity, acoustic
             separation, residence time). SpatialValidator → microsecond analytical pre-check.
        ▲
SEARCH (CSP layer):  OuterSweep samples a bounded 6-D box (SweepVariables: Pc,OF,CR,L*,SF,Twist;
             uniform or center-biased Gaussian), runs the inner physics, filters by validity,
             scores via the shared EngineEvaluation; SweepResult re-scores on demand from live
             ScoringWeights. DesignSweep also computes a REAL non-dominated Pareto front
             (max Isp · min mass · min σ_thermal). Human holds the weights, machine ranks.
             pymoo (PymooSweep.py) = a SIMPLIFIED, NON-AUTHORITATIVE NSGA-III screen — clearly
             labelled as such in code + UI; the C# physics is the source of truth.
```

CHANNELS, not TPMS/gyroid — channels have inlet, outlet, flow direction, mass-flow cross-section.
("Channels not gyroids" is an ENGINE rule; heat exchangers use other primitives.)

## Key invariants (do not break)

- **SDF continuity:** every `IImplicit.fSignedDistance` must be spatially continuous. Per-index /
  per-channel step functions break marching cubes → non-watertight mesh. Use spatial functions
  (`atan2(y,x)`) or smooth blends. (This is the project's #1 hard-won lesson.)
- **Fluid-first order:** voids → mutual-exclusion offsets → grow wall → subtract → features → smooth.
- **One source of truth:** physics constants live in C# (`Thermochemistry`/`HeatTransfer`); do not
  fork them (pymoo's copy is explicitly marked non-authoritative).
- **Shared evaluation:** validity checks + scoring live once in `Engine/CSP/ScoringWeights.cs`
  (`EngineEvaluation`); `DesignSweep`, `OuterSweep`, `SweepResult` all delegate to it (no copies).

## Run modes (`Program.cs`)

```
dotnet run -- --physics    # print the full physics chain, no voxels (fast sanity check)
dotnet run -- --headless   # full build + STL/cutaway/JSON export at 0.3 mm (~5.4 GB, ~36 s)
dotnet run -- --headless 0.2   # optional voxel override (16.8 GB, ~162 s) — practical maximum
dotnet run -- --sweep      # bounded design-space search + real Pareto front
dotnet run -- --single     # validate one variant
dotnet run                 # interactive viewer + live digital twin (0.5 mm core grid)
```

Voxel sizes are intentionally distinct: **0.3 mm** headless/STL (AeroSpec default), 0.5 mm viewer
core grid + ControlPanel default.

**C12 is not a sufficient resolution check.** `voxel < throatGap/2` passes at 1.5 mm (the annular
gap is 3.07 mm at 110 bar), but the binding feature is the **throat wall: 1.01 mm**, i.e. 2.5 voxels
at the old 0.4 mm default. Measured on 64 GB (peak RSS / wall-clock / voxel mass):
0.4 mm → 2.59 GB / 16 s / 1.048 kg · 0.3 mm → 5.36 GB / 36 s / 0.940 kg ·
0.25 mm → 9.61 GB / 65 s / 0.931 kg · 0.2 mm → 16.83 GB / 162 s / 0.888 kg.
Memory scales as voxel^-2.7. **Voxel mass has not converged even at 0.2 mm** (0.4→0.2 spans -15.3%),
so treat voxel mass and T/W as grid-dependent — never compare them across resolutions. The Pareto
front is unaffected: `DesignSweep` scores on the analytic `EngineEvaluation.MassEstimateKg`.

## Key PicoGK APIs

```csharp
new Voxels(iImplicit, bbox);          // render a continuous SDF field to voxels (the channel/wall path)
Lattice lat = new(); lat.AddBeam(...); // beams → voxels (manifolds, ports, flange, bores)
v.voxOffset(mm);                       // grow/shrink (mutual exclusion, wall margins)
v.BoolAdd / v.BoolSubtract;            // fluid-first union / carve
v.Smoothen(0.3f);                      // biological surface (NO OverOffset in the live path)
```

## Files that are NOT in the build path

- `Engine/SurfaceTurtle.cs` — surface-following turtle walk, retained for reference. The production
  router builds analytical helices instead (faster, and free of voxel-discretisation artefacts).
  Keep it for a future non-revolution surface.
- `Engine/EngineBodyImplicit.cs` — the "one fused SDF for the whole core" idea; not wired in.
- `Engine/SpikeTreeRouter.cs` — compiling skeleton of a 3-D branching-truss spike interior,
  reachable behind `AeroSpec.useSpikeTree` (default false, currently read nowhere).
- `Engine/ChannelFieldImplicit.cs` — used by the older `MeshBased_v4` / `Implicit_v5` modes only.

## Known frontier (open engineering questions, not code defects)

- **Spike interior topology.** The plug is currently cooled by longitudinal helical channels. A
  3-D branching truss reaches the gas-side heat load with less pressure drop and is the intended
  successor; it needs a real tree router plus a matching `IImplicit`, not just the skeleton.
- **Coolant-side margin on a methalox plug** is the least-constrained part of the model. Published
  hot-fire data is the calibration target; until then treat spike-side cooling as an estimate.
- **Bartz throat diameter for an annular throat** has more than one defensible definition, and the
  σ(M) wall-temperature correction uses the standard ω = 0.6 exponent — a textbook value, not one
  measured on this geometry.
- **Disconnected bodies.** The last headless run reported 31 shells where a printable part wants 1.
  The mesh is watertight; the fragments are small and their origin is not yet diagnosed.

## Build & verify

```bash
dotnet build OpenSpaceArch.csproj -c Debug
dotnet run -- --headless
python analyze_stl.py ~/Desktop/AerospikeV4.stl --json ~/Desktop/AerospikeV4_spec.json
```

## References

Physics sources are listed in [REFERENCES.md](./REFERENCES.md). Third-party components and their
licences are listed in [THIRD_PARTY_LICENSES.md](./THIRD_PARTY_LICENSES.md).
