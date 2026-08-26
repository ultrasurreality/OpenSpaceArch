# OpenSpaceArch

**Open Computational Architecture for Aerospace Hardware**

A rocket engine that is *computed*, not drawn. Give it thrust, chamber pressure and a
propellant pair; it derives the thermochemistry, sizes the chamber, solves the cooling,
routes the channels, grows the metal around them and writes a watertight STL.

The reference model is a **5 kN LOX/CH4 aerospike** with regenerative cooling on both the
shroud and the plug.

---

## The idea

Conventional CAD asks you to draw a wall and then drill channels through it. This does the
opposite, and the difference is the whole point:

1. **Physics produces numbers.** Thermochemistry → chamber sizing → Bartz heat flux →
   Barlow wall thickness → an iterative solve for how many cooling channels there must be
   and what cross-section each needs. No geometry yet.
2. **The coolant voids are built first**, as continuous signed-distance fields.
3. **The wall grows outward from the voids** by a per-station Barlow thickness, and the
   voids are subtracted. **The metal that remains is the engine.**

The outer surface is therefore a *consequence* of where the coolant had to go — never an
input. Everything downstream (ports, manifolds, flange) hangs off computed positions.

Channel start angles use golden-ratio phyllotaxis with a deterministic seeded jitter, so
the cooling jacket is aperiodic rather than an N-fold repeat — the same seed always yields
the same STL.

## What it currently produces

Values below come from `dotnet run -- --physics` at the default spec
(5 kN, Pc = 110 bar, O/F = 3.2, CuCrZr):

| Quantity | Value |
|---|---|
| Isp (sea level / vacuum) | 323.7 s / 349.6 s |
| Mass flow | 1.575 kg/s |
| Throat heat flux (Bartz, after film) | 69.1 MW/m² |
| Throat wall (Barlow, SF 1.5) | 1.01 mm |
| Cooling channels | 32 shroud (R 0.68 → 1.24 mm) + 24 spike |
| Overall length | 152.5 mm |
| Build time (0.3 mm voxels) | ~36 s, ~5.4 GB RAM |

The generated mesh is watertight. See [Honest limitations](#honest-limitations) before you
send anything to a printer.

## Requirements

- [.NET 9.0 SDK](https://dotnet.microsoft.com/download)
- Windows 10/11 x64
- 8 GB RAM for a 0.3 mm build; 24 GB+ if you push to 0.2 mm
- OpenGL 3.3+ for the interactive viewer (headless mode needs no GPU)
- Python 3.9+ with `numpy`, `scipy`, `trimesh`, `matplotlib` for `analyze_stl.py` (optional)

### Layout

The project consumes [LEAP 71's ShapeKernel](https://github.com/leap71/LEAP71_ShapeKernel)
**by source path**, so it expects a sibling directory:

```
your-workspace/
├── OpenSpaceArch/      ← this repository
└── PicoGK/
    └── LEAP71_ShapeKernel/
```

```bash
git clone https://github.com/ultrasurreality/OpenSpaceArch.git
git clone https://github.com/leap71/LEAP71_ShapeKernel.git PicoGK/LEAP71_ShapeKernel
cd OpenSpaceArch
dotnet build OpenSpaceArch.csproj -c Debug
```

The PicoGK kernel itself is vendored in `Core/` (a headless fork — see
[THIRD_PARTY_LICENSES.md](./THIRD_PARTY_LICENSES.md)), so you do not need it separately.

## Run

```bash
dotnet run -- --physics        # full physics chain, no voxels — a fast sanity check
dotnet run -- --headless       # build + export STL, cutaway and spec JSON at 0.3 mm
dotnet run -- --headless 0.2   # finer grid (~17 GB RAM, ~162 s)
dotnet run -- --sweep          # bounded design-space search + Pareto front
dotnet run                     # interactive viewer with a live digital twin
```

Exports land on your desktop: `AerospikeV4.stl`, `AerospikeV4_Cutaway.stl` and
`AerospikeV4_spec.json`. Then:

```bash
python analyze_stl.py ~/Desktop/AerospikeV4.stl --json ~/Desktop/AerospikeV4_spec.json
```

## Architecture

The pipeline, the invariants that must not be broken, and the parts deliberately left out
of the build path are documented in **[ARCHITECTURE.md](./ARCHITECTURE.md)**.

The one rule worth repeating here: **every `IImplicit.fSignedDistance` must be spatially
continuous.** A per-channel step function in a distance field breaks marching cubes and
silently produces a non-watertight mesh. This project learned that the expensive way.

Above the generator sits a bounded search layer: it samples a 6-dimensional box
(Pc, O/F, contraction ratio, L*, safety factor, channel twist), runs the physics on each
sample, discards invalid designs and computes a real non-dominated Pareto front over
Isp, mass and thermal stress. Scoring weights are exposed as sliders — **the human holds
the weights, the machine only ranks**.

## Honest limitations

This is a generator, not a qualified engine. Known open items:

- **~31 disconnected shells** in the last headless run, where a printable part wants one.
  The mesh is watertight and the fragments are small, but their origin is undiagnosed.
- **Voxel mass has not converged.** Between 0.4 mm and 0.2 mm the computed mass moves by
  15%, so voxel-derived mass and thrust-to-weight are grid-dependent numbers — never
  compare them across resolutions. The Pareto front uses an analytic mass estimate instead.
- **Overhang.** Printed spike-tip-down, several thousand downward-facing points still sit
  below the 45° self-supporting angle and would need supports.
- **The plug interior is modelled as longitudinal channels**, not the branching truss such
  a geometry really wants. A skeleton router exists but is not wired in.
- **No CFD, no FEA, no combustion stability analysis.** The thermal model is 1-D Bartz with
  a film-cooling knockdown; the structural model is Barlow plus a thermal-stress warning.

## License

OpenSpaceArch is licensed under the
[GNU Affero General Public License v3.0](./LICENSE). You may use, modify and distribute it
under those terms; a modified version made available over a network must also be offered
under the same license.

Third-party components — PicoGK and ShapeKernel (Apache 2.0), Silk.NET and ImGui.NET (MIT) —
are documented in [THIRD_PARTY_LICENSES.md](./THIRD_PARTY_LICENSES.md).

## Export Control Notice

This software is published as open source and freely available to the general public without restriction. It implements general scientific, mathematical, and engineering principles commonly taught in schools, colleges, and universities.

This software qualifies as "publicly available" under the EAR (15 CFR §734.3(b)(3), §734.7) and as information in the "public domain" under ITAR (22 CFR §120.11).

Users are responsible for ensuring their use complies with all applicable U.S. export control laws, including ITAR (22 CFR §§120-130) and EAR (15 CFR §§730-774).

Contributors must NOT submit material subject to ITAR/EAR restrictions, including classified data, information received under NDA, or data from defense contracts.

This notice does not constitute legal advice.

## Engineering Disclaimer

This software is a computational engineering tool intended solely to assist in design and development processes. It requires considerable engineering skill, expertise, and professional judgment for correct use and interpretation of computed results.

This software is **not a substitute** for independent engineering analysis, physical prototype testing, destructive and non-destructive testing of manufactured components, or compliance with applicable aerospace standards.

## Contributing

See [CONTRIBUTING.md](./CONTRIBUTING.md). Contributions are accepted under the same AGPL
v3.0 license; a CLA check runs on every pull request.

## References

Physics and engineering sources are listed in [REFERENCES.md](./REFERENCES.md).
