# OpenSpaceArch

**Open Computational Architecture for Aerospace Hardware**

An open-source system that turns physics equations into 3D-printable aerospace components. Input thrust, propellant, and chamber pressure — get a complete rocket engine geometry in 30 seconds.

---

## What It Does

- **Computes** nozzle contour (Rao's method), heat transfer (Bartz equation), cooling channel sizing — all from first principles
- **Generates** watertight 3D geometry via [PicoGK](https://github.com/leap71/PicoGK) voxel engine — three-layer walls with hidden regenerative cooling channels
- **Exports** production-ready STL files for metal 3D printing

This is not CAD. Not simulation. This is **computational engineering** — first-principles physics encoded into a deterministic algorithm that generates real hardware.

## Quick Start

### Prerequisites

- [.NET 9.0 SDK](https://dotnet.microsoft.com/download)
- Windows 10/11 (64-bit)

### Run

```bash
git clone https://github.com/ultrasurreality/OpenSpaceArch.git
cd OpenSpaceArch
dotnet run
```

Select "Bell Nozzle Engine" from the menu. In ~30 seconds you get:
- Engineering calculations log (Isp, c*, thrust coefficient, heat flux map)
- 3D viewer with the generated engine
- STL file on your desktop

### Example Output

```
Propellant:    LOX/Methane
Thrust:        5000 N
Pc:            55.0 bar
c*:            1772.9 m/s
Isp:           305.4 s
CF:            1.690
Throat d:      26.17 mm
Chamber d:     52.35 mm
Exit d:        70.33 mm
Expansion:     7.2:1
Rao angles:    θn=30.9° θe=11.0°
Max heat flux: 41.40 MW/m²
```

## Architecture

```
┌──────────────────────────────────────────────┐
│  Engines/          Concrete engine generators │  ← you work here
├──────────────────────────────────────────────┤
│  Physics/          First-principles math      │  ← pure calculations
│  Geometry/         PicoGK wrappers            │  ← voxels, SDF, VDB
├──────────────────────────────────────────────┤
│  PicoGK            Voxel geometry kernel      │  ← foundation (LEAP 71)
└──────────────────────────────────────────────┘
```

### Physics/

Pure engineering mathematics. No geometry, no dependencies beyond `System.Math`.

| File | What it computes |
|------|-----------------|
| `Propellants.cs` | Thermochemistry: Tc, γ, c*, molecular weight (NASA CEA data) |
| `Nozzle.cs` | Rao bell nozzle contour, isentropic relations, CF, Isp |
| `HeatTransfer.cs` | Bartz gas-side hg, Dittus-Boelter coolant-side hc, wall temperature |
| `Designer.cs` | Orchestrator: thrust + propellant + Pc → complete engine specification |

### Geometry/

PicoGK abstractions for memory-safe voxel operations.

| File | What it does |
|------|-------------|
| `RevolutionSDF.cs` | Implicit signed distance field for bodies of revolution |
| `MemoryGuard.cs` | Windows Job Object hard memory limit (prevents system freeze) |
| `VdbStaging.cs` | Save/load voxel fields to disk between stages (RAM economy) |

### Engines/

Concrete engine generators that combine Physics + Geometry.

| File | What it generates |
|------|------------------|
| `BellEngine.cs` | De Laval bell nozzle with regenerative cooling, injector, mounting flange |

## Engineering Basis

All dimensions are computed from physics, not hardcoded:

| Calculation | Method | Reference |
|-------------|--------|-----------|
| Nozzle contour | Rao parabolic approximation (quadratic Bézier) | NASA SP-8120 (1976) |
| Nozzle angles θn, θe | Interpolated from Rao's digitized charts | Rao (1961) |
| Throat geometry | Circular arcs Ru=1.5Rt (upstream), Rd=0.382Rt (downstream) | Sutton, RPE |
| Gas-side heat transfer | Bartz semi-empirical correlation | Bartz (1957) |
| Coolant-side heat transfer | Dittus-Boelter correlation | Dittus & Boelter (1930) |
| Characteristic velocity c* | Thermodynamic first principles | NASA CEA |
| Expansion ratio | Isentropic flow, optimal for sea-level (Pe=Pa) | Anderson, Gas Dynamics |
| Thrust coefficient CF | Momentum + pressure thrust | Sutton, RPE |

## Background

Until now, only one company in the world could computationally generate complete rocket engine geometry from abstract specifications — [LEAP 71](https://leap71.com) from Dubai, with their proprietary [Noyron](https://leap71.com/noyron/) system. Their engines work on the first attempt. Their code is closed.

OpenSpaceArch is the open alternative. Built on LEAP 71's open-source [PicoGK](https://github.com/leap71/PicoGK) geometry kernel, it implements the same paradigm: **physics-first computational engineering** — where geometry is derived from equations, not drawn by hand.

Today — rocket engines. Tomorrow — heat exchangers, turbines, satellite structures. The architecture is extensible: add your physics module, get a new class of aerospace components.

## System Requirements

| Resource | Minimum | Recommended |
|----------|---------|-------------|
| RAM | 4 GB | 8+ GB |
| .NET | 9.0 | 9.0 |
| OS | Windows 10 x64 | Windows 11 x64 |
| GPU | OpenGL 3.3 | OpenGL 4.0+ |

The built-in `MemoryGuard` enforces a hard memory limit via Windows Job Objects, preventing system freezes during heavy voxel operations.

## License

OpenSpaceArch is licensed under the [GNU Affero General Public License v3.0](./LICENSE).

You are free to use, modify, and distribute this software under the terms of the AGPL v3.0. Any modified version that is made available over a network must also be made available under the same license.

See [LICENSE](./LICENSE) for the full text.

### Third-Party Components

OpenSpaceArch depends on [PicoGK](https://github.com/leap71/PicoGK) by LEAP 71, licensed under the Apache License 2.0. Apache 2.0 is compatible with AGPL v3.0. PicoGK's license and attribution notices are preserved per its license terms.

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

See [CONTRIBUTING.md](./CONTRIBUTING.md) for guidelines.

## References

See [REFERENCES.md](./REFERENCES.md) for the complete list of academic and engineering sources used in this project.
