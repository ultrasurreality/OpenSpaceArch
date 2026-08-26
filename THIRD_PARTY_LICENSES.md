# Third-Party Licenses

OpenSpaceArch itself is licensed under the **GNU Affero General Public License v3.0**
(see [LICENSE](./LICENSE)). It builds on the third-party components listed below. All of
them carry permissive licences that are compatible with AGPL v3.0, and their own notices
are preserved in the files they cover.

## LEAP 71 — PicoGK

- **Project:** PicoGK ("Pico Geometry Kernel") — https://github.com/leap71/PicoGK
- **License:** Apache License 2.0
- **Copyright:** © 2023-2026 LEAP 71
- **Used as:** the voxel/SDF geometry kernel this project is built on.

`Core/` in this repository is a **fork** of PicoGK's C# layer, reduced to a headless build
(the upstream viewer is stripped). Every file in `Core/` retains its original
`SPDX-License-Identifier: Apache-2.0` header and LEAP 71 copyright notice. Modifications
are limited to removing viewer-dependent code paths.

`Core/native/` contains the prebuilt PicoGK runtime library (`picogk.1.7.dll`) and the
native libraries it links against (`blosc`, `lz4`, `tbb12`, `zlib1`, `zstd`). These are
redistributed unmodified as published by the PicoGK project; PicoGK is in turn a layer over
[OpenVDB](https://www.openvdb.org/) and its dependency stack. Refer to the PicoGK
repository for the authoritative licence terms of those binaries.

## LEAP 71 — ShapeKernel

- **Project:** LEAP71_ShapeKernel — https://github.com/leap71/LEAP71_ShapeKernel
- **License:** Apache License 2.0
- **Copyright:** © 2023-2026 LEAP 71
- **Used as:** shape/lattice helpers (`Lattice`, `LatticeManifold`, `Frames`, modulations).

Consumed by source inclusion via `<Compile Include="../PicoGK/LEAP71_ShapeKernel/...">` in
`OpenSpaceArch.csproj`, excluding the visualisation files that depend on the stripped
viewer API. Original Apache-2.0 headers are retained in every included file.

## LEAP 71 — SurfaceTurtleWalk example

`Engine/SurfaceTurtle.cs` adapts LEAP 71's publicly published `SurfaceTurtleWalk` example
(`Ex_SurfaceTurtleWalkShowCase`), released by its author under **CC0-1.0** (public domain
dedication). That file therefore carries a dual
`SPDX-License-Identifier: CC0-1.0 AND AGPL-3.0-or-later` notice: the adapted portions stay
CC0, and the surrounding OpenSpaceArch code is AGPL-3.0-or-later.

## Silk.NET

- **Project:** Silk.NET — https://github.com/dotnet/Silk.NET
- **License:** MIT
- **Packages:** `Silk.NET.OpenGL`, `Silk.NET.Windowing`, `Silk.NET.Input`,
  `Silk.NET.Maths`, `Silk.NET.OpenGL.Extensions.ImGui` (2.22.0)
- **Used as:** windowing, input and OpenGL bindings for the interactive viewer.

## ImGui.NET / Dear ImGui

- **Project:** ImGui.NET — https://github.com/ImGuiNET/ImGui.NET (1.91.6.1),
  wrapping [Dear ImGui](https://github.com/ocornut/imgui)
- **License:** MIT (both)
- **Used as:** the viewer's control, constraint and sweep panels.

Consumed as NuGet packages; they are not vendored into this repository.

## Python tooling

`analyze_stl.py` and `Engine/CSP/PymooSweep.py` import `numpy`, `scipy`, `trimesh`,
`matplotlib` and `pymoo` at runtime. These are not distributed with OpenSpaceArch — the
user installs them separately, under their own licences.

---

If you believe an attribution here is incomplete or incorrect, please open an issue.
