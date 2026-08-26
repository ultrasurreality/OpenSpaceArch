// SPDX-License-Identifier: AGPL-3.0-or-later
//
// OpenSpaceArch — Open Computational Architecture for Aerospace Hardware
// Copyright (C) 2025-2026 ultrasurreality
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

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
